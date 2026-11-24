#!/usr/bin/env python3
"""Local server for the YIG tuning web demo (Python standard library only).

    python3 web/server.py                 start MATLAB and serve the page
    python3 web/server.py --no-matlab     recorded runs only
    python3 web/server.py --record        re-record the fallback runs, then exit

Then open http://localhost:8137.  MATLAB is found on PATH, in /Applications,
or through the MATLAB_CMD environment variable.
"""
import argparse, atexit, glob, json, os, shutil, signal, socketserver, subprocess, sys, tempfile
import threading, time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

WEB = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(WEB, "static")
RECORDED = os.path.join(STATIC, "recorded")
BOARDS = sorted(os.path.splitext(os.path.basename(p))[0]
                for p in glob.glob(os.path.join(WEB, "..", "devices", "*.m")))


def find_matlab():
    if os.environ.get("MATLAB_CMD"):
        return os.environ["MATLAB_CMD"]
    found = sorted(glob.glob("/Applications/MATLAB_R*.app/bin/matlab"))
    return found[-1] if found else shutil.which("matlab")


class Worker:
    """One background MATLAB process, one job at a time."""

    def __init__(self, cmd):
        self.lock = threading.Lock()
        self.n = 0
        self.dir = tempfile.mkdtemp(prefix="yig_web_")
        self.log_path = os.path.join(self.dir, "matlab.log")
        self._beat_once()
        threading.Thread(target=self._beat, daemon=True).start()
        q = lambda s: s.replace("'", "''")
        self.log = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            [cmd, "-batch", f"cd('{q(WEB)}'); yig_web_worker('{q(self.dir)}')"],
            stdout=self.log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        atexit.register(self.stop)

    def _beat_once(self):
        with open(os.path.join(self.dir, "heartbeat"), "w") as f:
            f.write("1")

    def _beat(self):
        while True:
            time.sleep(2)
            try:
                self._beat_once()
            except OSError:
                return

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
        shutil.rmtree(self.dir, ignore_errors=True)

    def status(self):
        if self.proc.poll() is not None:
            return {"state": "stopped"}
        p = os.path.join(self.dir, "ready.json")
        if not os.path.exists(p):
            return {"state": "starting"}
        with open(p) as f:
            return {"state": "busy" if self.lock.locked() else "ready", **json.load(f)}

    def run(self, job, timeout=600):
        with self.lock:
            if self.status()["state"] in ("stopped", "starting"):
                raise RuntimeError("MATLAB is not ready")
            self.n += 1
            job = {**job, "id": self.n}
            res = os.path.join(self.dir, "result.json")
            tmp = os.path.join(self.dir, "job.tmp")
            with open(tmp, "w") as f:
                json.dump(job, f)
            os.replace(tmp, os.path.join(self.dir, "job.json"))
            end = time.time() + timeout
            while time.time() < end:
                if os.path.exists(res):
                    with open(res) as f:
                        r = json.load(f)
                    os.remove(res)
                    if r.get("id") == self.n:      # else: a job that timed out earlier
                        return r
                if self.proc.poll() is not None:
                    raise RuntimeError("MATLAB stopped; see the terminal")
                time.sleep(0.02)
            raise RuntimeError("MATLAB took too long")


def number(d, key, lo, hi):
    v = d.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
        raise ValueError(f"{key} must be a number from {lo} to {hi}")
    return v


def check(d):
    """Build a clean job from a request body."""
    if not isinstance(d, dict):
        raise ValueError("expected a JSON object")
    cmd = d.get("cmd")
    if cmd in ("session", "simulink"):
        if d.get("board") not in BOARDS:
            raise ValueError("unknown board")
        job = {"cmd": cmd, "board": d["board"], "strategy": int(number(d, "strategy", 1, 2))}
        if cmd == "session":
            job["seed"] = int(number(d, "seed", 0, 99999))
        return job
    if cmd == "step":
        return {"cmd": cmd, "f_GHz": number(d, "f_GHz", 2, 18), "T": number(d, "T", -40, 85),
                "B_mT": number(d, "B_mT", -5, 5), "dur_ms": number(d, "dur_ms", 5, 100)}
    raise ValueError("unknown command")


class Handler(SimpleHTTPRequestHandler):
    worker = None
    port = 8137

    def __init__(self, *a, **k):
        super().__init__(*a, directory=STATIC, **k)

    def log_message(self, *a):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _local(self):
        ok = {f"localhost:{self.port}", f"127.0.0.1:{self.port}"}
        origin = self.headers.get("Origin")
        return self.headers.get("Host") in ok and \
            (origin is None or origin.split("://")[-1] in ok)

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._local():
            return self._json(403, {"ok": False, "error": "local only"})
        if self.path == "/api/status":
            w = Handler.worker
            return self._json(200, {**(w.status() if w else {"state": "off"}), "boards": BOARDS})
        super().do_GET()

    def do_POST(self):
        if not self._local() or self.headers.get("Content-Type") != "application/json":
            return self._json(403, {"ok": False, "error": "local only"})
        try:
            if self.path != "/api/run" or Handler.worker is None:
                raise ValueError("MATLAB is not running")
            n = int(self.headers.get("Content-Length") or 0)
            if n > 10000:
                raise ValueError("request too large")
            self._json(200, Handler.worker.run(check(json.loads(self.rfile.read(n)))))
        except (ValueError, RuntimeError) as e:
            self._json(400, {"ok": False, "error": str(e)})


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def server_bind(self):
        # skip http.server's hostname lookup, which can stall for many
        # seconds on a network without working DNS
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


def record(w):
    """Save the Simulink full test for every board and strategy."""
    os.makedirs(RECORDED, exist_ok=True)
    while w.status()["state"] == "starting":
        time.sleep(0.5)
    index = []
    for b in BOARDS:
        for s in (1, 2):
            print(f"{b}, strategy {s} ...", end=" ", flush=True)
            r = w.run(check({"cmd": "simulink", "board": b, "strategy": s}), 900)
            if not r["ok"]:
                sys.exit(r["error"])
            r["recorded"] = time.strftime("%Y-%m-%d")
            name = f"{b}_s{s}.json"
            with open(os.path.join(RECORDED, name), "w") as f:
                json.dump(r, f, separators=(",", ":"))
            index.append({"board": b, "name": r["board"]["name"], "strategy": s, "file": name})
            print(f"{r['wall_s']:.1f} s")
    with open(os.path.join(RECORDED, "index.json"), "w") as f:
        json.dump(index, f, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8137)
    ap.add_argument("--no-matlab", action="store_true")
    ap.add_argument("--record", action="store_true")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # so atexit stops MATLAB
    cmd = None if a.no_matlab else find_matlab()
    if cmd:
        Handler.worker = Worker(cmd)
    if a.record:
        if not cmd:
            sys.exit("MATLAB not found")
        return record(Handler.worker)
    Handler.port = a.port
    srv = Server(("127.0.0.1", a.port), Handler)
    print(f"YIG demo: http://localhost:{a.port}   MATLAB: {cmd or 'off (recorded runs only)'}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
