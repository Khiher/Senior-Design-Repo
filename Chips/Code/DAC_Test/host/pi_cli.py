#!/usr/bin/env python3
"""Command-line control of the chip's phase interpolators (PIs) through the
NUCLEO-L152RE firmware in ../src/main.c.

Usage:
    python pi_cli.py                      interactive shell (port auto-detected)
    python pi_cli.py status               run one command and exit
    python pi_cli.py up 1 20
    python pi_cli.py --port /dev/cu.usbmodem1103 goto 2 -5

Commands (shell or one-shot):
    status                  PI positions, DAC levels, EOM_ERR level
    up <pi> [n]             step PI 1 or 2 up n times (default 1), stops at 7
    down <pi> [n]           step down (alias: dn), stops at 0
    goto <pi> <pos>         step to position 0..7
    reset [pi]              8 DN pulses, position = 0 (both PIs if omitted)
    jog <pi>                single keys: k/j = up/down, 0-7 = go to, r = reset, q = quit
    eom [ms]                count EOM errors for ms (default 100)
    dac <A|B> <mV>          set a DAC output
    raw <text>              send a firmware command verbatim
    help, quit

Each PI has 8 positions (0..7). The firmware tracks position by counting the
pulses it sends and never pulses past either end; it cannot read the chip back,
so positions are only meaningful after a reset.
"""
import argparse
import shlex
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing: python3 -m pip install pyserial")

BAUD = 115200
ST_VID = 0x0483              # STMicroelectronics (ST-LINK virtual COM port)
PI_IDS = (1, 2)
PI_STEPS = 8                 # positions per PI, PI_STEPS in pi.h
PI_POS_MAX = PI_STEPS - 1

JOG_HELP = "jog keys:  k / +  up    j / -  down    0-7  go to    r  reset    q  quit"


class CommandError(Exception):
    """The firmware answered ERR, or the input was invalid."""


class Board:
    def __init__(self, port, baud=BAUD, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.port = port
        time.sleep(0.05)
        self.ser.reset_input_buffer()   # drop a READY banner or stale replies

    def close(self):
        self.ser.close()

    def cmd(self, text, timeout=2.0):
        """Send one command line and return the text after 'OK'."""
        self.ser.reset_input_buffer()
        self.ser.write((text.strip() + "\n").encode("ascii"))
        deadline = time.monotonic() + timeout
        while True:
            self.ser.timeout = max(0.05, deadline - time.monotonic())
            line = self.ser.readline().decode("ascii", "replace").strip()
            if line.startswith("OK"):
                return line[2:].strip()
            if line.startswith("ERR"):
                raise CommandError(line[3:].strip() or "ERR")
            if not line and time.monotonic() >= deadline:
                raise CommandError(f"no reply to '{text.strip()}' within {timeout:.1f} s")
            # anything else (e.g. READY after a board reset) is skipped

    # ---- PI -------------------------------------------------------------
    def status(self):
        fields = dict(kv.split("=", 1) for kv in self.cmd("STATUS").split())
        return {k: int(v) for k, v in fields.items()}

    def position(self, pi):
        return self.status()[f"PI{pi}"]

    def step(self, pi, n):
        """Step by n (positive = up). The firmware stops at 0 and 7."""
        if n == 0:
            return self.position(pi)
        n = max(-PI_STEPS, min(PI_STEPS, n))          # more than 8 is never useful
        reply = self.cmd(f"{'UP' if n > 0 else 'DN'} {pi} {abs(n)}")   # "PI1=<pos>"
        return int(reply.split("=", 1)[1])

    def goto(self, pi, target):
        if not 0 <= target <= PI_POS_MAX:
            raise CommandError(f"position must be 0..{PI_POS_MAX}, got {target}")
        return self.step(pi, target - self.position(pi))

    def reset(self, pi=None):
        self.cmd("RST" if pi is None else f"RST {pi}")

    # ---- other firmware features ----------------------------------------
    def eom(self, ms):
        errors, total = (int(x) for x in self.cmd(f"EOM {ms}", timeout=ms / 1000 + 2).split())
        return errors, total

    def dac(self, ch, mv):
        return int(self.cmd(f"DAC {ch} {mv}"))


# ------------------------------------------------------------------ helpers

def find_port():
    ports = list(serial.tools.list_ports.comports())
    st = [p for p in ports if p.vid == ST_VID]
    if len(st) == 1:
        return st[0].device
    if not st:
        names = ", ".join(p.device for p in ports) or "none"
        sys.exit(f"No ST-LINK serial port found (ports: {names}). Plug in the NUCLEO "
                 "or pass --port.")
    sys.exit("Several ST-LINK ports found, pick one with --port: "
             + ", ".join(p.device for p in st))


def parse_pi(s):
    try:
        pi = int(s)
    except (TypeError, ValueError):
        pi = None
    if pi not in PI_IDS:
        raise CommandError(f"PI must be 1 or 2, got '{s}'")
    return pi


def parse_int(s, name):
    try:
        return int(s)
    except (TypeError, ValueError):
        raise CommandError(f"{name} must be an integer, got '{s}'")


def fmt_status(st):
    return (f"PI1 = {st['PI1']}/{PI_POS_MAX}   PI2 = {st['PI2']}/{PI_POS_MAX}   "
            f"DAC A = {st['DACA']} mV   DAC B = {st['DACB']} mV   EOM_ERR = {st['ERR']}")


def bar(pi, pos):
    """PI1 [...#....] 3/7"""
    cells = "".join("#" if i == pos else "." for i in range(PI_STEPS))
    return f"PI{pi} [{cells}] {pos}/{PI_POS_MAX}"


def read_key():
    """One keypress from the terminal, without waiting for Enter."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def jog(board, pi):
    if not sys.stdin.isatty():
        raise CommandError("jog needs an interactive terminal")
    keys = {"k": 1, "+": 1, "=": 1, "j": -1, "-": -1, "_": -1}
    print(JOG_HELP)
    pos = board.position(pi)
    print(f"\r{bar(pi, pos)}   ", end="", flush=True)
    while True:
        k = read_key()
        if k in ("q", "Q", "\x03", "\x1b"):
            break
        if k in keys:
            pos = board.step(pi, keys[k])
        elif k.isdigit() and int(k) <= PI_POS_MAX:
            pos = board.goto(pi, int(k))
        elif k == "r":
            board.reset(pi)
            pos = 0
        else:
            continue
        print(f"\r{bar(pi, pos)}   ", end="", flush=True)
    print()


# ------------------------------------------------------------- dispatcher

def run(board, argv):
    """Run one command; returns False when the shell should exit."""
    if not argv:
        return True
    c, args = argv[0].lower(), argv[1:]

    if c in ("quit", "exit", "q"):
        return False
    elif c in ("help", "?", "h"):
        print(__doc__.split("Commands (shell or one-shot):", 1)[1].split("The firmware", 1)[0].rstrip())
    elif c in ("status", "st"):
        print(fmt_status(board.status()))
    elif c in ("up", "down", "dn"):
        if not args:
            raise CommandError(f"usage: {c} <pi> [n]")
        pi = parse_pi(args[0])
        n = parse_int(args[1], "n") if len(args) > 1 else 1
        if n <= 0:
            raise CommandError("n must be positive")
        before = board.position(pi)
        pos = board.step(pi, n if c == "up" else -n)
        moved = abs(pos - before)
        note = "" if moved == n else f"  (stopped at {'top' if c == 'up' else 'bottom'} after {moved} of {n})"
        print(bar(pi, pos) + note)
    elif c == "goto":
        if len(args) != 2:
            raise CommandError("usage: goto <pi> <pos>")
        pi = parse_pi(args[0])
        print(bar(pi, board.goto(pi, parse_int(args[1], "pos"))))
    elif c in ("reset", "rst"):
        pi = parse_pi(args[0]) if args else None
        board.reset(pi)
        print("PI1 and PI2 reset to 0" if pi is None else f"PI{pi} reset to 0")
    elif c == "jog":
        if len(args) != 1:
            raise CommandError("usage: jog <pi>")
        jog(board, parse_pi(args[0]))
    elif c == "eom":
        ms = parse_int(args[0], "ms") if args else 100
        errors, total = board.eom(ms)
        ber = errors / total if total else float("nan")
        print(f"{errors} errors / {total} cycles  (error ratio {ber:.3e})")
    elif c == "dac":
        if len(args) != 2 or args[0].upper() not in ("A", "B"):
            raise CommandError("usage: dac <A|B> <mV>")
        code = board.dac(args[0].upper(), parse_int(args[1], "mV"))
        print(f"DAC {args[0].upper()} = {args[1]} mV (code {code})")
    elif c == "raw":
        print(board.cmd(" ".join(args)) or "OK")
    else:
        raise CommandError(f"unknown command '{c}' (try help)")
    return True


def shell(board):
    try:
        import readline  # noqa: F401  (history and line editing)
    except ImportError:
        pass
    print(f"Connected to {board.port}. Type 'help' for commands, 'quit' to exit.")
    try:
        print(fmt_status(board.status()))
    except CommandError as e:
        print(f"error: {e}")
    while True:
        try:
            line = input("pi> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        try:
            if not run(board, shlex.split(line)):
                break
        except (CommandError, ValueError) as e:
            print(f"error: {e}")
        except serial.SerialException as e:
            print(f"serial error: {e}")
            break


def main():
    ap = argparse.ArgumentParser(description="Control the chip's phase interpolators.",
                                 epilog="With no command, starts an interactive shell. "
                                        "Run 'help' in the shell for the command list.")
    ap.add_argument("--port", help="serial port (default: auto-detect the ST-LINK)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("command", nargs=argparse.REMAINDER,
                    help="one command to run, e.g. 'up 1 5' or 'status'")
    a = ap.parse_args()

    board = Board(a.port or find_port(), a.baud)
    try:
        board.cmd("ID")                          # fail early on the wrong port
    except CommandError as e:
        board.close()
        sys.exit(f"{board.port} is not answering like the chip controller ({e})")

    try:
        if a.command:
            try:
                run(board, a.command)
            except CommandError as e:
                sys.exit(f"error: {e}")
        else:
            shell(board)
    finally:
        board.close()


if __name__ == "__main__":
    main()
