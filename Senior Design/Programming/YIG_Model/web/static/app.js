'use strict';
const $ = id => document.getElementById(id);
const arr = v => v == null ? [] : Array.isArray(v) ? v : [v];   // jsonencode writes 1-element arrays as scalars
const NS = 'http://www.w3.org/2000/svg';
const KIND = {1: ['Reset', 'var(--series-3)'], 3: ['Write', 'var(--series-1)'], 8: ['Reverse correction', 'var(--series-2)']};
const MODE = m => m === 5 ? 'Tracking' : m === 6 || m === 7 ? 'Standby' : 'Tuning';

let tab = 'live', server = {state: 'off'}, recorded = [], busy = false, zoom = null;
const data = {live: null, full: null};      // {run, segs, board, caption}
let session = null;                         // "board|strategy|seed" of the running live session

/* ---------- data ---------- */
function blank() { return {run: {t: [], ref: [], f: [], trim: [], mode: [], pulse: []}, segs: [], board: null}; }

function normRun(r) {
  const o = {};
  for (const k of ['t', 'ref', 'f', 'trim', 'mode']) o[k] = arr(r[k]);
  const pt = arr(r.pulse.t), pv = arr(r.pulse.V), pk = arr(r.pulse.kind);
  o.pulse = pt.map((t, i) => ({t, V: pv[i], kind: pk[i]}));
  return o;
}

function normSegs(spec, run) {
  const g = k => arr(spec[k]), cmd = g('t_cmd'), end = run.t[run.t.length - 1];
  return g('f').map((f, i) => ({
    f, t0: cmd[i], t1: i + 1 < cmd.length ? cmd[i + 1] : end,
    resp: g('confirmed')[i] ? g('t_resp')[i] : null, err: g('confirmed')[i] ? g('max_err')[i] : null,
    out: g('t_out')[i], sb: g('standby_frac')[i], pow: g('P_standby')[i] ?? null,
    n: 0,
  })).map(s => (s.n = run.pulse.filter(p => p.t >= s.t0 && p.t < s.t1).length, s));
}

async function api(job) {
  const r = await fetch('/api/run', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(job)});
  const j = await r.json();
  if (!j.ok) throw new Error(j.error);
  return j;
}

function note(text, err) { $('note').textContent = text; $('note').classList.toggle('err', !!err); }

async function guard(msg, fn) {
  if (busy) return;
  busy = true; note(msg); $('results').classList.add('busy'); buttons();
  try { await fn(); } catch (e) { note(e.message, true); }
  busy = false; $('results').classList.remove('busy'); buttons(); draw();
}

function runLive() {
  const board = $('board').value, strategy = +$('strategy').value, seed = +$('seed').value;
  const job = {cmd: 'step', f_GHz: $('f').valueAsNumber, T: $('T').valueAsNumber, B_mT: $('B').valueAsNumber, dur_ms: +$('dur').value};
  if (![job.f_GHz, job.T, job.B_mT].every(Number.isFinite)) return note('Fill in the target, temperature and stray field.', true);
  const b = session && data.live && data.live.board;
  if (b && session.startsWith(board + '|') && (job.f_GHz < b.f_min || job.f_GHz > b.f_max))
    return note(`This board is calibrated from ${b.f_min} to ${b.f_max} GHz.`, true);
  guard('Running the model', async () => {
    const key = [board, strategy, seed].join('|');
    if (session !== key) {
      note('Starting a session. The first run on a board fits the magnet, which takes about 25 s.');
      const s = await api({cmd: 'session', board, strategy, seed});
      data.live = blank(); data.live.board = s.board; session = key; zoom = null;
      $('f').min = s.board.f_min; $('f').max = s.board.f_max;
      if (job.f_GHz < s.board.f_min || job.f_GHz > s.board.f_max)
        throw new Error(`This board is calibrated from ${s.board.f_min} to ${s.board.f_max} GHz.`);
      note('Running the model');
    }
    let r;
    try { r = await api(job); } catch (e) { session = null; throw e; }
    const run = normRun(r.run), d = data.live;
    const segs = normSegs(r.spec, run);
    for (const k of Object.keys(run)) d.run[k] = d.run[k].concat(run[k]);
    segs.forEach(s => { s.T = r.env.T; s.B = r.env.B_mT; });
    d.segs.push(...segs);
    zoom = null; fillView(String(d.segs.length - 1));
    note(`Simulated ${job.dur_ms} ms in ${r.wall_s.toFixed(1)} s with the script version of the model.`);
  });
}

function setFull(r, caption) {
  const run = normRun(r.run);
  data.full = {run, segs: normSegs(r.spec, run), board: r.board, key: r.board.id + '|' + r.strategy, caption};
  zoom = null;
  if (tab === 'full') { fillView('all'); note(caption); }
}

function runFull() {
  guard('Running the full test in Simulink. The first run on a board compiles the model, which can take a minute.', async () => {
    const r = await api({cmd: 'simulink', board: $('board').value, strategy: +$('strategy').value});
    setFull(r, `Simulink run: 0.4 s simulated in ${r.wall_s.toFixed(1)} s. Temperature rises from 25 to 37 °C, with a 0.5 mT stray-field step at 200 ms.`);
  });
}

async function loadRecorded() {
  const b = $('board').value, s = +$('strategy').value, key = b + '|' + s;
  if (data.full && data.full.key === key) return;
  const e = recorded.find(x => x.board === b && x.strategy === s);
  data.full = null;
  if (!e) return note('No recorded run for this setup.');
  const r = await (await fetch('recorded/' + e.file)).json();
  if ($('board').value + '|' + $('strategy').value !== key) return;   // changed while loading
  setFull(r, `Recorded Simulink run from ${r.recorded}. Temperature rises from 25 to 37 °C, with a 0.5 mT stray-field step at 200 ms.`);
}

/* ---------- page state ---------- */
function buttons() {
  const ready = server.state === 'ready' && !busy;
  $('run').disabled = !ready;
  $('runFull').disabled = !ready || !server.simulink;
  for (const id of ['board', 'strategy', 'reset', 'tabLive', 'tabFull']) $(id).disabled = busy;
}

function idleNote() {
  if (tab === 'full') return note(data.full ? data.full.caption : '');
  if (data.live && data.live.segs.length) return;
  note(server.state === 'ready' || server.state === 'busy' ? 'Pick a target and press Run. The magnet keeps its state between runs, like the real one.'
    : server.state === 'starting' ? 'MATLAB is starting. Live tuning unlocks when it is ready; the full test already has recorded runs.'
    : 'Live tuning needs MATLAB, which is not running. The full test has recorded runs.');
}

async function poll() {
  const old = server.state;
  try { server = await (await fetch('/api/status')).json(); } catch { server = {state: 'stopped'}; }
  $('status').dataset.s = server.state;
  $('statusText').textContent = {ready: 'MATLAB ' + (server.version || '') + ' ready', busy: 'MATLAB is running', starting: 'MATLAB is starting',
    stopped: 'MATLAB stopped', off: 'Recorded runs only'}[server.state];
  buttons();
  if (old !== server.state && !busy) idleNote();
}

function setTab(t) {
  tab = t; zoom = null;
  $('tabLive').setAttribute('aria-selected', t === 'live'); $('tabFull').setAttribute('aria-selected', t === 'full');
  document.querySelectorAll('.live-only').forEach(e => e.hidden = t !== 'live');
  document.querySelectorAll('.full-only').forEach(e => e.hidden = t !== 'full');
  const go = () => { fillView(t === 'live' && data.live && data.live.segs.length ? String(data.live.segs.length - 1) : 'all'); idleNote(); draw(); };
  if (t === 'full') loadRecorded().then(go, e => note(e.message, true)); else go();
}

function fillView(sel) {
  const d = data[tab], v = $('view');
  v.replaceChildren();
  const add = (val, txt) => { const o = document.createElement('option'); o.value = val; o.textContent = txt; v.append(o); };
  add('all', tab === 'live' ? 'Whole session' : 'Whole test');
  if (d) d.segs.forEach((s, i) => add(String(i), `${i + 1}. ${s.f} GHz`));
  v.value = sel;
}

/* ---------- drawing ---------- */
const M = {l: 58, r: 14, t: 8, b: 22};
const el = (p, n, a, txt) => { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (txt != null) e.textContent = txt; p.append(e); return e; };
function ticks(lo, hi, n) {
  const raw = (hi - lo) / n, p = 10 ** Math.floor(Math.log10(raw)), s = [1, 2, 5, 10].map(x => x * p).find(x => x >= raw);
  const o = []; for (let v = Math.ceil(lo / s - 1e-9) * s; v <= hi + s * 1e-6; v += s) o.push(+v.toFixed(10)); return o;
}
const fmt = (v, d = 2) => v == null ? '–' : v.toLocaleString('en-US', {maximumFractionDigits: d, minimumFractionDigits: d});
const lowerBound = (a, x) => { let lo = 0, hi = a.length; while (lo < hi) { const m = (lo + hi) >> 1; if (a[m] < x) lo = m + 1; else hi = m; } return lo; };

function windowNow() {
  const d = data[tab], v = $('view').value;
  const seg = v === 'all' ? null : d.segs[+v];
  const full = seg ? [seg.t0, seg.t1] : [d.run.t[0], d.run.t[d.run.t.length - 1]];
  return {seg, t0: seg ? seg.t0 : 0, range: zoom || full};
}

function frame(id, w, lo, hi, nt, showX) {
  const s = $(id), W = s.clientWidth || 800, H = +s.getAttribute('height');
  s.replaceChildren(); s.setAttribute('viewBox', `0 0 ${W} ${H}`);
  const [a, b] = w.range;
  const x = t => M.l + (t - a) / (b - a) * (W - M.l - M.r), y = v => H - M.b - (v - lo) / (hi - lo) * (H - M.t - M.b);
  const clip = 'clip-' + id, cp = el(el(s, 'defs', {}), 'clipPath', {id: clip});
  el(cp, 'rect', {x: M.l, y: M.t - 4, width: W - M.l - M.r, height: H - M.t - M.b + 8});
  for (const v of ticks(lo, hi, nt)) {
    el(s, 'line', {x1: M.l, x2: W - M.r, y1: y(v), y2: y(v), stroke: 'var(--grid)'});
    el(s, 'text', {x: M.l - 8, y: y(v) + 4, 'text-anchor': 'end'}, String(v));
  }
  const tx = ticks(a - w.t0, b - w.t0, Math.max(3, W / 120));
  for (const v of tx) el(s, 'text', {x: x(v + w.t0), y: H - 6, 'text-anchor': x(v + w.t0) > W - 34 ? 'end' : 'middle', class: 'xt'}, v + (showX && v === tx[tx.length - 1] ? ' ms' : ''));
  const g = el(s, 'g', {'clip-path': `url(#${clip})`});
  return {s, g, x, y, W, H};
}

function path(c, t, v, i0, i1, step) {
  let p = '';
  for (let i = i0; i < i1; i++) {
    const X = c.x(t[i]).toFixed(1), Y = c.y(v(i)).toFixed(1);
    p += i === i0 ? `M${X},${Y}` : step ? `H${X}V${Y}` : `L${X},${Y}`;
  }
  return p;
}

let cur = null;      // what the crosshair reads
function draw() {
  const d = data[tab];
  $('results').hidden = !d || !d.run.t.length;
  if ($('results').hidden) { cur = null; return; }
  const w = windowNow(), r = d.run, [a, b] = w.range;
  const i0 = Math.max(0, lowerBound(r.t, a) - 1), i1 = Math.min(r.t.length, lowerBound(r.t, b) + 1);
  $('zoomOut').hidden = !zoom;
  const line = (c, v, col, step) => el(c.g, 'path', {d: path(c, r.t, v, i0, i1, step), fill: 'none', stroke: col, 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round'});

  let lo = Infinity, hi = -Infinity, el_ = Infinity, eh = -Infinity, tm = 1;
  for (let i = i0; i < i1; i++) {
    lo = Math.min(lo, r.f[i], r.ref[i]); hi = Math.max(hi, r.f[i], r.ref[i]);
    const e = (r.f[i] - r.ref[i]) * 1e3; el_ = Math.min(el_, e); eh = Math.max(eh, e);
    tm = Math.max(tm, Math.abs(r.trim[i]));
  }
  const pad = Math.max((hi - lo) * .06, .02);
  const cF = frame('cF', w, lo - pad, hi + pad, 5);
  line(cF, i => r.ref[i], 'var(--muted)', true); line(cF, i => r.f[i], 'var(--series-1)');

  const es = +$('escale').value;
  const E = es ? [-es, es] : [Math.min(el_, -12) * 1.05, Math.max(eh, 12) * 1.05];
  const cE = frame('cE', w, E[0], E[1], 5);
  el(cE.g, 'rect', {x: M.l, width: cE.W - M.l - M.r, y: cE.y(10), height: cE.y(-10) - cE.y(10), fill: 'var(--band)'});
  line(cE, i => Math.max(-1e4, Math.min(1e4, (r.f[i] - r.ref[i]) * 1e3)), 'var(--series-1)');
  if (es && (eh > es || el_ < -es)) el(cE.s, 'text', {x: cE.W - M.r - 4, y: cE.H - M.b - 6, 'text-anchor': 'end'}, 'Goes off this scale');
  if (w.seg && w.seg.resp != null) {
    const t = w.seg.t0 + w.seg.resp;
    if (t > a && t < b) {
      el(cE.g, 'line', {x1: cE.x(t), x2: cE.x(t), y1: M.t, y2: cE.H - M.b, stroke: 'var(--text-secondary)'});
      el(cE.s, 'text', {x: cE.x(t) + 6, y: M.t + 12}, `In spec at ${fmt(w.seg.resp)} ms`);
    }
  }

  const ps = r.pulse.filter(p => p.t >= a && p.t <= b), pm = Math.max(10, ...ps.map(p => Math.abs(p.V))) * 1.12;
  const cP = frame('cP', w, -pm, pm, 4);
  el(cP.s, 'line', {x1: M.l, x2: cP.W - M.r, y1: cP.y(0), y2: cP.y(0), stroke: 'var(--axis)'});
  for (const p of ps) {
    const k = KIND[p.kind] || ['Pulse', 'var(--muted)'];
    el(cP.g, 'line', {x1: cP.x(p.t), x2: cP.x(p.t), y1: cP.y(0), y2: cP.y(p.V), stroke: k[1], 'stroke-width': 2});
    el(cP.g, 'circle', {cx: cP.x(p.t), cy: cP.y(p.V), r: 4, fill: k[1], stroke: 'var(--surface-1)', 'stroke-width': 2});
  }
  const cT = frame('cT', w, -tm * 1.1, tm * 1.1, 4);
  line(cT, i => r.trim[i], 'var(--series-1)');

  const cM = frame('cM', w, 0, 1, 1, true);
  cM.s.querySelectorAll('text:not(.xt)').forEach(t => t.remove());
  const mc = {Tuning: 'var(--m-tune)', Tracking: 'var(--m-track)', Standby: 'var(--m-stby)'};
  for (let i = i0; i < i1;) {
    let j = i; const m = MODE(r.mode[i]);
    while (j < i1 && MODE(r.mode[j]) === m) j++;
    const xa = cM.x(r.t[i]), xb = cM.x(r.t[Math.min(j, i1 - 1)]);
    el(cM.g, 'rect', {x: xa, width: Math.max(1, xb - xa), y: M.t, height: cM.H - M.t - M.b, fill: mc[m]});
    i = j;
  }
  cur = {w, r, i0, i1, charts: [cF, cE, cP, cT, cM], x: cF.x};
  hookCharts();
  tiles(d, w); tables(d, w);
}

function verdict(id, ok, good, bad) { $(id).className = 'foot ' + (ok == null ? '' : ok ? 'ok' : 'no'); $(id).textContent = ok == null ? good : ok ? 'Meets spec' : bad; }

function tiles(d, w) {
  const segs = w.seg ? [w.seg] : d.segs, r = d.run;
  const last = lowerBound(r.t, w.seg ? w.seg.t1 : Infinity) - 1, f = r.f[last];
  $('heroLabel').textContent = `True frequency at the end of ${w.seg ? 'this run' : tab === 'live' ? 'the session' : 'the test'}`;
  $('heroValue').textContent = fmt(f, 3) + ' GHz';
  $('heroFoot').textContent = `Target ${fmt(r.ref[last], 3)} GHz, error ${fmt((f - r.ref[last]) * 1e3, 1)} MHz`;
  const allSet = segs.every(s => s.resp != null), worst = k => Math.max(...segs.map(s => s[k] ?? -Infinity));
  $('vResp').textContent = allSet ? fmt(worst('resp')) + ' ms' : 'Not settled';
  verdict('sResp', allSet && worst('resp') < 1, '', allSet ? 'Misses spec' : 'Not in spec for a full 1 ms');
  $('vErr').textContent = allSet ? fmt(worst('err'), 1) + ' MHz' : '–';
  if (allSet) verdict('sErr', worst('err') <= 10, '', `Misses spec, out for ${fmt(segs.reduce((x, s) => x + s.out, 0))} ms in total`); else verdict('sErr', null, '');
  const sb = segs.filter(s => s.pow != null && s.sb > 0);
  const wsum = sb.reduce((x, s) => x + s.sb * (s.t1 - s.t0), 0), pow = sb.reduce((x, s) => x + s.pow * s.sb * (s.t1 - s.t0), 0) / wsum;
  $('vPow').textContent = sb.length ? fmt(pow) + ' mW' : '–';
  if (sb.length) verdict('sPow', pow < 1, '', 'Misses spec'); else verdict('sPow', null, 'Standby not reached yet');
  const a = w.seg ? w.seg.t0 : -Infinity, b = w.seg ? w.seg.t1 : Infinity;
  const p = r.pulse.filter(q => q.t >= a && q.t < b), c = k => p.filter(q => q.kind === k).length;
  $('vPul').textContent = String(p.length);
  $('sPul').textContent = `${c(3)} write, ${c(8)} reverse, ${c(1)} reset`;
}

function table(id, head, rows) {
  const t = $(id); t.replaceChildren();
  const tr = (cells, tag) => { const e = document.createElement('tr'); for (const c of cells) { const x = document.createElement(tag); x.textContent = c.t ?? c; if (c.c) x.className = c.c; e.append(x); } t.append(e); };
  tr(head, 'th'); rows.forEach(r => tr(r, 'td'));
}

function tables(d, w) {
  table('tSpec', ['Target', 'Response', 'Worst error after', 'Time out of spec', 'In standby', 'Standby power', 'Pulses', ...(tab === 'live' ? ['Temperature', 'Stray field'] : [])],
    d.segs.map(s => [`${s.f} GHz`,
      s.resp == null ? {t: 'Not settled', c: 'no'} : {t: fmt(s.resp) + ' ms', c: s.resp < 1 ? 'ok' : 'no'},
      s.err == null ? '–' : {t: fmt(s.err, 1) + ' MHz', c: s.err <= 10 ? 'ok' : 'no'},
      s.resp == null ? '–' : fmt(s.out) + ' ms', fmt(s.sb * 100, 0) + '%', s.pow == null ? '–' : fmt(s.pow) + ' mW', String(s.n),
      ...(tab === 'live' ? [s.T + ' °C', s.B + ' mT'] : [])]));
  const [a, b] = w.range;
  table('tPulse', ['Pulse', 'Time', 'Voltage'], d.run.pulse.filter(p => p.t >= a && p.t <= b).map(p => [(KIND[p.kind] || ['Pulse'])[0], fmt(p.t - w.t0, 3) + ' ms', fmt(p.V) + ' V']));
}

/* ---------- crosshair and zoom ---------- */
function hookCharts() {
  for (const c of cur.charts) {
    c.cross = el(c.s, 'line', {y1: M.t, y2: c.H - M.b, stroke: 'var(--axis)', visibility: 'hidden'});
    c.sel = el(c.s, 'rect', {y: M.t, height: c.H - M.t - M.b, fill: 'var(--band)', visibility: 'hidden'});
  }
}
const tAt = ev => { const c = cur.charts[0], box = c.s.getBoundingClientRect(), [a, b] = cur.w.range; return Math.min(b, Math.max(a, a + (ev.clientX - box.left - M.l) / (box.width - M.l - M.r) * (b - a))); };
let drag = null;
function move(ev) {
  if (!cur || busy) return;
  const t = tAt(ev), r = cur.r;
  let i = lowerBound(r.t, t); if (i > 0 && (i >= r.t.length || t - r.t[i - 1] < r.t[i] - t)) i--;
  const x = cur.x(r.t[i]);
  for (const c of cur.charts) { c.cross.setAttribute('x1', x); c.cross.setAttribute('x2', x); c.cross.setAttribute('visibility', 'visible');
    if (drag != null) { const p = cur.x(drag), q = cur.x(t); c.sel.setAttribute('x', Math.min(p, q)); c.sel.setAttribute('width', Math.abs(p - q)); c.sel.setAttribute('visibility', 'visible'); } }
  const near = (cur.w.range[1] - cur.w.range[0]) / 120, p = r.pulse.filter(q => Math.abs(q.t - r.t[i]) < near).sort((m, n) => Math.abs(m.t - r.t[i]) - Math.abs(n.t - r.t[i]))[0];
  const rows = [[fmt(r.t[i] - cur.w.t0, 3) + ' ms', 'Time'], [fmt(r.f[i], 4) + ' GHz', 'True frequency'], [fmt(r.ref[i], 4) + ' GHz', 'Target'],
    [fmt((r.f[i] - r.ref[i]) * 1e3, 1) + ' MHz', 'Error'], [fmt(r.trim[i], 1) + ' mA', 'Trim'], [MODE(r.mode[i]), 'Controller']];
  if (p) rows.push([fmt(p.V) + ' V', (KIND[p.kind] || ['Pulse'])[0] + ' pulse']);
  const tip = $('tip'); tip.replaceChildren();
  for (const [v, n] of rows) { const q = document.createElement('div'), s = document.createElement('b'), k = document.createElement('span'); s.textContent = v + ' '; k.textContent = n; q.append(s, k); tip.append(q); }
  tip.hidden = false;
  tip.style.left = Math.min(ev.clientX + 16, innerWidth - tip.offsetWidth - 8) + 'px';
  tip.style.top = Math.min(ev.clientY + 16, innerHeight - tip.offsetHeight - 8) + 'px';
}
function leave() { $('tip').hidden = true; if (cur) for (const c of cur.charts) { c.cross.setAttribute('visibility', 'hidden'); c.sel.setAttribute('visibility', 'hidden'); } }
$('charts').addEventListener('pointermove', move);
$('charts').addEventListener('pointerleave', () => { drag = null; leave(); });
$('charts').addEventListener('pointerdown', ev => { if (cur && !busy && ev.target.closest('svg')) drag = tAt(ev); });
$('charts').addEventListener('pointerup', ev => {
  if (drag == null || !cur) return;
  const t = tAt(ev), [a, b] = cur.w.range, q = [Math.min(drag, t), Math.max(drag, t)]; drag = null;
  if (q[1] - q[0] > (b - a) / 100 && q[1] - q[0] > 0.05) { zoom = q; leave(); draw(); }
});

/* ---------- start ---------- */
$('tabLive').onclick = () => setTab('live');
$('tabFull').onclick = () => setTab('full');
$('run').onclick = runLive;
$('runFull').onclick = runFull;
$('f').addEventListener('keydown', e => { if (e.key === 'Enter' && !$('run').disabled) runLive(); });
$('reset').onclick = () => { session = null; data.live = null; zoom = null; fillView('all'); idleNote(); draw(); };
$('view').onchange = () => { zoom = null; draw(); };
$('escale').onchange = draw;
$('zoomOut').onclick = () => { zoom = null; draw(); };
for (const id of ['board', 'strategy']) $(id).onchange = () => { if (tab === 'full') setTab('full'); else if (session) note('The next run starts a new session with this setup.'); };
new ResizeObserver(() => { if (cur) draw(); }).observe($('charts'));

(async () => {
  await poll();
  try { recorded = await (await fetch('recorded/index.json')).json(); } catch { recorded = []; }
  const names = new Map(recorded.map(e => [e.board, e.name]));
  for (const b of server.boards || [...names.keys()]) { const o = document.createElement('option'); o.value = b; o.textContent = names.get(b) || b; $('board').append(o); }
  setTab(server.state === 'off' ? 'full' : 'live');
  setInterval(poll, 1500);
})();
