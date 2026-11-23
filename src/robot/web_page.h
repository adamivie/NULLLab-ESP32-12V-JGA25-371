#pragma once
// The driving page served at http://<robot>/ . Self-contained (no internet
// needed - the robot's own Wi-Fi has none). Protocol: see ControlProtocol.h.

static const char kIndexHtml[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#101418">
<title>MakerBot</title>
<style>
:root{--bg:#101418;--panel:#1a2027;--line:#2a323c;--text:#e8edf2;--dim:#8a96a3;--blue:#3d7bff;
--green:#2fbf71;--amber:#f2a516;--red:#e5484d}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;height:100dvh;background:var(--bg);color:var(--text);
font:15px/1.35 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;overscroll-behavior:none}
body{display:flex;flex-direction:column;user-select:none;-webkit-user-select:none;
padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
header{display:flex;gap:8px;align-items:center;padding:10px 14px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0 auto 0 0;font-weight:650}
.pill{padding:3px 10px;border-radius:99px;background:var(--panel);border:1px solid var(--line);font-size:13px;white-space:nowrap}
.pill.ok{border-color:var(--green);color:var(--green)}.pill.warn{border-color:var(--amber);color:var(--amber)}
.pill.bad{border-color:var(--red);color:var(--red)}.pill.info{border-color:var(--blue);color:#9dbcff}
.banner{margin:10px 14px 0;padding:10px 12px;border-radius:10px;display:flex;gap:10px;align-items:center;justify-content:space-between}
.banner.warn{background:#3a2c0c;border:1px solid var(--amber)}.banner.bad{background:#3a1416;border:1px solid var(--red)}
.banner[hidden]{display:none}
main{flex:1;display:grid;grid-template-columns:1fr;grid-template-rows:minmax(220px,1fr) auto;gap:14px;padding:14px;min-height:0}
@media (orientation:landscape) and (min-width:600px){main{grid-template-columns:1fr minmax(260px,340px);grid-template-rows:1fr}}
#stickWrap{position:relative;min-height:0;min-width:0;display:flex;align-items:center;justify-content:center;overflow:hidden}
#stick{touch-action:none;display:block}
#side{display:flex;flex-direction:column;gap:12px;min-height:0;overflow-y:auto}
.row{display:flex;gap:10px}
button{font:inherit;color:var(--text);background:var(--panel);border:1px solid var(--line);border-radius:12px;
padding:10px 14px;cursor:pointer;touch-action:manipulation}
button:active{filter:brightness(1.3)}
.big{flex:1;font-size:20px;font-weight:700;padding:18px 10px;letter-spacing:.04em}
#arm.armed{background:#14361f;border-color:var(--green);color:#bff3d2}
#arm.fault{background:#3a1416;border-color:var(--red)}
#stop{background:var(--red);border-color:#ff7a7e;color:#fff}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px}
label{display:flex;justify-content:space-between;color:var(--dim);font-size:13px;margin-bottom:6px}
input[type=range]{width:100%;accent-color:var(--blue);height:28px}
.wheels{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.wheel{font-size:12px;color:var(--dim);min-width:0}
.wl{display:flex;justify-content:space-between;gap:6px;white-space:nowrap}
.wl span:first-child{overflow:hidden;text-overflow:ellipsis}
.wl span:last-child{color:var(--text);font-variant-numeric:tabular-nums}
.bar{position:relative;height:12px;background:#0c0f12;border-radius:6px;overflow:hidden;margin-top:4px}
.bar i{position:absolute;top:0;bottom:0;background:var(--blue);left:50%;width:0}
.bar b{position:absolute;top:-2px;bottom:-2px;width:2px;background:var(--text);left:50%}
.small{font-size:12px;color:var(--dim)}
fieldset{border:0;margin:0;padding:0;min-width:0;display:grid;grid-template-columns:1fr 1fr;gap:10px}
fieldset[disabled]{opacity:.5}
fieldset>div{min-width:0}fieldset>div.wide{grid-column:1/-1}fieldset>div[hidden]{display:none}
select,input[type=color]{width:100%;height:38px;font:inherit;color:var(--text);background:#0c0f12;border:1px solid var(--line);
border-radius:8px;padding:0 8px}
input[type=color]{padding:3px}
.check{display:flex;gap:8px;align-items:center;color:var(--text);font-size:14px;margin:0;justify-content:flex-start}
.check input{width:20px;height:20px;accent-color:var(--blue)}
#toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);background:#000c;padding:10px 16px;border-radius:10px;
opacity:0;transition:opacity .2s;pointer-events:none}
#toast.show{opacity:1}
</style>
</head>
<body>
<header>
  <h1>MakerBot</h1>
  <span id="conn" class="pill">connecting…</span>
  <span id="state" class="pill">—</span>
</header>
<div id="spectator" class="banner warn" hidden><span>Another device is driving.</span><button id="claim">Take control</button></div>
<div id="fault" class="banner bad" hidden><span id="faultText"></span></div>
<main>
  <section id="stickWrap"><canvas id="stick" aria-label="Drive joystick"></canvas></section>
  <section id="side">
    <div class="row">
      <button id="arm" class="big">ARM</button>
      <button id="stop" class="big">STOP</button>
    </div>
    <div class="card">
      <label for="lim"><span>Speed limit</span><span id="limVal">–</span></label>
      <input id="lim" type="range" min="10" max="100" step="5" value="50">
      <label for="expo" style="margin-top:10px"><span>Stick softness (expo)</span><span id="expoVal">–</span></label>
      <input id="expo" type="range" min="0" max="100" step="5" value="35">
      <label for="turn" style="margin-top:10px"><span>Turn rate</span><span id="turnVal">–</span></label>
      <input id="turn" type="range" min="20" max="100" step="5" value="65">
    </div>
    <div class="card">
      <div class="wheels" id="wheels"></div>
      <div class="small" id="info" style="margin-top:8px"></div>
    </div>
    <div class="card">
      <fieldset id="leds">
        <div class="wide"><label for="ledMode"><span>Lights</span></label>
          <select id="ledMode">
            <option value="0">Status colours</option>
            <option value="1">Solid colour</option>
            <option value="2">Pulse (fade)</option>
            <option value="3">Blink</option>
            <option value="4">Spin (fade)</option>
            <option value="5">Spin (blink)</option>
            <option value="6">Front / back colours</option>
            <option value="7">Palette sweep</option>
            <option value="8">Rainbow</option>
          </select></div>
        <div id="ledC1Box"><label for="ledC1"><span id="ledC1Name">Colour</span></label><input id="ledC1" type="color" value="#00ff40"></div>
        <div id="ledC2Box"><label for="ledC2"><span id="ledC2Name">Second colour</span></label><input id="ledC2" type="color" value="#000000"></div>
        <div id="ledPalBox" class="wide"><label for="ledPal"><span>Palette</span></label>
          <select id="ledPal">
            <option value="0">Fire</option><option value="1">Ocean</option><option value="2">Forest</option>
            <option value="3">Sunset</option><option value="4">Party</option><option value="5">Ice</option>
            <option value="6">My two colours</option>
          </select></div>
        <div id="ledSpeedBox" class="wide"><label for="ledSpeed"><span>Effect speed</span><span id="ledSpeedVal"></span></label>
          <input id="ledSpeed" type="range" min="1" max="100" value="50"></div>
        <div id="ledBriBox" class="wide"><label for="ledBri"><span>Brightness</span><span id="ledBriVal"></span></label>
          <input id="ledBri" type="range" min="1" max="255" value="40"></div>
        <div id="ledMovBox" class="wide"><label class="check"><input id="ledMov" type="checkbox" checked>Animate only while moving</label></div>
      </fieldset>
      <div class="small" style="margin-top:8px">Red and yellow warning blinks always show, whatever is chosen here.</div>
    </div>
    <div class="small">Keyboard: arrows / WASD to drive, Space = STOP, Enter = arm/disarm.</div>
  </section>
</main>
<div id="toast"></div>
<script>
"use strict";
const $ = id => document.getElementById(id);
let ws = null, myId = 0, owner = false, state = "disarmed", maxMps = 1, names = [];
let stick = {x: 0, y: 0}, keys = new Set(), limDragging = false, retryMs = 500;

// ---------- connection ----------
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { retryMs = 500; setPill("conn", "connected", "ok"); };
  ws.onclose = () => {
    setPill("conn", "reconnecting…", "bad"); setPill("state", "—", ""); owner = false;
    setTimeout(connect, retryMs); retryMs = Math.min(retryMs * 2, 4000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = ev => { try { handle(JSON.parse(ev.data)); } catch (e) {} };
}
function send(text) { if (ws && ws.readyState === 1) ws.send(text); }

function handle(m) {
  if (m.hello !== undefined) { myId = m.hello; maxMps = m.maxMps || 1; names = m.names || []; buildWheels(); return; }
  if (m.msg) { toast(m.msg); return; }
  if (m.pong !== undefined) { pingGot(m.pong); return; }
  if (m.led) { if (Date.now() - ledSentAt > 1000) ledShow(m.led); return; }
  if (m.st === undefined) return;
  if (m.st === "fault" && state !== "fault" && navigator.vibrate) navigator.vibrate([200, 100, 200]);
  if (m.st === "armed" && state !== "armed" && navigator.vibrate) navigator.vibrate(40);
  state = m.st; owner = m.own === myId;
  $("leds").disabled = !owner;
  if (!limDragging && m.ex !== undefined) {
    $("expo").value = Math.round(m.ex * 100); $("turn").value = Math.round(m.tg * 100); feelLabels();
  }
  $("spectator").hidden = owner;
  const labels = {disarmed: ["DISARMED", "info"], armed: ["ARMED", "ok"], stopping: ["STOPPING", "warn"], fault: ["FAULT", "bad"]};
  const [txt, cls] = labels[state] || [state, ""];
  setPill("state", m.link || state !== "armed" ? txt : "ARMED · NO LINK", m.link || state !== "armed" ? cls : "bad");
  const arm = $("arm");
  arm.textContent = state === "fault" ? "RESET" : (state === "armed" ? "DISARM" : "ARM");
  arm.className = "big " + (state === "armed" ? "armed" : state === "fault" ? "fault" : "");
  arm.disabled = !owner;
  $("fault").hidden = state !== "fault" && m.hw !== 0;
  if (m.hw === 0) $("faultText").textContent = "Motor/encoder hardware failed to start - check the serial log.";
  else if (state === "fault") $("faultText").textContent =
    `Wheel ${names[m.fw] || m.fw} stalled (blocked or jammed). Clear it, then press RESET.`;
  if (!limDragging) { $("lim").value = Math.round(m.lim * 100); $("limVal").textContent = `${Math.round(m.lim * 100)}% · ${(m.lim * maxMps).toFixed(2)} m/s`; }
  (m.w || []).forEach((w, i) => updateWheel(i, w[0], w[1], w[2]));
  draw();  // knob colour / hint depend on state and ownership
  $("info").textContent = `top speed ${maxMps.toFixed(2)} m/s` + (m.rssi ? ` · Wi-Fi ${m.rssi} dBm` : "") +
    (pingMs !== null ? ` · ping ${Math.round(pingMs)} ms` + (pingWorst > 80 ? ` (worst ${Math.round(pingWorst)})` : "") : "") +
    ` · loop ${m.loopUs} µs` + (m.ovr ? ` · late ${m.ovr}` : "");
}

function setPill(id, text, cls) { const e = $(id); e.textContent = text; e.className = "pill " + cls; }
let toastTimer;
function toast(t) { const e = $("toast"); e.textContent = t; e.classList.add("show"); clearTimeout(toastTimer); toastTimer = setTimeout(() => e.classList.remove("show"), 2500); }

// ---------- wheel bars ----------
function buildWheels() {
  // Lay the bars out like the robot seen from above (FL FR / RL RR), using the
  // position in each wheel's name; unknown positions keep port order.
  const rank = n => (/front/i.test(n) ? 0 : /rear/i.test(n) ? 2 : 1) * 3 + (/left/i.test(n) ? 0 : /right/i.test(n) ? 2 : 1);
  const order = names.map((n, i) => i).sort((a, b) => rank(names[a]) - rank(names[b]) || a - b);
  $("wheels").innerHTML = order.map(i =>
    `<div class="wheel" title="${names[i]}"><div class="wl"><span>${names[i].replace(/^M\d /, "")}</span><span id="wv${i}"></span></div>` +
    `<div class="bar"><i id="wb${i}"></i><b id="wt${i}"></b></div></div>`).join("");
}
function updateWheel(i, target, speed, duty) {
  const bar = $("wb" + i), tick = $("wt" + i), val = $("wv" + i);
  if (!bar) return;
  const f = Math.max(-1, Math.min(1, speed / maxMps)), t = Math.max(-1, Math.min(1, target / maxMps));
  bar.style.left = (f >= 0 ? 50 : 50 + f * 50) + "%"; bar.style.width = Math.abs(f) * 50 + "%";
  tick.style.left = `calc(${50 + t * 50}% - 1px)`;
  // Speed and motor effort (duty): a wheel near 100 % is working at its limit.
  val.textContent = `${speed.toFixed(2)} m/s · ${Math.round(Math.abs(duty) * 100)}%`;
  val.title = `target ${target.toFixed(2)} m/s`;
}

// ---------- joystick ----------
const cv = $("stick"), ctx = cv.getContext("2d");
let activePointer = null;
function resize() {
  // Square joystick that fits the space available, up to 420 CSS px.
  const wrap = $("stickWrap"), side = Math.max(120, Math.min(wrap.clientWidth, wrap.clientHeight, 420));
  const d = window.devicePixelRatio || 1;
  cv.style.width = cv.style.height = side + "px";
  cv.width = cv.height = Math.round(side * d); draw();
}
function draw() {
  const w = cv.width, h = cv.height, cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 * 0.92, kr = R * 0.28;
  ctx.clearRect(0, 0, w, h);
  ctx.lineWidth = Math.max(2, R * 0.012);
  ctx.strokeStyle = "#2a323c"; ctx.fillStyle = "#151a20";
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
  const live = owner && state === "armed";
  const kx = cx + stick.x * (R - kr), ky = cy - stick.y * (R - kr);
  ctx.fillStyle = live ? "#3d7bff" : "#3a4552";
  ctx.beginPath(); ctx.arc(kx, ky, kr, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = "#8a96a3"; ctx.font = `${Math.round(R * 0.09)}px system-ui`; ctx.textAlign = "center";
  ctx.fillText(live ? "" : (owner ? "press ARM to drive" : "watching"), cx, cy + R * 0.62);
}
function setFromPointer(e) {
  const r = cv.getBoundingClientRect(), R = Math.min(r.width, r.height) / 2 * 0.92 * (1 - 0.28);
  let x = (e.clientX - r.left - r.width / 2) / R, y = -(e.clientY - r.top - r.height / 2) / R;
  const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; }
  stick = {x, y}; draw(); stickChanged();
}
cv.addEventListener("pointerdown", e => { if (activePointer !== null) return; activePointer = e.pointerId; cv.setPointerCapture(e.pointerId); setFromPointer(e); });
cv.addEventListener("pointermove", e => { if (e.pointerId === activePointer) setFromPointer(e); });
const release = e => { if (e.pointerId !== activePointer) return; activePointer = null; stick = {x: 0, y: 0}; draw(); stickChanged(); };
cv.addEventListener("pointerup", release); cv.addEventListener("pointercancel", release); cv.addEventListener("lostpointercapture", release);

// ---------- keyboard ----------
const keyMap = {ArrowUp: "u", KeyW: "u", ArrowDown: "d", KeyS: "d", ArrowLeft: "l", KeyA: "l", ArrowRight: "r", KeyD: "r"};
addEventListener("keydown", e => {
  if (e.code === "Space") { e.preventDefault(); send("S"); return; }
  if (e.code === "Enter") { e.preventDefault(); $("arm").click(); return; }
  if (keyMap[e.code]) { e.preventDefault(); keys.add(keyMap[e.code]); keyStick(); }
});
addEventListener("keyup", e => { if (keyMap[e.code]) { keys.delete(keyMap[e.code]); keyStick(); } });
function keyStick() {
  if (activePointer !== null) return;
  stick = {x: (keys.has("r") ? 1 : 0) - (keys.has("l") ? 1 : 0), y: (keys.has("u") ? 1 : 0) - (keys.has("d") ? 1 : 0)};
  draw(); stickChanged();
}

// ---------- buttons ----------
$("stop").onclick = () => send("S");
$("arm").onclick = () => {
  if (!owner) return;
  if (state === "armed" || state === "stopping" || state === "fault") send("A 0");
  else send("A 1");
};
$("claim").onclick = () => send("C");
const lim = $("lim");
lim.addEventListener("pointerdown", () => limDragging = true);
addEventListener("pointerup", () => limDragging = false);
lim.addEventListener("input", () => {
  $("limVal").textContent = `${lim.value}% · ${(lim.value / 100 * maxMps).toFixed(2)} m/s`;
  send(`L ${(lim.value / 100).toFixed(2)}`);
});

// ---------- driving feel ----------
function feelLabels() {
  const e = +$("expo").value;
  $("expoVal").textContent = e === 0 ? "off (linear)" : e + "%";
  $("turnVal").textContent = $("turn").value + "%";
}
["expo", "turn"].forEach(id => {
  $(id).addEventListener("pointerdown", () => limDragging = true);
  $(id).addEventListener("input", () => {
    feelLabels(); send(`T ${($("expo").value / 100).toFixed(2)} ${($("turn").value / 100).toFixed(2)}`);
  });
});
feelLabels();

// ---------- lights ----------
// Wire format (see ControlProtocol.h): E mode rgb1 rgb2 palette speed brightness movingOnly
let ledSentAt = 0, ledTimer = null;
const hex = n => "#" + n.toString(16).padStart(6, "0");
function ledShow(f) {
  $("ledMode").value = f[0]; $("ledC1").value = hex(f[1]); $("ledC2").value = hex(f[2]); $("ledPal").value = f[3];
  $("ledSpeed").value = f[4]; $("ledBri").value = f[5]; $("ledMov").checked = !!f[6];
  ledLayout();
}
function ledLayout() {
  // Show only the controls the chosen effect uses.
  const mode = +$("ledMode").value, custom = mode === 7 && +$("ledPal").value === 6;
  $("ledC1Box").hidden = !([1, 2, 3, 4, 5, 6].includes(mode) || custom);
  $("ledC2Box").hidden = !([3, 4, 5, 6].includes(mode) || custom);
  $("ledC1Name").textContent = mode === 6 ? "Front colour" : "Colour";
  $("ledC2Name").textContent = mode === 6 ? "Back colour" : "Second colour";
  $("ledPalBox").hidden = mode !== 7;
  const animated = [2, 3, 4, 5, 7, 8].includes(mode);
  $("ledSpeedBox").hidden = $("ledMovBox").hidden = !animated;
  $("ledBriBox").hidden = mode === 0;
  $("ledSpeedVal").textContent = $("ledSpeed").value + "%";
  $("ledBriVal").textContent = Math.round($("ledBri").value / 2.55) + "%";
}
function ledSend() {
  ledLayout();
  if (ledTimer) return;  // colour pickers fire constantly: send at most ~12x per second
  ledTimer = setTimeout(() => {
    ledTimer = null; ledSentAt = Date.now();
    send("E " + [+$("ledMode").value, parseInt($("ledC1").value.slice(1), 16), parseInt($("ledC2").value.slice(1), 16),
      +$("ledPal").value, +$("ledSpeed").value, +$("ledBri").value, $("ledMov").checked ? 1 : 0].join(" "));
  }, 80);
}
["ledMode", "ledC1", "ledC2", "ledPal", "ledSpeed", "ledBri", "ledMov"].forEach(id => $(id).addEventListener("input", ledSend));
ledLayout();

// Joystick stream = heartbeat. If this page is hidden (phone locked, app
// switched) the stream stops and the robot halts by itself; we also ask for a
// gentle stop right away.
// A stick movement (above all: letting go) is sent at once, at most every
// 20 ms; with the stick still, a heartbeat goes out every 50 ms.
let stickSentAt = 0, stickTimer = null;
function sendStick() {
  if (!owner || document.hidden) return;
  stickSentAt = performance.now(); send(`D ${stick.x.toFixed(3)} ${stick.y.toFixed(3)}`);
}
function stickChanged() {
  const wait = 20 - (performance.now() - stickSentAt);
  if (wait <= 0) sendStick();
  else if (!stickTimer) stickTimer = setTimeout(() => { stickTimer = null; sendStick(); }, wait);
}
setInterval(() => { if (performance.now() - stickSentAt >= 50) sendStick(); }, 25);

// Round-trip time to the robot, shown in the info line.
let pingSeq = 0, pingSentAt = 0, pingMs = null, pingWorst = 0;
function pingGot(n) {
  if (n !== pingSeq) return;
  const rtt = performance.now() - pingSentAt;
  pingMs = pingMs === null ? rtt : pingMs * 0.7 + rtt * 0.3;
  pingWorst = Math.max(pingWorst * 0.9, rtt);  // recent worst case, fades out
}
setInterval(() => { if (!document.hidden) { pingSeq = (pingSeq + 1) % 1000000; pingSentAt = performance.now(); send("P " + pingSeq); } }, 1000);
document.addEventListener("visibilitychange", () => { if (document.hidden && owner) send("A 0"); });
addEventListener("blur", () => { keys.clear(); keyStick(); });
addEventListener("resize", resize);
if (window.ResizeObserver) new ResizeObserver(resize).observe($("stickWrap"));
resize(); connect();
</script>
</body>
</html>
)HTML";
