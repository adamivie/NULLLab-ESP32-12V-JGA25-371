// Mock robot: serves the real page from web_page.h and emulates the firmware's
// WebSocket behaviour. Every incoming message is validated by the REAL C++
// parser (parse_cli) so page <-> firmware protocol drift is caught.
const http = require("http");
const fs = require("fs");
const path = require("path");
const { spawn } = require("child_process");
const { WebSocketServer } = require("ws");

const header = fs.readFileSync(path.join(__dirname, "../../src/robot/web_page.h"), "utf8");
const html = header.split('R"HTML(')[1].split(')HTML"')[0];

const parser = spawn(path.join(__dirname, "build", "parse_cli"));
const pending = [];
let buf = "";
parser.stdout.on("data", d => {
  buf += d;
  let i;
  while ((i = buf.indexOf("\n")) >= 0) {
    const line = buf.slice(0, i); buf = buf.slice(i + 1);
    const cb = pending.shift(); if (cb) cb(line.split(" "));
  }
});
const parse = text => new Promise(res => { pending.push(res); parser.stdin.write(text.replace(/\n/g, " ") + "\n"); });

const robot = { state: "disarmed", lim: 0.5, owner: 0, stick: [0, 0], lastCmd: 0, nextId: 1,
  bat: { bs: "ok", bv: 11.84, ba: 2.3, bc: 3, bp: 63, bmah: 412 } };
const log = [];  // {client, text, type}
const clients = new Map();

const server = http.createServer((req, res) => {
  if (req.url === "/log") { res.setHeader("content-type", "application/json"); return res.end(JSON.stringify({ log, robot })); }
  if (req.url === "/reset-log") { log.length = 0; return res.end("ok"); }
  if (req.url === "/force-fault") { robot.state = "fault"; return res.end("ok"); }  // test hook
  if (req.url.startsWith("/battery?")) {  // test hook: /battery?bs=low&bv=10.3&bp=8
    const q = new URL(req.url, "http://x").searchParams;
    for (const [k, v] of q) robot.bat[k] = k === "bs" ? v : Number(v);
    if (robot.bat.bs === "critical" && robot.state === "armed") robot.state = "disarmed";  // firmware ramps to a stop
    return res.end("ok");
  }
  res.setHeader("content-type", "text/html"); res.end(html);
});
const wss = new WebSocketServer({ server, path: "/ws" });
wss.on("connection", ws => {
  const id = robot.nextId++;
  clients.set(id, ws);
  if (!robot.owner) robot.owner = id;
  ws.send(JSON.stringify({ hello: id, maxMps: 1.6, batCap: 0.5, names: ["M1 rear-left", "M2 front-left", "M3 rear-right", "M4 front-right"] }));
  ws.on("close", () => { clients.delete(id); if (robot.owner === id) { robot.owner = 0; if (robot.state === "armed") robot.state = "disarmed"; } });
  ws.on("message", async raw => {
    const text = raw.toString();
    const [type, a, b] = await parse(text);
    log.push({ client: id, text, type });
    if (type === "Invalid") return;
    if (type === "Stop") { robot.state = robot.state === "fault" ? "fault" : "disarmed"; return; }
    if (type === "Claim") { if (robot.owner !== id) { robot.owner = id; if (robot.state === "armed") robot.state = "disarmed"; } return; }
    if (robot.owner !== id) return;
    if (type === "Drive") { robot.stick = [+a, +b]; robot.lastCmd = Date.now(); }
    if (type === "SpeedLimit") robot.lim = +a;
    if (type === "Arm") {
      if (+a === 0) { robot.state = "disarmed"; return; }
      if (robot.state === "fault") return ws.send(JSON.stringify({ msg: "Press RESET first" }));
      if (robot.bat.bs === "critical") return ws.send(JSON.stringify({ msg: "Battery is flat - replace or recharge it" }));
      if (Math.abs(robot.stick[0]) > 0.06 || Math.abs(robot.stick[1]) > 0.06) return ws.send(JSON.stringify({ msg: "Let go of the joystick to arm" }));
      robot.state = "armed";
    }
  });
});
setInterval(() => {
  const link = Date.now() - robot.lastCmd < 500;
  const capped = (robot.bat.bs === "low" || robot.bat.bs === "critical") && robot.lim > 0.5;
  const v = robot.state === "armed" ? robot.stick[1] * (capped ? 0.5 : robot.lim) * 1.6 : 0;
  const t = JSON.stringify({ st: robot.state, fw: robot.state === "fault" ? 2 : -1, link: link ? 1 : 0, lim: robot.lim, own: robot.owner, hw: 1, rssi: -55, ovr: 0, loopUs: 140, ...robot.bat, cap: capped ? 1 : 0,
    w: [0, 1, 2, 3].map(() => [v, v * 0.97, v / 1.6]) });
  for (const ws of clients.values()) ws.send(t);
}, 100);
const port = Number(process.env.PORT || 8099);
server.listen(port, () => console.log(`mock robot on http://localhost:${port}`));
module.exports = { server, parser };
