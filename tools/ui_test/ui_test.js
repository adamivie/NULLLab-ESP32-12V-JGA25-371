const { chromium, devices } = require("playwright");
const URL = `http://localhost:${process.env.PORT || 8099}/`;
const SHOTS = process.env.SHOTS_DIR || require("path").join(__dirname, "build");
const sleep = ms => new Promise(r => setTimeout(r, ms));
let failures = 0;
function check(cond, what) { console.log(`${cond ? "PASS" : "FAIL"}  ${what}`); if (!cond) failures++; }
const getLog = async () => (await (await fetch(URL + "log")).json());

(async () => {
  require("fs").mkdirSync(SHOTS, { recursive: true });
  const mock = require("./mock_robot.js");
  await new Promise(r => setTimeout(r, 300));
  const browser = await chromium.launch();
  const phone = await browser.newContext({ ...devices["Pixel 7"], deviceScaleFactor: 1 });
  const a = await phone.newPage();
  const errors = [];
  a.on("pageerror", e => errors.push(e.message));
  a.on("console", m => { if (m.type() === "error") errors.push(m.text()); });

  await a.goto(URL);
  await a.waitForFunction(() => document.getElementById("conn").textContent === "connected");
  await sleep(400);
  check(await a.textContent("#state") === "DISARMED", "shows DISARMED after connect");
  check(await a.isEnabled("#arm"), "ARM enabled for the first (owner) client");
  check(await a.isHidden("#spectator"), "no spectator banner for owner");

  // Heartbeat rate
  await fetch(URL + "reset-log");
  await sleep(1000);
  let { log } = await getLog();
  const drives = log.filter(l => l.type === "Drive");
  check(drives.length >= 15 && drives.length <= 25, `joystick heartbeat ~20/s (got ${drives.length})`);
  check(log.every(l => l.type !== "Invalid"), "every message accepted by the firmware parser");

  // Joystick geometry: drag up-right, while holding try to arm -> refused
  const box = await a.locator("#stick").boundingBox();
  const cx = box.x + box.width / 2, cy = box.y + box.height / 2;
  await a.mouse.move(cx, cy);
  await a.mouse.down();
  await a.mouse.move(cx + box.width * 0.45, cy - box.height * 0.45, { steps: 5 });
  await sleep(200);
  ({ log } = await getLog());
  const last = log.filter(l => l.type === "Drive").pop().text.split(" ").map(Number);
  check(last[1] > 0.5 && last[2] > 0.5, `drag up-right -> turn>0, throttle>0 (${last.slice(1)})`);
  check(Math.hypot(last[1], last[2]) <= 1.0001, "stick magnitude clamped to 1");
  await a.locator("#arm").dispatchEvent("click");  // click via event: mouse is busy holding the stick
  await sleep(300);
  check((await getLog()).robot.state === "disarmed", "arming refused while stick deflected");
  check((await a.textContent("#toast")).includes("Let go"), "user told why arming was refused");
  await a.mouse.up();
  await sleep(150);
  ({ log } = await getLog());
  const rel = log.filter(l => l.type === "Drive").pop().text;
  check(rel === "D 0.000 0.000", `stick recentres on release (${rel})`);

  // Arm
  await a.click("#arm");
  await sleep(300);
  check((await getLog()).robot.state === "armed", "ARM arms the robot");
  check(await a.textContent("#arm") === "DISARM", "button flips to DISARM");
  check(await a.textContent("#state") === "ARMED", "state pill ARMED");

  // Speed limit slider
  await a.locator("#lim").fill("80");
  await sleep(200);
  check(Math.abs((await getLog()).robot.lim - 0.8) < 1e-6, "speed slider sends limit 0.80");

  // Keyboard
  await a.keyboard.down("ArrowUp");
  await sleep(150);
  ({ log } = await getLog());
  check(log.filter(l => l.type === "Drive").pop().text === "D 0.000 1.000", "ArrowUp = full throttle");
  await a.keyboard.up("ArrowUp");
  await sleep(150);

  // Second device: spectator
  const other = await browser.newContext({ ...devices["iPhone 13"], deviceScaleFactor: 1 });
  const b = await other.newPage();
  b.on("pageerror", e => errors.push("B: " + e.message));
  await b.goto(URL);
  await b.waitForFunction(() => document.getElementById("conn").textContent === "connected");
  await sleep(400);
  check(await b.isVisible("#spectator"), "second device sees 'Another device is driving'");
  check(await b.isDisabled("#arm"), "second device cannot arm");
  await fetch(URL + "reset-log");
  await sleep(300);
  ({ log } = await getLog());
  check(log.filter(l => l.client !== 1).length === 0, "spectator does not stream joystick");

  // Spectator STOP works
  await b.click("#stop");
  await sleep(250);
  check((await getLog()).robot.state === "disarmed", "STOP from spectator stops the robot");

  // Take control
  await b.click("#claim");
  await sleep(400);
  check(await b.isHidden("#spectator") && await a.isVisible("#spectator"), "take control swaps driver");
  check(await b.isEnabled("#arm") && await a.isDisabled("#arm"), "only new driver can arm");

  // Fault display + reset
  await b.click("#arm");
  await sleep(200);
  await fetch(URL + "force-fault");
  await sleep(300);
  check(await b.isVisible("#fault") && (await b.textContent("#faultText")).includes("M3 front-right"), "fault banner names the stalled wheel");
  check(await b.textContent("#arm") === "RESET", "button becomes RESET during fault");
  await fetch(URL + "reset-log");
  await b.click("#arm");
  await sleep(200);
  ({ log } = await getLog());
  check(log.some(l => l.text === "A 0"), "RESET sends disarm (acknowledge fault)");

  // Page hidden -> gentle stop
  await fetch(URL + "reset-log");
  await b.evaluate(() => { Object.defineProperty(document, "hidden", { value: true, configurable: true }); document.dispatchEvent(new Event("visibilitychange")); });
  await sleep(300);
  ({ log } = await getLog());
  check(log.some(l => l.text === "A 0" && l.client === 2), "hiding the page sends a stop");
  check(!log.some(l => l.type === "Drive" && l.client === 2), "hidden page stops the heartbeat");

  // Screenshots for a visual check
  await a.screenshot({ path: SHOTS + "/shot_spectator.png" });
  const layoutOk = async (page, label) => {
    const r = await page.evaluate(() => {
      const b = document.getElementById("stick").getBoundingClientRect();
      const side = document.getElementById("stop").getBoundingClientRect();
      return { w: b.width, h: b.height, top: b.top, bottom: b.bottom, left: b.left, right: b.right,
               vw: innerWidth, vh: innerHeight, stopBottom: side.bottom, scrollW: document.documentElement.scrollWidth };
    });
    check(Math.abs(r.w - r.h) < 1 && r.w >= 180, `${label}: joystick is square and usable (${r.w.toFixed(0)}x${r.h.toFixed(0)})`);
    check(r.top >= 0 && r.bottom <= r.vh && r.left >= 0 && r.right <= r.vw, `${label}: joystick fully on screen`);
    check(r.stopBottom <= r.vh, `${label}: STOP button on screen without scrolling`);
    check(r.scrollW <= r.vw, `${label}: no horizontal scroll`);
  };
  await layoutOk(a, "portrait phone");
  const land = await browser.newContext({ viewport: { width: 844, height: 390 }, deviceScaleFactor: 1, hasTouch: true, isMobile: true });
  const c = await land.newPage();
  c.on("pageerror", e => errors.push("C: " + e.message));
  await c.goto(URL);
  await sleep(600);
  await layoutOk(c, "landscape phone");
  await c.screenshot({ path: SHOTS + "/shot_landscape.png" });
  // Driver's view while armed and steering
  await b.evaluate(() => { Object.defineProperty(document, "hidden", { value: false, configurable: true }); });
  await b.click("#arm"); await sleep(300);
  const bb = await b.locator("#stick").boundingBox();
  await b.mouse.move(bb.x + bb.width / 2, bb.y + bb.height / 2); await b.mouse.down();
  await b.mouse.move(bb.x + bb.width * 0.62, bb.y + bb.height * 0.2, { steps: 4 }); await sleep(400);
  await b.screenshot({ path: SHOTS + "/shot_driving.png" });
  await b.mouse.up();

  check(errors.length === 0, `no JS errors (${errors.join(" | ")})`);
  await browser.close();
  console.log(failures ? `\n${failures} FAILED` : "\nALL PASSED");
  mock.server.close(); mock.parser.kill();
  process.exit(failures ? 1 : 0);
})();
