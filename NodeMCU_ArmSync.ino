/*
  ╔══════════════════════════════════════════════════════════╗
  ║  ArmSync v4  —  Dual-Mode Robotic Arm Controller         ║
  ║  Board : NodeMCU 1.0 (ESP-12E / ESP8266)                 ║
  ╠══════════════════════════════════════════════════════════╣
  ║  PIN ASSIGNMENTS                                          ║
  ║  D1 (GPIO5)  → Base servo   (continuous rotation)        ║
  ║  D2 (GPIO4)  → Shoulder servo                            ║
  ║  D4 (GPIO2)  → Elbow servo  ← MOVED from D6              ║
  ║  D5 (GPIO14) → Gripper servo                             ║
  ║  D6 (GPIO12) → HC-SR04  TRIG                             ║
  ║  D7 (GPIO13) → HC-SR04  ECHO  (use voltage divider!)     ║
  ╠══════════════════════════════════════════════════════════╣
  ║  HC-SR04 ECHO wiring:                                    ║
  ║    ECHO pin → 1 kΩ → D7  AND  2 kΩ → GND                ║
  ║  (divides 5 V echo to ~3.3 V safe for NodeMCU)          ║
  ╚══════════════════════════════════════════════════════════╝
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// ═══════════════════════════════════════════════════════════
//  ①  EDIT THESE  ─ Wi-Fi credentials
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID = "OPPO Reno14";
const char* WIFI_PASS = "navajyot";
// Fallback AP (no router needed)
const char* AP_SSID   = "ArmSync";
const char* AP_PASS   = "armsync123";

// ═══════════════════════════════════════════════════════════
//  ②  EDIT THESE  ─ Tune pick/place angles for your arm
//     {Shoulder °, Elbow °, Gripper °}
//     GRIP_OPEN  = gripper wide open
//     GRIP_CLOSE = gripper gripping
// ═══════════════════════════════════════════════════════════
// ─── ② TUNE THESE: Gripper angles ────────────────────────
// GRIP_OPEN  = angle when gripper is fully open (wide)
// GRIP_CLOSE = angle when gripper is gripping tight
const int GRIP_OPEN  = 30;    // tune ↑↓
const int GRIP_CLOSE = 160;   // tune ↑↓

// Object detection distance (cm) — arm triggers when closer than this
const float DETECT_CM = 12.0f;

// ─── ② TUNE THESE: Arm angles for pick/place ──────────────
// Each SeqStep = { Shoulder°, Elbow°, Gripper°, holdMs }
// Shoulder: LOWER angle = arm leans BACK, HIGHER angle = arm leans FORWARD
// Elbow   : LOWER angle = elbow raised UP, HIGHER angle = elbow bent DOWN toward floor
// PICK sequence: open → lower to object → grab → lift up
struct SeqStep { int s, e, g; unsigned long ms; };

const SeqStep PICK_SEQ[] = {
  {120, 120,  GRIP_OPEN,  700},  // Step 1: lean forward & raise elbow, open gripper (ready)
  {60, 160, GRIP_OPEN,  800},  // Step 2: lower elbow DOWN toward object
  {55, 140, GRIP_CLOSE, 850},  // Step 3: GRAB — close gripper, wait firmly
  { 80, 60,  GRIP_CLOSE, 800},  // Step 4: LIFT — raise elbow & pull shoulder back with object
};
const int PICK_STEPS = 4;

// PLACE sequence: lower to drop spot → open gripper → raise arm back up
const SeqStep PLACE_SEQ[] = {
  {120, 60,  GRIP_CLOSE, 600},  // Step 1: lean forward over drop spot, elbow raised
  {60, 160, GRIP_CLOSE, 700},  // Step 2: lower elbow DOWN to drop height
  {50, 140, GRIP_OPEN,  800},  // Step 3: RELEASE — open gripper
  { 80, 60,  GRIP_OPEN,  600},  // Step 4: raise arm back to ready
};
const int PLACE_STEPS = 4;

// ─── ② TUNE: Home position joint angles ──────────────────
// These are the angles the arm snaps to when you press HOME
// Shoulder=140 (arm leaning forward toward sensor), Elbow=45 (raised), Gripper=75 (half open)

// ─── ② TUNE: Home base rotation ─────────────────────────
// The base rotates to face the ultrasonic sensor direction.
// HOME_BASE_DIR: 1=CW, -1=CCW
// HOME_BASE_MS : ms to spin (0 = don't rotate, arm already faces sensor)
const int          HOME_BASE_DIR = 1;
const unsigned long HOME_BASE_MS = 0;    // set to e.g. 1200 if base needs to rotate to face sensor

// ═══════════════════════════════════════════════════════════
//  HARDWARE PINS
// ═══════════════════════════════════════════════════════════
const uint8_t BASE_PIN = D1;
const uint8_t TRIG_PIN = D6;
const uint8_t ECHO_PIN = D7;

// Joint servo pins: index 0=Shoulder, 1=Elbow, 2=Gripper
const uint8_t JOINT_PIN[3]  = {D2,  D4,  D5};
const char*   JOINT_NAME[3] = {"Shoulder", "Elbow", "Gripper"};
const int     MIN_ANGLE[3]  = { 20,  20, 30};
const int     MAX_ANGLE[3]  = {160, 160, 120};
const int     HOME_ANGLE[3] = {140, 140, 75};  // Shoulder=140 (forward), Elbow=45 (raised), Gripper=75

// PWM pulse range for SG90
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

// ─── Base continuous-rotation servo ───────────────────────
const int   BASE_DIR         = 1;       // flip to -1 if rotation is reversed
const int   BASE_STOP_US     = 1500;
const int   BASE_SPEED_US[3] = {30, 45, 60};   // us offset per speed level
const float BASE_DPS[3]      = {10.0f, 20.0f, 30.0f}; // deg/s per speed level

// ─── Joint motion constants ────────────────────────────────
const float JOINT_DPS[3]      = { 5.0f, 12.0f, 25.0f};  // deg/s per speed level
const float JOINT_SCALE[3]    = { 0.80f,  1.0f,  1.0f};
const float JOINT_ACCEL[3]    = {30.0f,  50.0f, 80.0f};
const int   UPHILL_DIR[3]     = {1, 0, 0};
const float UPHILL_SCALE[3]   = {0.75f, 1.0f, 1.0f};
const float MULTI_MOVE_SCALE  = 0.80f;
const float SNAP_DEG          = 0.05f;
const unsigned long STEP_MS   = 20;

// ═══════════════════════════════════════════════════════════
//  STATE VARIABLES
// ═══════════════════════════════════════════════════════════
ESP8266WebServer server(80);

Servo baseServo;
Servo joint[3];

float current[3], target[3], vel[3];
int   lastUs[3];
int   speedLevel = 1;

// Active mode: 1 = Manual, 2 = Auto sense-and-place
int activeMode = 1;

// Base servo state
bool  baseAttached  = false;
int   baseDir       = 0;
bool  turnActive    = false;
unsigned long baseDeadline  = 0;
unsigned long turnEnd       = 0;
unsigned long baseIdleSince = 0;

// HC-SR04
float         lastDistCm  = 999.0f;
unsigned long lastDistRead = 0;

// Auto-mode state machine
enum AutoState {
  AUTO_IDLE,      // waiting for object in range
  AUTO_PICKING,   // executing pick sequence
  AUTO_HOLDING,   // holding object — waiting for drop direction
  AUTO_TURNING,   // rotating base to drop location
  AUTO_PLACING,   // executing place sequence
  AUTO_RETURNING, // rotating base back to sensor / center
  AUTO_DONE       // brief pause before resetting to IDLE
};

AutoState     autoState = AUTO_IDLE;
int           autoStep  = 0;
unsigned long stepDone  = 0;   // millis() deadline for current step
int           dropDir   = 0;   // −1=left, 0=straight, +1=right
int           pickCount = 0;

unsigned long lastStep = 0;

// ─── Runtime-tunable settings (changed via /settings from dashboard) ─
int   RT_GRIP_OPEN  = 60;    // gripper open angle
int   RT_GRIP_CLOSE = 160;   // gripper close angle
float RT_DETECT_CM  = 11.0f; // auto-pick trigger distance
// Home angles (overrides HOME_ANGLE[] at runtime)
int   RT_HOME[3]    = {140, 140, 75}; // Shoulder, Elbow, Gripper
// Pick reach position (step 2: shoulder & elbow when lowering to object)
int   RT_PICK_S = 120;  // shoulder angle when reaching for object
int   RT_PICK_E = 140;  // elbow angle when lowering to object (higher = more down)
// Lift position (step 4: after grab, lift up)
int   RT_LIFT_S = 80;   // shoulder after grab
int   RT_LIFT_E = 60;   // elbow after grab (lower = more up)
// Duration in ms to turn 90° for drop and return back to sensor
unsigned long RT_TURN90_MS = 1500;

// ═══════════════════════════════════════════════════════════
//  HTML DASHBOARD  (served from PROGMEM)
// ═══════════════════════════════════════════════════════════
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ArmSync</title>
<style>
*{box-sizing:border-box}
body{margin:0;padding:16px;background:#0b1220;color:#e5edf8;font-family:Arial,sans-serif;user-select:none;-webkit-user-select:none}
.wrap{max-width:560px;margin:auto}
header{display:flex;justify-content:space-between;align-items:center;margin:4px 2px 16px}
h1{font-size:21px;margin:0}
.status{font-size:12px;color:#7dd3fc}
.card{background:#172338;border:1px solid #263750;border-radius:16px;padding:16px;margin-bottom:14px}
h2{font-size:14px;color:#a9bad1;margin:0 0 14px;font-weight:600}
.row{display:flex;gap:10px;align-items:center}
button{border:0;border-radius:11px;padding:13px;background:#30445f;color:white;font-size:15px;flex:1;touch-action:manipulation;cursor:pointer}
button.on{background:#087eb5}
button.stop{background:#b91c1c;font-weight:bold}
button.drop{background:#065f3b}
button:disabled{opacity:.4;cursor:not-allowed}
.speed button{padding:10px 5px}

/* ── Tabs ── */
.tabs{display:flex;gap:8px;margin-bottom:14px}
.tab{flex:1;padding:13px 4px;border-radius:12px;border:2px solid #263750;background:#172338;color:#a9bad1;font-size:13px;font-weight:700;cursor:pointer;text-align:center;transition:all .2s}
.tab.active{border-color:#38bdf8;color:#38bdf8;background:#0c2035}
.panel{display:none}
.panel.active{display:block}
/* ── Settings panel ── */
.srow{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.srow label{font-size:14px;color:#c8d8ea}
.sval{color:#38bdf8;font-weight:bold;min-width:38px;text-align:right;font-variant-numeric:tabular-nums}
input.srange{width:100%;height:28px;accent-color:#38bdf8;touch-action:pan-y;margin-bottom:10px}
.ssave{background:#065f3b;padding:12px;font-size:15px;font-weight:700;width:100%;border-radius:11px;margin-top:4px}
.ssave:active{background:#044028}
.sgroup{border-top:1px solid #263750;padding-top:12px;margin-top:12px}

/* ── Joystick ── */
.joy-wrap{display:flex;justify-content:center;padding:5px 0 10px}
#joy{width:min(76vw,280px);height:min(76vw,280px);max-width:280px;max-height:280px;border-radius:50%;position:relative;touch-action:none;
  background:radial-gradient(circle,#304866 0 2px,transparent 3px),
    linear-gradient(90deg,transparent 49.5%,#526780 49.5% 50.5%,transparent 50.5%),
    linear-gradient(transparent 49.5%,#526780 49.5% 50.5%,transparent 50.5%),#223650;
  border:2px solid #526780;box-shadow:inset 0 0 0 18px #1c2c43}
#stick{position:absolute;width:27%;height:27%;left:50%;top:50%;transform:translate(-50%,-50%);border-radius:50%;
  background:linear-gradient(145deg,#38bdf8,#0369a1);border:3px solid #bae6fd;box-shadow:0 5px 14px #0008;pointer-events:none;z-index:2}
.axis{display:flex;justify-content:space-between;color:#8fa7c4;font-size:12px;margin-top:8px}

/* ── Sliders ── */
.control{margin:17px 0 5px}
.control-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.control-head label{font-size:15px}
.value{color:#7dd3fc;font-weight:bold;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;height:32px;accent-color:#38bdf8;touch-action:pan-y}
.actions{display:flex;gap:10px}
.hint{font-size:12px;color:#91a4bd;line-height:1.5;margin:10px 0 0}

/* ── Auto mode stats ── */
.stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px}
.stat-box{background:#0e1e30;border-radius:10px;padding:14px;text-align:center}
.stat-label{font-size:11px;color:#7a93b0;margin-bottom:5px}
.stat-val{font-size:26px;font-weight:bold;color:#38bdf8;font-variant-numeric:tabular-nums}
.stat-unit{font-size:11px;color:#7a93b0}
.state-badge{display:inline-block;padding:5px 14px;border-radius:20px;font-size:13px;font-weight:700;background:#263750;color:#a9bad1;transition:all .3s}
.state-badge.detecting{background:#163320;color:#4ade80}
.state-badge.picking{background:#12264a;color:#60a5fa}
.state-badge.holding{background:#3b1f00;color:#fb923c}
.state-badge.turning{background:#2a1545;color:#a78bfa}
.state-badge.placing{background:#2d1b50;color:#c084fc}
.state-badge.returning{background:#1e3a5f;color:#38bdf8}
.dist-bar-bg{background:#0e1e30;border-radius:8px;height:14px;overflow:hidden;margin-top:10px}
.dist-bar{height:100%;background:#38bdf8;border-radius:8px;transition:width .25s,background .25s;width:0%}
.dist-bar.alert{background:#ef4444}
.drop-label{font-size:13px;color:#7a93b0;text-align:center;margin-bottom:8px}
</style>
</head>

<body>
<div class="wrap">

<header>
  <h1>⚙️ ArmSync</h1>
  <span class="status" id="status">Connecting…</span>
</header>

<!-- Mode tabs -->
<div class="tabs">
  <div class="tab active" id="tab1" onclick="switchMode(1)">🕹️ Manual</div>
  <div class="tab"        id="tab2" onclick="switchMode(2)">🤖 Auto</div>
  <div class="tab"        id="tab3" onclick="switchMode(3)">⚙️ Settings</div>
</div>

<!-- Emergency stop — always visible -->
<div class="card">
  <div class="actions">
    <button class="stop" onclick="doStop()">⛔ EMERGENCY STOP</button>
  </div>
</div>

<!-- ═══════════════════════════ MODE 1 PANEL ═══════════════════════════ -->
<div class="panel active" id="panel1">

  <div class="card">
    <h2>XY JOYSTICK</h2>
    <div class="joy-wrap">
      <div id="joy"><div id="stick"></div></div>
    </div>
    <div class="axis">
      <span>◀ LEFT</span>
      <span>BASE ROTATION</span>
      <span>RIGHT ▶</span>
    </div>
    <p class="hint">Up / Down → Elbow &nbsp;|&nbsp; Left / Right → Base rotation<br>Release the joystick to stop the base.</p>
  </div>

  <div class="card">
    <h2>ARM POSITION</h2>
    <div class="control">
      <div class="control-head">
        <label for="height">Height / Shoulder</label>
        <span class="value" id="heightVal">90°</span>
      </div>
      <input id="height" type="range" min="20" max="160" value="90">
    </div>
    <div class="control">
      <div class="control-head">
        <label for="grip">Gripper</label>
        <span class="value" id="gripVal">90°</span>
      </div>
      <input id="grip" type="range" min="30" max="120" value="90">
    </div>
  </div>

  <div class="card">
    <h2>MOTION SPEED</h2>
    <div class="row speed" id="speed"></div>
  </div>

  <div class="card">
    <div class="actions">
      <button onclick="doHome()">🏠 HOME</button>
    </div>
  </div>

</div><!-- /panel1 -->

<!-- ═══════════════════════════ MODE 2 PANEL ═══════════════════════════ -->
<div class="panel" id="panel2">

  <div class="card">
    <h2>SENSOR &amp; STATE</h2>
    <div class="stat-grid">
      <div class="stat-box">
        <div class="stat-label">Distance</div>
        <div class="stat-val" id="distVal">—</div>
        <div class="stat-unit">cm</div>
      </div>
      <div class="stat-box">
        <div class="stat-label">Total Picks</div>
        <div class="stat-val" id="pickCount">0</div>
      </div>
    </div>
    <div class="dist-bar-bg">
      <div class="dist-bar" id="distBar"></div>
    </div>
    <div style="margin-top:12px;text-align:center">
      State: <span class="state-badge" id="autoState">IDLE</span>
    </div>
    <p class="hint" style="text-align:center;margin-top:8px">
      Place an object within <strong>12 cm</strong> of the sensor to trigger automatic pickup.
    </p>
  </div>

  <div class="card">
    <h2>DROP DIRECTION</h2>
    <p class="drop-label">Active only while arm is holding an object</p>
    <div class="row" style="gap:8px">
      <button class="drop" id="btnLeft"  onclick="doDrop(-1)" disabled>◀ Left 90°</button>
      <button class="drop" id="btnFront" onclick="doDrop(0)"  disabled>⬇ In Front</button>
      <button class="drop" id="btnRight" onclick="doDrop(1)"  disabled>Right 90° ▶</button>
    </div>
  </div>

  <div class="card">
    <h2>🎯 BASE ALIGNMENT TO SENSOR</h2>
    <p class="hint" style="margin:0 0 10px">Jog the base so the arm faces the ultrasonic sensor before picking.</p>
    <div class="row" style="gap:8px">
      <button onclick="nudgeBase(-1)">◀ Nudge Left</button>
      <button onclick="nudgeBase(0)" class="stop" style="flex:0.6">Stop</button>
      <button onclick="nudgeBase(1)">Nudge Right ▶</button>
    </div>
  </div>

</div><!-- /panel2 -->

<!-- ════════════════════════ SETTINGS PANEL ════════════════════════ -->
<div class="panel" id="panel3">

  <div class="card">
    <h2>🦾 GRIPPER</h2>

    <div class="srow"><label>Open angle</label><span class="sval" id="vgo">30°</span></div>
    <input class="srange" type="range" id="sgo" min="20" max="120" value="30"
      oninput="$('vgo').textContent=this.value+'°'">

    <div class="srow"><label>Close angle (grip tight)</label><span class="sval" id="vgc">110°</span></div>
    <input class="srange" type="range" id="sgc" min="20" max="160" value="110"
      oninput="$('vgc').textContent=this.value+'°'">
  </div>

  <div class="card">
    <h2>🏠 HOME POSITION</h2>

    <div class="srow"><label>Shoulder</label><span class="sval" id="vhs">140°</span></div>
    <input class="srange" type="range" id="shs" min="20" max="160" value="140"
      oninput="$('vhs').textContent=this.value+'°'">

    <div class="srow"><label>Elbow</label><span class="sval" id="vhe">45°</span></div>
    <input class="srange" type="range" id="she" min="20" max="160" value="45"
      oninput="$('vhe').textContent=this.value+'°'">

    <div class="srow"><label>Gripper</label><span class="sval" id="vhg">75°</span></div>
    <input class="srange" type="range" id="shg" min="20" max="120" value="75"
      oninput="$('vhg').textContent=this.value+'°'">
  </div>

  <div class="card">
    <h2>📡 AUTO DETECT</h2>

    <div class="srow"><label>Trigger distance (cm)</label><span class="sval" id="vdc">12 cm</span></div>
    <input class="srange" type="range" id="sdc" min="3" max="40" value="12"
      oninput="$('vdc').textContent=this.value+' cm'">
  </div>

  <div class="card">
    <h2>🤖 PICK POSITION</h2>
    <p class="hint" style="margin:0 0 12px">Arm position used to reach down and grab the object.</p>

    <div class="srow"><label>Shoulder (lean)</label><span class="sval" id="vps">120°</span></div>
    <input class="srange" type="range" id="sps" min="20" max="160" value="120"
      oninput="$('vps').textContent=this.value+'°'">

    <div class="srow"><label>Elbow (lower = up, higher = down)</label><span class="sval" id="vpe">140°</span></div>
    <input class="srange" type="range" id="spe" min="20" max="160" value="140"
      oninput="$('vpe').textContent=this.value+'°'">
  </div>

  <div class="card">
    <h2>📦 LIFT POSITION</h2>
    <p class="hint" style="margin:0 0 12px">Arm position after grabbing — holds object high.</p>

    <div class="srow"><label>Shoulder</label><span class="sval" id="vls">80°</span></div>
    <input class="srange" type="range" id="sls" min="20" max="160" value="80"
      oninput="$('vls').textContent=this.value+'°'">

    <div class="srow"><label>Elbow</label><span class="sval" id="vle">60°</span></div>
    <input class="srange" type="range" id="sle" min="20" max="160" value="60"
      oninput="$('vle').textContent=this.value+'°'">
  </div>

  <div class="card">
    <h2>🔄 BASE 90° TURN DURATION</h2>
    <p class="hint" style="margin:0 0 12px">Time in milliseconds to turn 90° for drop and return back.</p>

    <div class="srow"><label>Turn Time (ms)</label><span class="sval" id="vt90">1500 ms</span></div>
    <input class="srange" type="range" id="st90" min="400" max="4000" step="50" value="1500"
      oninput="$('vt90').textContent=this.value+' ms'">
  </div>

  <button class="ssave" onclick="saveSettings()">💾 APPLY SETTINGS</button>
  <p class="hint" style="text-align:center;margin-top:10px">Settings apply instantly. Changes are lost on power-off.<br>Test with HOME or switch to Auto mode.</p>

</div><!-- /panel3 -->

</div><!-- /wrap -->

<script>
const $ = id => document.getElementById(id);
const get = url => fetch(url, {cache:'no-store'}).catch(()=>{});

let cfg = null;
let baseDir = 0, keepAlive = null;
let lastElbow = -999, lastElbowSend = 0;
let curMode = 1;
let pollTimer = null;
let joystickActive = false;   // true only while finger/mouse is held down on joystick

// ─── Mode switching ──────────────────────────────────────
function switchMode(m) {
  if (m === curMode) return;
  curMode = m;
  // Update tabs & panels (including settings tab 3)
  [1,2,3].forEach(i => {
    $('tab'  +i).className = 'tab'  + (m===i ? ' active' : '');
    $('panel'+i).className = 'panel'+ (m===i ? ' active' : '');
  });
  // Stop joystick if leaving manual
  releaseJoystick();
  // Only tell firmware for modes 1 & 2 (settings = UI-only tab)
  if (m === 1 || m === 2) get('/mode?m=' + m);
  // Poll only in Mode 2
  if (m === 2) startPoll(); else stopPoll();
}

// ─── Mode 2 polling ──────────────────────────────────────
function startPoll() {
  if (!pollTimer) pollTimer = setInterval(pollAuto, 500);
}
function stopPoll() {
  clearInterval(pollTimer);
  pollTimer = null;
}

function pollAuto() {
  fetch('/auto', {cache:'no-store'})
    .then(r => r.json())
    .then(d => {
      const dist = parseFloat(d.dist);
      $('distVal').textContent = (dist < 400) ? dist.toFixed(1) : '—';

      // Distance bar: 0 cm → 100%, 50 cm → 0%
      const pct = Math.max(0, Math.min(100, (1 - dist / 50) * 100));
      const bar = $('distBar');
      bar.style.width = pct + '%';
      bar.className   = 'dist-bar' + (dist < 13 ? ' alert' : '');

      $('pickCount').textContent = d.picks;

      // State badge
      const state = d.state;
      const badge = $('autoState');
      badge.textContent = state;
      const cls = {
        DETECTING:' detecting', PICKING:' picking',
        HOLDING:' holding',    TURNING:' turning',
        PLACING:' placing',    RETURNING:' returning'
      }[state] || '';
      badge.className = 'state-badge' + cls;

      // Drop buttons: only enabled when HOLDING
      const holding = (state === 'HOLDING');
      ['btnLeft','btnFront','btnRight'].forEach(id => $(id).disabled = !holding);

      $('status').textContent = 'Mode 2 · ' + state;
    })
    .catch(() => {
      $('status').textContent = 'Connection lost';
    });
}

function doDrop(dir) {
  get('/drop?dir=' + dir);
}

function nudgeBase(dir) {
  get('/nudgebase?dir=' + dir);
}

// ─── Manual mode ─────────────────────────────────────────
function setJoint(i, a) {
  get('/set?j=' + i + '&a=' + Math.round(a));
}

function stopBase() {
  if (keepAlive) { clearInterval(keepAlive); keepAlive = null; }
  if (baseDir !== 0) { baseDir = 0; get('/base?v=0'); }
}

function driveBase(dir) {
  if (dir === baseDir) return;
  if (keepAlive) { clearInterval(keepAlive); keepAlive = null; }
  baseDir = dir;
  if (dir === 0) { get('/base?v=0'); return; }
  get('/base?v=' + dir);
  keepAlive = setInterval(() => get('/base?v=' + dir), 150);
}

function updateJoystick(e) {
  if (!joystickActive) return;   // ignore stray events after release
  const rect   = $('joy').getBoundingClientRect();
  const cx     = rect.left + rect.width  / 2;
  const cy     = rect.top  + rect.height / 2;
  const radius = rect.width * 0.34;
  let dx = (e.clientX - cx) / radius;
  let dy = (cy - e.clientY) / radius;
  const mag = Math.hypot(dx, dy);
  if (mag > 1) { dx /= mag; dy /= mag; }
  $('stick').style.left = (50 + dx * 34) + '%';
  $('stick').style.top  = (50 - dy * 34) + '%';
  driveBase(Math.abs(dx) > 0.22 ? (dx > 0 ? 1 : -1) : 0);
  if (!cfg) return;
  const j     = cfg.joints[1];
  // Negate dy so joystick UP = arm raises (lower angle) and DOWN = arm lowers
  const elbow = Math.round(j.min + Math.max(0, Math.min(1, (-dy + 1) / 2)) * (j.max - j.min));
  const now   = Date.now();
  if (Math.abs(elbow - lastElbow) >= 2 && now - lastElbowSend > 65) {
    lastElbow = elbow; lastElbowSend = now;
    setJoint(1, elbow);
    $('status').textContent = 'Elbow ' + elbow + '°';
  }
}

function releaseJoystick() {
  joystickActive = false;
  lastElbow = -999;             // reset so next drag starts fresh
  stopBase();
  get('/hold?j=1');             // freeze elbow at its current position
  $('stick').style.left = '50%';
  $('stick').style.top  = '50%';
}

function buildUI(c) {
  ['Slow','Medium','Fast'].forEach((name, i) => {
    const b = document.createElement('button');
    b.textContent = name; b.id = 's' + i;
    b.onclick = () => { get('/speed?l=' + i); markSpeed(i); };
    $('speed').appendChild(b);
  });
  $('height').min = c.joints[0].min; $('height').max = c.joints[0].max;
  $('grip').min   = c.joints[2].min; $('grip').max   = c.joints[2].max;
  $('height').value = c.joints[0].pos;
  $('grip').value   = c.joints[2].pos;
  $('height').oninput = () => { $('heightVal').textContent = $('height').value + '°'; setJoint(0, $('height').value); };
  $('grip').oninput   = () => { $('gripVal').textContent   = $('grip').value   + '°'; setJoint(2, $('grip').value);   };
  const joy = $('joy');
  joy.addEventListener('pointerdown', e => { e.preventDefault(); joystickActive = true; joy.setPointerCapture(e.pointerId); updateJoystick(e); });
  joy.addEventListener('pointermove', e => { if (joy.hasPointerCapture(e.pointerId)) updateJoystick(e); });
  ['pointerup','pointercancel','lostpointercapture'].forEach(ev => joy.addEventListener(ev, releaseJoystick));
}

function markSpeed(l) {
  for (let i = 0; i < 3; i++) $('s' + i).className = (i === l ? 'on' : '');
}

// ─── Settings ─────────────────────────────────────────────
function saveSettings() {
  const p = new URLSearchParams({
    go:  $('sgo').value,   // gripper open
    gc:  $('sgc').value,   // gripper close
    hs:  $('shs').value,   // home shoulder
    he:  $('she').value,   // home elbow
    hg:  $('shg').value,   // home gripper
    dc:  $('sdc').value,   // detect cm
    ps:  $('sps').value,   // pick shoulder
    pe:  $('spe').value,   // pick elbow
    ls:  $('sls').value,   // lift shoulder
    le:  $('sle').value,   // lift elbow
    t90: $('st90').value   // 90° turn ms
  });
  get('/settings?' + p.toString())
    .then(() => { $('status').textContent = '✔ Settings applied'; });
}

function doHome() { releaseJoystick(); get('/home').then(() => setTimeout(load, 700)); }

function doStop() {
  releaseJoystick();
  get('/stop').then(() => setTimeout(() => { load(); if (curMode===2) pollAuto(); }, 350));
}

function load() {
  fetch('/cfg', {cache:'no-store'})
    .then(r => r.json())
    .then(c => {
      cfg = c;
      if (!$('s0')) buildUI(c);
      markSpeed(c.speed);
      $('height').value = c.joints[0].pos; $('heightVal').textContent = c.joints[0].pos + '°';
      $('grip').value   = c.joints[2].pos; $('gripVal').textContent   = c.joints[2].pos + '°';
      $('status').textContent = 'Connected';
    })
    .catch(() => { $('status').textContent = 'Connection lost'; setTimeout(load, 1500); });
}

load();
</script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════
//  HARDWARE HELPERS
// ═══════════════════════════════════════════════════════════

int angleToUs(float a) {
  return SERVO_MIN_US + (int)(a / 180.0f * (SERVO_MAX_US - SERVO_MIN_US) + 0.5f);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000UL); // 25 ms timeout ≈ 430 cm
  if (dur == 0) return 999.0f;
  return dur * 0.0343f / 2.0f;
}

// ─── Base servo ───────────────────────────────────────────
void baseDrive(int dir) {
  if (!baseAttached) { baseServo.attach(BASE_PIN, 1000, 2000); baseAttached = true; }
  baseServo.writeMicroseconds(BASE_STOP_US + dir * BASE_DIR * BASE_SPEED_US[speedLevel]);
  baseDir = dir;
}

void baseStop() {
  if (baseAttached) baseServo.writeMicroseconds(BASE_STOP_US);
  baseDir = 0; turnActive = false; baseIdleSince = millis();
}

void updateBase(unsigned long now) {
  if  (turnActive && now >= turnEnd)                           baseStop();
  else if (!turnActive && baseDir != 0 && now > baseDeadline) baseStop();
  if (baseDir == 0 && baseAttached && now - baseIdleSince > 150) {
    baseServo.detach(); baseAttached = false;
  }
}

// ─── Joint smooth motion ──────────────────────────────────
void moveJoints(float dt) {
  int moving = 0;
  for (int i = 0; i < 3; i++) if (fabsf(target[i] - current[i]) > SNAP_DEG) moving++;
  float share = (moving > 1) ? MULTI_MOVE_SCALE : 1.0f;

  for (int i = 0; i < 3; i++) {
    float d = target[i] - current[i], ad = fabsf(d);
    if (ad <= SNAP_DEG) { current[i] = target[i]; vel[i] = 0; }
    else {
      int sgn = (d > 0) ? 1 : -1;
      float vmax = JOINT_DPS[speedLevel] * JOINT_SCALE[i] * share;
      if (sgn == UPHILL_DIR[i]) vmax *= UPHILL_SCALE[i];
      float vdes  = sgn * fminf(vmax, sqrtf(JOINT_ACCEL[i] * ad));
      float dvMax = JOINT_ACCEL[i] * dt;
      float dv    = vdes - vel[i];
      if (dv >  dvMax) dv =  dvMax;
      if (dv < -dvMax) dv = -dvMax;
      vel[i] += dv;
      float step = vel[i] * dt;
      if (step * d > 0 && fabsf(step) >= ad) { current[i] = target[i]; vel[i] = 0; }
      else current[i] += step;
    }
    int us = angleToUs(current[i]);
    if (us != lastUs[i]) { joint[i].writeMicroseconds(us); lastUs[i] = us; }
  }
}

// ─── Helpers for auto mode ────────────────────────────────
void setTargets(int s, int e, int g) {
  target[0] = constrain(s, MIN_ANGLE[0], MAX_ANGLE[0]);
  target[1] = constrain(e, MIN_ANGLE[1], MAX_ANGLE[1]);
  target[2] = constrain(g, MIN_ANGLE[2], MAX_ANGLE[2]);
}

bool jointsSettled() {
  for (int i = 0; i < 3; i++) if (fabsf(target[i] - current[i]) > 2.0f) return false;
  return true;
}

const char* autoStateName() {
  switch (autoState) {
    case AUTO_IDLE:     return "IDLE";
    case AUTO_PICKING:  return "PICKING";
    case AUTO_HOLDING:  return "HOLDING";
    case AUTO_TURNING:   return "TURNING";
    case AUTO_PLACING:   return "PLACING";
    case AUTO_RETURNING: return "RETURNING";
    case AUTO_DONE:      return "DONE";
  }
  return "IDLE";
}

// ═══════════════════════════════════════════════════════════
//  AUTO-MODE STATE MACHINE  (runs only when activeMode == 2)
// ═══════════════════════════════════════════════════════════
void autoLoop(unsigned long now) {
  if (activeMode != 2) return;

  // Read HC-SR04 every 200 ms (non-blocking cadence)
  if (now - lastDistRead >= 200) {
    lastDistRead = now;
    lastDistCm   = readDistanceCm();
  }

  switch (autoState) {

    // ── IDLE: watch for object ───────────────────────────
    case AUTO_IDLE:
      if (lastDistCm < RT_DETECT_CM) {
        autoState = AUTO_PICKING;
        autoStep  = 0;
        stepDone  = 0;
        Serial.println(F("AUTO: object detected → PICKING"));
      }
      break;

    // ── PICKING: run pick sequence step by step ──────────
    // Step 0: ready position (open gripper, raise elbow)
    // Step 1: lower to object using RT_PICK_S / RT_PICK_E
    // Step 2: close gripper (grab)
    // Step 3: lift using RT_LIFT_S / RT_LIFT_E
    case AUTO_PICKING: {
      if (autoStep >= PICK_STEPS) {
        autoState = AUTO_HOLDING;
        Serial.println(F("AUTO: pick done → HOLDING (awaiting drop command)"));
        break;
      }
      if (stepDone == 0) {
        // Override steps 1 and 3 with dashboard-tuned runtime values
        int s = PICK_SEQ[autoStep].s;
        int e = PICK_SEQ[autoStep].e;
        int g = PICK_SEQ[autoStep].g;
        if (autoStep == 1) { s = RT_PICK_S; e = RT_PICK_E; g = RT_GRIP_OPEN;  }
        if (autoStep == 2) { s = RT_PICK_S; e = RT_PICK_E; g = RT_GRIP_CLOSE; }
        if (autoStep == 3) { s = RT_LIFT_S; e = RT_LIFT_E; g = RT_GRIP_CLOSE; }
        setTargets(s, e, g);
        stepDone = now + PICK_SEQ[autoStep].ms;
      } else if (now >= stepDone && jointsSettled()) {
        autoStep++;
        stepDone = 0;
      }
      break;
    }

    // ── HOLDING: wait for /drop command from dashboard ───
    case AUTO_HOLDING:
      // Nothing to do — handleDrop() moves us out of this state
      break;

    // ── TURNING: wait for timed base rotation to finish ─
    case AUTO_TURNING:
      if (!turnActive) {
        autoState = AUTO_PLACING;
        autoStep  = 0;
        stepDone  = 0;
        Serial.println(F("AUTO: turn done → PLACING"));
      }
      break;

    // ── PLACING: run place sequence step by step ─────────
    // Uses same RT_PICK_S/E for the lowering step and RT_LIFT_S/E for raise-back
    case AUTO_PLACING: {
      if (autoStep >= PLACE_STEPS) {
        if (dropDir != 0) {
          // Object dropped at left/right: rotate base BACK to center/sensor
          baseDrive(-dropDir);
          turnActive = true;
          turnEnd    = millis() + RT_TURN90_MS;
          autoState  = AUTO_RETURNING;
          Serial.println(F("AUTO: place done → RETURNING base to sensor"));
        } else {
          autoState = AUTO_DONE;
          stepDone  = now + 600;
          Serial.println(F("AUTO: place done → DONE"));
        }
        break;
      }
      if (stepDone == 0) {
        int s = PLACE_SEQ[autoStep].s;
        int e = PLACE_SEQ[autoStep].e;
        int g = PLACE_SEQ[autoStep].g;
        if (autoStep == 1) { s = RT_PICK_S; e = RT_PICK_E; g = RT_GRIP_CLOSE; }
        if (autoStep == 2) { s = RT_PICK_S; e = RT_PICK_E; g = RT_GRIP_OPEN;  }
        if (autoStep == 3) { s = RT_LIFT_S; e = RT_LIFT_E; g = RT_GRIP_OPEN;  }
        setTargets(s, e, g);
        stepDone = now + PLACE_SEQ[autoStep].ms;
      } else if (now >= stepDone && jointsSettled()) {
        autoStep++;
        stepDone = 0;
      }
      break;
    }

    // ── RETURNING: wait for base to rotate back to sensor ─
    case AUTO_RETURNING:
      if (!turnActive) {
        baseStop();
        dropDir   = 0;
        autoState = AUTO_DONE;
        stepDone  = now + 400;
        Serial.println(F("AUTO: returned to sensor → DONE"));
      }
      break;

    // ── DONE: brief pause, increment counter, reset ──────
    case AUTO_DONE:
      if (now >= stepDone) {
        pickCount++;
        setTargets(RT_HOME[0], RT_HOME[1], RT_HOME[2]);
        autoState = AUTO_IDLE;
        Serial.print(F("AUTO: cycle complete. Total picks: "));
        Serial.println(pickCount);
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════
//  WEB ROUTE HANDLERS
// ═══════════════════════════════════════════════════════════

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleCfg() {
  String s = F("{\"speed\":");
  s += speedLevel;
  s += F(",\"mode\":");
  s += activeMode;
  s += F(",\"joints\":[");
  for (int i = 0; i < 3; i++) {
    if (i) s += ',';
    s += F("{\"name\":\""); s += JOINT_NAME[i];
    s += F("\",\"min\":"); s += MIN_ANGLE[i];
    s += F(",\"max\":"); s += MAX_ANGLE[i];
    s += F(",\"pos\":"); s += (int)roundf(target[i]);
    s += '}';
  }
  s += "]}";
  server.send(200, "application/json", s);
}

// /hold — freeze a joint at its current position (called on joystick release)
void handleHold() {
  if (server.hasArg("j")) {
    int j = constrain(server.arg("j").toInt(), 0, 2);
    target[j] = current[j];
    vel[j]    = 0;
  } else {
    for (int i = 0; i < 3; i++) { target[i] = current[i]; vel[i] = 0; }
  }
  server.send(200, "text/plain", "OK");
}

// /settings — update runtime-tunable values from Settings panel
void handleSettings() {
  #define GARG(k, def) (server.hasArg(k) ? server.arg(k).toInt() : (def))
  RT_GRIP_OPEN  = constrain(GARG("go", RT_GRIP_OPEN),  20, 120);
  RT_GRIP_CLOSE = constrain(GARG("gc", RT_GRIP_CLOSE), 20, 160);
  RT_HOME[0]    = constrain(GARG("hs", RT_HOME[0]),    MIN_ANGLE[0], MAX_ANGLE[0]);
  RT_HOME[1]    = constrain(GARG("he", RT_HOME[1]),    MIN_ANGLE[1], MAX_ANGLE[1]);
  RT_HOME[2]    = constrain(GARG("hg", RT_HOME[2]),    MIN_ANGLE[2], MAX_ANGLE[2]);
  RT_DETECT_CM  = (float)constrain(GARG("dc", (int)RT_DETECT_CM), 2, 60);
  RT_PICK_S     = constrain(GARG("ps", RT_PICK_S), MIN_ANGLE[0], MAX_ANGLE[0]);
  RT_PICK_E     = constrain(GARG("pe", RT_PICK_E), MIN_ANGLE[1], MAX_ANGLE[1]);
  RT_LIFT_S     = constrain(GARG("ls", RT_LIFT_S), MIN_ANGLE[0], MAX_ANGLE[0]);
  RT_LIFT_E     = constrain(GARG("le", RT_LIFT_E), MIN_ANGLE[1], MAX_ANGLE[1]);
  RT_TURN90_MS  = (unsigned long)constrain(GARG("t90", (int)RT_TURN90_MS), 300, 6000);
  #undef GARG
  Serial.println(F("Settings updated from dashboard."));
  server.send(200, "text/plain", "OK");
}

// /nudgebase — jog base left/right in Mode 2 for alignment to sensor
void handleNudgeBase() {
  int d = constrain(server.arg("dir").toInt(), -1, 1);
  if (d == 0) {
    baseStop();
  } else {
    baseDrive(d);
    turnActive = true;
    turnEnd    = millis() + 300; // jog for 300ms
  }
  server.send(200, "text/plain", "OK");
}

// /set — move a joint (Manual mode only)
void handleSet() {
  if (activeMode != 1) { server.send(403, "text/plain", "manual off"); return; }
  if (!server.hasArg("j") || !server.hasArg("a")) { server.send(400, "text/plain", "missing"); return; }
  int j = server.arg("j").toInt(), a = server.arg("a").toInt();
  if (j < 0 || j >= 3) { server.send(400, "text/plain", "bad joint"); return; }
  target[j] = constrain(a, MIN_ANGLE[j], MAX_ANGLE[j]);
  server.send(200, "text/plain", "OK");
}

// /base — drive base CW/CCW (Manual mode only)
void handleBase() {
  if (activeMode != 1) { server.send(403, "text/plain", "manual off"); return; }
  int v = constrain(server.arg("v").toInt(), -1, 1);
  if (v == 0) baseStop();
  else { baseDrive(v); turnActive = false; baseDeadline = millis() + 400; }
  server.send(200, "text/plain", "OK");
}

// /turn — timed base turn (Manual mode only)
void handleTurn() {
  if (activeMode != 1) { server.send(403, "text/plain", "manual off"); return; }
  int deg = constrain(server.arg("deg").toInt(), -720, 720);
  if (deg == 0) baseStop();
  else {
    baseDrive(deg > 0 ? 1 : -1);
    turnActive = true;
    turnEnd    = millis() + (unsigned long)(fabsf((float)deg) / BASE_DPS[speedLevel] * 1000.0f);
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  speedLevel = constrain(server.arg("l").toInt(), 0, 2);
  if (baseDir != 0) baseDrive(baseDir);
  server.send(200, "text/plain", "OK");
}

void handleHome() {
  // Use runtime-tunable home angles (edited from Settings panel)
  for (int i = 0; i < 3; i++) target[i] = RT_HOME[i];
  // Only rotate base if HOME_BASE_MS is non-zero (tune in constants above)
  if (HOME_BASE_MS > 0) {
    baseDrive(HOME_BASE_DIR);
    turnActive = true;
    turnEnd    = millis() + HOME_BASE_MS;
  } else {
    baseStop();  // ensure base stops if already spinning
  }
  server.send(200, "text/plain", "OK");
}

// /stop — emergency stop (works in any mode)
void handleStop() {
  for (int i = 0; i < 3; i++) { target[i] = current[i]; vel[i] = 0; }
  baseStop();
  if (activeMode == 2) { autoState = AUTO_IDLE; autoStep = 0; stepDone = 0; }
  server.send(200, "text/plain", "OK");
}

// /mode — switch between Mode 1 and Mode 2
void handleMode() {
  int m = constrain(server.arg("m").toInt(), 1, 2);
  if (m == activeMode) { server.send(200, "text/plain", "OK"); return; }

  // Hold current position (don't snap) and stop base
  for (int i = 0; i < 3; i++) { target[i] = current[i]; vel[i] = 0; }
  baseStop();

  if (m == 2) {
    // Reset auto state machine when entering Mode 2
    autoState  = AUTO_IDLE;
    autoStep   = 0;
    stepDone   = 0;
    dropDir    = 0;
    lastDistCm = 999.0f;
    // Set joints to Home / Ready position for auto-pick
    for (int i = 0; i < 3; i++) target[i] = RT_HOME[i];
  }

  activeMode = m;
  Serial.print(F("Mode → ")); Serial.println(m);
  server.send(200, "text/plain", "OK");
}

// /auto — return sensor + state JSON (Mode 2 polling)
void handleAuto() {
  String s = F("{\"dist\":");
  s += lastDistCm;
  s += F(",\"state\":\"");
  s += autoStateName();
  s += F("\",\"picks\":");
  s += pickCount;
  s += '}';
  server.send(200, "application/json", s);
}

// /drop — issue drop direction command (Mode 2 HOLDING state only)
void handleDrop() {
  if (activeMode != 2)           { server.send(403, "text/plain", "not in Mode 2"); return; }
  if (autoState != AUTO_HOLDING) { server.send(400, "text/plain", "not holding");   return; }

  dropDir = constrain(server.arg("dir").toInt(), -1, 1);

  if (dropDir == 0) {
    // Drop in front — no base rotation needed
    autoState = AUTO_PLACING;
    autoStep  = 0;
    stepDone  = 0;
    Serial.println(F("DROP: in front"));
  } else {
    // Turn 90° then place
    baseDrive(dropDir);
    turnActive = true;
    turnEnd    = millis() + RT_TURN90_MS;
    autoState  = AUTO_TURNING;
    Serial.print(F("DROP: turning ")); Serial.println(dropDir > 0 ? "right" : "left");
  }
  server.send(200, "text/plain", "OK");
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);

  // HC-SR04 pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Attach joint servos and move to HOME one at a time
  for (int i = 0; i < 3; i++) {
    current[i] = target[i] = HOME_ANGLE[i];
    vel[i]    = 0;
    lastUs[i] = angleToUs(current[i]);
    joint[i].attach(JOINT_PIN[i], SERVO_MIN_US, SERVO_MAX_US);
    joint[i].writeMicroseconds(lastUs[i]);
    delay(400);
  }

  // Wi-Fi
  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("Connecting to WiFi"));
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250); Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("✔ Open http://"));
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print(F("AP mode — join '"));
    Serial.print(AP_SSID);
    Serial.print(F("' then open http://"));
    Serial.println(WiFi.softAPIP());
  }

  // Register all routes
  server.on("/",      handleRoot);
  server.on("/cfg",   handleCfg);
  server.on("/set",   handleSet);
  server.on("/base",  handleBase);
  server.on("/turn",  handleTurn);
  server.on("/speed", handleSpeed);
  server.on("/home",  handleHome);
  server.on("/stop",  handleStop);
  server.on("/mode",     handleMode);
  server.on("/hold",     handleHold);
  server.on("/auto",     handleAuto);
  server.on("/drop",      handleDrop);
  server.on("/nudgebase", handleNudgeBase);
  server.on("/settings",  handleSettings);

  server.begin();
  lastStep = millis();
  Serial.println(F("Server started. ArmSync ready."));
}

// ═══════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastStep >= STEP_MS) {
    float dt = (now - lastStep) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    lastStep = now;

    moveJoints(dt);   // always runs (smooth motion toward target)
    updateBase(now);  // base rotation timeout / detach
    autoLoop(now);    // auto state machine (only when Mode 2)
  }
}s