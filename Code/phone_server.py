from flask import Flask, Response, jsonify, request
import json
import time
import os

app = Flask(__name__)

MISSION_STATE_PATH = "/tmp/mission_state.json"
TRASH_DETECTION_PATH = "/tmp/trash_detection.json"
FRONT_DETECTION_PATH = "/tmp/front_detection.json"
FRONT_FRAME_PATH = "/tmp/front_frame.jpg"
BOTTOM_FRAME_PATH = "/tmp/bottom_frame.jpg"
MANUAL_CONTROL_PATH = "/tmp/manual_control.json"
EMERGENCY_COMMAND_PATH = "/tmp/emergency_command.json"


def read_json(path):
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def mjpeg_stream(frame_path):
    while True:
        if os.path.exists(frame_path):
            with open(frame_path, "rb") as f:
                frame_bytes = f.read()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
        time.sleep(0.1)


@app.route("/stream/front")
def stream_front():
    return Response(mjpeg_stream(FRONT_FRAME_PATH), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream/bottom")
def stream_bottom():
    return Response(mjpeg_stream(BOTTOM_FRAME_PATH), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/status")
def status():
    return jsonify({
        "mission": read_json(MISSION_STATE_PATH),
        "trash": read_json(TRASH_DETECTION_PATH),
        "front": read_json(FRONT_DETECTION_PATH)
    })


@app.route("/manual_control", methods=["POST"])
def manual_control():
    command = request.get_json()
    command["timestamp"] = time.time()
    with open(MANUAL_CONTROL_PATH, "w") as f:
        json.dump(command, f)
    return jsonify({"ok": True})


@app.route("/emergency_rtl", methods=["POST"])
def emergency_rtl():
    with open(EMERGENCY_COMMAND_PATH, "w") as f:
        json.dump({"command": "rtl", "timestamp": time.time()}, f)
    return jsonify({"ok": True})


INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
<title>Drone Monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<style>
:root {
  --bg: #0a0a0b;
  --surface: #151517;
  --surface-2: #1c1c1f;
  --border: #262629;
  --text: #e8e8ea;
  --text-dim: #8a8a90;
  --accent: #6fd3c7;
  --accent-dim: #3a6e68;
  --danger: #d16565;
}
* { box-sizing: border-box; -webkit-user-select: none; user-select: none; }
body {
  font-family: -apple-system, system-ui, sans-serif;
  background: var(--bg);
  color: var(--text);
  margin: 0;
  padding: 16px;
  max-width: 480px;
  margin-inline: auto;
}
h1 {
  font-size: 15px;
  font-weight: 600;
  letter-spacing: 0.02em;
  color: var(--text-dim);
  text-transform: uppercase;
  margin: 0 0 12px 2px;
}

.streams { display: flex; gap: 8px; margin-bottom: 12px; }
.streams img {
  width: 50%;
  border-radius: 10px;
  border: 1px solid var(--border);
  background: var(--surface);
  display: block;
}

.card {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 14px;
  margin-bottom: 12px;
}
.card-label {
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--text-dim);
  margin-bottom: 8px;
}
.stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.stat { }
.stat-value { font-size: 17px; font-weight: 600; }
.stat-key { font-size: 11px; color: var(--text-dim); margin-top: 2px; }

.flags { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 10px; }
.flag {
  font-size: 11px;
  padding: 4px 9px;
  border-radius: 20px;
  background: var(--surface-2);
  color: var(--text-dim);
  border: 1px solid var(--border);
}
.flag.on { background: var(--accent-dim); color: var(--accent); border-color: var(--accent-dim); }

.mode-row {
  display: flex;
  background: var(--surface-2);
  border-radius: 10px;
  padding: 4px;
  margin-bottom: 12px;
  border: 1px solid var(--border);
}
.mode-btn {
  flex: 1;
  padding: 10px;
  font-size: 13px;
  font-weight: 600;
  letter-spacing: 0.03em;
  border: none;
  background: transparent;
  color: var(--text-dim);
  border-radius: 8px;
}
.mode-btn.active { background: var(--accent-dim); color: var(--accent); }

.rtl-btn {
  width: 100%;
  padding: 14px;
  font-size: 14px;
  font-weight: 600;
  letter-spacing: 0.03em;
  background: transparent;
  color: var(--danger);
  border: 1px solid var(--danger);
  border-radius: 10px;
  margin-bottom: 20px;
}

#joystick-zone {
  position: relative;
  width: 200px;
  height: 200px;
  margin: 8px auto 20px;
  background: var(--surface);
  border-radius: 50%;
  border: 1px solid var(--border);
  touch-action: none;
}
#joystick-stick {
  position: absolute;
  width: 64px;
  height: 64px;
  border-radius: 50%;
  background: var(--accent-dim);
  border: 1px solid var(--accent);
  top: 68px;
  left: 68px;
}
</style>
</head>
<body>

<h1>Drone Monitor</h1>

<div class="streams">
  <img src="/stream/front">
  <img src="/stream/bottom">
</div>

<div class="card">
  <div class="card-label">Status</div>
  <div class="stat-grid" id="statGrid">
    <div class="stat"><div class="stat-value" id="statPos">—</div><div class="stat-key">position</div></div>
    <div class="stat"><div class="stat-value" id="statHeading">—</div><div class="stat-key">heading</div></div>
    <div class="stat"><div class="stat-value" id="statCoverage">—</div><div class="stat-key">visited cells</div></div>
    <div class="stat"><div class="stat-value" id="statTrash">—</div><div class="stat-key">trash seen</div></div>
  </div>
  <div class="flags" id="obstacleFlags"></div>
</div>

<div class="mode-row">
  <button class="mode-btn active" id="autoBtn" onclick="setMode('auto')">AUTO</button>
  <button class="mode-btn" id="manualBtn" onclick="setMode('manual')">MANUAL</button>
</div>

<button class="rtl-btn" onclick="emergencyRTL()">Return to Launch</button>

<div id="joystick-zone">
  <div id="joystick-stick"></div>
</div>

<script>
let currentMode = "auto";
let joystickActive = false;
let joystickInterval = null;

function setMode(mode) {
    currentMode = mode;
    document.getElementById("autoBtn").classList.toggle("active", mode === "auto");
    document.getElementById("manualBtn").classList.toggle("active", mode === "manual");
}

function emergencyRTL() {
    if (!confirm("Return to launch now?")) return;
    fetch("/emergency_rtl", { method: "POST" });
}

function sendManualCommand(dx, dy) {
    fetch("/manual_control", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: "manual", dx: dx, dy: dy})
    });
}

const zone = document.getElementById("joystick-zone");
const stick = document.getElementById("joystick-stick");
const zoneRadius = 100;
const stickRadius = 32;
let currentDx = 0, currentDy = 0;

function updateStickPosition(clientX, clientY) {
    const rect = zone.getBoundingClientRect();
    const centerX = rect.left + zoneRadius;
    const centerY = rect.top + zoneRadius;

    let offsetX = clientX - centerX;
    let offsetY = clientY - centerY;
    const distance = Math.hypot(offsetX, offsetY);
    const maxDistance = zoneRadius - stickRadius;

    if (distance > maxDistance) {
        offsetX = (offsetX / distance) * maxDistance;
        offsetY = (offsetY / distance) * maxDistance;
    }

    stick.style.left = (zoneRadius - stickRadius + offsetX) + "px";
    stick.style.top = (zoneRadius - stickRadius + offsetY) + "px";

    currentDx = offsetX / maxDistance;
    currentDy = -offsetY / maxDistance;
}

function resetStick() {
    stick.style.left = "68px";
    stick.style.top = "68px";
    currentDx = 0;
    currentDy = 0;
}

function startJoystick(x, y) {
    if (currentMode !== "manual") return;
    joystickActive = true;
    updateStickPosition(x, y);
    if (joystickInterval) clearInterval(joystickInterval);
    joystickInterval = setInterval(() => {
        if (joystickActive) sendManualCommand(currentDx, currentDy);
    }, 150);
}

function moveJoystick(x, y) {
    if (!joystickActive) return;
    updateStickPosition(x, y);
}

function endJoystick() {
    joystickActive = false;
    if (joystickInterval) clearInterval(joystickInterval);
    resetStick();
}

zone.addEventListener("touchstart", e => { e.preventDefault(); startJoystick(e.touches[0].clientX, e.touches[0].clientY); });
zone.addEventListener("touchmove", e => { e.preventDefault(); moveJoystick(e.touches[0].clientX, e.touches[0].clientY); });
zone.addEventListener("touchend", e => { e.preventDefault(); endJoystick(); });

zone.addEventListener("mousedown", e => startJoystick(e.clientX, e.clientY));
window.addEventListener("mousemove", e => moveJoystick(e.clientX, e.clientY));
window.addEventListener("mouseup", endJoystick);

function renderFlags(nearby) {
    const container = document.getElementById("obstacleFlags");
    container.innerHTML = "";
    if (!nearby) return;
    for (const [direction, isNear] of Object.entries(nearby)) {
        const el = document.createElement("div");
        el.className = "flag" + (isNear ? " on" : "");
        el.textContent = direction;
        container.appendChild(el);
    }
}

setInterval(() => {
    fetch("/status").then(r => r.json()).then(data => {
        const mission = data.mission;
        if (mission && mission.pose) {
            document.getElementById("statPos").textContent =
                mission.pose.x.toFixed(1) + ", " + mission.pose.y.toFixed(1) + " m";
            document.getElementById("statHeading").textContent =
                (mission.pose.theta * 180 / Math.PI).toFixed(0) + "°";
        }
        if (mission && mission.coverage) {
            document.getElementById("statCoverage").textContent = mission.coverage.visited_cells;
        }
        if (data.trash && data.trash.detections) {
            document.getElementById("statTrash").textContent = data.trash.detections.length;
        }
        if (mission) renderFlags(mission.nearby);
    }).catch(() => {});
}, 1000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return INDEX_HTML


if __name__ == "__main__":
    # threaded=True matters here — without it, Flask's dev server is
    # single-threaded, and the MJPEG stream routes (which loop forever)
    # would block every other request, including /status and manual control.
    app.run(host="0.0.0.0", port=5000, threaded=True)