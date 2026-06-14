#!/usr/bin/env python3
"""
HIL Servo Control Testbed — Real-Time Dashboard Backend
════════════════════════════════════════════════════════
Flask + SocketIO server providing:
  - Real-time serial data ingestion from Controller MCU
  - Live plot data streaming via WebSocket
  - PID tuning API
  - Performance metrics storage (SQLite)
  - CSV data export

Author: HIL Testbed Project
License: MIT
"""

import os, sys, csv, json, time, struct, threading, io
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports
from flask import Flask, render_template, request, jsonify, send_file
from flask_socketio import SocketIO, emit
import sqlite3

# ─── Configuration ─────────────────────────────────────────────────────────────
APP_HOST    = "0.0.0.0"
APP_PORT    = 5000
SERIAL_PORT = os.environ.get("HIL_PORT", "COM3")  # or "/dev/ttyUSB0"
SERIAL_BAUD = 115200
BUFFER_LEN  = 2000   # samples to keep in memory

# ─── Flask + SocketIO ─────────────────────────────────────────────────────────
app = Flask(__name__, template_folder="templates", static_folder="static")
app.config["SECRET_KEY"] = "hil-testbed-secret"
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ─── Data Store ───────────────────────────────────────────────────────────────
data_lock = threading.Lock()
time_buf   = deque(maxlen=BUFFER_LEN)
sp_buf     = deque(maxlen=BUFFER_LEN)
pv_buf     = deque(maxlen=BUFFER_LEN)
cv_buf     = deque(maxlen=BUFFER_LEN)
err_buf    = deque(maxlen=BUFFER_LEN)

metrics_store = {}
run_id        = 0
ser           = None
serial_thread = None
running       = False

# ─── SQLite Setup ─────────────────────────────────────────────────────────────
DB_PATH = "hil_results.db"

def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute("""
        CREATE TABLE IF NOT EXISTS runs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            controller TEXT,
            plant TEXT,
            kp REAL, ki REAL, kd REAL,
            setpoint REAL,
            rise_time REAL,
            settling_time REAL,
            overshoot REAL,
            ss_error REAL,
            iae REAL,
            control_effort REAL
        )
    """)
    c.execute("""
        CREATE TABLE IF NOT EXISTS samples (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            run_id INTEGER,
            t REAL, sp REAL, pv REAL, cv REAL, error REAL
        )
    """)
    conn.commit()
    conn.close()

init_db()

# ─── Serial Reader Thread ─────────────────────────────────────────────────────
def serial_reader():
    """Parse CSV lines from Controller MCU: time,sp,pv,cv,error"""
    global ser, running, run_id
    while running:
        try:
            if ser and ser.is_open:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line or line.startswith("["):
                    continue
                parts = line.split(",")
                if len(parts) < 4:
                    continue
                t_val  = float(parts[0])
                sp_val = float(parts[1])
                pv_val = float(parts[2])
                cv_val = float(parts[3])
                err_val= float(parts[4]) if len(parts) > 4 else sp_val - pv_val

                with data_lock:
                    time_buf.append(t_val)
                    sp_buf.append(sp_val)
                    pv_buf.append(pv_val)
                    cv_buf.append(cv_val)
                    err_buf.append(err_val)

                # Emit every 10 samples (100Hz → 10Hz to browser)
                if len(time_buf) % 10 == 0:
                    sio.emit("data", {
                        "t":   t_val,
                        "sp":  round(sp_val,  4),
                        "pv":  round(pv_val,  4),
                        "cv":  round(cv_val,  4),
                        "err": round(err_val, 4),
                    })
        except Exception as e:
            sio.emit("error", {"msg": str(e)})
            time.sleep(0.5)

# ─── Routes ───────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return render_template("index.html", ports=ports)

@app.route("/api/connect", methods=["POST"])
def connect():
    global ser, serial_thread, running
    data = request.json
    port = data.get("port", SERIAL_PORT)
    baud = int(data.get("baud", SERIAL_BAUD))
    try:
        if ser and ser.is_open:
            ser.close()
        ser = serial.Serial(port, baud, timeout=1)
        running = True
        serial_thread = threading.Thread(target=serial_reader, daemon=True)
        serial_thread.start()
        return jsonify({"status": "ok", "port": port, "baud": baud})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route("/api/disconnect", methods=["POST"])
def disconnect():
    global running, ser
    running = False
    if ser and ser.is_open:
        ser.close()
    return jsonify({"status": "ok"})

@app.route("/api/tune", methods=["POST"])
def tune_pid():
    """Send PID gains to MCU via serial"""
    d = request.json
    if not ser or not ser.is_open:
        return jsonify({"status": "error", "msg": "Not connected"}), 400
    try:
        cmds = []
        if "kp" in d: cmds.append(f"kp:{d['kp']}\n")
        if "ki" in d: cmds.append(f"ki:{d['ki']}\n")
        if "kd" in d: cmds.append(f"kd:{d['kd']}\n")
        if "sp" in d: cmds.append(f"sp:{d['sp']}\n")
        if "ctrl" in d: cmds.append(f"ctrl:{d['ctrl']}\n")
        for cmd in cmds:
            ser.write(cmd.encode())
            time.sleep(0.05)
        return jsonify({"status": "ok", "commands": cmds})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 500

@app.route("/api/command", methods=["POST"])
def send_command():
    d = request.json
    cmd = d.get("cmd", "")
    if not ser or not ser.is_open:
        return jsonify({"status": "error", "msg": "Not connected"}), 400
    ser.write((cmd + "\n").encode())
    return jsonify({"status": "ok"})

@app.route("/api/snapshot")
def get_snapshot():
    with data_lock:
        return jsonify({
            "t":   list(time_buf)[-200:],
            "sp":  list(sp_buf)[-200:],
            "pv":  list(pv_buf)[-200:],
            "cv":  list(cv_buf)[-200:],
            "err": list(err_buf)[-200:],
        })

@app.route("/api/metrics", methods=["GET", "POST"])
def metrics_api():
    global metrics_store
    if request.method == "POST":
        metrics_store = request.json
        _save_run_to_db(metrics_store)
        return jsonify({"status": "saved"})
    return jsonify(metrics_store)

@app.route("/api/history")
def history():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT * FROM runs ORDER BY timestamp DESC LIMIT 50"
    ).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])

@app.route("/api/export/csv")
def export_csv():
    with data_lock:
        rows = list(zip(time_buf, sp_buf, pv_buf, cv_buf, err_buf))
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["time_s", "setpoint", "process_variable", "control_output", "error"])
    w.writerows(rows)
    buf.seek(0)
    return send_file(
        io.BytesIO(buf.getvalue().encode()),
        mimetype="text/csv",
        as_attachment=True,
        download_name=f"hil_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

@app.route("/api/ports")
def list_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return jsonify(ports)

# ─── SocketIO Events ─────────────────────────────────────────────────────────
@sio.on("connect")
def on_connect():
    emit("status", {"connected": ser is not None and ser.is_open if ser else False})

@sio.on("ping_server")
def on_ping():
    emit("pong")

# ─── DB Helper ────────────────────────────────────────────────────────────────
def _save_run_to_db(m):
    conn = sqlite3.connect(DB_PATH)
    conn.execute("""
        INSERT INTO runs
        (timestamp, controller, plant, kp, ki, kd, setpoint,
         rise_time, settling_time, overshoot, ss_error, iae, control_effort)
        VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)
    """, (
        datetime.now().isoformat(),
        m.get("controller", "PID"),
        m.get("plant", "SERVO"),
        m.get("kp", 0), m.get("ki", 0), m.get("kd", 0),
        m.get("setpoint", 0),
        m.get("riseTime", 0), m.get("settlingTime", 0),
        m.get("overshoot", 0), m.get("ssError", 0),
        m.get("iae", 0), m.get("controlEffort", 0),
    ))
    conn.commit()
    conn.close()

# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("╔══════════════════════════════════════════╗")
    print("║   HIL Testbed Dashboard  v2.0            ║")
    print(f"║   Open → http://localhost:{APP_PORT}          ║")
    print("╚══════════════════════════════════════════╝")
    sio.run(app, host=APP_HOST, port=APP_PORT, debug=False)
