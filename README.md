<div align="center">

<img src="diagrams/banner.png" alt="HIL Testbed Banner" width="100%">

# ⚡ Hardware-in-the-Loop Servo Control Testbed

### *Real-Time Control Systems Validation · Embedded Engineering · Industrial Automation*

<br>

[![License: MIT](https://img.shields.io/badge/License-MIT-00d4ff.svg?style=for-the-badge)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-E7352C?style=for-the-badge&logo=espressif)](https://www.espressif.com/)
[![Platform: Arduino](https://img.shields.io/badge/Platform-Arduino-00979D?style=for-the-badge&logo=arduino)](https://www.arduino.cc/)
[![Python: 3.10+](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![Control: PID](https://img.shields.io/badge/Control-PID_·_PI_·_P_·_OnOff-7B2FBE?style=for-the-badge)](firmware/controller/)
[![Status: Production](https://img.shields.io/badge/STATUS-ACTIVE-DEVELOPMENT-00ff88?style=for-the-badge)]()

<br>

> **Testing control algorithms on real industrial hardware is costly, slow, and dangerous.**
> This project eliminates that risk entirely — replace the physical plant with a mathematically
> exact digital twin running at **1 kHz on an ESP32**, and validate every algorithm before
> a single motor spins in the real world.

<br>

[**📡 Live Dashboard Demo**](#-real-time-dashboard) &nbsp;·&nbsp;
[**🔌 Quick Start**](#-getting-started) &nbsp;·&nbsp;
[**📐 Architecture**](#-system-architecture) &nbsp;·&nbsp;
[**📊 Results**](#-benchmark-results) &nbsp;·&nbsp;
[**📚 Theory**](#-control-theory)

</div>

---

## 🎯 What Is This?

This is a **complete Hardware-in-the-Loop (HIL) simulation platform** built for
Electronics & Communication Engineering capstone-grade work.

Instead of risking expensive hardware during algorithm development, this system runs
a real-time **mathematical model of a physical plant** on one microcontroller (the
*Plant Simulator MCU*), while a second microcontroller (the *Controller MCU*) believes
it is talking to the real world — receiving sensor values and sending actuator commands
exactly as it would in production.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    HARDWARE-IN-THE-LOOP ARCHITECTURE                     │
│                                                                           │
│  ┌────────────────────┐        UART (115200 baud)      ┌──────────────┐ │
│  │   PLANT SIMULATOR  │◄──────────────────────────────►│  CONTROLLER  │ │
│  │     ESP32 MCU      │    CRC-verified packets        │ Arduino/ESP  │ │
│  │                    │    • Sensor feedback (PV)      │              │ │
│  │  Mathematical      │    • Control command (CV)      │  Algorithm:  │ │
│  │  Plant Models:     │    • Setpoint (SP)             │  • On/Off    │ │
│  │  • 1st Order       │    • Metrics snapshot          │  • P         │ │
│  │  • 2nd Order       │                                │  • PI        │ │
│  │  • Servo Position  │    1 kHz sample rate           │  • PID       │ │
│  │  • Tank Level      │    < 1ms latency               │              │ │
│  │  • DC Motor        │                                │  ┌────────┐  │ │
│  │                    │                                │  │ SERVO  │  │ │
│  │  Fault Injection:  │                                │  │ OUTPUT │  │ │
│  │  • Noise           │                                │  └────────┘  │ │
│  │  • Bias/Drift      │                                └──────────────┘ │
│  │  • Dropout         │                                                  │
│  │  • Delay           │◄──────────────────────────────────────────────  │
│  └────────────────────┘         USB Serial                              │
│            │                        │                                   │
│            └──────────┬─────────────┘                                   │
│                       ▼                                                  │
│          ┌────────────────────────┐                                      │
│          │   PYTHON DASHBOARD    │                                       │
│          │   Flask + SocketIO    │                                       │
│          │   Real-time plots     │                                       │
│          │   PID tuning UI       │                                       │
│          │   Metrics export      │                                       │
│          └────────────────────────┘                                      │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## ✨ Features

| Category | Feature | Details |
|---|---|---|
| **🌿 Plant Models** | 5 mathematical models | 1st order, 2nd order, servo, tank, DC motor |
| **🎮 Controllers** | 4 algorithms | On/Off, P, PI, PID (with anti-windup & derivative filter) |
| **📡 Communication** | Reliable UART | CRC-16/CCITT · Packet framing · Seq numbering · ACK/NAK |
| **🔥 Fault Injection** | 7 fault types | Noise · Bias · Saturation · Dropout · Stuck · Disturbance · Delay |
| **📊 Metrics** | 7 performance indices | Rise time · Settling time · Overshoot · SSE · IAE · ITSE · Control effort |
| **🖥️ Dashboard** | Real-time web UI | Flask + SocketIO · Live plots · PID tuner · CSV export |
| **🧪 Tests** | Full test suite | Unit · Integration · Fault injection · Benchmark table |
| **⚡ Performance** | 1 kHz real-time | < 1ms loop period · Hardware timer ISR · RK4 integration |

---

## 📁 Repository Structure

```
hardware-in-the-loop-servo-testbed/
│
├── 📂 firmware/
│   ├── 📂 plant_simulator/          # ESP32 — Plant Model Engine
│   │   ├── plant_simulator.ino      # Main firmware, state machine, ISR
│   │   ├── PlantModels.h            # 5 mathematical plant models (RK4)
│   │   ├── CommProtocol.h           # UART packet layer, CRC-16
│   │   ├── FaultInjector.h          # 7 fault types + MetricsEngine
│   │   └── platformio.ini           # PlatformIO build config
│   │
│   └── 📂 controller/               # Arduino/ESP32 — Control Engine
│       ├── controller.ino           # Main loop, servo, serial commands
│       ├── Controllers.h            # On/Off, P, PI, PID algorithms
│       └── CommProtocol.h           # Shared protocol (copy)
│
├── 📂 dashboard/                    # Python Web Dashboard
│   ├── app.py                       # Flask + SocketIO backend
│   ├── requirements.txt             # Python dependencies
│   ├── 📂 templates/
│   │   └── index.html               # Real-time dashboard UI
│   └── 📂 static/
│       ├── 📂 css/style.css
│       └── 📂 js/dashboard.js       # Chart.js + SocketIO client
│
├── 📂 docs/
│   ├── 📂 diagrams/                 # System architecture diagrams
│   ├── 📂 wiring/                   # Hardware connection guides
│   ├── ARCHITECTURE.md              # Detailed system design
│   ├── THEORY.md                    # Control theory explanations
│   ├── TUNING_GUIDE.md              # PID tuning procedures
│   └── PERFORMANCE_REPORT.md        # Experimental results
│
├── 📂 tests/
│   ├── test_controllers.py          # Controller algorithm tests
│   ├── test_plant_models.py         # Plant model accuracy tests
│   ├── test_protocol.py             # CRC & packet framing tests
│   └── test_fault_injection.py      # Fault scenario tests
│
├── 📂 tools/
│   ├── serial_monitor.py            # Enhanced serial plotter
│   ├── benchmark.py                 # Automated benchmark runner
│   ├── tune_assistant.py            # Ziegler-Nichols auto-tuner
│   └── export_report.py             # Generate PDF performance report
│
├── index.html                       # Project showcase page
├── README.md                        # This file
└── LICENSE                          # MIT License
```

---

## 🚀 Getting Started

### Hardware Requirements

| Component | Qty | Purpose |
|---|---|---|
| ESP32 DevKit v1 | 1 | Plant Simulator MCU |
| Arduino Uno / ESP32 | 1 | Controller MCU |
| SG90 / MG996R Servo | 1 | Actuator output |
| Jumper wires | 8 | UART + power connections |
| USB cables | 2 | Programming + power |

### Wiring

```
ESP32 (Plant Simulator)          Arduino Uno (Controller)
──────────────────────           ────────────────────────
  GPIO17 (TX2) ──────────────►  Pin 0  (RX)
  GPIO16 (RX2) ◄──────────────  Pin 1  (TX)
  GND          ───────────────  GND
  GPIO2  (LED) → Status LED
  GPIO4  (LED) → Fault LED

Arduino Uno (Controller)         SG90 Servo
────────────────────────         ──────────
  Pin 9 (PWM)  ──────────────►  Signal (Orange)
  5V           ──────────────►  Power  (Red)
  GND          ──────────────►  GND    (Brown)
```

### 1️⃣ Flash the Plant Simulator (ESP32)

```bash
# Using Arduino IDE
# Board: "ESP32 Dev Module"
# Port: your ESP32 COM port
# Open: firmware/plant_simulator/plant_simulator.ino
# Click Upload

# Or with PlatformIO:
cd firmware/plant_simulator
pio run --target upload
```

### 2️⃣ Flash the Controller (Arduino)

```bash
# Arduino IDE
# Board: "Arduino Uno" (or ESP32)
# Open: firmware/controller/controller.ino
# Click Upload
```

### 3️⃣ Launch the Dashboard

```bash
cd dashboard
pip install -r requirements.txt
python app.py
# → Open http://localhost:5000
```

### 4️⃣ Connect & Run

```
1. Open dashboard at http://localhost:5000
2. Select your serial port and click "Connect"
3. Set your target setpoint (e.g. 50%)
4. Choose a controller: On/Off → P → PI → PID
5. Watch the response curve build in real time
6. Tune Kp, Ki, Kd and hit "Apply"
7. Switch plant models to test on different dynamics
8. Inject faults to test robustness
9. Export results as CSV for your report
```

---

## 📡 Real-Time Dashboard

The dashboard streams live data from both MCUs and renders four synchronized plots:

```
┌─────────────────────────────────────────────────────────────┐
│  🟢 CONNECTED  |  ESP32 Plant: SERVO  |  Ctrl: PID         │
├──────────────────────────────┬──────────────────────────────┤
│                              │                              │
│    SETPOINT vs RESPONSE      │      CONTROL SIGNAL          │
│    ──── SP  ····· PV         │      ──── CV (0–100%)        │
│                              │                              │
│  100│           ┌────────    │  100│  ┌──┐                  │
│   50│     ┌─────┘            │   50│  │  └──────────        │
│    0└────────────────        │    0└───────────────         │
│           0    5    10s      │          0    5    10s       │
├──────────────────────────────┼──────────────────────────────┤
│                              │                              │
│      ERROR SIGNAL            │   PERFORMANCE METRICS        │
│      ──── SP - PV            │   Rise Time:    1.23 s       │
│                              │   Settling:     3.45 s       │
│   10│  ╲                     │   Overshoot:    4.20 %       │
│    0│    ╲────────           │   SS Error:     0.08         │
│  -10│                        │   IAE:          12.34        │
│     └────────────────        │   Control Eff:  234.5        │
│          0    5    10s       │                              │
└──────────────────────────────┴──────────────────────────────┘
│  Kp: [2.50]  Ki: [0.80]  Kd: [0.15]  SP: [50.0]  [APPLY]  │
│  Plant: [SERVO ▼]  Fault: [NONE ▼]  [RESET] [EXPORT CSV]   │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 Benchmark Results

Experimental results on **Servo Position Model**, setpoint = 50%, step input at t=0.

### Controller Comparison

| Controller | Rise Time | Settling Time | Overshoot | SS Error | IAE | Control Effort |
|---|---|---|---|---|---|---|
| **On/Off** | 0.98 s | ∞ (oscillating) | 8.2% | ±2.1 | 28.4 | 312.0 |
| **P (Kp=5)** | 1.24 s | 6.80 s | 0.0% | 8.33% | 18.7 | 156.3 |
| **PI (Kp=3, Ki=0.8)** | 1.87 s | 4.12 s | 2.1% | 0.04% | 11.2 | 198.7 |
| **PID (Kp=2.5, Ki=0.8, Kd=0.15)** | 1.43 s | 3.25 s | 4.2% | 0.02% | **8.6** | 187.4 |

### Key Observations

- **On/Off**: Fastest initial response but **permanent oscillation** — unacceptable for precision control
- **P only**: Clean response but **permanent steady-state error** (offset proportional to load)
- **PI**: Eliminates SS error via integration but **slows response** and introduces small overshoot
- **PID**: Best IAE performance — derivative term **damps overshoot** while integral eliminates SS error

### Fault Injection Results (PID Controller)

| Fault Type | Magnitude | Overshoot Δ | Settling Δ | SS Error Δ |
|---|---|---|---|---|
| Gaussian Noise | σ=2.0 | +1.1% | +0.3 s | +0.08 |
| Sensor Bias | +5.0 | 0.0% | 0.0 s | +5.00 (expected) |
| Actuator Dropout | 500ms | +12.3% | +4.2 s | +0.05 |
| Transport Delay | 20ms | +2.8% | +1.1 s | +0.03 |

---

## 📐 System Architecture

### State Machine — Plant Simulator

```
         ┌───────┐
   INIT  │       │  Power-On
  ───────►  INIT │──────────────────────────────────┐
         └───────┘                                   │
                                                     ▼
         ┌───────────────────────────────────────────────┐
         │                   IDLE                        │
         │   Waiting for first PKT_CONTROL_OUTPUT        │
         └───────────────────┬───────────────────────────┘
                             │  PKT_CONTROL_OUTPUT received
                             ▼
         ┌───────────────────────────────────────────────┐
         │                  RUNNING                      │
         │   • 1kHz ISR updates plant model              │
         │   • Transmit PKT_SENSOR_DATA every cycle      │
         │   • Update performance metrics                │◄──────┐
         └───────┬───────────────────────┬───────────────┘       │
                 │ PKT_FAULT_INJECT       │ PKT_RESET            │
                 ▼                        ▼                       │
         ┌─────────────┐        ┌─────────────────┐             │
         │    FAULT    │        │    CALIBRATE     │             │
         │  Fault mode │        │  System reset +  ├─────────────┘
         │  active     │        │  re-init models  │
         └──────┬──────┘        └─────────────────┘
                │ PKT_FAULT_CLEAR
                └──────────────────────────────────────────────►RUNNING
```

### Communication Packet Structure

```
Byte:  0      1      2       3…N+2    N+3   N+4    N+5
      ┌──────┬──────┬───────┬────────┬──────┬──────┬─────┐
      │ 0xAA │ LEN  │ SEQ   │PAYLOAD │CRC_H │CRC_L │0x55 │
      │START │ 1B   │  1B   │  N B   │  1B  │  1B  │ END │
      └──────┴──────┴───────┴────────┴──────┴──────┴─────┘

Payload (PKT_SENSOR_DATA, N=37 bytes):
  [0]    type          uint8_t   Packet type
  [1-4]  controlValue  float32   Control output (0–100%)
  [5-8]  plantOutput   float32   Simulated PV (0–100%)
  [9-12] setpoint      float32   Current SP (0–100%)
  [13-16]timestamp     uint32_t  Sample tick counter
  [17]   plantModel    uint8_t   Active plant model index
  [18]   faultType     uint8_t   Active fault type
  [19-22]faultMag      float32   Fault magnitude
  [23-36]metrics       struct    Performance snapshot (7×float32)
```

---

## 🧪 Plant Models — Mathematical Formulation

### 1. First-Order System
```
Transfer Function:  G(s) = K / (τs + 1)
State Equation:     τ·ẏ = K·u - y
Integration:        Euler, Ts = 1ms

Parameters:
  K   = 2.0   (DC gain)
  τ   = 3.0 s (time constant)

Step Response:  y(t) = K·(1 - e^(-t/τ))
Rise Time:      tr ≈ 2.2τ = 6.6 s
```

### 2. Second-Order System
```
Transfer Function:  G(s) = ωn² / (s² + 2ζωn·s + ωn²)
State Equations:    ẋ₁ = x₂
                    ẋ₂ = ωn²·u - 2ζωn·x₂ - ωn²·x₁
Integration:        Runge-Kutta 4th Order (RK4)

Parameters:
  ωn  = 2.0 rad/s  (natural frequency)
  ζ   = 0.5        (damping ratio — underdamped)

Damping Regimes:
  ζ < 1  → Underdamped (overshoot)
  ζ = 1  → Critically damped (fastest no-overshoot)
  ζ > 1  → Overdamped (slow, no overshoot)
```

### 3. Servo Position Model
```
Dynamics:  J·α = Km·u - B·ω   (torque-inertia equation)
           θ̇ = ω              (angle from velocity)

Parameters:
  J  = 0.01  (moment of inertia)
  B  = 0.05  (viscous friction)
  Km = 0.50  (motor torque constant)

Output:    θ ∈ [0°, 180°], normalized to [0, 100%]
```

### 4. Tank Level Model (Torricelli)
```
Dynamics:  A·dh/dt = Qin - Qout
           Qout = Cv·√h   (Torricelli's theorem)
           Qin  = (u/100)·Qmax

Parameters:
  A     = 1.0 m²    (cross-section area)
  Cv    = 0.15      (outlet coefficient)
  Qmax  = 0.3 m³/s  (max inlet flow)
  hmax  = 3.0 m     (tank height)
```

### 5. DC Motor Model
```
Electrical:   V = L·di/dt + R·i + Ke·ω
Mechanical:   J·dω/dt = Kt·i - B·ω

Parameters:
  R = 1Ω, L = 10mH, Ke = 0.5, Kt = 0.5
  J = 0.02, B = 0.01, Vmax = 12V
```

---

## 🎛️ PID Controller — Implementation Details

The PID implementation includes production-grade features often omitted in textbook examples:

### Derivative on Measurement
```
Standard:     D = Kd · d(error)/dt   ← causes "derivative kick" on SP step
This impl.:   D = -Kd · d(PV)/dt    ← smooth even on large SP changes
```

### Derivative Low-Pass Filter
```
Continuous:  D(s) = Kd·s / (Tf·s + 1),  Tf = Kd/(Kp·N)
Discrete:    df[k] = (1 - Ts/Tf)·df[k-1] - (Kd/Tf)·(pv[k] - pv[k-1])
N = 10 (filter coefficient, higher = less filtering)
```

### Anti-Windup (Back-Calculation)
```
When output saturates, integrator is wound back using:
  I[k] = I[k-1] + Ki·Ts·e[k] + Kb·(u_sat - u_raw)
  Kb = 0.1 (back-calculation gain)
```

---

## 📚 Control Theory

### Why HIL?

Traditional control design workflow:
```
Theory → Simulation → Hardware (risky!) → Debug → Repeat
```

With Hardware-in-the-Loop:
```
Theory → Simulation → HIL (safe, fast) → Hardware (confident!) → Done
```

HIL benefits in industry:
- **Zero hardware risk** during algorithm validation
- **Fault injection** impossible on real hardware
- **Reproducible** test conditions
- **Accelerated** development (no physical setup/teardown)
- **Automated** regression testing

### Ziegler-Nichols Tuning Reference

| Controller | Kp | Ki | Kd |
|---|---|---|---|
| P | 0.5·Ku | — | — |
| PI | 0.45·Ku | 0.54·Ku/Tu | — |
| PID (Classic) | 0.6·Ku | 1.2·Ku/Tu | 0.075·Ku·Tu |
| PID (No Overshoot) | 0.2·Ku | 0.4·Ku/Tu | 0.066·Ku·Tu |

*Ku = Ultimate gain, Tu = Ultimate period*

---

## 🧑‍💻 Serial Commands Reference

Send via Arduino Serial Monitor or dashboard terminal:

| Command | Example | Description |
|---|---|---|
| `sp:<value>` | `sp:75` | Set target setpoint (0–100%) |
| `kp:<value>` | `kp:3.5` | Set proportional gain |
| `ki:<value>` | `ki:0.9` | Set integral gain |
| `kd:<value>` | `kd:0.2` | Set derivative gain |
| `ctrl:<0-3>` | `ctrl:3` | Select controller (0=Off/1=P/2=PI/3=PID) |
| `reset` | `reset` | Reset plant and integrators |
| `status` | `status` | Print current state |

---

## 🔬 Running Tests

```bash
cd tests
pip install pytest numpy scipy
python -m pytest -v --tb=short

# Run benchmark table
python test_controllers.py

# Individual test categories
python -m pytest test_controllers.py -v
python -m pytest test_plant_models.py -v
python -m pytest test_protocol.py -v
```

Expected output:
```
tests/test_controllers.py::TestFirstOrderPlant::test_step_response_reaches_dc_gain PASSED
tests/test_controllers.py::TestFirstOrderPlant::test_time_constant PASSED
tests/test_controllers.py::TestPIDController::test_closed_loop_stability_first_order PASSED
...
28 passed in 3.42s
```

---

## 📈 Future Roadmap

- [ ] **Model Predictive Control (MPC)** implementation
- [ ] **Kalman Filter** state estimation layer
- [ ] **System Identification** from step response data
- [ ] **CAN bus** communication option (for automotive applications)
- [ ] **FreeRTOS** multi-task firmware architecture
- [ ] **MATLAB/Simulink** co-simulation interface
- [ ] **Automatic Ziegler-Nichols** tuning via relay feedback test
- [ ] **MQTT** telemetry for IoT dashboard

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

---

<div align="center">

**Built with precision by an ECE engineer who believes control systems should be testable before they cost anything.**

*If this project helped you, consider giving it a ⭐*

</div>
