# 🏗️ System Architecture — HIL Servo Control Testbed

> All diagrams use Mermaid syntax and render natively on GitHub.
> Dark-theme colours chosen for visibility on both light and dark backgrounds.

---

## 1. System Architecture Diagram

```mermaid
graph TB
    subgraph HOST["🖥️  Host Computer"]
        DASH["Flask + SocketIO\nDashboard\n:5000"]
        DB[("SQLite\nhil_results.db")]
        CSV["📄 CSV Export"]
        DASH <-->|"REST + WebSocket"| DB
        DASH --> CSV
    end

    subgraph CTRL_MCU["⚙️  Controller MCU  (Arduino / ESP32)"]
        direction TB
        SER_RX["Serial RX\nParser"]
        CTRL_SEL{"Controller\nSelector"}
        ON_OFF["On/Off\nController"]
        P_CTRL["P\nController"]
        PI_CTRL["PI + Anti-Windup\nController"]
        PID_CTRL["PID\nDerivative Filter\nBack-Calc AW"]
        SERVO_DRV["Servo\nPWM Driver\nPin 9"]
        SER_RX --> CTRL_SEL
        CTRL_SEL --> ON_OFF & P_CTRL & PI_CTRL & PID_CTRL
        ON_OFF & P_CTRL & PI_CTRL & PID_CTRL --> SERVO_DRV
    end

    subgraph PLANT_MCU["🌿  Plant Simulator MCU  (ESP32)"]
        direction TB
        PKT_RX["CommProtocol\nPacket Receiver\nCRC-16"]
        PLANT_SEL{"Plant\nModel\nSelector"}
        FO["1st-Order\nEuler"]
        SO["2nd-Order\nRK4"]
        SV["Servo\nPosition"]
        TK["Tank\nLevel"]
        DC["DC Motor\nElec+Mech"]
        FI["Fault\nInjector\n7 fault types"]
        ME["Metrics\nEngine"]
        PKT_TX["CommProtocol\nPacket Sender"]
        PKT_RX --> PLANT_SEL & FI
        PLANT_SEL --> FO & SO & SV & TK & DC
        FO & SO & SV & TK & DC --> FI
        FI --> ME
        FI --> PKT_TX
        ME --> PKT_TX
    end

    DASH   <-->|"USB Serial\nCSV lines\n115200 baud"| CTRL_MCU
    CTRL_MCU <-->|"UART 115200\nCRC packets\n< 1 ms latency"| PLANT_MCU
    DASH   <-->|"USB Serial\n(direct for faults)"| PLANT_MCU

    style HOST       fill:#1a1a2e,stroke:#00d4ff,color:#e0e0e0
    style CTRL_MCU   fill:#16213e,stroke:#7B2FBE,color:#e0e0e0
    style PLANT_MCU  fill:#0f3460,stroke:#00ff88,color:#e0e0e0
```

---

## 2. Data Flow Diagram

```mermaid
flowchart LR
    SP(["Setpoint\nSP"])

    subgraph CL["Closed-Loop Control"]
        direction LR
        SUM(["∑\nError"])
        CTRL["Controller\nAlgorithm\nu = f(e)"]
        PLANT["Plant\nModel\ny = G·u"]
        FAULT["Fault\nInjector\nApply faults"]
        METRICS["Metrics\nEngine\nIAE/ITSE/…"]
        SUM -->|"e = SP−PV"| CTRL
        CTRL -->|"CV (0–100%)"| PLANT
        PLANT -->|"y raw"| FAULT
        FAULT -->|"y faulted"| METRICS
    end

    SP --> SUM
    FAULT -->|"PV"| SUM
    METRICS -->|"Snapshot\nevery 100 ms"| PKT["UART\nPacket\nPKT_SENSOR_DATA"]
    PKT -->|"Serial\n115200 baud"| DASH["Dashboard\nBuffer\n2000 samples"]
    DASH -->|"WebSocket\n10 Hz"| BROWSER["Browser\nChart.js\nReal-time Plot"]
    DASH -->|"REST /api/metrics"| SQLITE[("SQLite")]
    DASH -->|"GET /api/export/csv"| CSVFILE["📄 CSV"]

    style CL fill:#1a1a2e,stroke:#00d4ff,color:#e0e0e0
```

---

## 3. UART Packet Flow Diagram

```mermaid
sequenceDiagram
    participant H  as 🖥️ Dashboard (Host)
    participant CM as ⚙️ Controller MCU
    participant PM as 🌿 Plant Simulator MCU

    Note over H,PM: System Startup
    H  ->>+ CM : kp:2.5\n / ki:0.8\n / sp:50\n  (ASCII serial cmd)
    CM -->> H  : [STATUS] gains updated

    H  ->>  PM : PKT_SET_PLANT (0x03)  model=SERVO
    PM -->> H  : PKT_ACK (0x0A)

    Note over CM,PM: Real-time control loop @ 1 kHz
    loop Every 1 ms
        CM ->>+ PM : PKT_CONTROL_OUTPUT (0x01)\n controlValue, setpoint, timestamp
        PM -->> PM : Integrate plant model (Euler / RK4)
        PM -->> PM : Apply fault injector
        PM -->> PM : Update MetricsEngine
        PM ->>- CM : PKT_SENSOR_DATA (0x02)\n plantOutput, metrics snapshot
        CM -->> CM : Run controller algorithm\n write servo PWM
    end

    Note over H,PM: Fault Injection
    H  ->>  PM : PKT_FAULT_INJECT (0x04)  type=NOISE, mag=3.0
    PM -->> H  : PKT_ACK
    H  ->>  PM : PKT_FAULT_CLEAR (0x05)
    PM -->> H  : PKT_ACK

    Note over CM,H: Metrics retrieval (every 5 s)
    CM ->>+ PM : PKT_GET_METRICS (0x07)
    PM ->>- CM : PKT_METRICS_REPORT (0x08)\n riseTime, settlingTime,\n overshoot, IAE, ITSE

    Note over H,CM: Dashboard reads Controller MCU via USB
    CM ->>  H  : CSV line: t,sp,pv,cv,error  (every 10 ms)
    H  -->> H  : Buffer → SocketIO → Browser
```

---

## 4. Plant Simulator State Machine

```mermaid
stateDiagram-v2
    [*] --> INIT : Power-on / reset

    INIT : INIT\nLoad default plant model\nInit CommProtocol\nStart 1 kHz ISR timer

    INIT --> IDLE : Setup complete

    IDLE : IDLE\nHeartbeat TX every 1 s\nAwait PKT from controller

    IDLE --> RUNNING : PKT_CONTROL_OUTPUT received

    RUNNING : RUNNING\n━━━━━━━━━━━━━━━\n1. Receive CV from controller\n2. Apply fault injector\n3. Integrate plant model (Euler/RK4)\n4. Update MetricsEngine\n5. Transmit PKT_SENSOR_DATA

    RUNNING --> FAULT : PKT_FAULT_INJECT received
    RUNNING --> IDLE  : PKT_RESET received
    RUNNING --> MODEL_SWITCH : PKT_SET_PLANT received

    FAULT : FAULT\nFault active on signal path\nAll other operations continue

    FAULT --> RUNNING : PKT_FAULT_CLEAR received
    FAULT --> IDLE    : PKT_RESET received

    MODEL_SWITCH : MODEL_SWITCH\nReset plant state\nSwitch active model\nReset MetricsEngine

    MODEL_SWITCH --> RUNNING : Transition complete

    CALIBRATE : CALIBRATE\nFull system reset\nRe-init all models\nZero MetricsEngine

    RUNNING --> CALIBRATE : PKT_RESET + re-init flag
    FAULT    --> CALIBRATE : PKT_RESET + re-init flag
    CALIBRATE --> IDLE : Complete
```

---

## 5. Controller MCU State Machine

```mermaid
stateDiagram-v2
    [*] --> BOOT : Power-on

    BOOT : BOOT\nInit UART Serial\nInit Servo on Pin 9\nParse default gains from EEPROM

    BOOT --> WAITING : Setup complete

    WAITING : WAITING\nListen on USB for commands\nListen on UART for plant data\nServo held at last position

    WAITING --> CONTROLLING : Valid PKT_SENSOR_DATA received

    CONTROLLING : CONTROLLING  ── 1 kHz loop ──\n━━━━━━━━━━━━━━━━━━━━━━━━\n1. Read PV from latest PKT_SENSOR_DATA\n2. Compute controller output u\n3. Clamp u ∈ [0, 100 %]\n4. Write servo PWM (0 – 180°)\n5. Transmit PKT_CONTROL_OUTPUT\n6. Print CSV to USB: t,sp,pv,cv,err

    CONTROLLING --> TUNING : Serial cmd kp/ki/kd/sp received

    TUNING : TUNING\nUpdate gains at runtime\nReset integrators if SP changed\nAck to dashboard

    TUNING --> CONTROLLING : Gains applied

    CONTROLLING --> WAITING : UART timeout > 500 ms

    CTRL_SWITCH : CTRL_SWITCH\nReset all controller state\nSelect new algorithm

    CONTROLLING --> CTRL_SWITCH : ctrl:N command received
    CTRL_SWITCH --> CONTROLLING : Transition complete
```

---

## 6. Dashboard Data Pipeline Diagram

```mermaid
flowchart TD
    MCU["⚙️ Controller MCU\nUSB → /dev/ttyUSBx\n115200 baud"]

    subgraph BACKEND["🐍 Flask + SocketIO Backend  (app.py)"]
        direction TB
        SRT["serial_reader()\ndaemon thread"]
        BUF["Circular Buffers\ndeque(maxlen=2000)\nt, sp, pv, cv, err"]
        SIO_EMIT["SocketIO emit()\nevery 10 samples\n→ 10 Hz to browser"]
        DB_SAVE["SQLite INSERT\nper run metrics"]
        CSV_EXP["CSV BytesIO\n/api/export/csv"]
        REST["REST API\n/api/connect\n/api/tune\n/api/snapshot\n/api/history\n/api/metrics"]
        SRT --> BUF
        BUF --> SIO_EMIT & CSV_EXP
        REST --> DB_SAVE
        REST --> BUF
    end

    subgraph FRONTEND["🌐 Browser  (index.html + Chart.js)"]
        direction TB
        SIO_CLIENT["socket.io client\non('data')"]
        CHARTS["Chart.js\nReal-time rolling plot\nSP / PV / CV / Error"]
        METRIC_CARDS["Metric Cards\nRise Time · OS · SSE\nIAE · ITSE · Effort"]
        PID_PANEL["PID Tuner Panel\nKp / Ki / Kd sliders"]
        CTRL_SEL["Controller Selector\nOn/Off · P · PI · PID"]
        PLANT_SEL["Plant Selector\n5 plant models"]
        FAULT_PANEL["Fault Injector Panel\n7 fault types"]
        SIO_CLIENT --> CHARTS & METRIC_CARDS
        PID_PANEL --> REST
        CTRL_SEL & PLANT_SEL & FAULT_PANEL --> REST
    end

    MCU       -->|"ASCII CSV lines\nt,sp,pv,cv,err"| SRT
    SIO_EMIT  -->|"WebSocket"| SIO_CLIENT
    REST      <-->|"HTTP"| PID_PANEL

    style BACKEND  fill:#1a1a2e,stroke:#00d4ff,color:#e0e0e0
    style FRONTEND fill:#16213e,stroke:#7B2FBE,color:#e0e0e0
```

---

## 7. Fault Injection Architecture Diagram

```mermaid
flowchart LR
    subgraph HOST["Host / Dashboard"]
        UI["Fault Injector\nPanel"]
        API["/api/command\nPKT_FAULT_INJECT"]
        UI --> API
    end

    subgraph PLANT["Plant Simulator ESP32"]
        direction TB
        PKT_IN["CommProtocol\nReceiver"]
        DISPATCH{"Fault\nDispatch"}
        NONE["FAULT_NONE\nPassthrough"]
        NOISE["FAULT_NOISE\nGaussian\nBox-Muller"]
        BIAS["FAULT_BIAS\nConstant\nOffset"]
        SAT["FAULT_SATURATE\nActuator\nClamping"]
        DROP["FAULT_DROPOUT\nSignal → 0"]
        STUCK["FAULT_STUCK\nLast-value\nFreeze"]
        DIST["FAULT_DISTURBANCE\nRandom step\npulses"]
        DELAY["FAULT_DELAY\nTransport delay\nbuffer 1–49 ms"]
        PKT_IN --> DISPATCH
        DISPATCH --> NONE & NOISE & BIAS & SAT & DROP & STUCK & DIST & DELAY
        MOUT(["y_faulted"])
        NONE & NOISE & BIAS & SAT & DROP & STUCK & DIST & DELAY --> MOUT
    end

    API       -->|"PKT_FAULT_INJECT\ntype + magnitude"| PKT_IN
    PLANT_IN(["Plant\nOutput y"]) --> DISPATCH
    MOUT      --> MET["MetricsEngine\nupdate(sp,pv,u)"]
    MET       --> FB(["Feedback\nto Controller"])

    style HOST  fill:#1a1a2e,stroke:#ff6b6b,color:#e0e0e0
    style PLANT fill:#0f3460,stroke:#00ff88,color:#e0e0e0
```

---

## Component Dependency Map

```mermaid
graph LR
    subgraph FIRMWARE["Firmware (Arduino/C++)"]
        A["controller.ino"] --> B["Controllers.h"]
        A --> C["CommProtocol.h"]
        D["plant_simulator.ino"] --> E["PlantModels.h"]
        D --> C
        D --> F["FaultInjector.h"]
        F -.->|"contains"| G["MetricsEngine\n(same file)"]
    end
    subgraph PYTHON["Python (Dashboard + Tools)"]
        H["app.py"] --> I["Flask"]
        H --> J["flask_socketio"]
        H --> K["pyserial"]
        H --> L["sqlite3"]
        M["benchmark.py"] --> N["numpy"]
        M --> O["MetricsEngine\n(Python port)"]
        P["test_controllers.py"] --> Q["pytest"]
        P --> N
        P --> R["scipy"]
    end
    A -.->|"UART packets"| D
    H -.->|"USB serial"| A
    H -.->|"USB serial (faults)"| D

    style FIRMWARE fill:#16213e,stroke:#7B2FBE,color:#e0e0e0
    style PYTHON   fill:#1a1a2e,stroke:#00d4ff,color:#e0e0e0
```

---

*Diagrams generated for the HIL Servo Control Testbed project — paste any block directly into a GitHub README or `ARCHITECTURE.md`.*
