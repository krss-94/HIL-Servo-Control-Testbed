# 🔌 Wiring Guide — HIL Servo Control Testbed

> **Safety first:** Always disconnect USB power before modifying wiring.
> Maximum current on Arduino 5 V pin: 500 mA. Use an external 5 V supply for the servo.

---

## Hardware Bill of Materials (BOM)

| # | Component | Specification | Qty | Role |
|---|---|---|---|---|
| 1 | **ESP32 DevKit v1** | Xtensa LX6, 240 MHz, 3.3 V I/O | 1 | Plant Simulator MCU |
| 2 | **Arduino Uno R3** | ATmega328P, 16 MHz, 5 V I/O | 1 | Controller MCU |
| 3 | **SG90 or MG996R Servo** | 180° travel, 4.8–6 V | 1 | Physical actuator output |
| 4 | **USB-A to Micro-USB cable** | — | 1 | ESP32 power + programming |
| 5 | **USB-A to USB-B cable** | — | 1 | Arduino Uno power + programming |
| 6 | **Breadboard** | 830-point | 1 | Signal routing |
| 7 | **Jumper wires M-M** | 20 cm | 6 | UART + GND connections |
| 8 | **Logic level shifter 5V↔3.3V** | TXS0102 or 4-channel | 1 | UART voltage translation |
| 9 | **External 5 V supply** | ≥ 1 A (2 A recommended) | 1 | Servo power (avoids USB noise) |
| 10 | **LED red 3 mm** | — | 1 | Status LED (ESP32 GPIO2) |
| 11 | **LED yellow 3 mm** | — | 1 | Fault LED (ESP32 GPIO4) |
| 12 | **Resistors 330 Ω** | 1/4 W | 2 | LED current-limiting |
| 13 | **Electrolytic cap 100 µF** | 6.3 V | 1 | Servo power decoupling |

**Estimated total cost:** USD 15–25 (educational components)

---

## Pin Mapping Tables

### ESP32 (Plant Simulator MCU)

| ESP32 Pin | Label | Direction | Connected To | Function |
|---|---|---|---|---|
| GPIO16 | RX2 | Input | Level shifter → Arduino TX | UART receive from Controller |
| GPIO17 | TX2 | Output | Level shifter → Arduino RX | UART transmit to Controller |
| GPIO2 | LED\_BLUE | Output | 330 Ω → LED → GND | Status LED (heartbeat blink) |
| GPIO4 | LED\_RED | Output | 330 Ω → LED → GND | Fault active indicator |
| GND | GND | — | Arduino GND, level shifter GND | Common ground reference |
| 3V3 | 3.3 V | Output | Level shifter LV rail | Logic supply to shifter |
| 5V (VIN) | 5 V | Input | USB or external | MCU power |

> **Note:** ESP32 GPIO is 3.3 V tolerant. Do NOT connect directly to Arduino 5 V UART without the level shifter.

### Arduino Uno (Controller MCU)

| Arduino Pin | Label | Direction | Connected To | Function |
|---|---|---|---|---|
| Pin 0 (RX) | RX | Input | Level shifter → ESP32 TX2 | UART receive from Plant Simulator |
| Pin 1 (TX) | TX | Output | Level shifter → ESP32 RX2 | UART transmit to Plant Simulator |
| Pin 9 | PWM | Output | Servo signal (orange wire) | Servo PWM control (50 Hz, 1–2 ms) |
| Pin 13 | LED\_BUILTIN | Output | On-board LED | Controller activity indicator |
| 5V | 5 V | Output | Level shifter HV rail | Level shifter high-voltage supply |
| GND | GND | — | ESP32 GND, servo GND | Common ground |

> **Important:** Arduino UART pins 0 and 1 are shared with the USB-serial bridge.
> Do NOT have the Plant Simulator connected to pins 0/1 while uploading firmware.
> Disconnect the UART TX/RX jumpers before clicking "Upload" in the IDE.

### SG90 / MG996R Servo

| Wire Colour | Connected To | Function |
|---|---|---|
| Orange (or white) | Arduino Pin 9 | PWM signal (3.3–5 V compatible) |
| Red | External 5 V supply (+) | Power supply |
| Brown (or black) | External 5 V supply (−) + Arduino GND | Ground |

---

## UART Connection Diagram

```
ESP32 (Plant Simulator)         LEVEL SHIFTER              Arduino Uno (Controller)
──────────────────────          ────────────               ─────────────────────────

   GPIO17 (TX2) ─────────► HV-A ──── LV-A ──────────────► Pin 0 (RX)

   GPIO16 (RX2) ◄───────── HV-B ──── LV-B ◄────────────── Pin 1 (TX)

   3V3 ──────────────────────────── LV VCC
                                    HV VCC ─────────────── 5V
   GND ─────────────────────────── GND (shared with Arduino GND)


Baud rate: 115 200 bps
Format: 8N1 (8 data bits, no parity, 1 stop bit)
Protocol: CRC-16/CCITT framed packets + ASCII CSV on USB-UART
```

---

## Servo Connection Diagram

```
                     ┌─────────────────────┐
                     │    Arduino Uno       │
                     │                      │
                     │  Pin 9 (PWM 50 Hz) ──┼────► SERVO SIGNAL (orange)
                     │                      │
                     │  GND ────────────────┼─┐
                     └─────────────────────┘ │
                                              │
                     ┌─────────────────┐      │
  AC adapter or      │  External 5 V   │      │
  USB power bank ───►│  Supply (+5V) ──┼──────┼──► SERVO POWER (red)
                     │  GND ───────────┼──────┴──► SERVO GND (brown)
                     └─────────────────┘

                     ⚠️  Add 100 µF capacitor across the servo
                          power rails to suppress voltage spikes.

Servo Pulse Width:
  0 %  →  1.0 ms pulse  →  0°
  50 % →  1.5 ms pulse  →  90°
  100% →  2.0 ms pulse  →  180°
  Frequency: 50 Hz (20 ms period)
```

---

## Power Distribution Diagram

```
                     USB Power (5 V, 500 mA max)
                         │
                         ▼
              ┌──────────────────┐
              │  Arduino Uno     │
              │  Onboard 3.3 V   │────────────► Level shifter LV rail (3.3 V)
              │  regulator       │
              │  5 V rail ───────┼────────────► Level shifter HV rail (5 V)
              └──────────────────┘
                         │
                        GND (common)
                         │
              ┌──────────┴──────────────────────────────────────┐
              │                                                   │
              ▼                                                   ▼
   ┌─────────────────────┐                          ┌─────────────────────┐
   │  ESP32 DevKit v1    │  ← USB power (separate)  │  External 5 V        │
   │  3.3 V internal     │                          │  Supply (servo)      │
   │  LDO regulator      │                          │  Rated ≥ 1 A         │
   └─────────────────────┘                          └─────────────────────┘
         ▲ 100 µF cap across ESP32 5V/GND                   │
                                                    SG90 / MG996R Servo
                                                    (Power: 150–600 mA stall)
```

> **Rule:** Keep servo power separate from logic power. Servo motor current spikes cause brown-outs on the Arduino if powered from USB alone.

---

## Assembly Instructions

### Step 1 — Prepare the Breadboard

```
1. Place the level shifter module in the centre of the breadboard.
2. Connect the HV rail to the breadboard positive bus (+5 V) → bridge to Arduino 5V.
3. Connect the LV rail to breadboard secondary bus → bridge to ESP32 3V3.
4. Connect both GND pins of the level shifter to the breadboard GND bus.
5. Run a jumper from Arduino GND → breadboard GND bus.
6. Run a jumper from ESP32 GND → same breadboard GND bus.
```

### Step 2 — UART Connections

```
1. Arduino Pin 1 (TX) ─► breadboard column A
2. Level shifter HV-B (input side) ← breadboard column A
3. Level shifter LV-B (output side) ─► ESP32 GPIO16 (RX2)

4. ESP32 GPIO17 (TX2) ─► breadboard column B
5. Level shifter LV-A (input side) ← breadboard column B
6. Level shifter HV-A (output side) ─► Arduino Pin 0 (RX)
```

### Step 3 — Status LEDs (optional)

```
1. Insert red LED anode (+) into GPIO4 column.
2. Connect 330 Ω resistor from LED cathode (−) to GND bus.
3. Repeat for yellow LED on GPIO2.
```

### Step 4 — Servo Connections

```
1. Connect servo signal wire (orange) to Arduino Pin 9.
2. Connect servo power (red) to EXTERNAL 5 V supply positive.
3. Connect servo GND (brown) to EXTERNAL 5 V supply negative AND
   bridge that negative to the breadboard GND bus (common ground).
4. Place 100 µF electrolytic capacitor across the servo power rails
   (+ to supply positive, − to supply negative / GND).
```

### Step 5 — USB Connections

```
1. Connect ESP32 DevKit to Host PC via Micro-USB.
2. Connect Arduino Uno to Host PC via USB-B.
   ⚠️  Disconnect Arduino UART jumpers (pins 0, 1) before uploading.
3. Both boards should appear as separate COM ports.
   Windows:  COM3, COM4 (check Device Manager)
   Linux:    /dev/ttyUSB0, /dev/ttyUSB1
   macOS:    /dev/cu.SLAB_USBtoUART, /dev/cu.usbmodem...
```

---

## Troubleshooting

### No communication between MCUs

| Symptom | Likely Cause | Fix |
|---|---|---|
| Heartbeat LED not blinking on ESP32 | Firmware not uploaded or crashed | Re-upload plant_simulator.ino; check Serial Monitor for errors |
| Arduino Serial Monitor shows garbled data | Wrong baud rate | Set Serial Monitor to 115200 |
| No serial data at all | TX/RX jumper wires swapped | Swap RX↔TX wire between MCUs |
| Intermittent packet errors | Missing level shifter | Add TXS0102 level shifter |
| CRC errors in logs | Loose jumper wire | Re-seat all connections; check GND continuity |

### Servo does not move

| Symptom | Likely Cause | Fix |
|---|---|---|
| Servo makes noise but doesn't rotate | Power insufficient | Use external 5 V supply ≥ 1 A |
| Servo jitters at rest | PWM signal noise | Add 100 µF cap across servo power; move servo wire away from UART wires |
| Servo only goes to one extreme | PID gains too high | Reduce Kp; check output clamping in Controllers.h |
| Servo not responding | Wrong pin or library | Confirm `Servo.attach(9)` in controller.ino; test with sweep sketch |

### Dashboard connection fails

| Symptom | Likely Cause | Fix |
|---|---|---|
| "Port not found" error | Wrong COM port | Check Device Manager / `ls /dev/tty*`; update HIL_PORT env var |
| `serial.SerialException` | Port in use by IDE | Close Arduino IDE Serial Monitor before starting dashboard |
| WebSocket keeps disconnecting | Firewall blocking port 5000 | Allow port 5000 in firewall; try `--port 5001` |
| Charts not updating | SocketIO not connecting | Check browser console for WS errors; try hard refresh |

### Power issues

| Symptom | Likely Cause | Fix |
|---|---|---|
| Arduino resets when servo moves | Current surge on USB | Use powered USB hub or external servo supply |
| ESP32 brownout on startup | 3.3 V regulator overloaded | Disconnect peripherals; power ESP32 from dedicated USB |
| Erratic plant output | Floating GND reference | Verify single common GND across all three boards |

---

## Hardware Validation Checklist

Before running control experiments:

```
□ All GND pins connected to common bus (continuity verified with multimeter)
□ Level shifter LV = 3.3 V (measure with multimeter)
□ Level shifter HV = 5.0 V (measure with multimeter)
□ ESP32 heartbeat LED blinks every 1 s
□ Arduino Serial Monitor shows CSV data (t,sp,pv,cv,error) at 10 Hz
□ Servo responds to `sp:25` and `sp:75` commands from dashboard
□ UART packet error rate < 1 % (visible in dashboard link stats)
□ CRC test passes: python -m pytest tests/test_protocol.py -v
```

---

*Wiring Guide v1.0 — HIL Servo Control Testbed*
*Refer to ARCHITECTURE.md for system-level context.*
