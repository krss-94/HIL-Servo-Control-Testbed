/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║     HARDWARE-IN-THE-LOOP SERVO CONTROL TESTBED                  ║
 * ║     Controller Firmware — Arduino Uno / ESP32                   ║
 * ║     Version: 2.0.0  |  Multi-Algorithm Control Engine          ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * This firmware runs on the Controller MCU.
 * It receives simulated sensor feedback from the Plant Simulator MCU,
 * computes the control output using the selected algorithm, and drives
 * the servo actuator.
 *
 * Control Algorithms:
 *   [0] On/Off Control     (Bang-Bang with hysteresis)
 *   [1] Proportional       (P only)
 *   [2] PI Control         (Proportional-Integral)
 *   [3] PID Control        (with anti-windup, derivative filter)
 *   [4] PID + Feedforward  (model-based enhancement)
 *
 * Author: HIL Testbed Project
 * License: MIT
 */

#include <Arduino.h>
#include <Servo.h>
#include "Controllers.h"
#include "CommProtocol.h"

// ─── Hardware Configuration ───────────────────────────────────────────────────
#define SERVO_PIN         9
#define STATUS_LED        13
#define MODE_BTN          7       // Cycle control algorithm
#define SP_POT            A0      // Setpoint potentiometer
#define UART_BAUD         115200

// ─── Control Parameters (tunable via Serial) ──────────────────────────────────
float Kp = 2.5f, Ki = 0.8f, Kd = 0.15f;
float setpoint    = 50.0f;   // 0–100% of plant range
float hysteresis  = 2.0f;    // On/Off hysteresis band

// ─── Global Objects ───────────────────────────────────────────────────────────
Servo         servo;
CommProtocol  comm;
PIDController pid;
PIController  pi;
PController   p;
OnOffController onoff;

uint8_t activeCtrl = 3;   // Default: PID

float   processVariable = 0.0f;
float   controlOutput   = 0.0f;
uint32_t tickCount      = 0;
bool    commOK          = false;

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  #if defined(ESP32)
    Serial2.begin(UART_BAUD, SERIAL_8N1, 16, 17);
    comm.init(&Serial2);
  #else
    Serial1.begin(UART_BAUD);
    comm.init(&Serial1);
  #endif

  servo.attach(SERVO_PIN);
  servo.write(90);  // centre

  pinMode(STATUS_LED, OUTPUT);
  pinMode(MODE_BTN,   INPUT_PULLUP);

  pid.init(Kp, Ki, Kd, 0.001f);
  pi.init(Kp, Ki, 0.001f);
  p.init(Kp);
  onoff.init(hysteresis);

  Serial.println(F("──────────────────────────────────────────"));
  Serial.println(F("  HIL Controller v2.0  |  Arduino/ESP32   "));
  Serial.println(F("──────────────────────────────────────────"));
  Serial.println(F("  Commands:"));
  Serial.println(F("    sp:<val>       Set setpoint (0-100)"));
  Serial.println(F("    kp:<val>       Set Kp"));
  Serial.println(F("    ki:<val>       Set Ki"));
  Serial.println(F("    kd:<val>       Set Kd"));
  Serial.println(F("    ctrl:<0-4>     Change controller"));
  Serial.println(F("    reset          Reset system"));
  Serial.println(F("──────────────────────────────────────────"));
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  handleSerialCommands();
  handleModeButton();

  // Receive plant feedback
  if (comm.dataAvailable()) {
    HILPacket feedback = comm.receive();
    if (feedback.type == PKT_SENSOR_DATA) {
      processVariable = feedback.plantOutput;
      commOK          = true;
      tickCount++;

      // Compute control output
      controlOutput = computeControl(setpoint, processVariable);
      controlOutput = constrain(controlOutput, 0.0f, 100.0f);

      // Drive servo (map 0-100% → 0°-180°)
      int servoAngle = (int)(controlOutput * 1.8f);
      servo.write(servoAngle);

      // Send control output back to plant simulator
      HILPacket cmd;
      cmd.type         = PKT_CONTROL_OUTPUT;
      cmd.controlValue = controlOutput;
      cmd.setpoint     = setpoint;
      cmd.timestamp    = tickCount;
      comm.transmit(cmd);

      // Status LED
      if ((tickCount % 500) == 0)
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
  }

  // If no comm, blink LED fast (alert)
  static uint32_t lastCommMs = 0;
  if (!commOK && (millis() - lastCommMs > 200)) {
    lastCommMs = millis();
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
}

// ─── Control Computation ─────────────────────────────────────────────────────
float computeControl(float sp, float pv) {
  switch (activeCtrl) {
    case 0: return onoff.compute(sp, pv);
    case 1: return p.compute(sp, pv);
    case 2: return pi.compute(sp, pv);
    case 3: return pid.compute(sp, pv);
    default: return 0.0f;
  }
}

// ─── Serial Command Parser ────────────────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("sp:")) {
    setpoint = cmd.substring(3).toFloat();
    setpoint = constrain(setpoint, 0.0f, 100.0f);
    Serial.printf("[CTRL] Setpoint → %.1f%%\n", setpoint);

  } else if (cmd.startsWith("kp:")) {
    Kp = cmd.substring(3).toFloat();
    pid.setGains(Kp, Ki, Kd);
    pi.setGains(Kp, Ki);
    p.setGain(Kp);
    Serial.printf("[CTRL] Kp → %.3f\n", Kp);

  } else if (cmd.startsWith("ki:")) {
    Ki = cmd.substring(3).toFloat();
    pid.setGains(Kp, Ki, Kd);
    pi.setGains(Kp, Ki);
    Serial.printf("[CTRL] Ki → %.3f\n", Ki);

  } else if (cmd.startsWith("kd:")) {
    Kd = cmd.substring(3).toFloat();
    pid.setGains(Kp, Ki, Kd);
    Serial.printf("[CTRL] Kd → %.3f\n", Kd);

  } else if (cmd.startsWith("ctrl:")) {
    activeCtrl = cmd.substring(5).toInt();
    pid.reset(); pi.reset();
    const char* names[] = { "On/Off", "P", "PI", "PID", "PID+FF" };
    Serial.printf("[CTRL] Controller → %s\n", names[activeCtrl]);

  } else if (cmd == "reset") {
    pid.reset(); pi.reset();
    HILPacket rst; rst.type = PKT_RESET;
    comm.transmit(rst);
    Serial.println(F("[CTRL] System reset"));

  } else if (cmd == "status") {
    printStatus();
  }
}

// ─── Mode Button Handler ─────────────────────────────────────────────────────
void handleModeButton() {
  static bool lastBtn = HIGH;
  bool btn = digitalRead(MODE_BTN);
  if (btn == LOW && lastBtn == HIGH) {
    activeCtrl = (activeCtrl + 1) % 4;
    pid.reset(); pi.reset();
    Serial.printf("[CTRL] Mode → %d\n", activeCtrl);
  }
  lastBtn = btn;
}

// ─── Status Report ────────────────────────────────────────────────────────────
void printStatus() {
  Serial.println(F("─── Controller Status ────────────────────"));
  Serial.printf("Active Ctrl:  %d (0=OnOff 1=P 2=PI 3=PID)\n", activeCtrl);
  Serial.printf("Setpoint:     %.2f%%\n", setpoint);
  Serial.printf("Process Var:  %.2f%%\n", processVariable);
  Serial.printf("Control Out:  %.2f%%\n", controlOutput);
  Serial.printf("Error:        %.2f%%\n", setpoint - processVariable);
  Serial.printf("Gains:        Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
  Serial.printf("Link OK:      %s\n", commOK ? "YES" : "NO");
  Serial.println(F("──────────────────────────────────────────"));
}
