/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║     HARDWARE-IN-THE-LOOP SERVO CONTROL TESTBED                  ║
 * ║     Plant Simulator Firmware — ESP32                            ║
 * ║     Version: 2.0.0  |  Real-Time Plant Model Engine            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * This firmware runs on the Plant Simulator MCU (ESP32).
 * It implements multiple mathematical plant models updated at 1kHz,
 * communicates over UART with the Controller MCU, and handles
 * fault injection for robustness testing.
 *
 * Plant Models Implemented:
 *   [0] First-Order System    (thermal, RC circuit analog)
 *   [1] Second-Order System   (mass-spring-damper)
 *   [2] Servo Position Model  (DC servo with inertia)
 *   [3] Tank Level Model      (hydraulic process)
 *   [4] DC Motor Model        (back-EMF, inertia, friction)
 *
 * Author: HIL Testbed Project
 * License: MIT
 */

#include <Arduino.h>
#include "PlantModels.h"
#include "CommProtocol.h"
#include "FaultInjector.h"
#include "MetricsEngine.h"

// ─── Hardware Configuration ───────────────────────────────────────────────────
#define UART_BAUD         115200
#define UART_TX_PIN       17
#define UART_RX_PIN       16
#define STATUS_LED        2
#define FAULT_LED         4
#define SAMPLE_RATE_HZ    1000
#define SAMPLE_PERIOD_US  1000

// ─── System State Machine ─────────────────────────────────────────────────────
enum SystemState {
  STATE_INIT,
  STATE_IDLE,
  STATE_RUNNING,
  STATE_FAULT,
  STATE_CALIBRATE
};

// ─── Global Objects ───────────────────────────────────────────────────────────
PlantModels     plant;
CommProtocol    comm;
FaultInjector   faultInject;
MetricsEngine   metrics;

SystemState     sysState       = STATE_INIT;
PlantModelType  activePlant    = PLANT_SERVO;
uint8_t         activeControl  = CTRL_PID;

volatile bool   sampleFlag     = false;
hw_timer_t*     sampleTimer    = nullptr;

float           controlInput   = 0.0f;
float           plantOutput    = 0.0f;
float           setpoint       = 0.0f;
uint32_t        tickCount      = 0;

// ─── ISR: 1kHz Sample Timer ───────────────────────────────────────────────────
void IRAM_ATTR onSampleTimer() {
  sampleFlag = true;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(FAULT_LED,  OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(FAULT_LED,  LOW);

  plant.init(PLANT_SERVO);
  comm.init(&Serial2);
  faultInject.init();
  metrics.init();

  // Configure 1kHz hardware timer
  sampleTimer = timerBegin(0, 80, true);          // 80MHz / 80 = 1MHz
  timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
  timerAlarmWrite(sampleTimer, SAMPLE_PERIOD_US, true);
  timerAlarmEnable(sampleTimer);

  sysState = STATE_IDLE;
  Serial.println(F("[HIL] Plant Simulator v2.0 — ESP32 Ready"));
  Serial.println(F("[HIL] Active Model: SERVO_POSITION"));
  blinkLED(STATUS_LED, 3, 100);
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  // Parse any incoming command from Controller MCU
  if (comm.dataAvailable()) {
    HILPacket pkt = comm.receive();
    processPacket(pkt);
  }

  // Process at 1kHz
  if (sampleFlag) {
    sampleFlag = false;
    tickCount++;

    if (sysState == STATE_RUNNING) {
      // 1. Apply fault injection if active
      float injectedInput = faultInject.apply(controlInput);

      // 2. Update plant model
      plantOutput = plant.update(injectedInput);

      // 3. Update performance metrics
      metrics.update(setpoint, plantOutput, injectedInput);

      // 4. Transmit sensor feedback to Controller MCU
      HILPacket feedback;
      feedback.type        = PKT_SENSOR_DATA;
      feedback.timestamp   = tickCount;
      feedback.plantOutput = plantOutput;
      feedback.setpoint    = setpoint;
      feedback.metrics     = metrics.getSnapshot();
      comm.transmit(feedback);

      // 5. Serial Plotter output (CSV: time,sp,pv,cv,error)
      if ((tickCount % 10) == 0) {  // 100Hz to serial plotter
        float t = tickCount * 0.001f;
        Serial.printf("%.3f,%.4f,%.4f,%.4f,%.4f\n",
          t, setpoint, plantOutput,
          injectedInput, setpoint - plantOutput);
      }

      // Status LED heartbeat every 500ms
      if ((tickCount % 500) == 0) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      }
    }
  }
}

// ─── Packet Handler ───────────────────────────────────────────────────────────
void processPacket(const HILPacket& pkt) {
  switch (pkt.type) {
    case PKT_CONTROL_OUTPUT:
      controlInput = constrain(pkt.controlValue, -100.0f, 100.0f);
      setpoint     = pkt.setpoint;
      if (sysState != STATE_RUNNING) {
        sysState = STATE_RUNNING;
        Serial.println(F("[HIL] Control loop STARTED"));
      }
      break;

    case PKT_SET_PLANT:
      changePlant((PlantModelType)pkt.plantModel);
      break;

    case PKT_FAULT_INJECT:
      faultInject.setFault((FaultType)pkt.faultType, pkt.faultMagnitude);
      digitalWrite(FAULT_LED, HIGH);
      Serial.printf("[HIL] Fault Injected: type=%d mag=%.2f\n",
        pkt.faultType, pkt.faultMagnitude);
      break;

    case PKT_FAULT_CLEAR:
      faultInject.clearFault();
      digitalWrite(FAULT_LED, LOW);
      Serial.println(F("[HIL] Fault Cleared"));
      break;

    case PKT_RESET:
      plant.reset();
      metrics.reset();
      tickCount    = 0;
      controlInput = 0.0f;
      plantOutput  = 0.0f;
      sysState     = STATE_IDLE;
      Serial.println(F("[HIL] System RESET"));
      break;

    case PKT_GET_METRICS:
      sendMetricsReport();
      break;

    default:
      Serial.printf("[HIL] Unknown packet type: 0x%02X\n", pkt.type);
      break;
  }
}

// ─── Plant Model Switch ───────────────────────────────────────────────────────
void changePlant(PlantModelType newPlant) {
  activePlant = newPlant;
  plant.init(newPlant);
  metrics.reset();
  const char* names[] = {
    "FIRST_ORDER", "SECOND_ORDER", "SERVO_POSITION", "TANK_LEVEL", "DC_MOTOR"
  };
  Serial.printf("[HIL] Plant changed to: %s\n", names[newPlant]);
}

// ─── Metrics Broadcast ───────────────────────────────────────────────────────
void sendMetricsReport() {
  PerformanceMetrics m = metrics.getFinal();
  HILPacket rpt;
  rpt.type    = PKT_METRICS_REPORT;
  rpt.metrics = m;
  comm.transmit(rpt);

  Serial.println(F("─── Performance Report ───────────────────"));
  Serial.printf("Rise Time:       %.3f s\n",  m.riseTime);
  Serial.printf("Settling Time:   %.3f s\n",  m.settlingTime);
  Serial.printf("Overshoot:       %.2f %%\n", m.overshoot);
  Serial.printf("SS Error:        %.4f\n",    m.steadyStateError);
  Serial.printf("Control Effort:  %.4f\n",    m.controlEffort);
  Serial.println(F("──────────────────────────────────────────"));
}

// ─── Utility ─────────────────────────────────────────────────────────────────
void blinkLED(int pin, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH); delay(delayMs);
    digitalWrite(pin, LOW);  delay(delayMs);
  }
}
