/**
 * PlantModels.h — Mathematical Plant Model Library
 * ══════════════════════════════════════════════════
 * Implements 5 continuous-time plant models discretized via
 * Euler/Runge-Kutta 4th order integration at 1kHz.
 *
 * All models accept a normalized control input u ∈ [-100, 100]%
 * and return a normalized output y.
 */

#pragma once
#include <Arduino.h>
#include <cmath>

enum PlantModelType : uint8_t {
  PLANT_FIRST_ORDER  = 0,
  PLANT_SECOND_ORDER = 1,
  PLANT_SERVO        = 2,
  PLANT_TANK         = 3,
  PLANT_DC_MOTOR     = 4
};

// ─── First-Order Plant ─────────────────────────────────────────────────────
// Transfer function: G(s) = K / (τs + 1)
// State equation:    τ·ẏ = K·u - y
// Parameters: K=gain, τ=time_constant
struct FirstOrderState {
  float y   = 0.0f;   // output
  float K   = 2.0f;   // DC gain
  float tau = 3.0f;   // time constant [s]
  const float Ts = 0.001f;

  float update(float u) {
    // Euler integration: y[k+1] = y[k] + Ts*(K*u - y[k])/tau
    y += (Ts / tau) * (K * u - y);
    return y;
  }
  void reset() { y = 0.0f; }
};

// ─── Second-Order Plant ────────────────────────────────────────────────────
// Transfer function: G(s) = ωn² / (s² + 2ζωn·s + ωn²)
// State equations: ẋ1 = x2
//                  ẋ2 = ωn²·u - 2ζωn·x2 - ωn²·x1
struct SecondOrderState {
  float x1  = 0.0f;   // position
  float x2  = 0.0f;   // velocity
  float wn  = 2.0f;   // natural frequency [rad/s]
  float zeta= 0.5f;   // damping ratio
  const float Ts = 0.001f;

  float update(float u) {
    // RK4 integration for accuracy
    float dx1, dx2;
    // k1
    float k1_x1 = x2;
    float k1_x2 = wn*wn*u - 2*zeta*wn*x2 - wn*wn*x1;
    // k2
    float k2_x1 = x2 + 0.5f*Ts*k1_x2;
    float k2_x2 = wn*wn*u - 2*zeta*wn*(x2+0.5f*Ts*k1_x2) - wn*wn*(x1+0.5f*Ts*k1_x1);
    // k3
    float k3_x1 = x2 + 0.5f*Ts*k2_x2;
    float k3_x2 = wn*wn*u - 2*zeta*wn*(x2+0.5f*Ts*k2_x2) - wn*wn*(x1+0.5f*Ts*k2_x1);
    // k4
    float k4_x1 = x2 + Ts*k3_x2;
    float k4_x2 = wn*wn*u - 2*zeta*wn*(x2+Ts*k3_x2) - wn*wn*(x1+Ts*k3_x1);

    x1 += (Ts/6.0f)*(k1_x1 + 2*k2_x1 + 2*k3_x1 + k4_x1);
    x2 += (Ts/6.0f)*(k1_x2 + 2*k2_x2 + 2*k3_x2 + k4_x2);
    return x1;
  }
  void reset() { x1 = x2 = 0.0f; }
};

// ─── Servo Position Model ─────────────────────────────────────────────────
// G(s) = Km / (s(Js + B))  → position output
// Includes: inertia J, viscous friction B, motor gain Km
// State: θ (angle), ω (angular velocity)
struct ServoState {
  float theta = 0.0f;   // position [deg, normalized to 0-100]
  float omega = 0.0f;   // angular velocity
  float J     = 0.01f;  // moment of inertia
  float B     = 0.05f;  // viscous friction
  float Km    = 0.5f;   // motor torque constant
  float maxTheta = 180.0f;
  const float Ts = 0.001f;

  float update(float u) {
    // τ = Km*u - B*ω  (torque equation)
    // J*α = τ  →  α = (Km*u - B*ω) / J
    float torque = Km * u - B * omega;
    float alpha  = torque / J;

    // Euler: ω, θ update
    omega += Ts * alpha;
    theta += Ts * omega * (180.0f / M_PI);  // rad/s → deg/s

    // Clamp to physical limits
    theta = constrain(theta, 0.0f, maxTheta);
    if ((theta <= 0.0f && omega < 0) || (theta >= maxTheta && omega > 0))
      omega = 0.0f;

    return theta / maxTheta * 100.0f;  // normalize to %
  }
  void reset() { theta = omega = 0.0f; }
};

// ─── Tank Level Model ─────────────────────────────────────────────────────
// dh/dt = (Qin - Qout) / A
// Qout = Cv * sqrt(h)  (Torricelli's theorem)
// u controls inlet valve (0-100% → Qin)
struct TankState {
  float h       = 0.0f;   // level [m]
  float A       = 1.0f;   // cross-section area [m²]
  float Cv      = 0.15f;  // outlet coefficient
  float Qmax    = 0.3f;   // max inlet flow [m³/s]
  float hMax    = 3.0f;   // max level [m]
  const float Ts = 0.001f;

  float update(float u) {
    float Qin  = (u / 100.0f) * Qmax;
    float Qout = Cv * sqrtf(fabsf(h));
    float dhdt = (Qin - Qout) / A;
    h += Ts * dhdt;
    h = constrain(h, 0.0f, hMax);
    return (h / hMax) * 100.0f;  // normalize to %
  }
  void reset() { h = 0.0f; }
};

// ─── DC Motor Model ───────────────────────────────────────────────────────
// V = L·di/dt + R·i + Ke·ω
// J·dω/dt = Kt·i - B·ω - TL
// State: i (current), ω (speed)
struct DCMotorState {
  float i     = 0.0f;   // armature current [A]
  float omega = 0.0f;   // angular velocity [rad/s]
  float R     = 1.0f;   // armature resistance [Ω]
  float L     = 0.01f;  // armature inductance [H]
  float Ke    = 0.5f;   // back-EMF constant
  float Kt    = 0.5f;   // torque constant
  float J     = 0.02f;  // inertia
  float B     = 0.01f;  // friction
  float Vmax  = 12.0f;  // supply voltage [V]
  float wMax  = 300.0f; // max speed [rad/s]
  const float Ts = 0.001f;

  float update(float u) {
    float V = (u / 100.0f) * Vmax;
    float didt  = (V - R*i - Ke*omega) / L;
    float domdt = (Kt*i - B*omega) / J;
    i     += Ts * didt;
    omega += Ts * domdt;
    omega  = constrain(omega, -wMax, wMax);
    return (omega / wMax) * 100.0f;  // normalize to %
  }
  void reset() { i = omega = 0.0f; }
};

// ─── PlantModels Class ────────────────────────────────────────────────────
class PlantModels {
public:
  PlantModelType activeModel;
  FirstOrderState  fo;
  SecondOrderState so;
  ServoState       sv;
  TankState        tk;
  DCMotorState     dc;

  void init(PlantModelType type) {
    activeModel = type;
    reset();
  }

  float update(float u) {
    switch (activeModel) {
      case PLANT_FIRST_ORDER:  return fo.update(u);
      case PLANT_SECOND_ORDER: return so.update(u);
      case PLANT_SERVO:        return sv.update(u);
      case PLANT_TANK:         return tk.update(u);
      case PLANT_DC_MOTOR:     return dc.update(u);
      default:                 return 0.0f;
    }
  }

  void reset() {
    fo.reset(); so.reset(); sv.reset(); tk.reset(); dc.reset();
  }

  // Tune plant parameters at runtime
  void setParameter(const char* param, float value) {
    // E.g. "tau", "zeta", "wn", etc.
    // Parsed from incoming PKT_PARAM_SET packets
  }
};
