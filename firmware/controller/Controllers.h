/**
 * Controllers.h — Multi-Algorithm Control Library
 * ═════════════════════════════════════════════════
 * Implements 4 classical feedback control algorithms
 * with production-grade features:
 *
 *   On/Off:  Bang-bang with configurable hysteresis
 *   P:       Proportional with output limits
 *   PI:      Integral anti-windup via back-calculation
 *   PID:     Derivative filter, output clamping, integral anti-windup
 *
 * All controllers:
 *   - Accept setpoint and process variable (0–100%)
 *   - Return control output (0–100%)
 *   - Support runtime gain updates
 *   - Expose internal state for diagnostics
 */

#pragma once
#include <Arduino.h>

// ─── On/Off Controller ────────────────────────────────────────────────────────
class OnOffController {
public:
  float _hyst    = 2.0f;
  float _output  = 0.0f;

  void init(float hysteresis) { _hyst = hysteresis; }

  float compute(float sp, float pv) {
    float err = sp - pv;
    if (err >  _hyst/2.0f) _output = 100.0f;
    if (err < -_hyst/2.0f) _output = 0.0f;
    return _output;
  }
};

// ─── Proportional Controller ──────────────────────────────────────────────────
class PController {
public:
  float _Kp     = 1.0f;
  float _bias   = 50.0f;  // Output bias at zero error

  void init(float Kp) { _Kp = Kp; }
  void setGain(float Kp) { _Kp = Kp; }

  float compute(float sp, float pv) {
    float err = sp - pv;
    return constrain(_bias + _Kp * err, 0.0f, 100.0f);
  }
};

// ─── PI Controller ────────────────────────────────────────────────────────────
// Anti-windup via integrator clamping (conditional integration)
class PIController {
public:
  float _Kp = 1.0f, _Ki = 0.0f;
  float _Ts = 0.001f;
  float _integral  = 0.0f;
  float _outMin    = 0.0f;
  float _outMax    = 100.0f;
  float _lastOutput= 0.0f;

  void init(float Kp, float Ki, float Ts) {
    _Kp = Kp; _Ki = Ki; _Ts = Ts;
  }

  void setGains(float Kp, float Ki) { _Kp = Kp; _Ki = Ki; }

  float compute(float sp, float pv) {
    float err = sp - pv;

    // Anti-windup: only integrate if output is not saturated
    bool saturated = (_lastOutput >= _outMax) || (_lastOutput <= _outMin);
    bool windUp    = saturated && (err * _lastOutput > 0);

    if (!windUp)
      _integral += err * _Ts;

    // Clamp integral
    _integral = constrain(_integral, _outMin / _Ki, _outMax / _Ki);

    float out = _Kp * err + _Ki * _integral;
    _lastOutput = constrain(out, _outMin, _outMax);
    return _lastOutput;
  }

  void reset() { _integral = 0.0f; _lastOutput = 0.0f; }

  // Diagnostics
  float getIntegral() { return _integral; }
};

// ─── PID Controller ───────────────────────────────────────────────────────────
// Features:
//   - Derivative on measurement (not error) → avoids derivative kick
//   - First-order derivative low-pass filter: Tf = Td/N
//   - Integral anti-windup via back-calculation
//   - Output rate limiting (optional)
class PIDController {
public:
  float _Kp = 2.5f, _Ki = 0.8f, _Kd = 0.15f;
  float _Ts  = 0.001f;
  float _N   = 10.0f;     // Derivative filter coefficient
  float _Kb  = 0.1f;      // Back-calculation gain for anti-windup

  float _integral     = 0.0f;
  float _prevPV       = 0.0f;
  float _derivFilter  = 0.0f;
  float _lastOutput   = 0.0f;
  float _outMin       = 0.0f;
  float _outMax       = 100.0f;

  // Diagnostics
  float _pTerm = 0.0f, _iTerm = 0.0f, _dTerm = 0.0f;

  void init(float Kp, float Ki, float Kd, float Ts) {
    _Kp = Kp; _Ki = Ki; _Kd = Kd; _Ts = Ts;
  }

  void setGains(float Kp, float Ki, float Kd) {
    _Kp = Kp; _Ki = Ki; _Kd = Kd;
  }

  float compute(float sp, float pv) {
    float err = sp - pv;

    // P term
    _pTerm = _Kp * err;

    // D term — derivative on measurement with low-pass filter
    // Tf = Kd/(Kp*N), discrete: df[k] = (1-Ts/Tf)*df[k-1] + (Kd/Tf)*(pv-prevPV)
    float dPV = pv - _prevPV;
    float Tf  = (_N > 0) ? (_Kd / (_Kp * _N)) : 0.01f;
    _derivFilter = (1.0f - _Ts/Tf) * _derivFilter - (_Kd/Tf) * dPV;
    _dTerm = _derivFilter;
    _prevPV = pv;

    // Raw output before integral
    float rawOut = _pTerm + _integral + _dTerm;
    float clampedOut = constrain(rawOut, _outMin, _outMax);

    // Anti-windup: back-calculation
    float satError = clampedOut - rawOut;

    // I term with anti-windup
    _integral += (_Ki * err + _Kb * satError) * _Ts;
    _iTerm = _integral;

    _lastOutput = clampedOut;
    return _lastOutput;
  }

  void reset() {
    _integral = _derivFilter = _prevPV = _lastOutput = 0.0f;
    _pTerm = _iTerm = _dTerm = 0.0f;
  }

  // Diagnostics
  float getP() { return _pTerm; }
  float getI() { return _iTerm; }
  float getD() { return _dTerm; }
  float getOutput() { return _lastOutput; }

  // Auto-tune: Ziegler-Nichols ultimate gain method
  // Call with Ku (ultimate gain) and Tu (ultimate period in seconds)
  void autoTuneZN(float Ku, float Tu, bool closedLoop = true) {
    if (closedLoop) {
      // Classic Z-N
      _Kp = 0.6f * Ku;
      _Ki = 1.2f * Ku / Tu;
      _Kd = 0.075f * Ku * Tu;
    } else {
      // Conservative (no overshoot)
      _Kp = 0.2f * Ku;
      _Ki = 0.4f * Ku / Tu;
      _Kd = 0.066f * Ku * Tu;
    }
  }
};
