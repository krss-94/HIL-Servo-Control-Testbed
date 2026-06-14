/**
 * FaultInjector.h — Real-Time Fault Injection Engine
 * ════════════════════════════════════════════════════
 * Simulates real-world failures for robustness testing:
 *   - Signal noise (Gaussian)
 *   - Sensor bias/offset drift
 *   - Actuator saturation
 *   - Signal dropout / stuck-at faults
 *   - Random disturbance pulses
 */

#pragma once
#include <Arduino.h>

enum FaultType : uint8_t {
  FAULT_NONE       = 0x00,
  FAULT_NOISE      = 0x01,   // Add Gaussian noise
  FAULT_BIAS       = 0x02,   // Constant offset on output
  FAULT_SATURATE   = 0x03,   // Clamp actuator at reduced range
  FAULT_DROPOUT    = 0x04,   // Signal goes to zero
  FAULT_STUCK      = 0x05,   // Signal freezes at last value
  FAULT_DISTURBANCE= 0x06,   // Step disturbance on plant input
  FAULT_DELAY      = 0x07,   // Simulate transport delay
};

class FaultInjector {
public:
  FaultType _type     = FAULT_NONE;
  float     _mag      = 0.0f;
  float     _lastVal  = 0.0f;
  uint8_t   _delayBuf[50] = {};  // max 50ms delay
  uint8_t   _delayHead = 0;
  uint32_t  _seed      = 42;

  void init()  { _type = FAULT_NONE; _mag = 0.0f; }

  void setFault(FaultType t, float magnitude) {
    _type = t;
    _mag  = magnitude;
  }

  void clearFault() {
    _type = FAULT_NONE;
    _mag  = 0.0f;
  }

  float apply(float u) {
    switch (_type) {
      case FAULT_NOISE:
        return u + _mag * gaussianRand();

      case FAULT_BIAS:
        return u + _mag;

      case FAULT_SATURATE:
        return constrain(u, -_mag, _mag);

      case FAULT_DROPOUT:
        return 0.0f;

      case FAULT_STUCK:
        return _lastVal;   // ignore new input, return frozen value

      case FAULT_DISTURBANCE: {
        static uint32_t nextPulse = 5000;
        static bool active = false;
        static uint32_t t = 0;
        t++;
        if (t >= nextPulse) { active = !active; nextPulse = t + 500 + rand()%2000; }
        return u + (active ? _mag : 0.0f);
      }

      case FAULT_DELAY: {
        uint8_t delaySteps = (uint8_t)constrain(_mag, 1, 49);
        // Shift buffer
        for (int i = delaySteps; i > 0; i--)
          _delayBuf[i] = _delayBuf[i-1];
        _delayBuf[0] = (uint8_t)constrain(u + 100.0f, 0, 200);
        return (float)_delayBuf[delaySteps] - 100.0f;
      }

      default:
        _lastVal = u;
        return u;
    }
  }

private:
  // Box-Muller Gaussian random (unit variance)
  float gaussianRand() {
    float u1 = (float)(rand()) / RAND_MAX;
    float u2 = (float)(rand()) / RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
  }
};


/**
 * MetricsEngine.h — Real-Time Performance Metrics Calculator
 * ═══════════════════════════════════════════════════════════
 * Calculates classical control performance indices in real-time.
 */
class MetricsEngine {
public:
  float    _sp         = 0.0f;  // setpoint
  float    _pv         = 0.0f;  // process variable
  float    _finalVal   = 0.0f;
  float    _peakVal    = 0.0f;
  float    _sumAbsErr  = 0.0f;  // IAE accumulator
  float    _sumCtrlEff = 0.0f;  // Control effort
  float    _itseSumE2  = 0.0f;  // ITSE
  uint32_t _n          = 0;
  float    _Ts         = 0.001f;

  // Rise time detection
  bool     _rt10Passed = false;
  bool     _rt90Passed = false;
  float    _rt10Time   = 0.0f;
  float    _riseTime   = -1.0f;

  // Settling time (2% band)
  float    _settlingTime = -1.0f;
  uint32_t _lastOutOfBand = 0;

  PerformanceMetrics _snapshot;

  void init()  { reset(); }

  void reset() {
    _n = _lastOutOfBand = 0;
    _peakVal = _sumAbsErr = _sumCtrlEff = _itseSumE2 = 0.0f;
    _riseTime = _settlingTime = -1.0f;
    _rt10Passed = _rt90Passed = false;
    memset(&_snapshot, 0, sizeof(_snapshot));
  }

  void update(float sp, float pv, float u) {
    _sp = sp; _pv = pv; _n++;
    float t   = _n * _Ts;
    float err = sp - pv;

    // Accumulate metrics
    _sumAbsErr  += fabsf(err)  * _Ts;
    _sumCtrlEff += fabsf(u)    * _Ts;
    _itseSumE2  += t * err * err * _Ts;

    // Peak overshoot detection
    if (pv > _peakVal) _peakVal = pv;

    // Rise time (10% → 90% of setpoint)
    if (!_rt10Passed && pv >= 0.1f * sp) {
      _rt10Passed = true; _rt10Time = t;
    }
    if (!_rt90Passed && pv >= 0.9f * sp) {
      _rt90Passed = true;
      _riseTime   = t - _rt10Time;
    }

    // Settling time (last exit from 2% band)
    if (fabsf(err) > 0.02f * fabsf(sp) && sp != 0.0f) {
      _lastOutOfBand = _n;
    }

    // Update snapshot every 100ms
    if (_n % 100 == 0) _snapshot = getFinal();
  }

  PerformanceMetrics getSnapshot() { return _snapshot; }

  PerformanceMetrics getFinal() {
    PerformanceMetrics m;
    m.riseTime        = (_riseTime > 0) ? _riseTime : 0.0f;
    m.settlingTime    = _lastOutOfBand * _Ts;
    m.overshoot       = (_sp > 0 && _peakVal > _sp) ?
                        (_peakVal - _sp) / _sp * 100.0f : 0.0f;
    m.steadyStateError= _sp - _pv;
    m.controlEffort   = _sumCtrlEff;
    m.iae             = _sumAbsErr;
    m.itse            = _itseSumE2;
    m.sampleCount     = _n;
    return m;
  }
};
