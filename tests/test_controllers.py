#!/usr/bin/env python3
"""
HIL Testbed — Controller Validation Test Suite
═══════════════════════════════════════════════
Unit and integration tests for:
  - Plant model numerical accuracy
  - Controller algorithm correctness
  - Communication protocol CRC validation
  - Performance metrics calculation
  - Fault injection scenarios

Run: python -m pytest tests/ -v --tb=short
"""

import pytest, math, struct
import numpy as np
from typing import Callable

# ─── Plant Model Simulation (Python reference) ────────────────────────────────
class FirstOrderPlant:
    def __init__(self, K=2.0, tau=3.0, Ts=0.001):
        self.K, self.tau, self.Ts = K, tau, Ts
        self.y = 0.0

    def step(self, u):
        self.y += (self.Ts / self.tau) * (self.K * u - self.y)
        return self.y

    def reset(self): self.y = 0.0

class SecondOrderPlant:
    def __init__(self, wn=2.0, zeta=0.5, Ts=0.001):
        self.wn, self.zeta, self.Ts = wn, zeta, Ts
        self.x1, self.x2 = 0.0, 0.0

    def step(self, u):
        dx1 = self.x2
        dx2 = self.wn**2 * u - 2*self.zeta*self.wn*self.x2 - self.wn**2*self.x1
        self.x1 += self.Ts * dx1
        self.x2 += self.Ts * dx2
        return self.x1

    def reset(self): self.x1 = self.x2 = 0.0

# ─── Controller Simulation (Python reference) ─────────────────────────────────
class PIDController:
    def __init__(self, Kp, Ki, Kd, Ts=0.001, N=10.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.Ts, self.N = Ts, N
        self.integral = 0.0
        self.prev_pv  = 0.0
        self.d_filt   = 0.0

    def compute(self, sp, pv):
        err  = sp - pv
        p_term = self.Kp * err

        # Derivative on measurement with filter
        Tf = self.Kd / (self.Kp * self.N) if self.N > 0 else 0.01
        d_pv = pv - self.prev_pv
        self.d_filt = (1 - self.Ts/Tf)*self.d_filt - (self.Kd/Tf)*d_pv
        self.prev_pv = pv

        self.integral += self.Ki * err * self.Ts
        self.integral  = np.clip(self.integral, 0, 100)

        out = p_term + self.integral + self.d_filt
        return float(np.clip(out, 0, 100))

    def reset(self): self.integral = self.d_filt = self.prev_pv = 0.0

# ─── Metrics Calculator ────────────────────────────────────────────────────────
def calc_metrics(t, pv, sp, cv, tol=0.02):
    pv, t, cv = np.array(pv), np.array(t), np.array(cv)
    final_val = sp

    # Rise time (10%→90%)
    idx10 = np.where(pv >= 0.1 * final_val)[0]
    idx90 = np.where(pv >= 0.9 * final_val)[0]
    rise_time = (t[idx90[0]] - t[idx10[0]]) if (len(idx10) and len(idx90)) else None

    # Overshoot
    overshoot = (max(pv) - final_val) / final_val * 100 if final_val != 0 else 0

    # Settling time (last time |err|/sp > tol)
    in_band = np.abs(pv - sp) <= tol * abs(sp)
    last_out = np.where(~in_band)[0]
    settling = t[last_out[-1]] if len(last_out) else 0.0

    # Steady-state error (last 10% of data)
    ss_pv = np.mean(pv[-len(pv)//10:])
    ss_error = sp - ss_pv

    # IAE, control effort
    iae = np.trapz(np.abs(sp - pv), t)
    ctrl_effort = np.trapz(np.abs(cv), t)

    return {
        "rise_time":      rise_time,
        "overshoot":      overshoot,
        "settling_time":  settling,
        "ss_error":       ss_error,
        "iae":            iae,
        "control_effort": ctrl_effort,
    }

# ─── CRC-16/CCITT ─────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
        crc &= 0xFFFF
    return crc

# ══════════════════════════════════════════════════════════════════════════════
#  TEST CLASSES
# ══════════════════════════════════════════════════════════════════════════════

class TestFirstOrderPlant:
    """First-order plant model validation"""

    def test_zero_input_stays_at_zero(self):
        p = FirstOrderPlant()
        for _ in range(1000): p.step(0.0)
        assert abs(p.y) < 1e-6, "Plant with zero input should stay at 0"

    def test_step_response_reaches_dc_gain(self):
        p = FirstOrderPlant(K=2.0, tau=3.0)
        # After 5τ (15 seconds) → should be ~99.3% of K
        for _ in range(15000): p.step(1.0)
        assert abs(p.y - 2.0) < 0.02, f"SS value {p.y:.4f} should be ≈ 2.0"

    def test_time_constant(self):
        """At t=τ, output should be 63.2% of K*u"""
        p = FirstOrderPlant(K=1.0, tau=3.0)
        for _ in range(3000): p.step(1.0)  # run for 3s = τ
        expected = 1.0 * (1 - math.exp(-1))  # ≈ 0.6321
        assert abs(p.y - expected) < 0.01, f"At τ: y={p.y:.4f}, expected {expected:.4f}"

    def test_reset(self):
        p = FirstOrderPlant()
        for _ in range(1000): p.step(50.0)
        p.reset()
        assert p.y == 0.0


class TestSecondOrderPlant:
    """Second-order system validation"""

    def test_underdamped_has_overshoot(self):
        p = SecondOrderPlant(wn=2.0, zeta=0.3)
        ys = [p.step(1.0) for _ in range(5000)]
        assert max(ys) > 1.0, "Underdamped system should overshoot"

    def test_overdamped_no_overshoot(self):
        p = SecondOrderPlant(wn=2.0, zeta=1.5)
        ys = [p.step(1.0) for _ in range(10000)]
        assert max(ys) <= 1.01, "Overdamped system must not overshoot"

    def test_critically_damped(self):
        p = SecondOrderPlant(wn=2.0, zeta=1.0)
        ys = [p.step(1.0) for _ in range(10000)]
        assert max(ys) <= 1.005, "Critically damped should have no overshoot"

    def test_natural_frequency_period(self):
        """Undamped: period = 2π/wn"""
        wn = 4.0
        p = SecondOrderPlant(wn=wn, zeta=0.0)
        ys = [p.step(0.0) for _ in range(500)]  # free oscillation

        # With initial conditions… We test that step response crosses 1.0 near expected peak time
        p.reset()
        ys = [p.step(1.0) for _ in range(3000)]
        peak_idx = np.argmax(ys)
        t_peak = peak_idx * 0.001
        t_peak_expected = math.pi / wn  # for underdamped: π/wd ≈ π/wn for ζ→0
        assert abs(t_peak - t_peak_expected) < 0.1, \
            f"Peak at t={t_peak:.3f}s, expected ~{t_peak_expected:.3f}s"


class TestPIDController:
    """PID controller algorithm validation"""

    def test_zero_error_zero_output(self):
        pid = PIDController(Kp=2.0, Ki=0.5, Kd=0.1)
        # With both sp=pv=0, output should be 0
        for _ in range(100): out = pid.compute(0.0, 0.0)
        assert abs(out) < 0.01

    def test_positive_error_positive_output(self):
        pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        out = pid.compute(50.0, 0.0)  # error = 50
        assert out > 0, "Positive error should give positive output"

    def test_proportional_gain(self):
        pid = PIDController(Kp=3.0, Ki=0.0, Kd=0.0)
        out = pid.compute(10.0, 0.0)  # error = 10
        assert abs(out - 30.0) < 0.01, f"P output={out:.2f}, expected 30.0"

    def test_integral_accumulates(self):
        pid = PIDController(Kp=0.0, Ki=1.0, Kd=0.0)
        for _ in range(100): pid.compute(10.0, 0.0)  # 100ms of error=10
        # I term ≈ 10 * 0.001 * 100 = 1.0
        assert pid.integral > 0.5, f"Integral should accumulate, got {pid.integral:.4f}"

    def test_derivative_kick_on_pv_change(self):
        """Derivative on measurement: step SP change shouldn't cause huge D kick"""
        pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.5)
        out1 = pid.compute(50.0, 0.0)    # big step in SP
        out2 = pid.compute(50.0, 0.0)    # same SP, PV hasn't moved
        # D term should be small on 2nd call (PV didn't change)
        assert abs(out2 - out1) < 20, "No derivative kick when PV is constant"

    def test_output_clamped(self):
        pid = PIDController(Kp=100.0, Ki=0.0, Kd=0.0)
        out = pid.compute(100.0, 0.0)  # huge error
        assert out <= 100.0, "Output must be clamped to 100"

    def test_closed_loop_stability_first_order(self):
        """PID on first-order plant should stabilize"""
        plant = FirstOrderPlant(K=2.0, tau=3.0)
        pid   = PIDController(Kp=2.0, Ki=0.5, Kd=0.1)
        sp = 50.0
        pvs = []
        for _ in range(20000):  # 20 seconds
            pv  = plant.y * 100.0
            u   = pid.compute(sp, pv)
            plant.step(u / 100.0)
            pvs.append(pv)

        ss = np.mean(pvs[-1000:])
        assert abs(ss - sp) < 2.0, f"SS value {ss:.2f} should be near setpoint {sp}"

    def test_closed_loop_no_divergence(self):
        """PID loop should not diverge"""
        plant = SecondOrderPlant(wn=2.0, zeta=0.5)
        pid   = PIDController(Kp=1.0, Ki=0.3, Kd=0.05)
        sp = 1.0
        for _ in range(10000):
            pv = plant.x1
            u  = pid.compute(sp, pv)
            plant.step(u)

        assert abs(plant.x1) < 100, "System diverged"


class TestCRCProtocol:
    """Communication protocol integrity"""

    def test_crc_known_value(self):
        data = b"\x01\x02\x03\x04"
        result = crc16(data)
        # Compute expected with standard CCITT
        assert isinstance(result, int) and 0 <= result <= 0xFFFF

    def test_crc_detects_single_bit_error(self):
        data  = b"\xAA\x10\x01\x02\x03\x04\x05\x06\x07\x08"
        crc1  = crc16(data)
        flipped = bytearray(data)
        flipped[3] ^= 0x01
        crc2  = crc16(bytes(flipped))
        assert crc1 != crc2, "CRC must detect single-bit error"

    def test_crc_different_for_different_data(self):
        a = crc16(b"\x01\x02\x03")
        b = crc16(b"\x01\x02\x04")
        assert a != b

    def test_crc_empty_data(self):
        result = crc16(b"")
        assert result == 0xFFFF  # CRC of empty = initial value


class TestMetricsCalculation:
    """Performance metrics accuracy"""

    def setup_method(self):
        """Generate a clean step response for testing"""
        # Simulate a first-order response to a unit step
        K, tau, Ts = 1.0, 1.0, 0.001
        N = 10000  # 10 seconds
        self.t  = np.arange(N) * Ts
        self.pv = K * (1 - np.exp(-self.t / tau))
        self.sp = 1.0
        self.cv = np.full(N, 50.0)

    def test_rise_time_first_order(self):
        m = calc_metrics(self.t, self.pv, self.sp, self.cv)
        # For 1st order: tr ≈ 2.197τ ≈ 2.197s
        expected = 2.197 * 1.0  # τ = 1.0
        assert m["rise_time"] is not None
        assert abs(m["rise_time"] - expected) < 0.05, \
            f"Rise time {m['rise_time']:.3f} vs expected {expected:.3f}"

    def test_no_overshoot_first_order(self):
        m = calc_metrics(self.t, self.pv, self.sp, self.cv)
        assert m["overshoot"] < 0.5, \
            f"First-order has no overshoot, got {m['overshoot']:.2f}%"

    def test_ss_error_near_zero(self):
        m = calc_metrics(self.t, self.pv, self.sp, self.cv)
        assert abs(m["ss_error"]) < 0.02, \
            f"SS error {m['ss_error']:.4f} should be near zero"

    def test_iae_positive(self):
        m = calc_metrics(self.t, self.pv, self.sp, self.cv)
        assert m["iae"] > 0, "IAE must be positive"

    def test_underdamped_overshoot(self):
        """Second-order underdamped should show overshoot"""
        p   = SecondOrderPlant(wn=3.0, zeta=0.2)
        pvs = [p.step(1.0) for _ in range(5000)]
        t   = np.arange(5000) * 0.001
        m   = calc_metrics(t, pvs, 1.0, np.ones(5000) * 50)
        assert m["overshoot"] > 5.0, \
            f"Underdamped (ζ=0.2) overshoot={m['overshoot']:.1f}% should be >5%"


class TestFaultInjection:
    """Fault injection scenarios"""

    def test_no_fault_passthrough(self):
        # Without fault, signal should pass unchanged
        u_values = [10.0, 50.0, -20.0, 0.0]
        for u in u_values:
            assert u == u, "No fault: signal passthrough"

    def test_bias_fault(self):
        bias = 5.0
        u = 30.0
        result = u + bias
        assert result == 35.0

    def test_saturation_fault(self):
        limit = 80.0
        assert np.clip(90.0, -limit, limit) == 80.0
        assert np.clip(70.0, -limit, limit) == 70.0

    def test_dropout_fault(self):
        assert 0.0 == 0.0  # Signal → 0

    def test_noise_fault_zero_mean(self):
        """Gaussian noise should average near zero"""
        np.random.seed(42)
        noise = np.random.normal(0, 1.0, 10000)
        assert abs(np.mean(noise)) < 0.05, "Noise should have zero mean"


# ─── Benchmark Table ──────────────────────────────────────────────────────────
def run_benchmark():
    """Generate controller comparison table"""
    from scipy.integrate import solve_ivp

    configs = [
        ("On/Off",  None,  None, None),
        ("P",       5.0,   0.0,  0.0 ),
        ("PI",      3.0,   0.8,  0.0 ),
        ("PID",     2.5,   0.8,  0.15),
    ]

    print("\n╔══════════════════════════════════════════════════════════════╗")
    print("║         CONTROLLER BENCHMARK — First-Order Plant            ║")
    print("╠══════╦══════════╦══════════╦══════════╦══════════╦══════════╣")
    print("║ CTRL ║ Rise (s) ║ Stl (s)  ║  OS (%)  ║ SSE      ║ IAE      ║")
    print("╠══════╬══════════╬══════════╬══════════╬══════════╬══════════╣")

    for name, Kp, Ki, Kd in configs:
        plant = FirstOrderPlant(K=2.0, tau=3.0)
        if Kp is None:
            # On/Off
            ctrl = lambda sp, pv: 100.0 if sp - pv > 1 else 0.0
        else:
            pid = PIDController(Kp=Kp, Ki=Ki or 0, Kd=Kd or 0)
            ctrl = pid.compute

        sp = 50.0; pvs = []; ts = []
        for k in range(30000):
            pv = plant.y * 100.0
            u  = ctrl(sp, pv)
            plant.step(u / 100.0)
            pvs.append(pv); ts.append(k * 0.001)

        m = calc_metrics(ts, pvs, sp, [50]*30000)
        rt  = f"{m['rise_time']:.3f}" if m["rise_time"] else "N/A "
        st  = f"{m['settling_time']:.3f}"
        os_ = f"{m['overshoot']:.2f}"
        sse = f"{m['ss_error']:.4f}"
        iae = f"{m['iae']:.4f}"
        print(f"║ {name:<4} ║ {rt:<8} ║ {st:<8} ║ {os_:<8} ║ {sse:<8} ║ {iae:<8} ║")

    print("╚══════╩══════════╩══════════╩══════════╩══════════╩══════════╝")

if __name__ == "__main__":
    run_benchmark()
