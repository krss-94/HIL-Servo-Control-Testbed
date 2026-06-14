#!/usr/bin/env python3
"""
HIL Testbed — Advanced Control Features
════════════════════════════════════════
Implements three high-impact advanced control features:

Feature 1 — Kalman Filter State Estimator
  Reduces sensor noise impact before the PID controller sees it.
  Uses a linear discrete Kalman filter (steady-state / Luenberger form
  is sufficient for scalar systems; full form included for generality).

Feature 2 — Ziegler-Nichols Relay Feedback Auto-Tuner
  Drives the plant into limit-cycle oscillation via a relay (on-off)
  feedback element, measures the ultimate gain Ku and period Tu, and
  automatically computes PID gains using Z-N rules.
  No manual trial-and-error required.

Feature 3 — Self-Tuning Adaptive PID (MIT Gradient Rule)
  Continuously updates Kp online using a gradient-descent law on the
  instantaneous squared error. Ki and Kd are kept at nominal values to
  maintain stability. This handles slowly varying plant gains (e.g.
  temperature-dependent motor resistance, fluid viscosity changes).

Usage (standalone demo):
    python advanced_controllers.py

Usage (integrate with benchmark.py):
    from advanced_controllers import KalmanFilter, RelayAutoTuner, AdaptivePID

Author: HIL Testbed Project
License: MIT
"""

from __future__ import annotations
import math
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np

# ──────────────────────────────────────────────────────────────────────────────
#  Helper — First-Order Plant (mirrors PlantModels.h FirstOrderState)
# ──────────────────────────────────────────────────────────────────────────────
class _FirstOrderPlant:
    def __init__(self, K=2.0, tau=3.0, ts=0.001, noise_std=0.0):
        self.K, self.tau, self.ts = K, tau, ts
        self.noise_std = noise_std
        self.y = 0.0

    def step(self, u: float) -> float:
        self.y += (self.ts / self.tau) * (self.K * u - self.y)
        noise = np.random.normal(0, self.noise_std) if self.noise_std > 0 else 0.0
        return self.y * 100.0 + noise   # % + noise

    def reset(self): self.y = 0.0


# ══════════════════════════════════════════════════════════════════════════════
#  FEATURE 1 — KALMAN FILTER STATE ESTIMATOR
# ══════════════════════════════════════════════════════════════════════════════

class KalmanFilter:
    """
    Discrete Scalar Kalman Filter for 1-D process variable estimation.

    Models the sensor measurement as:
        x[k+1] = A·x[k] + B·u[k]   (process model)
        z[k]   = H·x[k] + v[k]      (measurement model)

    where:
        A = 1 - Ts/tau  (first-order discrete-time state transition)
        B = K·Ts/tau
        H = 1
        Q = process noise variance
        R = measurement noise variance

    The filter alternates between Predict and Update steps each timestep.

    Parameters
    ----------
    Q : float
        Process noise variance (tune lower to trust model more).
    R : float
        Measurement noise variance (tune to actual sensor noise power).
    tau : float
        Plant time constant used for A and B matrices.
    K : float
        Plant DC gain (used in B matrix).
    ts : float
        Sample period [s].

    Usage
    -----
    kf = KalmanFilter(Q=0.01, R=9.0, tau=3.0, K=2.0)
    for each sample:
        x_est = kf.update(u=cv, z=pv_noisy)
    """

    def __init__(
        self,
        Q:   float = 0.01,   # process noise
        R:   float = 9.0,    # measurement noise (≈ noise_std²)
        tau: float = 3.0,
        K:   float = 2.0,
        ts:  float = 0.001,
    ):
        self.A  = 1.0 - ts / tau        # state transition
        self.B  = K * ts / tau          # input gain
        self.H  = 1.0                   # observation
        self.Q  = Q                     # process noise covariance
        self.R  = R                     # measurement noise covariance

        # Initial state and covariance
        self.x_hat: float = 0.0        # estimated state
        self.P:     float = 1.0        # estimated covariance

    # ── Core algorithm ────────────────────────────────────────────────────────

    def predict(self, u: float) -> None:
        """Predict step: propagate state and covariance forward."""
        self.x_hat = self.A * self.x_hat + self.B * u
        self.P     = self.A**2 * self.P + self.Q

    def correct(self, z: float) -> float:
        """Update step: incorporate measurement z; return filtered estimate."""
        # Kalman gain
        S = self.H**2 * self.P + self.R
        K_gain = self.P * self.H / S

        # State update
        innovation = z - self.H * self.x_hat
        self.x_hat = self.x_hat + K_gain * innovation

        # Covariance update (Joseph form for numerical stability)
        self.P = (1.0 - K_gain * self.H)**2 * self.P + K_gain**2 * self.R

        return self.x_hat

    def update(self, u: float, z: float) -> float:
        """
        One full Kalman cycle: predict then correct.

        Parameters
        ----------
        u : control input (used for state prediction)
        z : noisy measurement

        Returns
        -------
        Filtered state estimate
        """
        self.predict(u)
        return self.correct(z)

    def reset(self) -> None:
        self.x_hat = 0.0
        self.P     = 1.0

    @property
    def estimate(self) -> float:
        """Current state estimate (read-only)."""
        return self.x_hat


# ══════════════════════════════════════════════════════════════════════════════
#  FEATURE 2 — ZIEGLER-NICHOLS RELAY FEEDBACK AUTO-TUNER
# ══════════════════════════════════════════════════════════════════════════════

class RelayAutoTuner:
    """
    Åström-Hägglund Relay Feedback Auto-Tuner.

    Algorithm
    ---------
    1. Replace the feedback controller with a relay (on-off element)
       of amplitude d.
    2. The closed loop enters limit-cycle oscillation at the ultimate
       frequency of the plant.
    3. From the relay output amplitude d and PV oscillation amplitude a:
           Ku = 4d / (π·a)
    4. The period of the limit cycle gives the ultimate period Tu.
    5. Apply Ziegler-Nichols rules to compute PID gains.

    Parameters
    ----------
    relay_amplitude : float
        Relay switching amplitude d  (% of CV range).
    setpoint : float
        Operating point setpoint [% full scale].
    min_cycles : int
        Minimum number of full oscillation cycles before terminating.
    timeout_s : float
        Maximum auto-tune duration [seconds of sim time].

    Usage
    -----
    tuner = RelayAutoTuner(relay_amplitude=5.0, setpoint=50.0)
    while not tuner.done:
        pv = plant.step(tuner.compute(pv))
    kp, ki, kd = tuner.get_pid_gains()
    """

    def __init__(
        self,
        relay_amplitude: float = 5.0,
        setpoint:        float = 50.0,
        min_cycles:      int   = 4,
        timeout_s:       float = 120.0,
        ts:              float = 0.001,
    ):
        self.d          = relay_amplitude
        self.sp         = setpoint
        self.min_cycles = min_cycles
        self.timeout    = timeout_s
        self.ts         = ts

        # Internal state
        self._output: float        = self.d    # start high
        self._prev_err: float      = 0.0
        self._zero_crossings: list = []        # timestamps of zero-crossings
        self._pk_values: list      = []        # |pk-pk| of each half-cycle
        self._half_peak: float     = 0.0
        self._elapsed:  float      = 0.0
        self.done:      bool       = False

        # Results
        self.Ku: float = 0.0
        self.Tu: float = 0.0

    def compute(self, pv: float) -> float:
        """Return relay output; call once per timestep."""
        if self.done:
            return self.sp   # hold at setpoint after tuning

        err = self.sp - pv
        self._elapsed += self.ts

        # Detect zero crossing (error sign change)
        if self._prev_err * err < 0:
            t = self._elapsed
            self._zero_crossings.append(t)
            self._pk_values.append(abs(self._half_peak))
            self._half_peak = 0.0

            # Check completion
            if len(self._zero_crossings) >= 2 * self.min_cycles:
                self._finalize()

        # Track peak within current half-cycle
        if abs(err) > abs(self._half_peak):
            self._half_peak = err

        # Relay switching
        self._output = self.d if err > 0 else -self.d
        self._prev_err = err

        # Timeout guard
        if self._elapsed >= self.timeout:
            self._finalize()

        return self.sp + self._output   # relay output centred on setpoint

    def _finalize(self) -> None:
        """Compute Ku and Tu from accumulated data."""
        if len(self._pk_values) < 2:
            self.done = True
            return

        # Amplitude: mean of oscillation half-amplitudes
        a = float(np.mean(self._pk_values[-self.min_cycles * 2:]))
        if a < 1e-6:
            self.done = True
            return

        # Ku from describing function
        self.Ku = 4.0 * self.d / (math.pi * a)

        # Tu from period between zero-crossings (two crossings = one half period)
        crossings = self._zero_crossings[-self.min_cycles * 2:]
        half_periods = np.diff(crossings)
        self.Tu = float(np.mean(half_periods)) * 2.0

        self.done = True

    def get_pid_gains(
        self,
        method: str = "classic",
    ) -> Tuple[float, float, float]:
        """
        Compute PID gains from Ku and Tu.

        Parameters
        ----------
        method : "classic" | "no_overshoot" | "some_overshoot"

        Returns
        -------
        (Kp, Ki, Kd)
        """
        if not self.done or self.Ku == 0 or self.Tu == 0:
            raise RuntimeError("Auto-tuner has not completed. Run the relay test first.")

        Ku, Tu = self.Ku, self.Tu

        if method == "classic":
            Kp = 0.60 * Ku
            Ki = 1.20 * Ku / Tu
            Kd = 0.075 * Ku * Tu
        elif method == "no_overshoot":
            Kp = 0.20 * Ku
            Ki = 0.40 * Ku / Tu
            Kd = 0.066 * Ku * Tu
        elif method == "some_overshoot":
            Kp = 0.33 * Ku
            Ki = 0.66 * Ku / Tu
            Kd = 0.11 * Ku * Tu
        else:
            raise ValueError(f"Unknown Z-N method: {method}")

        return Kp, Ki, Kd

    def get_pi_gains(self) -> Tuple[float, float]:
        Ku, Tu = self.Ku, self.Tu
        return 0.45 * Ku, 0.54 * Ku / Tu

    @property
    def summary(self) -> str:
        return (
            f"RelayAutoTuner: Ku={self.Ku:.4f}  Tu={self.Tu:.4f} s  "
            f"Elapsed={self._elapsed:.1f} s  "
            f"Crossings={len(self._zero_crossings)}"
        )


# ══════════════════════════════════════════════════════════════════════════════
#  FEATURE 3 — SELF-TUNING ADAPTIVE PID (MIT Gradient Rule)
# ══════════════════════════════════════════════════════════════════════════════

class AdaptivePID:
    """
    Self-Tuning Adaptive PID using the MIT Gradient Descent Rule.

    The MIT rule adjusts Kp online to minimise J = 0.5 · e²:

        dKp/dt = −γ · e · ∂e/∂Kp

    For a P-dominant loop, ∂y/∂Kp ≈ e (error driven by proportional action),
    giving:

        Kp[k] = Kp[k-1] + γ · e[k] · ∂y/∂Kp

    Ki and Kd adapt more slowly using smaller learning rates to maintain
    closed-loop stability. This is a simplified but production-grade
    implementation suitable for slowly varying plants.

    Parameters
    ----------
    kp0, ki0, kd0 : float
        Initial PID gains.
    gamma_p, gamma_i, gamma_d : float
        Learning rates. Typical range: 0.001–0.1.
        Too large → instability. Too small → slow adaptation.
    kp_bounds, ki_bounds, kd_bounds : (min, max)
        Hard bounds on each gain to prevent runaway.
    adapt_every : int
        Update gains every N timesteps (reduces sensitivity to noise).

    Usage
    -----
    apid = AdaptivePID(kp0=2.5, ki0=0.8, kd0=0.15)
    for each sample:
        cv = apid.compute(sp, pv)
    print(apid.gains)  # current adapted gains
    """

    def __init__(
        self,
        kp0: float = 2.5,
        ki0: float = 0.8,
        kd0: float = 0.15,
        gamma_p: float  = 0.005,
        gamma_i: float  = 0.001,
        gamma_d: float  = 0.0005,
        kp_bounds: Tuple[float, float] = (0.1, 20.0),
        ki_bounds: Tuple[float, float] = (0.0, 10.0),
        kd_bounds: Tuple[float, float] = (0.0,  2.0),
        adapt_every: int = 10,
        ts: float = 0.001,
        N:  float = 10.0,
        Kb: float = 0.1,
    ):
        self.kp = kp0
        self.ki = ki0
        self.kd = kd0

        self.gamma_p = gamma_p
        self.gamma_i = gamma_i
        self.gamma_d = gamma_d

        self.kp_bounds = kp_bounds
        self.ki_bounds = ki_bounds
        self.kd_bounds = kd_bounds

        self.adapt_every = adapt_every
        self.ts = ts
        self.N  = N
        self.Kb = Kb

        # Controller state (same as PIDController)
        self._integral    = 0.0
        self._prev_pv     = 0.0
        self._deriv_filt  = 0.0
        self._last_output = 0.0

        # Adaptation state
        self._n: int   = 0
        self._err_history = deque(maxlen=adapt_every)

        # Sensitivity model (approximate ∂y/∂Kp as a first-order IIR)
        self._sens_kp: float = 0.0
        self._sens_ki: float = 0.0
        self._sens_kd: float = 0.0

        # History for diagnostics
        self.kp_history: list = [kp0]
        self.ki_history: list = [ki0]
        self.kd_history: list = [kd0]

    # ── Core controller ───────────────────────────────────────────────────────

    def compute(self, sp: float, pv: float) -> float:
        """Compute control output and adapt gains."""
        err    = sp - pv
        p_term = self.kp * err

        # Derivative on measurement with filter
        Tf = (self.kd / (self.kp * self.N)) if (self.kp > 0 and self.N > 0) else 0.01
        dPV = pv - self._prev_pv
        self._deriv_filt = (1.0 - self.ts / Tf) * self._deriv_filt \
                           - (self.kd / Tf) * dPV
        self._prev_pv = pv

        raw_out   = p_term + self._integral + self._deriv_filt
        clamped   = float(np.clip(raw_out, 0.0, 100.0))
        sat_error = clamped - raw_out

        # Anti-windup back-calculation
        self._integral    += (self.ki * err + self.Kb * sat_error) * self.ts
        self._last_output  = clamped

        self._n += 1
        self._err_history.append(err)

        # Adapt gains every `adapt_every` steps
        if self._n % self.adapt_every == 0:
            self._adapt(err, pv)

        return clamped

    # ── MIT gradient adaptation ───────────────────────────────────────────────

    def _adapt(self, err: float, pv: float) -> None:
        """Update gains using smoothed gradient estimate."""
        # Mean error over the adaptation window (less noise sensitive)
        e_mean = float(np.mean(self._err_history))

        # Sensitivity signal ∂y/∂K: first-order recursive model
        # ∂y/∂Kp ≈ err (proportional sensitivity)
        # ∂y/∂Ki ≈ ∫e dt  (integral sensitivity)
        # ∂y/∂Kd ≈ de/dt  (derivative sensitivity)
        alpha = 0.9   # IIR decay (smoothing)
        self._sens_kp = alpha * self._sens_kp + (1 - alpha) * abs(e_mean)
        self._sens_ki = alpha * self._sens_ki + (1 - alpha) * self._integral
        self._sens_kd = alpha * self._sens_kd + (1 - alpha) * abs(self._deriv_filt)

        # Gradient descent: Δθ = -γ · e · sensitivity
        # Sign: positive error → increase kp to reduce error faster
        delta_kp =  self.gamma_p * e_mean * self._sens_kp
        delta_ki =  self.gamma_i * e_mean * self._sens_ki
        delta_kd =  self.gamma_d * e_mean * self._sens_kd

        self.kp = float(np.clip(self.kp + delta_kp, *self.kp_bounds))
        self.ki = float(np.clip(self.ki + delta_ki, *self.ki_bounds))
        self.kd = float(np.clip(self.kd + delta_kd, *self.kd_bounds))

        self.kp_history.append(self.kp)
        self.ki_history.append(self.ki)
        self.kd_history.append(self.kd)

    # ── Diagnostics ───────────────────────────────────────────────────────────

    @property
    def gains(self) -> dict:
        return {"kp": round(self.kp, 4), "ki": round(self.ki, 4), "kd": round(self.kd, 4)}

    def reset(self) -> None:
        self._integral = self._deriv_filt = self._prev_pv = self._last_output = 0.0
        self._sens_kp = self._sens_ki = self._sens_kd = 0.0
        self._err_history.clear()
        self._n = 0


# ══════════════════════════════════════════════════════════════════════════════
#  DEMONSTRATION
# ══════════════════════════════════════════════════════════════════════════════

def demo_kalman(noise_std=3.0, ts=0.001, sim_time=20.0):
    """Show Kalman filter noise reduction on a first-order plant."""
    print("\n" + "═" * 60)
    print("  DEMO 1 — Kalman Filter State Estimator")
    print("═" * 60)

    np.random.seed(42)
    plant  = _FirstOrderPlant(K=2.0, tau=3.0, ts=ts, noise_std=noise_std)
    kf     = KalmanFilter(Q=0.01, R=noise_std**2, tau=3.0, K=2.0, ts=ts)

    n      = int(sim_time / ts)
    sp     = 50.0
    raw_mse  = 0.0
    filt_mse = 0.0

    for k in range(n):
        cv      = 100.0   # open-loop step
        pv_noisy = plant.step(cv)
        pv_true  = plant.y * 100.0   # noiseless truth
        pv_filt  = kf.update(u=cv, z=pv_noisy)

        raw_mse  += (pv_noisy - pv_true)**2
        filt_mse += (pv_filt  - pv_true)**2

    raw_rmse  = math.sqrt(raw_mse  / n)
    filt_rmse = math.sqrt(filt_mse / n)
    reduction = (1.0 - filt_rmse / raw_rmse) * 100.0

    print(f"  Noise std dev    : {noise_std:.1f} %")
    print(f"  Raw RMSE         : {raw_rmse:.4f} %")
    print(f"  Filtered RMSE    : {filt_rmse:.4f} %")
    print(f"  Noise reduction  : {reduction:.1f} %")
    print(f"  Kalman gain K∞   : {kf.P * kf.H / (kf.H**2 * kf.P + kf.R):.4f}")


def demo_auto_tuner(ts=0.001):
    """Run relay auto-tuner on first-order plant and print Z-N gains."""
    print("\n" + "═" * 60)
    print("  DEMO 2 — Ziegler-Nichols Relay Feedback Auto-Tuner")
    print("═" * 60)

    plant  = _FirstOrderPlant(K=2.0, tau=3.0, ts=ts)
    tuner  = RelayAutoTuner(relay_amplitude=5.0, setpoint=50.0,
                             min_cycles=5, ts=ts, timeout_s=200.0)

    steps  = 0
    while not tuner.done:
        pv  = plant.step(tuner.compute(plant.y * 100.0))
        steps += 1

    print(f"  {tuner.summary}")
    print(f"  Simulation steps : {steps}  ({steps * ts:.1f} s)")

    for method in ("classic", "no_overshoot", "some_overshoot"):
        kp, ki, kd = tuner.get_pid_gains(method)
        print(f"\n  Z-N [{method}]:")
        print(f"    Kp = {kp:.4f}   Ki = {ki:.4f}   Kd = {kd:.4f}")


def demo_adaptive_pid(ts=0.001, sim_time=60.0):
    """Run adaptive PID on a plant whose gain suddenly doubles at t=30s."""
    print("\n" + "═" * 60)
    print("  DEMO 3 — Self-Tuning Adaptive PID (MIT Rule)")
    print("═" * 60)

    np.random.seed(7)
    sp  = 50.0
    ts  = 0.001
    n   = int(sim_time / ts)

    apid = AdaptivePID(kp0=2.5, ki0=0.8, kd0=0.15,
                       gamma_p=0.004, gamma_i=0.0008, gamma_d=0.0002)

    sse_fixed_list   = []   # reference: non-adaptive PID
    sse_adaptive_list = []

    # Non-adaptive reference
    from benchmark import PIDController as StaticPID
    static = StaticPID(kp=2.5, ki=0.8, kd=0.15, ts=ts)

    plant_fast  = _FirstOrderPlant(K=4.0, tau=3.0, ts=ts)  # after gain change
    plant_slow  = _FirstOrderPlant(K=2.0, tau=3.0, ts=ts)  # normal
    plant_fast2 = _FirstOrderPlant(K=4.0, tau=3.0, ts=ts)

    for k in range(n):
        t    = k * ts
        # At t=30s, plant gain doubles (simulates load change)
        pv_adapt  = plant_slow.step(apid.compute(sp, plant_slow.y * 100.0)) \
                    if t < 30 else plant_fast.step(apid.compute(sp, plant_fast.y * 100.0))
        pv_static = plant_slow.step(static.compute(sp, plant_slow.y * 100.0)) \
                    if t < 30 else plant_fast2.step(static.compute(sp, plant_fast2.y * 100.0))

        if t >= 31.0:
            sse_adaptive_list.append(sp - pv_adapt)
            sse_fixed_list.append(sp - pv_static)

    sse_adapt  = float(np.mean(np.abs(sse_adaptive_list)))
    sse_fixed  = float(np.mean(np.abs(sse_fixed_list)))
    improvement = (1.0 - sse_adapt / sse_fixed) * 100.0 if sse_fixed > 0 else 0

    final_gains = apid.gains
    print(f"  Plant gain change at t=30 s: K=2.0 → K=4.0")
    print(f"  Mean |SSE| (static PID)  : {sse_fixed:.4f} %")
    print(f"  Mean |SSE| (adaptive PID): {sse_adapt:.4f} %")
    print(f"  SSE improvement          : {improvement:.1f} %")
    print(f"  Final adapted gains      : Kp={final_gains['kp']}  "
          f"Ki={final_gains['ki']}  Kd={final_gains['kd']}")
    print(f"  Adaptation steps recorded: {len(apid.kp_history)}")


if __name__ == "__main__":
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   HIL Testbed — Advanced Control Features Demo           ║")
    print("╚══════════════════════════════════════════════════════════╝")

    t0 = time.perf_counter()
    demo_kalman()
    demo_auto_tuner()
    demo_adaptive_pid()
    print(f"\n  All demos complete in {time.perf_counter()-t0:.2f} s")
