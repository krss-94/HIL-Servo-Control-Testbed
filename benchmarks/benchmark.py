#!/usr/bin/env python3
"""
HIL Testbed — Automated Benchmark Framework
════════════════════════════════════════════
Runs every controller × plant combination in simulation, computes all
performance indices, exports CSV results, and prints formatted tables.

Metrics computed:
  Rise Time       — 10 % → 90 % of setpoint [s]
  Settling Time   — last exit from ±2 % band [s]
  Overshoot       — (peak − SP) / SP × 100 [%]
  Steady-State Error — SP − mean(last 10 % of data)
  IAE             — Integral Absolute Error  [unit·s]
  ITSE            — Integral Time-Squared Error [unit·s³]
  Control Effort  — ∫|u(t)| dt  [%·s]

Usage:
  python benchmark.py                     # full benchmark, all combos
  python benchmark.py --plant servo       # single plant
  python benchmark.py --ctrl pid          # single controller
  python benchmark.py --setpoint 60       # custom setpoint
  python benchmark.py --sim-time 40       # simulation duration [s]
  python benchmark.py --export results/   # custom CSV output directory

Author: HIL Testbed Project
License: MIT
"""

from __future__ import annotations
import argparse
import csv
import math
import os
import time
from dataclasses import dataclass, fields, asdict
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

# ─── Simulation Constants ─────────────────────────────────────────────────────
TS          = 0.001          # sample period [s]
DEFAULT_SP  = 50.0           # setpoint [% of full scale]
DEFAULT_SIM = 30.0           # simulation duration [s]

# ─── Performance Metrics Dataclass ────────────────────────────────────────────
@dataclass
class PerformanceMetrics:
    controller:     str   = ""
    plant:          str   = ""
    setpoint:       float = 0.0
    kp:             float = 0.0
    ki:             float = 0.0
    kd:             float = 0.0
    rise_time:      float = float("nan")
    settling_time:  float = float("nan")
    overshoot_pct:  float = 0.0
    ss_error:       float = 0.0
    iae:            float = 0.0
    itse:           float = 0.0
    control_effort: float = 0.0
    sim_time_s:     float = 0.0
    samples:        int   = 0
    stable:         bool  = True


# ══════════════════════════════════════════════════════════════════════════════
#  METRICS ENGINE
# ══════════════════════════════════════════════════════════════════════════════

class MetricsEngine:
    """
    Real-time performance metrics calculator.

    Computes classical control performance indices incrementally as each
    sample arrives. Call ``update(t, sp, pv, u)`` once per timestep, then
    retrieve results with ``get_metrics()``.

    All calculations mirror the on-MCU MetricsEngine in FaultInjector.h.
    """

    def __init__(self, ts: float = TS, tol: float = 0.02):
        self.ts  = ts
        self.tol = tol          # settling band (fraction of SP)
        self.reset()

    # ── Public interface ──────────────────────────────────────────────────────

    def reset(self) -> None:
        self._n            = 0
        self._t_arr:  List[float] = []
        self._pv_arr: List[float] = []
        self._u_arr:  List[float] = []

        self._sp           = 0.0
        self._peak_pv      = -math.inf
        self._sum_abs_err  = 0.0
        self._sum_ctrl_eff = 0.0
        self._itse_acc     = 0.0

        # Rise-time state
        self._rt10_t:  Optional[float] = None
        self._rt90_t:  Optional[float] = None
        self._rise_time: Optional[float] = None

        # Settling-time: last sample index outside the 2 % band
        self._last_out_of_band = 0

    def update(self, t: float, sp: float, pv: float, u: float) -> None:
        """Feed one sample to the metrics engine."""
        self._n  += 1
        self._sp  = sp
        err       = sp - pv

        self._t_arr.append(t)
        self._pv_arr.append(pv)
        self._u_arr.append(u)

        # Accumulate integrals (trapezoidal where cheap to do so)
        self._sum_abs_err  += abs(err) * self.ts
        self._sum_ctrl_eff += abs(u)   * self.ts
        self._itse_acc     += t * err * err * self.ts

        # Peak value (for overshoot)
        if pv > self._peak_pv:
            self._peak_pv = pv

        # Rise time (10 % → 90 % of SP)
        if self._rt10_t is None and pv >= 0.10 * sp:
            self._rt10_t = t
        if self._rt10_t is not None and self._rt90_t is None and pv >= 0.90 * sp:
            self._rt90_t = t
            self._rise_time = self._rt90_t - self._rt10_t

        # Settling time: record last sample outside the ±tol band
        if sp != 0.0 and abs(err) > self.tol * abs(sp):
            self._last_out_of_band = self._n

    def get_metrics(self) -> dict:
        """Return a dictionary of all computed performance indices."""
        t   = np.array(self._t_arr)
        pv  = np.array(self._pv_arr)
        sp  = self._sp
        n   = self._n

        # Steady-state error: mean of last 10 % of data
        tail = max(1, n // 10)
        ss_pv     = float(np.mean(pv[-tail:]))
        ss_error  = sp - ss_pv

        # Overshoot
        overshoot = 0.0
        if sp > 0 and self._peak_pv > sp:
            overshoot = (self._peak_pv - sp) / sp * 100.0

        # ITSE via numpy for accuracy
        itse = float(np.trapz(t * (sp - pv) ** 2, t)) if n > 1 else 0.0

        return {
            "rise_time":      self._rise_time,
            "settling_time":  self._last_out_of_band * self.ts,
            "overshoot_pct":  overshoot,
            "ss_error":       ss_error,
            "iae":            self._sum_abs_err,
            "itse":           itse,
            "control_effort": self._sum_ctrl_eff,
            "samples":        n,
            "stable":         abs(ss_error) < 0.05 * abs(sp) if sp != 0 else True,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  PLANT MODELS
# ══════════════════════════════════════════════════════════════════════════════

class FirstOrderPlant:
    """G(s) = K / (τs + 1) — Euler integration"""
    name = "first_order"
    label = "1st-Order  (K=2, τ=3s)"

    def __init__(self, K=2.0, tau=3.0, ts=TS):
        self.K, self.tau, self.ts = K, tau, ts
        self.y = 0.0

    def step(self, u: float) -> float:
        self.y += (self.ts / self.tau) * (self.K * u - self.y)
        return self.y * 100.0          # normalise to %

    def reset(self): self.y = 0.0


class SecondOrderPlant:
    """G(s) = ωn² / (s² + 2ζωns + ωn²) — RK4 integration"""
    name = "second_order"
    label = "2nd-Order  (ωn=2, ζ=0.5)"

    def __init__(self, wn=2.0, zeta=0.5, ts=TS):
        self.wn, self.zeta, self.ts = wn, zeta, ts
        self.x1 = self.x2 = 0.0

    def _deriv(self, x1, x2, u):
        dx1 = x2
        dx2 = self.wn**2 * u - 2 * self.zeta * self.wn * x2 - self.wn**2 * x1
        return dx1, dx2

    def step(self, u: float) -> float:
        h = self.ts
        k1a, k1b = self._deriv(self.x1, self.x2, u)
        k2a, k2b = self._deriv(self.x1 + .5*h*k1a, self.x2 + .5*h*k1b, u)
        k3a, k3b = self._deriv(self.x1 + .5*h*k2a, self.x2 + .5*h*k2b, u)
        k4a, k4b = self._deriv(self.x1 + h*k3a,    self.x2 + h*k3b,    u)
        self.x1 += (h / 6.0) * (k1a + 2*k2a + 2*k3a + k4a)
        self.x2 += (h / 6.0) * (k1b + 2*k2b + 2*k3b + k4b)
        return np.clip(self.x1, 0, 1) * 100.0    # normalise to %

    def reset(self): self.x1 = self.x2 = 0.0


class ServoPlant:
    """J·α = Km·u − B·ω  → position"""
    name = "servo"
    label = "Servo  (J=0.01, B=0.05, Km=0.5)"

    def __init__(self, J=0.01, B=0.05, Km=0.5, maxTheta=180.0, ts=TS):
        self.J, self.B, self.Km, self.maxTheta, self.ts = J, B, Km, maxTheta, ts
        self.theta = self.omega = 0.0

    def step(self, u: float) -> float:
        # u is in %, map to torque range −1…+1
        u_norm   = u / 100.0
        torque   = self.Km * u_norm - self.B * self.omega
        alpha    = torque / self.J
        self.omega += self.ts * alpha
        self.theta += self.ts * self.omega * (180.0 / math.pi)
        self.theta = float(np.clip(self.theta, 0.0, self.maxTheta))
        if (self.theta <= 0.0 and self.omega < 0) or \
           (self.theta >= self.maxTheta and self.omega > 0):
            self.omega = 0.0
        return self.theta / self.maxTheta * 100.0

    def reset(self): self.theta = self.omega = 0.0


class TankPlant:
    """A·dh/dt = Qin − Cv·√h   (Torricelli)"""
    name = "tank"
    label = "Tank  (A=1.0, Cv=0.15, Qmax=0.3)"

    def __init__(self, A=1.0, Cv=0.15, Qmax=0.3, hMax=3.0, ts=TS):
        self.A, self.Cv, self.Qmax, self.hMax, self.ts = A, Cv, Qmax, hMax, ts
        self.h = 0.0

    def step(self, u: float) -> float:
        Qin  = (u / 100.0) * self.Qmax
        Qout = self.Cv * math.sqrt(abs(self.h))
        self.h += self.ts * (Qin - Qout) / self.A
        self.h = float(np.clip(self.h, 0.0, self.hMax))
        return self.h / self.hMax * 100.0

    def reset(self): self.h = 0.0


class DCMotorPlant:
    """Electrical + mechanical DC motor model"""
    name = "dc_motor"
    label = "DC Motor  (R=1Ω, L=10mH, Kt=0.5)"

    def __init__(self, R=1.0, L=0.01, Ke=0.5, Kt=0.5, J=0.02,
                 B=0.01, Vmax=12.0, wMax=300.0, ts=TS):
        self.R, self.L, self.Ke, self.Kt = R, L, Ke, Kt
        self.J, self.B, self.Vmax, self.wMax, self.ts = J, B, Vmax, wMax, ts
        self.i = self.omega = 0.0

    def step(self, u: float) -> float:
        V       = (u / 100.0) * self.Vmax
        didt    = (V - self.R * self.i - self.Ke * self.omega) / self.L
        domdt   = (self.Kt * self.i - self.B * self.omega) / self.J
        self.i     += self.ts * didt
        self.omega += self.ts * domdt
        self.omega  = float(np.clip(self.omega, -self.wMax, self.wMax))
        return self.omega / self.wMax * 100.0

    def reset(self): self.i = self.omega = 0.0


PLANTS = [FirstOrderPlant, SecondOrderPlant, ServoPlant, TankPlant, DCMotorPlant]
PLANT_MAP = {p.name: p for p in PLANTS}


# ══════════════════════════════════════════════════════════════════════════════
#  CONTROLLER IMPLEMENTATIONS
# ══════════════════════════════════════════════════════════════════════════════

class OnOffController:
    name  = "onoff"
    label = "On/Off"
    kp = ki = kd = 0.0

    def __init__(self, hysteresis=2.0):
        self.hyst    = hysteresis
        self._output = 0.0

    def compute(self, sp: float, pv: float) -> float:
        err = sp - pv
        if err >  self.hyst / 2.0: self._output = 100.0
        if err < -self.hyst / 2.0: self._output = 0.0
        return self._output

    def reset(self): self._output = 0.0


class PController:
    name  = "p"
    label = "P"

    def __init__(self, kp=5.0, bias=50.0):
        self.kp = kp; self.ki = 0.0; self.kd = 0.0
        self.bias = bias

    def compute(self, sp: float, pv: float) -> float:
        return float(np.clip(self.bias + self.kp * (sp - pv), 0.0, 100.0))

    def reset(self): pass


class PIController:
    name  = "pi"
    label = "PI"

    def __init__(self, kp=3.0, ki=0.8, ts=TS):
        self.kp = kp; self.ki = ki; self.kd = 0.0
        self.ts  = ts
        self._integral   = 0.0
        self._last_output = 0.0

    def compute(self, sp: float, pv: float) -> float:
        err      = sp - pv
        saturated = (self._last_output >= 100.0) or (self._last_output <= 0.0)
        windup    = saturated and (err * self._last_output > 0)
        if not windup:
            self._integral += err * self.ts
        self._integral = float(np.clip(self._integral, 0.0, 100.0 / max(self.ki, 1e-9)))
        out = self.kp * err + self.ki * self._integral
        self._last_output = float(np.clip(out, 0.0, 100.0))
        return self._last_output

    def reset(self): self._integral = self._last_output = 0.0


class PIDController:
    name  = "pid"
    label = "PID"

    def __init__(self, kp=2.5, ki=0.8, kd=0.15, ts=TS, N=10.0, Kb=0.1):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.ts = ts; self.N  = N;  self.Kb = Kb
        self._integral    = 0.0
        self._prev_pv     = 0.0
        self._deriv_filt  = 0.0
        self._last_output = 0.0

    def compute(self, sp: float, pv: float) -> float:
        err    = sp - pv
        p_term = self.kp * err

        # Derivative on measurement with low-pass filter
        Tf = (self.kd / (self.kp * self.N)) if self.N > 0 else 0.01
        dPV = pv - self._prev_pv
        self._deriv_filt = (1.0 - self.ts / Tf) * self._deriv_filt \
                           - (self.kd / Tf) * dPV
        self._prev_pv = pv

        raw_out    = p_term + self._integral + self._deriv_filt
        clamped    = float(np.clip(raw_out, 0.0, 100.0))
        sat_error  = clamped - raw_out

        # Anti-windup back-calculation
        self._integral += (self.ki * err + self.Kb * sat_error) * self.ts
        self._last_output = clamped
        return clamped

    def reset(self):
        self._integral = self._deriv_filt = self._prev_pv = self._last_output = 0.0


CONTROLLERS = [OnOffController, PController, PIController, PIDController]
CTRL_MAP    = {c.name: c for c in CONTROLLERS}

# Default tunings per controller
DEFAULT_GAINS = {
    "onoff": {},
    "p":     {"kp": 5.0},
    "pi":    {"kp": 3.0, "ki": 0.8},
    "pid":   {"kp": 2.5, "ki": 0.8, "kd": 0.15},
}


# ══════════════════════════════════════════════════════════════════════════════
#  BENCHMARK RUNNER
# ══════════════════════════════════════════════════════════════════════════════

def run_single(
    plant_cls,
    ctrl_cls,
    setpoint: float = DEFAULT_SP,
    sim_time: float = DEFAULT_SIM,
    ts: float       = TS,
    gains: dict     = None,
) -> Tuple[PerformanceMetrics, np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate one (controller, plant) pair and return metrics + time-series.

    Returns
    -------
    metrics  : PerformanceMetrics
    t_arr    : np.ndarray  — time axis
    pv_arr   : np.ndarray  — process variable
    cv_arr   : np.ndarray  — control output
    """
    gains  = gains or DEFAULT_GAINS.get(ctrl_cls.name, {})
    plant  = plant_cls(ts=ts)
    ctrl   = ctrl_cls(**gains) if gains else ctrl_cls()
    engine = MetricsEngine(ts=ts)

    n_steps = int(sim_time / ts)
    t_arr   = np.empty(n_steps)
    pv_arr  = np.empty(n_steps)
    cv_arr  = np.empty(n_steps)

    pv = plant.step(0.0)   # initial output
    for k in range(n_steps):
        t  = k * ts
        cv = ctrl.compute(setpoint, pv)
        pv = plant.step(cv)
        engine.update(t, setpoint, pv, cv)
        t_arr[k]  = t
        pv_arr[k] = pv
        cv_arr[k] = cv

    raw = engine.get_metrics()
    pm  = PerformanceMetrics(
        controller     = ctrl_cls.label,
        plant          = plant_cls.label,
        setpoint       = setpoint,
        kp             = getattr(ctrl, "kp", 0.0),
        ki             = getattr(ctrl, "ki", 0.0),
        kd             = getattr(ctrl, "kd", 0.0),
        rise_time      = raw["rise_time"] or float("nan"),
        settling_time  = raw["settling_time"],
        overshoot_pct  = raw["overshoot_pct"],
        ss_error       = raw["ss_error"],
        iae            = raw["iae"],
        itse           = raw["itse"],
        control_effort = raw["control_effort"],
        sim_time_s     = sim_time,
        samples        = raw["samples"],
        stable         = raw["stable"],
    )
    return pm, t_arr, pv_arr, cv_arr


def run_full_benchmark(
    plants:   List = None,
    ctrls:    List = None,
    setpoint: float = DEFAULT_SP,
    sim_time: float = DEFAULT_SIM,
    ts:       float = TS,
    verbose:  bool  = True,
) -> List[PerformanceMetrics]:
    """Run every (plant × controller) combination and return all metrics."""
    plants = plants or PLANTS
    ctrls  = ctrls  or CONTROLLERS
    results: List[PerformanceMetrics] = []

    total = len(plants) * len(ctrls)
    done  = 0

    for plant_cls in plants:
        for ctrl_cls in ctrls:
            t0 = time.perf_counter()
            pm, *_ = run_single(plant_cls, ctrl_cls, setpoint, sim_time, ts)
            elapsed = time.perf_counter() - t0
            done += 1
            if verbose:
                flag = "✓" if pm.stable else "!"
                print(f"  [{done:>2}/{total}] {flag} {ctrl_cls.label:<8} × "
                      f"{plant_cls.label:<38}  ({elapsed*1000:.0f} ms)")
            results.append(pm)

    return results


# ══════════════════════════════════════════════════════════════════════════════
#  EXPORT
# ══════════════════════════════════════════════════════════════════════════════

def export_csv(results: List[PerformanceMetrics], path: str) -> str:
    """Write results to CSV; return the path written."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fnames = [f.name for f in fields(PerformanceMetrics)]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fnames)
        w.writeheader()
        for pm in results:
            row = asdict(pm)
            # Format floats to 6 d.p.; keep nan as empty
            for k, v in row.items():
                if isinstance(v, float):
                    row[k] = "" if math.isnan(v) else round(v, 6)
            w.writerow(row)
    return path


# ══════════════════════════════════════════════════════════════════════════════
#  PRETTY TABLES
# ══════════════════════════════════════════════════════════════════════════════

def _fmt(val, w=9, decimals=3) -> str:
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return f"{'N/A':>{w}}"
    return f"{val:>{w}.{decimals}f}"


def print_table_for_plant(plant_name: str, results: List[PerformanceMetrics]) -> None:
    rows = [r for r in results if r.plant.startswith(plant_name.replace("_", " ").split("(")[0].strip())]
    if not rows:
        return

    hdr = (f"{'Controller':<10} {'Rise(s)':>9} {'Settle(s)':>10} "
           f"{'OS(%)':>8} {'SSE':>10} {'IAE':>10} {'ITSE':>12} {'Effort':>10} {'OK':>4}")
    div = "─" * len(hdr)

    print(f"\n  Plant: {rows[0].plant}")
    print(f"  {div}")
    print(f"  {hdr}")
    print(f"  {div}")

    for r in rows:
        ok  = "✓" if r.stable else "✗"
        rt  = _fmt(r.rise_time, 9, 3)
        st  = _fmt(r.settling_time, 10, 3)
        os_ = _fmt(r.overshoot_pct, 8, 2)
        sse = _fmt(r.ss_error, 10, 4)
        iae = _fmt(r.iae, 10, 3)
        its = _fmt(r.itse, 12, 3)
        eff = _fmt(r.control_effort, 10, 2)
        print(f"  {r.controller:<10} {rt} {st} {os_} {sse} {iae} {its} {eff} {ok:>4}")

    print(f"  {div}")


def print_summary_table(results: List[PerformanceMetrics]) -> None:
    """Best controller per plant (by IAE)."""
    from collections import defaultdict
    by_plant: dict[str, List[PerformanceMetrics]] = defaultdict(list)
    for r in results:
        by_plant[r.plant].append(r)

    print("\n" + "═" * 80)
    print("  SUMMARY — Best Controller per Plant (lowest IAE)")
    print("═" * 80)
    hdr = (f"  {'Plant':<38} {'Best Ctrl':<10} {'IAE':>10} "
           f"{'Rise(s)':>9} {'OS(%)':>8} {'SSE':>10}")
    print(hdr)
    print("  " + "─" * 78)

    for plant, rows in by_plant.items():
        stable_rows = [r for r in rows if r.stable] or rows
        best = min(stable_rows, key=lambda r: r.iae)
        print(f"  {best.plant:<38} {best.controller:<10} "
              f"{_fmt(best.iae,10,3)} {_fmt(best.rise_time,9,3)} "
              f"{_fmt(best.overshoot_pct,8,2)} {_fmt(best.ss_error,10,4)}")

    print("  " + "─" * 78)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="HIL Testbed — Automated Benchmark Framework"
    )
    parser.add_argument("--plant",    choices=list(PLANT_MAP), default=None,
                        help="Restrict to one plant model")
    parser.add_argument("--ctrl",     choices=list(CTRL_MAP),  default=None,
                        help="Restrict to one controller")
    parser.add_argument("--setpoint", type=float, default=DEFAULT_SP,
                        help=f"Target setpoint 0–100 %% (default {DEFAULT_SP})")
    parser.add_argument("--sim-time", type=float, default=DEFAULT_SIM,
                        help=f"Simulation duration [s] (default {DEFAULT_SIM})")
    parser.add_argument("--export",   default="benchmark_results",
                        help="Output directory for CSV file")
    parser.add_argument("--no-csv",   action="store_true",
                        help="Skip CSV export")
    parser.add_argument("--quiet",    action="store_true",
                        help="Suppress per-run progress lines")
    args = parser.parse_args()

    plants = [PLANT_MAP[args.plant]] if args.plant else PLANTS
    ctrls  = [CTRL_MAP[args.ctrl]]  if args.ctrl  else CONTROLLERS

    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║              HIL Testbed — Automated Controller Benchmark               ║")
    print(f"║  SP={args.setpoint:.1f}%   T={args.sim_time:.0f}s   "
          f"Plants×Ctrls = {len(plants)}×{len(ctrls)} = {len(plants)*len(ctrls)} runs      ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝\n")

    t0      = time.perf_counter()
    results = run_full_benchmark(plants, ctrls, args.setpoint,
                                 args.sim_time, verbose=not args.quiet)
    elapsed = time.perf_counter() - t0

    # Per-plant tables
    seen = []
    for r in results:
        key = r.plant
        if key not in seen:
            seen.append(key)
            print_table_for_plant(key, results)

    print_summary_table(results)
    print(f"\n  Benchmark complete — {len(results)} runs in {elapsed:.2f} s\n")

    if not args.no_csv:
        stamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
        csvpath = os.path.join(args.export, f"benchmark_{stamp}.csv")
        export_csv(results, csvpath)
        print(f"  📄 CSV exported → {csvpath}\n")


if __name__ == "__main__":
    main()
