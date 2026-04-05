"""
Adaptive PID Controller — Universal
====================================
Full suite: oscillation detection, slow response detection,
steady-state error detection, gain scheduling, CSV logging.

Usage:
    pid = AdaptivePID(kp=1.0, ki=0.1, kd=0.05)
    pid.set_schedule("high_load", kp=0.7, ki=0.08, kd=0.04)

    while running:
        output = pid.compute(setpoint, measured_value, dt)
"""

import time
import csv
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


@dataclass
class AdaptiveConfig:
    # Gain nudge step sizes
    nudge_p: float = 0.01
    nudge_i: float = 0.005
    nudge_d: float = 0.002

    # Gain limits (prevent runaway)
    kp_min: float = 0.0
    kp_max: float = 10.0
    ki_min: float = 0.0
    ki_max: float = 5.0
    kd_min: float = 0.0
    kd_max: float = 2.0

    # Oscillation detection
    osc_window: int = 20        # samples to watch
    osc_flip_threshold: int = 6 # sign flips in window = oscillating
    osc_near_sp_band: float = 0.1  # fraction of setpoint to consider "near"

    # Slow response detection
    slow_response_time: float = 2.0   # seconds to reach within tolerance
    slow_response_tolerance: float = 0.05  # fraction of setpoint

    # Steady-state error detection
    steady_state_window: float = 1.0  # seconds of stable but non-zero error
    steady_state_threshold: float = 0.02  # fraction of setpoint

    # Adaptation enable flags
    adapt_p: bool = True
    adapt_i: bool = True
    adapt_d: bool = False  # D adaptation is risky, off by default

    # Logging
    log_enabled: bool = True
    log_path: str = "pid_log.csv"


class AdaptivePID:
    """
    Universal Adaptive PID Controller.

    Parameters
    ----------
    kp, ki, kd : float
        Initial gains.
    output_limits : tuple[float, float] | None
        Clamp output to (min, max). None = unclamped.
    config : AdaptiveConfig | None
        Tuning behaviour config. Uses defaults if None.
    name : str
        Label for logging (useful when running multiple PIDs).
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.1,
        kd: float = 0.05,
        output_limits: Optional[tuple] = None,
        config: Optional[AdaptiveConfig] = None,
        name: str = "pid",
    ):
        self.gains = PIDGains(kp=kp, ki=ki, kd=kd)
        self._initial_gains = PIDGains(kp=kp, ki=ki, kd=kd)
        self.output_limits = output_limits
        self.config = config or AdaptiveConfig()
        self.name = name

        # Internal state
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_measurement: float = 0.0

        # Detection buffers
        self._error_history: deque = deque(maxlen=self.config.osc_window)
        self._time_since_last_sp_change: float = 0.0
        self._time_at_sp: float = 0.0        # how long we've been "near" SP
        self._time_leaving_sp: float = 0.0   # how long since we left SP band
        self._last_setpoint: float = float("nan")

        # Steady-state tracking
        self._ss_accumulator: float = 0.0
        self._ss_sample_count: int = 0

        # Schedule store: name → PIDGains
        self._schedules: dict[str, PIDGains] = {}
        self._active_schedule: Optional[str] = None

        # Logging
        self._log_file = None
        self._log_writer = None
        if self.config.log_enabled:
            self._init_log()

        self._t_start: float = time.monotonic()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """
        Compute PID output and run one adaptation step.

        Parameters
        ----------
        setpoint    : desired value
        measurement : current measured value
        dt          : time delta since last call (seconds)

        Returns
        -------
        float : control output
        """
        if dt <= 0:
            return 0.0

        # Detect setpoint change → reset integral wind-up, timers
        if setpoint != self._last_setpoint:
            self._on_setpoint_change(setpoint)
        self._last_setpoint = setpoint

        error = setpoint - measurement

        # PID terms
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt

        output = (
            self.gains.kp * error
            + self.gains.ki * self._integral
            + self.gains.kd * derivative
        )

        # Clamp output + anti-windup
        if self.output_limits:
            lo, hi = self.output_limits
            if output > hi:
                output = hi
                self._integral -= error * dt  # undo windup
            elif output < lo:
                output = lo
                self._integral -= error * dt

        # Update detection state
        self._error_history.append(error)
        self._update_timers(error, setpoint, dt)

        # Adapt gains
        self._adapt(error, setpoint, dt)

        # Log
        if self.config.log_enabled and self._log_writer:
            elapsed = time.monotonic() - self._t_start
            self._log_writer.writerow({
                "time": f"{elapsed:.4f}",
                "setpoint": setpoint,
                "measurement": measurement,
                "error": f"{error:.6f}",
                "output": f"{output:.6f}",
                "kp": f"{self.gains.kp:.6f}",
                "ki": f"{self.gains.ki:.6f}",
                "kd": f"{self.gains.kd:.6f}",
                "schedule": self._active_schedule or "",
            })

        self._prev_error = error
        self._prev_measurement = measurement
        return output

    def set_schedule(self, name: str, kp: float, ki: float, kd: float):
        """Register a named gain schedule."""
        self._schedules[name] = PIDGains(kp=kp, ki=ki, kd=kd)

    def apply_schedule(self, name: str):
        """Switch to a named gain schedule immediately."""
        if name not in self._schedules:
            raise KeyError(f"Schedule '{name}' not registered.")
        g = self._schedules[name]
        self.gains.kp = g.kp
        self.gains.ki = g.ki
        self.gains.kd = g.kd
        self._integral = 0.0  # reset integral on schedule change
        self._active_schedule = name

    def reset(self):
        """Reset internal state (keep gains)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._error_history.clear()
        self._time_at_sp = 0.0
        self._ss_accumulator = 0.0
        self._ss_sample_count = 0

    def reset_gains(self):
        """Restore gains to initial values."""
        self.gains = PIDGains(
            kp=self._initial_gains.kp,
            ki=self._initial_gains.ki,
            kd=self._initial_gains.kd,
        )
        self._active_schedule = None

    @property
    def current_gains(self) -> dict:
        return {"kp": self.gains.kp, "ki": self.gains.ki, "kd": self.gains.kd}

    def close(self):
        """Flush and close log file."""
        if self._log_file:
            self._log_file.flush()
            self._log_file.close()

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def _is_oscillating(self, setpoint: float) -> bool:
        """True if error sign flips too frequently near the setpoint."""
        if len(self._error_history) < self.config.osc_window:
            return False

        # Only detect near setpoint to avoid false positives on step changes
        band = abs(setpoint) * self.config.osc_near_sp_band + 1e-9
        if abs(self._prev_error) > band:
            return False

        flips = sum(
            1
            for i in range(1, len(self._error_history))
            if (self._error_history[i] * self._error_history[i - 1]) < 0
        )
        return flips >= self.config.osc_flip_threshold

    def _is_responding_slowly(self, setpoint: float) -> bool:
        """True if we haven't reached SP within slow_response_time."""
        tol = abs(setpoint) * self.config.slow_response_tolerance + 1e-9
        not_there = abs(self._prev_error) > tol
        return not_there and self._time_leaving_sp > self.config.slow_response_time

    def _has_steady_state_error(self, setpoint: float) -> bool:
        """True if error is small but non-zero over a sustained window."""
        if self._ss_sample_count == 0:
            return False
        avg_error = self._ss_accumulator / self._ss_sample_count
        tol_hi = abs(setpoint) * self.config.steady_state_threshold + 1e-9
        tol_lo = 1e-9
        return tol_lo < abs(avg_error) < tol_hi

    # ------------------------------------------------------------------
    # Adaptation
    # ------------------------------------------------------------------

    def _adapt(self, error: float, setpoint: float, dt: float):
        cfg = self.config

        if self._is_oscillating(setpoint):
            if cfg.adapt_p:
                self.gains.kp = max(cfg.kp_min, self.gains.kp - cfg.nudge_p)
            if cfg.adapt_d:
                self.gains.kd = min(cfg.kd_max, self.gains.kd + cfg.nudge_d)

        elif self._is_responding_slowly(setpoint):
            if cfg.adapt_p:
                self.gains.kp = min(cfg.kp_max, self.gains.kp + cfg.nudge_p)

        if self._has_steady_state_error(setpoint):
            if cfg.adapt_i:
                self.gains.ki = min(cfg.ki_max, self.gains.ki + cfg.nudge_i)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _on_setpoint_change(self, new_sp: float):
        self._integral = 0.0
        self._time_at_sp = 0.0
        self._time_leaving_sp = 0.0
        self._ss_accumulator = 0.0
        self._ss_sample_count = 0
        self._error_history.clear()

    def _update_timers(self, error: float, setpoint: float, dt: float):
        tol = abs(setpoint) * self.config.slow_response_tolerance + 1e-9
        ss_tol = abs(setpoint) * self.config.steady_state_threshold + 1e-9

        if abs(error) <= tol:
            self._time_at_sp += dt
            self._time_leaving_sp = 0.0
        else:
            self._time_leaving_sp += dt
            self._time_at_sp = 0.0

        # Steady-state accumulator: running once we've been near SP for a bit
        if self._time_at_sp > 0.2:
            if abs(error) < ss_tol:
                self._ss_accumulator += error
                self._ss_sample_count += 1

    def _init_log(self):
        fields = ["time", "setpoint", "measurement", "error",
                  "output", "kp", "ki", "kd", "schedule"]
        self._log_file = open(self.config.log_path, "w", newline="")
        self._log_writer = csv.DictWriter(self._log_file, fieldnames=fields)
        self._log_writer.writeheader()


# ------------------------------------------------------------------
# Convenience: run a quick simulation for testing
# ------------------------------------------------------------------

def simulate(
    pid: AdaptivePID,
    setpoints: list,
    plant_gain: float = 1.0,
    plant_lag: float = 0.3,
    dt: float = 0.01,
    steps_per_sp: int = 500,
):
    """
    Simulate a first-order plant: output += (input - output) * (1 / lag) * dt

    Returns list of dicts with time, setpoint, measurement, output, kp, ki, kd.
    """
    measurement = 0.0
    t = 0.0
    log = []

    for sp in setpoints:
        for _ in range(steps_per_sp):
            output = pid.compute(sp, measurement, dt)
            measurement += (output * plant_gain - measurement) * (1.0 / plant_lag) * dt
            log.append({
                "t": round(t, 4),
                "sp": sp,
                "y": round(measurement, 5),
                "u": round(output, 5),
                **pid.current_gains,
            })
            t += dt

    return log


if __name__ == "__main__":
    cfg = AdaptiveConfig(
        nudge_p=0.02,
        nudge_i=0.005,
        osc_flip_threshold=5,
        slow_response_time=1.5,
        log_path="pid_log.csv",
    )
    pid = AdaptivePID(kp=1.2, ki=0.1, kd=0.05, output_limits=(-10, 10), config=cfg)

    # Register some gain schedules
    pid.set_schedule("light_load", kp=1.4, ki=0.12, kd=0.06)
    pid.set_schedule("heavy_load", kp=0.8, ki=0.08, kd=0.04)

    results = simulate(pid, setpoints=[1.0, 2.0, 0.5, 1.5], dt=0.01, steps_per_sp=400)
    pid.close()

    print(f"Ran {len(results)} steps. Final gains: {pid.current_gains}")
    print("Log saved to pid_log.csv")
