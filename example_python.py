"""
example_python.py — Basic usage example for the Python adaptive PID

Run:
    python example_python.py
"""

import sys
sys.path.insert(0, "../python")
from adaptive_pid import AdaptivePID, AdaptiveConfig, simulate


def main():
    cfg = AdaptiveConfig(
        nudge_p=0.02,
        nudge_i=0.005,
        osc_flip_threshold=5,
        slow_response_time=1.5,
        log_path="python_pid_log.csv",
    )

    pid = AdaptivePID(kp=1.2, ki=0.1, kd=0.05,
                      output_limits=(-10, 10), config=cfg, name="demo")

    pid.set_schedule("light_load", kp=1.4, ki=0.12, kd=0.06)
    pid.set_schedule("heavy_load", kp=0.8, ki=0.08, kd=0.04)

    setpoints = [1.0, 2.0, 0.5, 1.5]
    dt = 0.01
    y = 0.0

    for i, sp in enumerate(setpoints):
        print(f"--- Setpoint: {sp} ---")

        if i == 1:
            pid.apply_schedule("heavy_load")
        elif i == 2:
            pid.apply_schedule("light_load")

        for _ in range(400):
            u = pid.compute(sp, y, dt)
            # first-order plant: y += (u - y) * (1/lag) * dt
            y += (u - y) * (1.0 / 0.3) * dt

        g = pid.current_gains
        print(f"  Final output: {y:.4f}  kp={g['kp']:.4f} "
              f"ki={g['ki']:.4f} kd={g['kd']:.4f}")

    pid.close()
    print("Log saved to python_pid_log.csv")


if __name__ == "__main__":
    main()
