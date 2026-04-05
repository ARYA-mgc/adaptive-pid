# Adaptive PID Controller — Universal

A universal adaptive PID controller implemented in **Python**, **C**, and **C++**.

Full suite:
- Oscillation detection → reduces P gain
- Slow response detection → increases P gain
- Steady-state error detection → increases I gain
- Gain scheduling (named presets, hot-swap at runtime)
- Anti-windup (clamped integral on output saturation)
- CSV logging (time, setpoint, measurement, error, output, kp, ki, kd)

---

## Repository structure

```
adaptive-pid/
├── python/
│   └── adaptive_pid.py         # Python implementation
├── c/
│   ├── adaptive_pid.h          # C header
│   └── adaptive_pid.c          # C implementation
├── cpp/
│   ├── adaptive_pid.hpp        # C++ header
│   └── adaptive_pid.cpp        # C++ implementation
└── examples/
    ├── example_python.py
    ├── example_c.c
    └── example_cpp.cpp
```

---

## Quick start

### Python

```python
from adaptive_pid import AdaptivePID, AdaptiveConfig

cfg = AdaptiveConfig(nudge_p=0.02, nudge_i=0.005, log_path="pid_log.csv")
pid = AdaptivePID(kp=1.2, ki=0.1, kd=0.05, output_limits=(-10, 10), config=cfg)

# Optional gain schedules
pid.set_schedule("heavy_load", kp=0.8, ki=0.08, kd=0.04)
pid.apply_schedule("heavy_load")

# Control loop
while True:
    output = pid.compute(setpoint, measurement, dt)
```

### C

```c
#include "adaptive_pid.h"

AdaptiveConfig cfg;
AdaptiveConfig_default(&cfg);

AdaptivePID pid;
AdaptivePID_init(&pid, 1.2f, 0.1f, 0.05f, &cfg);
AdaptivePID_set_limits(&pid, -10.0f, 10.0f);
AdaptivePID_set_schedule(&pid, "heavy_load", 0.8f, 0.08f, 0.04f);

float output = AdaptivePID_compute(&pid, setpoint, measurement, dt);

AdaptivePID_close(&pid);
```

Compile:
```bash
gcc main.c adaptive_pid.c -o controller -lm
```

### C++

```cpp
#include "adaptive_pid.hpp"

AdaptiveConfig cfg;
cfg.nudge_p = 0.02;
cfg.log_path = "pid_log.csv";

AdaptivePID pid(1.2, 0.1, 0.05, cfg);
pid.setLimits(-10.0, 10.0);
pid.setSchedule("heavy_load", 0.8, 0.08, 0.04);

double output = pid.compute(setpoint, measurement, dt);
```

Compile:
```bash
g++ -std=c++17 main.cpp adaptive_pid.cpp -o controller
```

---

## How adaptation works

The controller checks three conditions every step and nudges gains accordingly:

| Condition | Detection | Action |
|---|---|---|
| Oscillation | Error sign flips rapidly near setpoint | P -= nudge_p |
| Slow response | Error remains large beyond `slow_response_time` | P += nudge_p |
| Steady-state error | Small persistent error after settling | I += nudge_i |

Gains are bounded by `kp_min/kp_max`, `ki_min/ki_max`, `kd_min/kd_max` to prevent runaway.

### Oscillation detection — false positive guard

Detection only fires when the measurement is **already near the setpoint** (within `osc_near_sp_band * |setpoint|`). This prevents reducing P during normal large step responses.

---

## Configuration reference

| Parameter | Default | Description |
|---|---|---|
| `nudge_p` | 0.01 | P gain step size |
| `nudge_i` | 0.005 | I gain step size |
| `nudge_d` | 0.002 | D gain step size |
| `kp_min/max` | 0 / 10 | P gain bounds |
| `ki_min/max` | 0 / 5 | I gain bounds |
| `kd_min/max` | 0 / 2 | D gain bounds |
| `osc_window` | 20 | Samples in oscillation detection window |
| `osc_flip_threshold` | 6 | Sign flips in window to trigger |
| `osc_near_sp_band` | 0.1 | Fraction of setpoint for "near SP" |
| `slow_response_time` | 2.0s | Seconds before slow response triggers |
| `slow_response_tolerance` | 0.05 | Fraction of setpoint for "reached SP" |
| `steady_state_threshold` | 0.02 | Fraction of setpoint for SS error |
| `adapt_p` | true | Enable P adaptation |
| `adapt_i` | true | Enable I adaptation |
| `adapt_d` | false | Enable D adaptation (risky, off by default) |
| `log_enabled` | true | Enable CSV logging |
| `log_path` | pid_log.csv | Log file path |

---

## Log format

```
time,setpoint,measurement,error,output,kp,ki,kd,schedule
0.0100,1.000000,0.033333,0.966667,1.160000,1.200000,0.100000,0.050000,
...
```

---

## License

MIT
