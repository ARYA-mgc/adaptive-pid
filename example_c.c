/**
 * example_c.c — Basic usage example for the C adaptive PID
 *
 * Compile:
 *   gcc example_c.c ../c/adaptive_pid.c -o example_c -lm
 */

#include <stdio.h>
#include <string.h>
#include "../c/adaptive_pid.h"

/* Simple first-order plant: y += (u * gain - y) * (1/lag) * dt */
static float plant(float y, float u, float gain, float lag, float dt) {
    return y + (u * gain - y) * (1.0f / lag) * dt;
}

int main(void) {
    AdaptiveConfig cfg;
    AdaptiveConfig_default(&cfg);
    cfg.nudge_p              = 0.02f;
    cfg.nudge_i              = 0.005f;
    cfg.osc_flip_threshold   = 5;
    cfg.slow_response_time   = 1.5f;
    cfg.log_enabled          = 1;
    strncpy(cfg.log_path, "c_pid_log.csv", sizeof(cfg.log_path) - 1);

    AdaptivePID pid;
    AdaptivePID_init(&pid, 1.2f, 0.1f, 0.05f, &cfg);
    AdaptivePID_set_limits(&pid, -10.0f, 10.0f);
    AdaptivePID_set_schedule(&pid, "light_load", 1.4f, 0.12f, 0.06f);
    AdaptivePID_set_schedule(&pid, "heavy_load", 0.8f, 0.08f, 0.04f);

    float setpoints[] = {1.0f, 2.0f, 0.5f, 1.5f};
    int   n_sp        = 4;
    float dt          = 0.01f;
    float y           = 0.0f;

    for (int s = 0; s < n_sp; s++) {
        float sp = setpoints[s];
        printf("--- Setpoint: %.1f ---\n", sp);

        /* Switch schedule at setpoint 2 */
        if (s == 1) AdaptivePID_apply_schedule(&pid, "heavy_load");
        if (s == 2) AdaptivePID_apply_schedule(&pid, "light_load");

        for (int i = 0; i < 400; i++) {
            float u = AdaptivePID_compute(&pid, sp, y, dt);
            y = plant(y, u, 1.0f, 0.3f, dt);
        }
        printf("  Final output: %.4f  kp=%.4f ki=%.4f kd=%.4f\n",
               y, pid.gains.kp, pid.gains.ki, pid.gains.kd);
    }

    AdaptivePID_close(&pid);
    printf("Log saved to c_pid_log.csv\n");
    return 0;
}
