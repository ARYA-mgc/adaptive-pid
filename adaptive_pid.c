/**
 * adaptive_pid.c — Universal Adaptive PID Controller (C implementation)
 */

#include "adaptive_pid.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float fabsf_safe(float v) { return v < 0.0f ? -v : v; }

static void history_push(AdaptivePID *pid, float err) {
    pid->err_history[pid->err_history_idx] = err;
    pid->err_history_idx = (pid->err_history_idx + 1) % APID_OSC_WINDOW;
    if (pid->err_history_count < APID_OSC_WINDOW)
        pid->err_history_count++;
}

static bool is_oscillating(AdaptivePID *pid, float setpoint) {
    if (pid->err_history_count < APID_OSC_WINDOW)
        return false;

    float band = fabsf_safe(setpoint) * pid->cfg.osc_near_sp_band + 1e-9f;
    if (fabsf_safe(pid->prev_error) > band)
        return false;

    int flips = 0;
    for (int i = 1; i < pid->err_history_count; i++) {
        int cur  = (pid->err_history_idx - 1 - i + APID_OSC_WINDOW) % APID_OSC_WINDOW;
        int prev = (pid->err_history_idx - 2 - i + APID_OSC_WINDOW) % APID_OSC_WINDOW;
        if (pid->err_history[cur] * pid->err_history[prev] < 0.0f)
            flips++;
    }
    return flips >= pid->cfg.osc_flip_threshold;
}

static bool is_responding_slowly(AdaptivePID *pid, float setpoint) {
    float tol = fabsf_safe(setpoint) * pid->cfg.slow_response_tolerance + 1e-9f;
    return fabsf_safe(pid->prev_error) > tol
        && pid->time_leaving_sp > pid->cfg.slow_response_time;
}

static bool has_steady_state_error(AdaptivePID *pid, float setpoint) {
    if (pid->ss_sample_count == 0) return false;
    float avg = pid->ss_accumulator / (float)pid->ss_sample_count;
    float tol_hi = fabsf_safe(setpoint) * pid->cfg.steady_state_threshold + 1e-9f;
    return fabsf_safe(avg) > 1e-9f && fabsf_safe(avg) < tol_hi;
}

static void update_timers(AdaptivePID *pid, float error, float setpoint, float dt) {
    float tol    = fabsf_safe(setpoint) * pid->cfg.slow_response_tolerance + 1e-9f;
    float ss_tol = fabsf_safe(setpoint) * pid->cfg.steady_state_threshold  + 1e-9f;

    if (fabsf_safe(error) <= tol) {
        pid->time_at_sp     += dt;
        pid->time_leaving_sp = 0.0f;
    } else {
        pid->time_leaving_sp += dt;
        pid->time_at_sp       = 0.0f;
    }

    if (pid->time_at_sp > 0.2f && fabsf_safe(error) < ss_tol) {
        pid->ss_accumulator  += error;
        pid->ss_sample_count++;
    }
}

static void adapt(AdaptivePID *pid, float setpoint) {
    AdaptiveConfig *c = &pid->cfg;

    if (is_oscillating(pid, setpoint)) {
        if (c->adapt_p)
            pid->gains.kp = clampf(pid->gains.kp - c->nudge_p, c->kp_min, c->kp_max);
        if (c->adapt_d)
            pid->gains.kd = clampf(pid->gains.kd + c->nudge_d, c->kd_min, c->kd_max);
    } else if (is_responding_slowly(pid, setpoint)) {
        if (c->adapt_p)
            pid->gains.kp = clampf(pid->gains.kp + c->nudge_p, c->kp_min, c->kp_max);
    }

    if (has_steady_state_error(pid, setpoint)) {
        if (c->adapt_i)
            pid->gains.ki = clampf(pid->gains.ki + c->nudge_i, c->ki_min, c->ki_max);
    }
}

static void on_setpoint_change(AdaptivePID *pid) {
    pid->integral        = 0.0f;
    pid->time_at_sp      = 0.0f;
    pid->time_leaving_sp = 0.0f;
    pid->ss_accumulator  = 0.0f;
    pid->ss_sample_count = 0;
    pid->err_history_idx   = 0;
    pid->err_history_count = 0;
}

static void write_log(AdaptivePID *pid, float sp, float meas,
                       float err, float out) {
    if (!pid->log_file) return;
    const char *sched = (pid->active_schedule >= 0)
                        ? pid->schedules[pid->active_schedule].name : "";
    fprintf(pid->log_file,
        "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
        pid->elapsed, sp, meas, err, out,
        pid->gains.kp, pid->gains.ki, pid->gains.kd, sched);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void AdaptiveConfig_default(AdaptiveConfig *cfg) {
    cfg->nudge_p = 0.01f;
    cfg->nudge_i = 0.005f;
    cfg->nudge_d = 0.002f;
    cfg->kp_min  = 0.0f;  cfg->kp_max = 10.0f;
    cfg->ki_min  = 0.0f;  cfg->ki_max = 5.0f;
    cfg->kd_min  = 0.0f;  cfg->kd_max = 2.0f;
    cfg->osc_flip_threshold      = 6;
    cfg->osc_near_sp_band        = 0.1f;
    cfg->slow_response_time      = 2.0f;
    cfg->slow_response_tolerance = 0.05f;
    cfg->steady_state_window     = 1.0f;
    cfg->steady_state_threshold  = 0.02f;
    cfg->adapt_p = true;
    cfg->adapt_i = true;
    cfg->adapt_d = false;
    cfg->log_enabled = true;
    strncpy(cfg->log_path, "pid_log.csv", sizeof(cfg->log_path) - 1);
}

void AdaptivePID_init(AdaptivePID *pid, float kp, float ki, float kd,
                      AdaptiveConfig *cfg) {
    memset(pid, 0, sizeof(*pid));
    pid->gains         = (PIDGains){kp, ki, kd};
    pid->initial_gains = (PIDGains){kp, ki, kd};
    pid->active_schedule = -1;
    pid->sp_initialized  = false;
    pid->last_setpoint   = 0.0f;
    pid->limits_enabled  = false;

    if (cfg) {
        pid->cfg = *cfg;
    } else {
        AdaptiveConfig_default(&pid->cfg);
    }

    if (pid->cfg.log_enabled) {
        pid->log_file = fopen(pid->cfg.log_path, "w");
        if (pid->log_file) {
            fprintf(pid->log_file,
                "time,setpoint,measurement,error,output,kp,ki,kd,schedule\n");
        }
    }
}

void AdaptivePID_set_limits(AdaptivePID *pid, float out_min, float out_max) {
    pid->out_min        = out_min;
    pid->out_max        = out_max;
    pid->limits_enabled = true;
}

float AdaptivePID_compute(AdaptivePID *pid, float setpoint,
                           float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    if (!pid->sp_initialized || setpoint != pid->last_setpoint) {
        on_setpoint_change(pid);
        pid->last_setpoint  = setpoint;
        pid->sp_initialized = true;
    }

    float error = setpoint - measurement;

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    float output = pid->gains.kp * error
                 + pid->gains.ki * pid->integral
                 + pid->gains.kd * derivative;

    if (pid->limits_enabled) {
        if (output > pid->out_max) {
            output = pid->out_max;
            pid->integral -= error * dt;
        } else if (output < pid->out_min) {
            output = pid->out_min;
            pid->integral -= error * dt;
        }
    }

    history_push(pid, error);
    update_timers(pid, error, setpoint, dt);
    adapt(pid, setpoint);

    pid->elapsed += dt;
    write_log(pid, setpoint, measurement, error, output);

    pid->prev_error       = error;
    pid->prev_measurement = measurement;
    return output;
}

int AdaptivePID_set_schedule(AdaptivePID *pid, const char *name,
                              float kp, float ki, float kd) {
    if (pid->schedule_count >= APID_MAX_SCHEDULES) return -1;
    GainSchedule *s = &pid->schedules[pid->schedule_count++];
    strncpy(s->name, name, APID_SCHEDULE_NAME_LEN - 1);
    s->gains = (PIDGains){kp, ki, kd};
    return 0;
}

int AdaptivePID_apply_schedule(AdaptivePID *pid, const char *name) {
    for (int i = 0; i < pid->schedule_count; i++) {
        if (strncmp(pid->schedules[i].name, name, APID_SCHEDULE_NAME_LEN) == 0) {
            pid->gains           = pid->schedules[i].gains;
            pid->integral        = 0.0f;
            pid->active_schedule = i;
            return 0;
        }
    }
    return -1;
}

void AdaptivePID_reset(AdaptivePID *pid) {
    pid->integral          = 0.0f;
    pid->prev_error        = 0.0f;
    pid->err_history_idx   = 0;
    pid->err_history_count = 0;
    pid->time_at_sp        = 0.0f;
    pid->time_leaving_sp   = 0.0f;
    pid->ss_accumulator    = 0.0f;
    pid->ss_sample_count   = 0;
}

void AdaptivePID_reset_gains(AdaptivePID *pid) {
    pid->gains           = pid->initial_gains;
    pid->active_schedule = -1;
}

void AdaptivePID_close(AdaptivePID *pid) {
    if (pid->log_file) {
        fflush(pid->log_file);
        fclose(pid->log_file);
        pid->log_file = NULL;
    }
}
