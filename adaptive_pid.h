/**
 * adaptive_pid.h — Universal Adaptive PID Controller
 * =====================================================
 * Full suite: oscillation detection, slow response detection,
 * steady-state error detection, gain scheduling, CSV logging.
 *
 * Usage (C):
 *   AdaptivePID pid;
 *   AdaptivePID_init(&pid, 1.0f, 0.1f, 0.05f);
 *   float out = AdaptivePID_compute(&pid, setpoint, measurement, dt);
 */

#ifndef ADAPTIVE_PID_H
#define ADAPTIVE_PID_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Limits                                                              */
/* ------------------------------------------------------------------ */
#define APID_MAX_SCHEDULES     8
#define APID_SCHEDULE_NAME_LEN 32
#define APID_OSC_WINDOW        20

/* ------------------------------------------------------------------ */
/* Structs                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    float kp, ki, kd;
} PIDGains;

typedef struct {
    /* Nudge step sizes */
    float nudge_p;
    float nudge_i;
    float nudge_d;

    /* Gain limits */
    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;

    /* Oscillation detection */
    int   osc_flip_threshold;   /* sign flips within window → oscillating */
    float osc_near_sp_band;     /* fraction of |setpoint| for "near SP" */

    /* Slow response */
    float slow_response_time;       /* seconds */
    float slow_response_tolerance;  /* fraction of |setpoint| */

    /* Steady-state error */
    float steady_state_window;    /* seconds of settled error */
    float steady_state_threshold; /* fraction of |setpoint| */

    /* Feature flags */
    bool adapt_p;
    bool adapt_i;
    bool adapt_d;

    /* Logging */
    bool  log_enabled;
    char  log_path[128];
} AdaptiveConfig;

typedef struct {
    char     name[APID_SCHEDULE_NAME_LEN];
    PIDGains gains;
} GainSchedule;

typedef struct {
    /* Current gains */
    PIDGains gains;
    PIDGains initial_gains;

    /* Config */
    AdaptiveConfig cfg;

    /* Output limits */
    float out_min, out_max;
    bool  limits_enabled;

    /* Internal state */
    float    integral;
    float    prev_error;
    float    prev_measurement;

    /* Oscillation buffer */
    float    err_history[APID_OSC_WINDOW];
    int      err_history_idx;
    int      err_history_count;

    /* Timers */
    float    time_at_sp;
    float    time_leaving_sp;
    float    elapsed;

    /* Steady-state tracking */
    float    ss_accumulator;
    int      ss_sample_count;

    /* Last setpoint */
    float    last_setpoint;
    bool     sp_initialized;

    /* Gain schedules */
    GainSchedule schedules[APID_MAX_SCHEDULES];
    int          schedule_count;
    int          active_schedule;   /* -1 = none */

    /* Log file */
    FILE    *log_file;
} AdaptivePID;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Populate a config with sensible defaults. */
void AdaptiveConfig_default(AdaptiveConfig *cfg);

/**
 * Initialise a PID controller.
 * Call AdaptiveConfig_default() first if you want defaults.
 */
void AdaptivePID_init(
    AdaptivePID     *pid,
    float            kp,
    float            ki,
    float            kd,
    AdaptiveConfig  *cfg    /* NULL = use defaults */
);

/** Set output clamp. */
void AdaptivePID_set_limits(AdaptivePID *pid, float out_min, float out_max);

/**
 * Compute one control step.
 * @param setpoint    desired value
 * @param measurement current measured value
 * @param dt          time delta in seconds
 * @return control output
 */
float AdaptivePID_compute(AdaptivePID *pid, float setpoint, float measurement, float dt);

/** Register a named gain schedule. */
int AdaptivePID_set_schedule(
    AdaptivePID *pid,
    const char  *name,
    float        kp,
    float        ki,
    float        kd
);

/** Switch to a named schedule immediately. Returns 0 on success. */
int AdaptivePID_apply_schedule(AdaptivePID *pid, const char *name);

/** Reset internal state (keep gains). */
void AdaptivePID_reset(AdaptivePID *pid);

/** Restore initial gains. */
void AdaptivePID_reset_gains(AdaptivePID *pid);

/** Flush log and close file handle. */
void AdaptivePID_close(AdaptivePID *pid);

#ifdef __cplusplus
}
#endif

#endif /* ADAPTIVE_PID_H */
