/**
 * adaptive_pid.hpp — Universal Adaptive PID Controller (C++)
 * ============================================================
 * Full suite: oscillation detection, slow response detection,
 * steady-state error detection, gain scheduling, CSV logging.
 *
 * Usage:
 *   AdaptivePID pid(1.0, 0.1, 0.05);
 *   pid.setSchedule("heavy", 0.7, 0.08, 0.04);
 *   double output = pid.compute(setpoint, measurement, dt);
 */

#pragma once

#include <string>
#include <unordered_map>
#include <deque>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <cmath>
#include <chrono>

struct PIDGains {
    double kp, ki, kd;
};

struct AdaptiveConfig {
    // Nudge step sizes
    double nudge_p = 0.01;
    double nudge_i = 0.005;
    double nudge_d = 0.002;

    // Gain limits
    double kp_min = 0.0, kp_max = 10.0;
    double ki_min = 0.0, ki_max = 5.0;
    double kd_min = 0.0, kd_max = 2.0;

    // Oscillation detection
    int    osc_window          = 20;
    int    osc_flip_threshold  = 6;
    double osc_near_sp_band    = 0.1;

    // Slow response
    double slow_response_time      = 2.0;
    double slow_response_tolerance = 0.05;

    // Steady-state
    double steady_state_window     = 1.0;
    double steady_state_threshold  = 0.02;

    // Feature flags
    bool adapt_p = true;
    bool adapt_i = true;
    bool adapt_d = false;

    // Logging
    bool        log_enabled = true;
    std::string log_path    = "pid_log.csv";
};

class AdaptivePID {
public:
    /**
     * @param kp, ki, kd     Initial gains
     * @param cfg             Configuration (uses defaults if not provided)
     * @param name            Label used in log file
     */
    explicit AdaptivePID(
        double kp = 1.0,
        double ki = 0.1,
        double kd = 0.05,
        AdaptiveConfig cfg = {},
        std::string name = "pid"
    );

    ~AdaptivePID();

    // Non-copyable, movable
    AdaptivePID(const AdaptivePID&) = delete;
    AdaptivePID& operator=(const AdaptivePID&) = delete;
    AdaptivePID(AdaptivePID&&) = default;

    /**
     * Compute one control step.
     * @param setpoint     desired value
     * @param measurement  current measured value
     * @param dt           time delta in seconds
     * @return control output
     */
    double compute(double setpoint, double measurement, double dt);

    /** Set output clamp. */
    void setLimits(double outMin, double outMax);

    /** Register a named gain schedule. */
    void setSchedule(const std::string& name, double kp, double ki, double kd);

    /** Switch to a named schedule immediately. Throws if not found. */
    void applySchedule(const std::string& name);

    /** Reset internal state (keep current gains). */
    void reset();

    /** Restore initial gains. */
    void resetGains();

    /** Flush and close log. */
    void close();

    /** Read current gains. */
    PIDGains gains() const { return gains_; }

    /** Current active schedule name, or empty string. */
    std::string activeSchedule() const { return activeSchedule_; }

private:
    PIDGains gains_;
    PIDGains initialGains_;
    AdaptiveConfig cfg_;
    std::string name_;

    // Output limits
    std::optional<std::pair<double,double>> limits_;

    // State
    double integral_         = 0.0;
    double prevError_        = 0.0;
    double prevMeasurement_  = 0.0;

    // Detection
    std::deque<double> errorHistory_;
    double timeAtSP_      = 0.0;
    double timeLeavingSP_ = 0.0;
    double ssAccumulator_ = 0.0;
    int    ssSampleCount_ = 0;

    double lastSetpoint_  = std::numeric_limits<double>::quiet_NaN();

    // Schedules
    std::unordered_map<std::string, PIDGains> schedules_;
    std::string activeSchedule_;

    // Logging
    std::ofstream logFile_;
    double elapsed_ = 0.0;

    // Internal methods
    void onSetpointChange();
    void pushHistory(double err);
    void updateTimers(double error, double setpoint, double dt);
    void adapt(double setpoint);

    bool isOscillating(double setpoint) const;
    bool isRespondingSlowly(double setpoint) const;
    bool hasSteadyStateError(double setpoint) const;

    void writeLog(double sp, double meas, double err, double out);
    void initLog();
};
