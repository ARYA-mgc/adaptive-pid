/**
 * adaptive_pid.cpp — Universal Adaptive PID Controller (C++ implementation)
 */

#include "adaptive_pid.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <sstream>

// -----------------------------------------------------------------------
// Construction / destruction
// -----------------------------------------------------------------------

AdaptivePID::AdaptivePID(double kp, double ki, double kd,
                         AdaptiveConfig cfg, std::string name)
    : gains_{kp, ki, kd}
    , initialGains_{kp, ki, kd}
    , cfg_(std::move(cfg))
    , name_(std::move(name))
{
    initLog();
}

AdaptivePID::~AdaptivePID() { close(); }

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

double AdaptivePID::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    if (std::isnan(lastSetpoint_) || setpoint != lastSetpoint_) {
        onSetpointChange();
        lastSetpoint_ = setpoint;
    }

    const double error = setpoint - measurement;

    integral_ += error * dt;
    const double derivative = (error - prevError_) / dt;

    double output = gains_.kp * error
                  + gains_.ki * integral_
                  + gains_.kd * derivative;

    // Clamp + anti-windup
    if (limits_) {
        auto [lo, hi] = *limits_;
        if (output > hi) {
            output = hi;
            integral_ -= error * dt;
        } else if (output < lo) {
            output = lo;
            integral_ -= error * dt;
        }
    }

    pushHistory(error);
    updateTimers(error, setpoint, dt);
    adapt(setpoint);

    elapsed_ += dt;
    writeLog(setpoint, measurement, error, output);

    prevError_       = error;
    prevMeasurement_ = measurement;
    return output;
}

void AdaptivePID::setLimits(double outMin, double outMax) {
    limits_ = {outMin, outMax};
}

void AdaptivePID::setSchedule(const std::string& name,
                               double kp, double ki, double kd) {
    schedules_[name] = PIDGains{kp, ki, kd};
}

void AdaptivePID::applySchedule(const std::string& name) {
    auto it = schedules_.find(name);
    if (it == schedules_.end())
        throw std::invalid_argument("Unknown schedule: " + name);
    gains_          = it->second;
    integral_       = 0.0;
    activeSchedule_ = name;
}

void AdaptivePID::reset() {
    integral_       = 0.0;
    prevError_      = 0.0;
    timeAtSP_       = 0.0;
    timeLeavingSP_  = 0.0;
    ssAccumulator_  = 0.0;
    ssSampleCount_  = 0;
    errorHistory_.clear();
}

void AdaptivePID::resetGains() {
    gains_          = initialGains_;
    activeSchedule_ = "";
}

void AdaptivePID::close() {
    if (logFile_.is_open()) {
        logFile_.flush();
        logFile_.close();
    }
}

// -----------------------------------------------------------------------
// Detection
// -----------------------------------------------------------------------

bool AdaptivePID::isOscillating(double setpoint) const {
    if ((int)errorHistory_.size() < cfg_.osc_window) return false;

    const double band = std::abs(setpoint) * cfg_.osc_near_sp_band + 1e-9;
    if (std::abs(prevError_) > band) return false;

    int flips = 0;
    for (int i = 1; i < (int)errorHistory_.size(); ++i)
        if (errorHistory_[i] * errorHistory_[i - 1] < 0.0)
            ++flips;
    return flips >= cfg_.osc_flip_threshold;
}

bool AdaptivePID::isRespondingSlowly(double setpoint) const {
    const double tol = std::abs(setpoint) * cfg_.slow_response_tolerance + 1e-9;
    return std::abs(prevError_) > tol
        && timeLeavingSP_ > cfg_.slow_response_time;
}

bool AdaptivePID::hasSteadyStateError(double setpoint) const {
    if (ssSampleCount_ == 0) return false;
    const double avg    = ssAccumulator_ / ssSampleCount_;
    const double tol_hi = std::abs(setpoint) * cfg_.steady_state_threshold + 1e-9;
    return std::abs(avg) > 1e-9 && std::abs(avg) < tol_hi;
}

// -----------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------

void AdaptivePID::onSetpointChange() {
    integral_      = 0.0;
    timeAtSP_      = 0.0;
    timeLeavingSP_ = 0.0;
    ssAccumulator_ = 0.0;
    ssSampleCount_ = 0;
    errorHistory_.clear();
}

void AdaptivePID::pushHistory(double err) {
    errorHistory_.push_back(err);
    if ((int)errorHistory_.size() > cfg_.osc_window)
        errorHistory_.pop_front();
}

void AdaptivePID::updateTimers(double error, double setpoint, double dt) {
    const double tol    = std::abs(setpoint) * cfg_.slow_response_tolerance + 1e-9;
    const double ss_tol = std::abs(setpoint) * cfg_.steady_state_threshold  + 1e-9;

    if (std::abs(error) <= tol) {
        timeAtSP_      += dt;
        timeLeavingSP_  = 0.0;
    } else {
        timeLeavingSP_ += dt;
        timeAtSP_       = 0.0;
    }

    if (timeAtSP_ > 0.2 && std::abs(error) < ss_tol) {
        ssAccumulator_ += error;
        ++ssSampleCount_;
    }
}

void AdaptivePID::adapt(double setpoint) {
    auto clamp = [](double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    };

    if (isOscillating(setpoint)) {
        if (cfg_.adapt_p)
            gains_.kp = clamp(gains_.kp - cfg_.nudge_p, cfg_.kp_min, cfg_.kp_max);
        if (cfg_.adapt_d)
            gains_.kd = clamp(gains_.kd + cfg_.nudge_d, cfg_.kd_min, cfg_.kd_max);
    } else if (isRespondingSlowly(setpoint)) {
        if (cfg_.adapt_p)
            gains_.kp = clamp(gains_.kp + cfg_.nudge_p, cfg_.kp_min, cfg_.kp_max);
    }

    if (hasSteadyStateError(setpoint)) {
        if (cfg_.adapt_i)
            gains_.ki = clamp(gains_.ki + cfg_.nudge_i, cfg_.ki_min, cfg_.ki_max);
    }
}

void AdaptivePID::initLog() {
    if (!cfg_.log_enabled) return;
    logFile_.open(cfg_.log_path, std::ios::out | std::ios::trunc);
    if (logFile_.is_open())
        logFile_ << "time,setpoint,measurement,error,output,kp,ki,kd,schedule\n";
}

void AdaptivePID::writeLog(double sp, double meas, double err, double out) {
    if (!logFile_.is_open()) return;
    logFile_ << std::fixed << std::setprecision(6)
             << elapsed_ << ","
             << sp       << ","
             << meas     << ","
             << err      << ","
             << out      << ","
             << gains_.kp << ","
             << gains_.ki << ","
             << gains_.kd << ","
             << activeSchedule_ << "\n";
}
