/**
 * example_cpp.cpp — Basic usage example for the C++ adaptive PID
 *
 * Compile:
 *   g++ -std=c++17 example_cpp.cpp ../cpp/adaptive_pid.cpp -o example_cpp
 */

#include <iostream>
#include <iomanip>
#include "../cpp/adaptive_pid.hpp"

static double plant(double y, double u, double gain, double lag, double dt) {
    return y + (u * gain - y) * (1.0 / lag) * dt;
}

int main() {
    AdaptiveConfig cfg;
    cfg.nudge_p             = 0.02;
    cfg.nudge_i             = 0.005;
    cfg.osc_flip_threshold  = 5;
    cfg.slow_response_time  = 1.5;
    cfg.log_enabled         = true;
    cfg.log_path            = "cpp_pid_log.csv";

    AdaptivePID pid(1.2, 0.1, 0.05, cfg, "demo");
    pid.setLimits(-10.0, 10.0);
    pid.setSchedule("light_load", 1.4, 0.12, 0.06);
    pid.setSchedule("heavy_load", 0.8, 0.08, 0.04);

    double setpoints[] = {1.0, 2.0, 0.5, 1.5};
    double dt = 0.01;
    double y  = 0.0;

    for (int s = 0; s < 4; s++) {
        double sp = setpoints[s];
        std::cout << "--- Setpoint: " << sp << " ---\n";

        if (s == 1) pid.applySchedule("heavy_load");
        if (s == 2) pid.applySchedule("light_load");

        for (int i = 0; i < 400; i++) {
            double u = pid.compute(sp, y, dt);
            y = plant(y, u, 1.0, 0.3, dt);
        }

        auto g = pid.gains();
        std::cout << std::fixed << std::setprecision(4)
                  << "  Final output: " << y
                  << "  kp=" << g.kp
                  << " ki=" << g.ki
                  << " kd=" << g.kd
                  << "\n";
    }

    std::cout << "Log saved to cpp_pid_log.csv\n";
    return 0;
}
