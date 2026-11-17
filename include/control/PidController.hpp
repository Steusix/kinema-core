#pragma once

#include <algorithm>

namespace kinema {

class PidController {
public:
    PidController(double kp, double ki, double kd, double maxOutput = 50.0)
        : kp_(kp), ki_(ki), kd_(kd), maxOutput_(maxOutput) {}

    double update(double target, double current, double dt) {
        if (dt <= 0.0) return 0.0;

        const double error = target - current;

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -10.0, 10.0);

        const double derivative = (error - prevError_) / dt;
        prevError_ = error;

        const double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        return std::clamp(output, -maxOutput_, maxOutput_);
    }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double maxOutput_;

    double integral_{0.0};
    double prevError_{0.0};
};

} 