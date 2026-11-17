#pragma once

#include <Eigen/Dense>
#include <optional>
#include <cmath>
#include <algorithm>

namespace kinema {

struct JointAngles {
    double theta1{0.0};
    double theta2{0.0}; 
};

struct ArmState {
    Eigen::Vector2d base{0.0, 0.0};
    Eigen::Vector2d elbow{0.0, 0.0};
    Eigen::Vector2d endEffector{0.0, 0.0};
};

class PlanarArmKinematics {
public:
    PlanarArmKinematics(double link1Length, double link2Length)
        : l1_(link1Length), l2_(link2Length) {}

    [[nodiscard]] ArmState forward(const JointAngles& angles) const {
        ArmState state;
        state.elbow.x() = l1_ * std::cos(angles.theta1);
        state.elbow.y() = l1_ * std::sin(angles.theta1);

        const double globalTheta2 = angles.theta1 + angles.theta2;
        state.endEffector.x() = state.elbow.x() + l2_ * std::cos(globalTheta2);
        state.endEffector.y() = state.elbow.y() + l2_ * std::sin(globalTheta2);

        return state;
    }

    [[nodiscard]] JointAngles inverse(const Eigen::Vector2d& target, bool elbowUp = false) const {
        Eigen::Vector2d clampedTarget = clampToWorkspace(target);
        const double x = clampedTarget.x();
        const double y = clampedTarget.y();
        const double rSq = x * x + y * y;

        double cosTheta2 = (rSq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
        cosTheta2 = std::clamp(cosTheta2, -1.0, 1.0);
        if (cosTheta2 > 1.0 - 1e-9) cosTheta2 = 1.0;
        if (cosTheta2 < -1.0 + 1e-9) cosTheta2 = -1.0;
        
        const double sinTheta2 = (elbowUp ? -1.0 : 1.0) * std::sqrt(std::max(0.0, 1.0 - cosTheta2 * cosTheta2));
        const double theta2 = std::atan2(sinTheta2, cosTheta2);

        const double k1 = l1_ + l2_ * cosTheta2;
        const double k2 = l2_ * sinTheta2;
        const double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        return JointAngles{theta1, theta2};
    }

    [[nodiscard]] Eigen::Vector2d clampToWorkspace(const Eigen::Vector2d& target) const {
        const double dist = target.norm();
        const double maxReach = l1_ + l2_;
        const double minReach = std::abs(l1_ - l2_);

        if (dist < 1e-6) {
            return Eigen::Vector2d{minReach, 0.0};
        }
        if (dist > maxReach) {
            return target * (maxReach / dist);
        }
        if (dist < minReach) {
            return target * (minReach / dist);
        }
        return target;
    }

    [[nodiscard]] double getL1() const noexcept { return l1_; }
    [[nodiscard]] double getL2() const noexcept { return l2_; }

private:
    double l1_;
    double l2_;
};

}