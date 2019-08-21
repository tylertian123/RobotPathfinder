#pragma once

#include <limits>

namespace rpf {
    struct RobotSpecs {
        RobotSpecs() {
        }
        RobotSpecs(double max_v, double max_a, double base_width)
                : max_v(max_v), max_a(max_a), base_width(base_width) {
        }
        RobotSpecs(double max_v, double max_a)
                : max_v(max_v), max_a(max_a), base_width(std::numeric_limits<double>::quiet_NaN()) {
        }

        double max_v, max_a;
        double base_width;
    };
} // namespace rpf
