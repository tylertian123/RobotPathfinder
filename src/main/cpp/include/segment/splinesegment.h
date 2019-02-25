#pragma once

#include "math/vec2d.h"

namespace rpf {
    class SplineSegment {
    public:
        virtual Vec2D at(double) const = 0;
        virtual Vec2D deriv_at(double) const = 0;
        virtual Vec2D second_deriv_at(double) const = 0;
    };
}
