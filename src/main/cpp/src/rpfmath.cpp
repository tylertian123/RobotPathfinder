#include "rpfmath.h"

double lerp(double a, double b, double f) {
    return (a * (1.0 - f)) + (b * f);
}
