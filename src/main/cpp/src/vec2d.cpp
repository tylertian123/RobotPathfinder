#include "vec2d.h"
#include <cmath>

double Vec2D::get_x() const {
    return x;
}
double Vec2D::get_y() const {
    return y;
}

double Vec2D::dist(const Vec2D &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
}

Vec2D Vec2D::operator+(const Vec2D &other) const {
    return Vec2D(x + other.x, y + other.y);
}
Vec2D& Vec2D::operator+=(const Vec2D &other) {
    x += other.x;
    y += other.y;
    return *this;
}
Vec2D Vec2D::operator-(const Vec2D &other) const {
    return Vec2D(x - other.x, y - other.y);
}
Vec2D& Vec2D::operator-=(const Vec2D &other) {
    x -= other.x;
    y -= other.y;
    return *this;
}
