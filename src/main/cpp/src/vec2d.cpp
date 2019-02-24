#include "vec2d.h"
#include <cmath>
#include "rpfmath.h"

double Vec2D::get_x() const {
    return x;
}
double Vec2D::get_y() const {
    return y;
}

double Vec2D::dist(const Vec2D &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
}
void Vec2D::normalize() {
    double mag = magnitude();
    x /= mag;
    y /= mag;
}
double Vec2D::magnitude() const {
    return std::sqrt(x * x + y * y);
}

Vec2D Vec2D::lerp(const Vec2D &a, const Vec2D &b, double f) {
    return Vec2D(rpf::lerp(a.x, b.x, f), rpf::lerp(a.y, b.y, f));
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
Vec2D Vec2D::operator*(double scalar) const {
    return Vec2D(x * scalar, y * scalar);
}
Vec2D operator*(double scalar, const Vec2D &vec) {
    return Vec2D(vec.x * scalar, vec.y * scalar);
}
Vec2D& Vec2D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}
Vec2D Vec2D::operator/(double scalar) const {
    return Vec2D(x / scalar, y / scalar);
}
Vec2D& Vec2D::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    return *this;
}
