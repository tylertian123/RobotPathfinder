#pragma once

namespace rpf {
    struct Vec2D {
        Vec2D(double x, double y) : x(x), y(y) {
        }
        Vec2D() : x(0), y(0) {
        }

        double dist(const Vec2D &) const;
        void normalize();
        double magnitude() const;
        double dot(const Vec2D &) const;
        Vec2D proj(const Vec2D &) const;
        Vec2D reflect(const Vec2D &) const;

        Vec2D operator+(const Vec2D &) const;
        Vec2D &operator+=(const Vec2D &);
        Vec2D operator-(const Vec2D &) const;
        Vec2D &operator-=(const Vec2D &);
        Vec2D operator*(double) const;
        Vec2D &operator*=(double);
        friend Vec2D operator*(double, const Vec2D &);
        Vec2D operator/(double) const;
        Vec2D &operator/=(double);

        static Vec2D lerp(const Vec2D &, const Vec2D &, double);

        double x, y;
    };
} // namespace rpf
