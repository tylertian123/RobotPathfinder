#pragma once

namespace rpf {
    class Vec2D {
    public:
        Vec2D(double x, double y) : x(x), y(y) {}

        double get_x() const;
        double get_y() const;
        double dist(const Vec2D &) const;
        void normalize();
        double magnitude() const;

        Vec2D operator+(const Vec2D &) const;
        Vec2D& operator+=(const Vec2D&);
        Vec2D operator-(const Vec2D &) const;
        Vec2D& operator-=(const Vec2D&);
        Vec2D operator*(double) const;
        Vec2D& operator*=(double);
        friend Vec2D operator*(double, const Vec2D &);
        Vec2D operator/(double) const;
        Vec2D& operator/=(double);

        static Vec2D lerp(const Vec2D &, const Vec2D &, double);

    protected:
        double x, y;
    };
}
