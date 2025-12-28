#pragma once
#include <cmath>

struct Vec2 {
    double x{0.0};
    double y{0.0};

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    // const because does not modify *this
    double norm() const noexcept {return std::hypot(x, y);};

    // take const& to avoid copies
    Vec2 operator+(const Vec2& o) const noexcept {
        return {x + o.x, y + o.y};
    }

    Vec2 operator-(const Vec2& o) const noexcept {
        return {x - o.x, y - o.y};
    }

    Vec2 operator*(double o) const noexcept {
        return {x * o, y * o};
    }

};