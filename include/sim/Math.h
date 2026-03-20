#pragma once

#include <algorithm>
#include <cmath>

namespace sim {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    constexpr Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    constexpr Vec2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    constexpr Vec2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
};

inline constexpr Vec2 operator*(double scalar, const Vec2& v) {
    return v * scalar;
}

inline double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

inline double lengthSquared(const Vec2& v) {
    return dot(v, v);
}

inline double length(const Vec2& v) {
    return std::sqrt(lengthSquared(v));
}

inline Vec2 normalize(const Vec2& v, const Vec2& fallback = {1.0, 0.0}) {
    const double len = length(v);
    if (len <= 1e-12) {
        return fallback;
    }
    return v / len;
}

inline double clamp(double value, double lo, double hi) {
    return std::clamp(value, lo, hi);
}

inline Vec2 closestPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& p) {
    const Vec2 ab = b - a;
    const double denom = lengthSquared(ab);
    if (denom <= 1e-12) {
        return a;
    }
    const double t = clamp(dot(p - a, ab) / denom, 0.0, 1.0);
    return a + ab * t;
}

inline Vec2 lerp(const Vec2& a, const Vec2& b, double t) {
    return a + (b - a) * t;
}

inline double segmentSegmentDistanceSquared(const Vec2& p1,
                                           const Vec2& q1,
                                           const Vec2& p2,
                                           const Vec2& q2) {
    const Vec2 d1 = q1 - p1;
    const Vec2 d2 = q2 - p2;
    const Vec2 r = p1 - p2;
    const double a = lengthSquared(d1);
    const double e = lengthSquared(d2);
    const double f = dot(d2, r);

    double s = 0.0;
    double t = 0.0;

    if (a <= 1e-12 && e <= 1e-12) {
        return lengthSquared(p1 - p2);
    }

    if (a <= 1e-12) {
        t = clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= 1e-12) {
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denom = a * e - b * b;
            if (denom != 0.0) {
                s = clamp((b * f - c * e) / denom, 0.0, 1.0);
            }
            const double tNom = b * s + f;
            if (tNom <= 0.0) {
                t = 0.0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (tNom >= e) {
                t = 1.0;
                s = clamp((b - c) / a, 0.0, 1.0);
            } else {
                t = tNom / e;
            }
        }
    }

    const Vec2 c1 = p1 + d1 * s;
    const Vec2 c2 = p2 + d2 * t;
    return lengthSquared(c1 - c2);
}

}  // namespace sim
