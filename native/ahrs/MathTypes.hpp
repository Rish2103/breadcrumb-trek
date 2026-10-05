#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

static inline float invSqrtSafe(float x) {
    constexpr float eps = 1.0e-12f;
    return 1.0f / std::sqrt(std::max(x, eps));
}

static inline Quaternion normalizeQ(Quaternion q) {
    const float r = invSqrtSafe(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w * r, q.x * r, q.y * r, q.z * r};
}

static inline Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

// Rotates vector from body frame to Earth NWU frame: v_e = q ⊗ v_b ⊗ q*
static inline Vec3 rotateBodyToEarth(const Quaternion& q, const Vec3& v) {
    const Quaternion vq{0.0f, v.x, v.y, v.z};
    const Quaternion qc{q.w, -q.x, -q.y, -q.z};
    const Quaternion r = quatMultiply(quatMultiply(q, vq), qc);
    return {r.x, r.y, r.z};
}

// Wraps angle in radians to (-π, π]
static inline double wrapPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

/**
 * Returns compass bearing (rad, clockwise from magnetic North) of the phone's forward direction.
 * Per §13.1: selects more-horizontal of +y_b (held flat) or -z_b (held upright).
 */
static inline double phoneForwardBearing(const Quaternion& q) {
    // Columns of R(q) in earth (North, West, Up) NWU frame
    const double yN = 2.0 * (q.x * q.y - q.w * q.z);
    const double yW = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    const double zN = 2.0 * (q.x * q.z + q.w * q.y);
    const double zW = 2.0 * (q.y * q.z - q.w * q.x);

    const double hA = std::hypot(yN, yW); // +y_b horizontal extent
    const double hB = std::hypot(zN, zW); // -z_b horizontal extent

    double fN, fW;
    if (hA >= hB) {
        fN =  yN; fW =  yW;
    } else {
        fN = -zN; fW = -zW;
    }

    // Convert NWU to ENU bearing: bearing = atan2(E, N) with E = -W
    return wrapPi(std::atan2(-fW, fN));
}
