#pragma once

#include "MathTypes.hpp"
#include <algorithm>
#include <cmath>

/**
 * 9-DOF Madgwick AHRS implementation with reliability-weighted magnetic gradient.
 * Adheres strictly to gpt_technical_final.md §8.3.
 *
 * Earth frame convention: NWU (x = North, y = West, z = Up).
 */
class Madgwick9DOF {
public:
    Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
    float beta = 0.08f; // Tunable filter gain
    bool decoupleYawAccelGradient = false; // Variant B diagnostic flag (false = Variant A, true = Variant B)

    /**
     * Updates quaternion orientation from sensor readings.
     *
     * @param gx, gy, gz Gyroscope in rad/s (bias-corrected)
     * @param ax, ay, az Accelerometer in m/s²
     * @param mx, my, mz Magnetometer in µT
     * @param Cmag       Continuous magnetic reliability weight in [0.0, 1.0]
     * @param dt         Time delta in seconds
     */
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float Cmag, float dt) {

        if (dt <= 0.0f || dt > 0.25f) return;

        const float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

        // Normalise accelerometer measurement
        const float aNorm2 = ax * ax + ay * ay + az * az;
        const bool hasAccel = aNorm2 > 1.0e-4f;
        if (hasAccel) {
            const float ia = invSqrtSafe(aNorm2);
            ax *= ia; ay *= ia; az *= ia;
        }

        // Normalise magnetometer measurement
        const float mNorm2 = mx * mx + my * my + mz * mz;
        const bool hasMag = mNorm2 > 1.0e-4f && Cmag > 0.001f;
        if (hasMag) {
            const float im = invSqrtSafe(mNorm2);
            mx *= im; my *= im; mz *= im;
        } else {
            Cmag = 0.0f;
        }
        Cmag = std::clamp(Cmag, 0.0f, 1.0f);

        // Rate of change of quaternion from gyroscope
        float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float qd1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        float qd2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        float qd3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        if (hasAccel) {
            const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
            const float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            const float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
            const float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
            const float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

            // Gravity residuals: f_g(q, a)
            const float f1 = 2.0f * (q1q3 - q0q2) - ax;
            const float f2 = 2.0f * (q0q1 + q2q3) - ay;
            const float f3 = 2.0f * (0.5f - q1q1 - q2q2) - az;

            // Gradient of gravity residuals: J_g^T * f_g
            const float a0 = -_2q2 * f1 + _2q1 * f2;
            const float a1 =  _2q3 * f1 + _2q0 * f2 - 4.0f * q1 * f3;
            const float a2 = -_2q0 * f1 + _2q3 * f2 - 4.0f * q2 * f3;
            const float a3 = decoupleYawAccelGradient ? 0.0f : (_2q1 * f1 + _2q2 * f2);

            float m0g = 0.0f, m1g = 0.0f, m2g = 0.0f, m3g = 0.0f;

            if (Cmag > 0.0f) {
                // Earth magnetic reference direction (x = North, z = Up)
                const float _2q0mx = 2.0f * q0 * mx;
                const float _2q0my = 2.0f * q0 * my;
                const float _2q0mz = 2.0f * q0 * mz;
                const float _2q1mx = 2.0f * q1 * mx;

                const float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
                               + 2.0f * q1 * my * q2 + 2.0f * q1 * mz * q3 - mx * q2q2 - mx * q3q3;
                const float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
                               - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
                const float _2bx = std::sqrt(hx * hx + hy * hy);
                const float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
                                 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
                const float _4bx = 2.0f * _2bx, _4bz = 2.0f * _2bz;

                // Magnetic residuals: f_b(q, b̂, m)
                const float m1 = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
                const float m2 = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
                const float m3 = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;

                // Gradient of magnetic residuals: J_b^T * f_b
                m0g = -_2bz * q2 * m1 + (-_2bx * q3 + _2bz * q1) * m2 + _2bx * q2 * m3;
                m1g =  _2bz * q3 * m1 + ( _2bx * q2 + _2bz * q0) * m2 + (_2bx * q3 - _4bz * q1) * m3;
                m2g = (-_4bx * q2 - _2bz * q0) * m1 + (_2bx * q1 + _2bz * q3) * m2 + (_2bx * q0 - _4bz * q2) * m3;
                m3g = (-_4bx * q3 + _2bz * q1) * m1 + (-_2bx * q0 + _2bz * q2) * m2 + _2bx * q1 * m3;
            }

            // Dynamic magnetic-gradient scaling: s = s_acc + Cmag * s_mag
            float s0 = a0 + Cmag * m0g;
            float s1 = a1 + Cmag * m1g;
            float s2 = a2 + Cmag * m2g;
            float s3 = a3 + Cmag * m3g;

            const float is = invSqrtSafe(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= is; s1 *= is; s2 *= is; s3 *= is;

            // Apply gradient descent correction step
            qd0 -= beta * s0;
            qd1 -= beta * s1;
            qd2 -= beta * s2;
            qd3 -= beta * s3;
        }

        // Integrate rate of change and normalise
        q.w += qd0 * dt;
        q.x += qd1 * dt;
        q.y += qd2 * dt;
        q.z += qd3 * dt;
        q = normalizeQ(q);
    }
};
