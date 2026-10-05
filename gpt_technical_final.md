# Breadcrumb Trek 2.2 --- Agent-Ready Engineering Specification

## Offline, Confidence-Aware Trail Backtracking for Pedestrian Safety

### Consolidated, Implementation-Ready Engineering Specification: iQOO Hackathon Edition

> **One-line pitch:** *Breadcrumb Trek remembers where you walked, knows
> how much to trust that memory, and guides you back along it, even when
> network connectivity is unavailable and GNSS cannot be relied upon.*

> **Agent instruction:** This document is the technical source of truth
> for implementing Breadcrumb Trek from an empty repository. Build the
> smallest complete end-to-end system first. Do not invent unsupported
> hardware capabilities, accuracy numbers, sensor behavior, or AI
> results. Treat thresholds and calibration values as tunable starting
> parameters, not scientific constants.

------------------------------------------------------------------------

## 0. About this document

This file merges and corrects three sources:

1.  `Initial_MD.txt` (v1): executive summary and problem statement (it
    stopped at "3. System Architecture").
2.  `gpt_technical_6_.md` (v2): the 100-section engineering
    specification.
3.  A review pass that found errors in v2, filled gaps, and added a
    hackathon execution plan.

### 0.1 Change log (v2 → v2.1)

  ------------------------------------------------------------------------------
  \#            Change                                    Why
  ------------- ----------------------------------------- ----------------------
  1             **Gravity sign fixed.** Linear accel =    Android accelerometers
                `R(q)·a_b − [0,0,+g]`                     report *specific
                                                          force*: a phone lying
                                                          flat reads **+9.81 on
                                                          z**. v2 subtracted
                                                          `[0,0,−g]` (gives ≈
                                                          2g).

  2             **One heading convention everywhere:**    v2 used `cosψ/sinψ`
                compass bearing ψ, clockwise from North;  for x/y (angle from
                `ΔE = L·sinψ`, `ΔN = L·cosψ`              East) in §24/§27 but
                                                          `atan2(dx, dy)`
                                                          (bearing) in guidance.
                                                          They are inconsistent.

  3             **Earth frame stated correctly.** The     v2 labelled it ENU.
                Madgwick MARG formulation yields x =      
                magnetic North, y = West, z = Up (NWU).   
                We convert to E/N/U for the trail         

  4             **Per-sensor push API**                   v2's skeleton called
                (`pushAccel/pushGyro/pushMag/pushBaro`)   `nativeUpdate` on
                with staleness checks                     every event of any
                                                          sensor, creating
                                                          duplicate samples
                                                          (dt≈0) and reusing
                                                          stale mag data.

  5             Pressure `0.0f` placeholder removed;      A zero pressure would
                barometer is optional and guarded         corrupt the vertical
                                                          estimate.

  6             Magnetic reliability adds a **dip-angle   Magnitude-only checks
                consistency** term                        miss disturbances that
                                                          rotate the field
                                                          without changing its
                                                          strength.

  7             Standard barometric formula used          v2's variant was a
                                                          slightly different
                                                          approximation.

  8             180° tie-break uses the **guidance        The guidance vector is
                vector**, not just the segment tangent    what steering actually
                                                          uses.

  9             **Haptics redesigned for a single motor** A phone has one
                (pulse *patterns*, not left/right pulses) vibration motor and
                                                          cannot pulse "left".

  10            "Airplane Mode" demo note: also switch    Airplane mode
                **Location off**                          generally does not
                                                          disable the GNSS
                                                          receiver (verify on
                                                          the demo phone).

  11            Removed stray tool-citation artifacts;    Housekeeping.
                fixed section numbering; renamed          
                overloaded symbols                        

  12            **Added** concrete step detector, PDR,    v2 only declared these
                forward-axis heading, reverse matcher,    or described them
                and guidance code; the AI model           abstractly.
                specification; calibration procedures;    
                uncertainty model; record/replay harness; 
                build plan and risk register              

  13            **Made the reverse-matching MVP equation  Expected progress is
                identical to the C++ implementation**     enforced by the
                                                          monotonic
                                                          progress-pointer
                                                          window, so a separate
                                                          progress term is
                                                          omitted from the MVP
                                                          cost.

  14            **Added an explicit agent execution       The implementation
                contract and acceptance gates**           agent can start from
                                                          an empty repository
                                                          without inventing the
                                                          architecture or
                                                          completion criteria.
  ------------------------------------------------------------------------------

------------------------------------------------------------------------

## 1. Executive summary

**Breadcrumb Trek** is an on-device pedestrian safety app for places
where:

-   cellular connectivity is unavailable,
-   cloud services cannot be reached,
-   GNSS is degraded, intermittent, or deliberately not relied on,
-   or a user needs to **retrace a locally travelled path** rather than
    learn a global position.

It does **not** claim perfect inertial positioning. It builds an
**ordered, local, confidence-aware memory** of the user's journey and
uses it for **closed-loop reverse navigation**.

**Technique stack:** accelerometer, gyroscope, magnetometer, barometer →
quaternion attitude estimation (9-DOF Madgwick with reliability-weighted
magnetic correction) → step detection → step-length estimation →
Pedestrian Dead Reckoning (PDR) → stationary detection and gyro-bias
recalibration → relative barometric elevation → lightweight on-device AI
→ confidence propagation → ordered trail memory → sequence-aware reverse
matching → carrot-point guidance with cross-track error → visual and
haptic feedback.

> **Core principle:** PDR estimates movement. Breadcrumb Trek turns that
> movement into a *confidence-aware journey memory* and then uses that
> memory for *closed-loop reverse navigation*.

------------------------------------------------------------------------

## 2. Problem statement

### 2.1 Cellular blackouts

Cloud-dependent navigation (online tiles, geocoding, remote route
calculation) stops working without data, unless maps were cached
beforehand.

### 2.2 Cellular loss ≠ GNSS loss (important correction)

Cellular and GNSS are separate systems. GNSS is receive-only and can
work with no network, and cellular loss does not automatically mean GNSS
loss. The target scenario is specifically where **GNSS is unreliable,
unavailable, or not relied upon**.

  -----------------------------------------------------------------------
  Do say                              Do not say
  ----------------------------------- -----------------------------------
  "Local backtracking without         "No network means GPS doesn't
  requiring network connectivity or a work."
  live GNSS fix."                     

  -----------------------------------------------------------------------

### 2.3 Why GNSS degrades in rugged terrain

-   **Multipath:** signals reflect off rock faces and wet canopy before
    reaching the antenna, causing pseudorange errors. v1 quotes position
    shifts of tens of metres (30--100+ m); treat that range as
    indicative unless you cite a source.
-   **Poor geometry (GDOP):** in gorges and canyons only a thin strip of
    sky is visible, so satellites are nearly collinear and precision
    dilutes.
-   **Battery drain:** repeated reacquisition and tracking loops can
    drain the battery quickly. v1 quotes 2--4 hours; this is
    device-dependent, so cite or measure before claiming.

### 2.4 The disorientation loop

In featureless terrain, humans systematically lose their straight line
when they lack landmarks or a sun reference; research on walking without
visual cues reports people veering into loops. v1 quoted "circles of
200--500 m"; that figure is unsourced here, so cite a paper before
showing a number to judges.

### 2.5 The reframed question

  -----------------------------------------------------------------------
  Traditional navigation              Breadcrumb Trek
  ----------------------------------- -----------------------------------
  "Where am I **globally**?" (needs   "Where have I been **locally**, and
  satellites, maps, network)          how do I retrace it?"

  -----------------------------------------------------------------------

The system needs no latitude/longitude to tell the user which way to
walk, whether they are drifting from their path, which segment comes
next, or when its own estimate has become unreliable.

------------------------------------------------------------------------

## 3. Engineering scope: three separable things

1.  **Deterministic navigation mathematics.** The navigation core must
    keep working if the AI model is absent.
2.  **AI-assisted sensor interpretation.** It adapts parameters within
    hard bounds; it never replaces the equations.
3.  **Android/NDK implementation.** The native layer is chosen for
    deterministic numerical code, low allocation pressure, a reusable
    testable core, and clean separation from Android I/O. **It is not
    because Kotlin cannot handle 50 Hz**, which is modest for a modern
    phone. Do not claim otherwise.

------------------------------------------------------------------------

## 4. The core insight: why local backtracking is easier than global positioning

*(This is reasoning added in v2.1; validate it experimentally before
presenting it as a result.)*

Suppose the heading estimate has a **constant bias** θ, or step length
is consistently scaled by s. The recorded outbound path is the true path
rotated by θ and scaled by s. On the way back the same biases act on the
live estimate, so following the *remembered* path in the *estimated*
frame brings the user back along the *physical* path: **constant errors
largely cancel**.

What does *not* cancel: - **drift** (errors that change during the walk,
e.g., gyro bias walk), - **transient disturbances** (magnetic anomalies
on the outbound but not the return, or the reverse), - **carry-mode
changes** between outbound and return, - **step-detection misses/false
positives** that differ between legs.

Therefore the engineering effort goes into: drift control, magnetic
reliability, stationary recalibration, and honest confidence reporting.

------------------------------------------------------------------------

## 5. System architecture

``` text
 Smartphone sensors:  Accelerometer · Gyroscope · Magnetometer · Barometer
                                  │
                                  ▼
                 Timestamp layer (per-sensor push, staleness checks)
                                  │
                                  ▼
                 Sensor quality gate (missing / stale / noisy)
                                  │
              ┌───────────────────┴────────────────────┐
              ▼                                        ▼
   On-device AI (motion, carry,               Deterministic navigation core
   turn, sensor reliability)                  9-DOF AHRS · step · PDR
              └───────────────────┬────────────────────┘
                                  ▼
                 Confidence & uncertainty estimation
                                  ▼
                 Ordered trail memory (nodes + segments)
                                  │  RETURN TO START
                                  ▼
          Reverse segment matcher + carrot guidance + XTE + steering
                         ┌────────┴────────┐
                         ▼                 ▼
                    Visual UI          Haptic UI
```

------------------------------------------------------------------------

## 6. Conventions (read before implementing)

### 6.1 Units

Acceleration m/s², angular rate **rad/s** (Android native), magnetic
field µT, pressure hPa, time ns (`SensorEvent.timestamp`), distances m,
angles rad.

### 6.2 Frames

-   **Body frame (b):** phone axes as reported by Android (x right, y
    toward top of the screen, z out of the screen when held portrait).
-   **AHRS earth frame (e):** the Madgwick MARG formulation puts **x =
    magnetic North, y = West, z = Up** (NWU). The quaternion `q` rotates
    body → earth: `v_e = R(q)·v_b`.
-   **Trail/navigation frame (n):** **E = −y_e, N = x_e, U = z_e**. The
    origin is the start of the journey, `p0 = (0, 0, 0)`. No
    latitude/longitude is required. Magnetic north is fine; only
    relative geometry matters.

### 6.3 Heading (bearing) convention

ψ = compass bearing, **clockwise from North**, wrapped to \[−π, π).

``` text
ΔE = L·sin ψ        ΔN = L·cos ψ
bearing of vector (dE, dN) = atan2(dE, dN)
bearing of earth-frame horizontal vector (vN, vW) = atan2(−vW, vN)
heading unit vector h = [sin ψ, cos ψ]  (E, N order)
steering error  Δψ = wrap(ψ_desired − ψ_user)     (Δψ > 0 ⇒ turn RIGHT / clockwise)
wrap(a) = atan2(sin a, cos a)  ∈ (−π, π]
```

### 6.4 Rotation matrix

``` text
R(q) = [ 1−2(y²+z²)   2(xy−wz)    2(xz+wy) ]
       [ 2(xy+wz)    1−2(x²+z²)   2(yz−wx) ]
       [ 2(xz−wy)    2(yz+wx)    1−2(x²+y²) ]
```

(w, x, y, z are the quaternion components.)

------------------------------------------------------------------------

## 7. Sensing layer

### 7.1 Sensors and roles

  -----------------------------------------------------------------------
  Sensor                  Role                    Failure mode
  ----------------------- ----------------------- -----------------------
  Accelerometer           gravity direction       corrupted during
                          (tilt), step events,    aggressive motion
                          stride features         

  Gyroscope               short-term orientation, **bias drift**
                          turn rate               

  Magnetometer            absolute yaw reference  distorted by metal,
                                                  rock, electronics

  Barometer *(if          relative height         weather drift; absent
  present)*                                       on some phones
  -----------------------------------------------------------------------

Barometer is **optional**: `getDefaultSensor(TYPE_PRESSURE) == null`
must be handled (vertical confidence = "unavailable").

### 7.2 Sampling

Request an explicit period (`registerListener(listener, sensor, 20_000)`
µs ≈ 50 Hz for accel/gyro; slower for mag and baro). Android 12+ caps
default sensor rates at 200 Hz without a special permission, so 50 Hz is
unaffected.

### 7.3 Synchronisation (simplified from v2 §50)

-   Each sensor has a **push** entry: `pushAccel`, `pushGyro`,
    `pushMag`, `pushBaro`, each with its own timestamp.
-   The AHRS update runs **on each gyro sample** (`dt` = gyro timestamp
    difference), using the *latest* accel/mag **only if fresh**:
    -   accel older than \~100 ms → treat as missing (zero vector ⇒ zero
        accel gradient),
    -   mag older than \~300 ms → `Cmag = 0`.
-   The step detector runs on each accel sample.
-   Full interpolation via per-sensor ring buffers (timestamp, x, y, z)
    is the production upgrade, not required for the MVP.
-   Reject non-monotonic timestamps; ignore `dt ≤ 0` or `dt > 0.25 s`
    (example guard, tune).

### 7.4 Sensor health state

Maintain `GOOD / DEGRADED / BAD` for accel, gyro, mag, baro, gait.
Display it so judges see *why* confidence moves. Use `onAccuracyChanged`
for the magnetometer and prompt a figure-8 calibration when accuracy
drops.

------------------------------------------------------------------------

## 8. Attitude estimation (AHRS)

### 8.1 Gravity removal (corrected)

At rest the accelerometer reads **+g along the up axis** (specific
force). Therefore:

``` text
a_e   = R(q) · a_b
a_lin = a_e − [0, 0, +g]ᵀ        (earth frame, z up)
```

Do not assume the AHRS is perfect in aggressive motion. For step
detection a scalar `|a_b| − g` signal is used and does not depend on
orientation.

### 8.2 Quaternion propagation

`q̇ = ½ q ⊗ [0, ω]`. First-order: `q ← normalize(q + q̇·dt)`. An exact
increment can be used instead:

``` text
δθ = ‖ω_c‖·dt ;  δq = [cos(δθ/2), u·sin(δθ/2)], u = ω_c/‖ω_c‖ ;  q ← q ⊗ δq
```

where `ω_c = ω_measured − b̂_g` (bias-corrected).

### 8.3 Madgwick 9-DOF with reliability-weighted magnetic gradient

``` text
s = s_acc + Cmag · s_mag         (gradient of the accel and mag residuals)
q̇ = ½ q ⊗ [0, ωx, ωy, ωz] − β · s/‖s‖
```

The gradient path does **not branch** on mag validity: a zero vector
normalises (with a positive floor) to a zero contribution. `β` is a
calibration parameter, not a constant of nature.

``` cpp
#include <algorithm>
#include <cmath>

struct Vec3 { float x, y, z; };
struct Quaternion { float w, x, y, z; };

static inline float invSqrtSafe(float x) {
    constexpr float eps = 1.0e-12f;
    return 1.0f / std::sqrt(std::max(x, eps));
}
static inline Quaternion normalizeQ(Quaternion q) {
    const float r = invSqrtSafe(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return {q.w*r, q.x*r, q.y*r, q.z*r};
}

class Madgwick9DOF {
public:
    Quaternion q{1.f, 0.f, 0.f, 0.f};
    float beta = 0.08f; // calibration parameter

    // gyro in rad/s (bias-corrected), accel in m/s², mag in µT; dt in seconds
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float Cmag, float dt) {

        const float q0=q.w, q1=q.x, q2=q.y, q3=q.z;

        const float ia=invSqrtSafe(ax*ax+ay*ay+az*az);
        ax*=ia; ay*=ia; az*=ia;
        const float im=invSqrtSafe(mx*mx+my*my+mz*mz);
        mx*=im; my*=im; mz*=im;
        Cmag=std::clamp(Cmag,0.f,1.f);

        float qd0=0.5f*(-q1*gx-q2*gy-q3*gz);
        float qd1=0.5f*( q0*gx+q2*gz-q3*gy);
        float qd2=0.5f*( q0*gy-q1*gz+q3*gx);
        float qd3=0.5f*( q0*gz+q1*gy-q2*gx);

        const float _2q0=2.f*q0, _2q1=2.f*q1;
        const float _2q2=2.f*q2, _2q3=2.f*q3;
        const float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
        const float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
        const float q2q2=q2*q2, q2q3=q2*q3, q3q3=q3*q3;

        // Gravity residuals
        const float f1=2.f*(q1q3-q0q2)-ax;
        const float f2=2.f*(q0q1+q2q3)-ay;
        const float f3=2.f*(0.5f-q1q1-q2q2)-az;

        const float a0=-_2q2*f1+_2q1*f2;
        const float a1= _2q3*f1+_2q0*f2-4.f*q1*f3;
        const float a2=-_2q0*f1+_2q3*f2-4.f*q2*f3;
        const float a3= _2q1*f1+_2q2*f2;

        // Earth magnetic reference direction (x = North, z = Up)
        const float _2q0mx=2.f*q0*mx;
        const float _2q0my=2.f*q0*my;
        const float _2q0mz=2.f*q0*mz;
        const float _2q1mx=2.f*q1*mx;

        const float hx=mx*q0q0-_2q0my*q3+_2q0mz*q2+mx*q1q1
                     +2.f*q1*my*q2+2.f*q1*mz*q3-mx*q2q2-mx*q3q3;
        const float hy=_2q0mx*q3+my*q0q0-_2q0mz*q1+_2q1mx*q2
                     -my*q1q1+my*q2q2+_2q2*mz*q3-my*q3q3;
        const float _2bx=std::sqrt(hx*hx+hy*hy);
        const float _2bz=-_2q0mx*q2+_2q0my*q1+mz*q0q0+_2q1mx*q3
                        -mz*q1q1+_2q2*my*q3-mz*q2q2+mz*q3q3;
        const float _4bx=2.f*_2bx, _4bz=2.f*_2bz;

        const float m1=_2bx*(0.5f-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx;
        const float m2=_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my;
        const float m3=_2bx*(q0q2+q1q3)+_2bz*(0.5f-q1q1-q2q2)-mz;

        const float m0g=-_2bz*q2*m1+(-_2bx*q3+_2bz*q1)*m2+_2bx*q2*m3;
        const float m1g=_2bz*q3*m1+(_2bx*q2+_2bz*q0)*m2+(_2bx*q3-_4bz*q1)*m3;
        const float m2g=(-_4bx*q2-_2bz*q0)*m1+(_2bx*q1+_2bz*q3)*m2+(_2bx*q0-_4bz*q2)*m3;
        const float m3g=(-_4bx*q3+_2bz*q1)*m1+(-_2bx*q0+_2bz*q2)*m2+_2bx*q1*m3;

        // Dynamic magnetic-gradient scaling
        float s0=a0+Cmag*m0g;
        float s1=a1+Cmag*m1g;
        float s2=a2+Cmag*m2g;
        float s3=a3+Cmag*m3g;
        const float is=invSqrtSafe(s0*s0+s1*s1+s2*s2+s3*s3);
        s0*=is; s1*=is; s2*=is; s3*=is;

        qd0-=beta*s0; qd1-=beta*s1; qd2-=beta*s2; qd3-=beta*s3;

        q.w+=qd0*dt; q.x+=qd1*dt; q.y+=qd2*dt; q.z+=qd3*dt;
        q=normalizeQ(q);
    }
};
```

**Validation requirement:** "complete" is not "production-ready".
Unit-test known rotations, variable `dt`, static gravity, magnetic
disturbance, quaternion norm, and profile on the target phone.

### 8.4 Quaternion helpers

``` cpp
// body → earth (NWU) using v_e = q ⊗ v_b ⊗ q*
Vec3 rotateBodyToEarth(const Quaternion& q, const Vec3& v) {
    Quaternion vq{0.f, v.x, v.y, v.z};
    Quaternion qc{q.w, -q.x, -q.y, -q.z};
    Quaternion r = quatMultiply(quatMultiply(q, vq), qc);
    return {r.x, r.y, r.z};
}
```

------------------------------------------------------------------------

## 9. Magnetic reliability

### 9.1 Do not use a fixed global baseline

At a stationary calibration stage, estimate the local field strength and
dip angle:

``` text
B0 = mean(‖m_i‖)            σB² = var(‖m_i‖)
dip0 = mean( angle between m_i and gravity )
```

### 9.2 Continuous reliability

``` text
d_B   = |B_k − B0| / max(B0, ε)                (strength deviation)
d_dip = |dip_k − dip0|                          (dip-angle deviation, rad)    ← added in v2.1
Cmag  = exp(−λ_B·d_B) · exp(−λ_σ·σ_B,window) · exp(−λ_dip·d_dip),  clamped to [0,1]
```

All λ are calibration parameters from recorded data. (The v2 reference
values `λ_B = 4`, `λ_σ = 2` are starting guesses only.)

``` cpp
double computeMagReliability(double B, double B0, double windowStd,
                             double dip, double dip0) {
    if (B0 < 1e-6) return 0.0;
    const double dB   = std::abs(B - B0) / B0;
    const double dDip = std::abs(dip - dip0);
    const double c = std::exp(-4.0*dB) * std::exp(-2.0*windowStd) * std::exp(-3.0*dDip);
    return std::clamp(c, 0.0, 1.0);
}
```

### 9.3 Use as a weight, not a switch

``` text
β_mag = Cmag · β_max
```

### 9.4 Observability limitation (required statement)

Without an external yaw reference, accelerometer + gyroscope cannot give
indefinitely drift-free yaw. **When magnetic correction is unavailable
the system continues short-term gyro propagation and explicitly
increases heading uncertainty.** This is a physical limit, not a bug.

------------------------------------------------------------------------

## 10. Gyro bias and stationary detection

### 10.1 Bias model

`ω_m = ω_true + b_g + n_g`. In high-confidence stationary periods:
`b̂_g ← (1−α)·b̂_g + α·ω_m`, 0 \< α \< 1.

### 10.2 Stationary detection

A candidate requires, over a minimum duration `T_stationary`:

``` text
| ‖a‖ − g | < ε_A     ‖ω‖ < ε_G     std(‖a‖) < σ_A     std(‖ω‖) < σ_G
```

Thresholds are device-dependent calibration parameters.

### 10.3 Naming

Call this **"Stationary Detection + Bias Recalibration"**, not ZUPT. A
true ZUPT needs a velocity state and an EKF zero-velocity update (state
`[p, v, q, b_a, b_g]`, measurement `z = [0,0,0]`, `h(x) = v`). That is a
stretch goal; do not use the term unless it is implemented.

------------------------------------------------------------------------

## 11. Step detection

``` text
|a| − g  →  slow-baseline removal + low-pass (band-pass)  →  peak candidates
        →  adaptive threshold (μ + kσ)  →  prominence check
        →  interval constraint  →  periodicity check  →  verified step
```

Adaptive threshold: `T = clip(μ_s + k·σ_s, T_min, T_max)`. Passband,
`k`, prominence, and intervals come from recorded walking data, not
assumed constants.

``` cpp
class StepDetector {
public:
    struct Config {
        float hpHz = 0.5f, lpHz = 4.0f;       // start values; tune on data
        float k = 0.5f, thrMin = 0.4f, thrMax = 4.0f;
        float minProminence = 0.5f;           // m/s²
        float minIntervalS = 0.25f, maxIntervalS = 1.5f;
    };
    struct Step { bool detected=false; uint64_t tNs=0; float aMax=0, aMin=0, conf=0; };

    Config cfg;

    // aMag = ‖a_b‖ in m/s²
    Step push(float aMag, uint64_t tNs) {
        Step out;
        if (!init_) { init_=true; slow_=aMag; lastT_=tNs; tPrev_=tNs;
                      rawMax_=rawMin_=aMag; return out; }
        const float dt = float(tNs - lastT_) * 1e-9f;
        lastT_ = tNs;
        if (dt <= 0.f || dt > 0.25f) return out;

        constexpr float TWO_PI = 6.2831853f;
        const float aHp = dt / (1.f/(TWO_PI*cfg.hpHz) + dt);
        const float aLp = dt / (1.f/(TWO_PI*cfg.lpHz) + dt);
        slow_ += aHp * (aMag - slow_);             // slow baseline (≈ gravity)
        lp_   += aLp * ((aMag - slow_) - lp_);     // band-limited gait signal
        const float s0 = lp_;

        rawMax_ = std::max(rawMax_, aMag);
        rawMin_ = std::min(rawMin_, aMag);
        valleyMin_ = std::min(valleyMin_, s0);

        // adaptive statistics (per-sample EMA)
        constexpr float aS = 0.02f;
        mu_  += aS * (s0 - mu_);
        var_ += aS * ((s0 - mu_)*(s0 - mu_) - var_);
        const float thr = std::clamp(mu_ + cfg.k*std::sqrt(std::max(var_,0.f)),
                                     cfg.thrMin, cfg.thrMax);

        // peak at the middle sample (s1_) of three
        if (s1_ > s2_ && s1_ >= s0 && s1_ > thr &&
            (s1_ - valleyMin_) > cfg.minProminence) {
            const float interval = lastStepT_ ? float(tPrev_ - lastStepT_) * 1e-9f : 1e9f;
            if (interval >= cfg.minIntervalS) {
                float conf;
                if (emaInt_ > 0.f && interval <= cfg.maxIntervalS)
                    conf = std::exp(-2.f * std::abs(interval - emaInt_) / emaInt_);
                else conf = 0.4f;                  // first step or after a pause
                if (interval <= cfg.maxIntervalS)
                    emaInt_ = emaInt_ > 0.f ? 0.8f*emaInt_ + 0.2f*interval : interval;
                out = {true, tPrev_, rawMax_, rawMin_, conf};
                lastStepT_ = tPrev_;
                rawMax_ = rawMin_ = aMag;
                valleyMin_ = s0;
            }
        }
        s2_ = s1_; s1_ = s0; tPrev_ = tNs;
        return out;
    }

private:
    bool init_=false;
    float slow_=0, lp_=0, mu_=0, var_=0, s1_=0, s2_=0;
    float rawMax_=0, rawMin_=0, valleyMin_=0, emaInt_=0;
    uint64_t lastT_=0, tPrev_=0, lastStepT_=0;
};
```

------------------------------------------------------------------------

## 12. Step length

### 12.1 Weinberg-style model

``` text
L_k = K_mode · (A_max,k − A_min,k)^(1/4)
```

`K_mode` depends on user, carry mode, motion mode, and walking style; it
is **not universal**.

### 12.2 Per-user calibration (do this before every demo)

Walk a tape-measured straight distance `D` (e.g., 20 m) naturally, then:

``` text
K = D / Σ_k (A_max,k − A_min,k)^(1/4)
```

### 12.3 AI stride correction (bounded)

``` text
K_k = clip( K_mode · (1 + δ_k), K_min, K_max ),   δ_k = f_θ(F_k)
```

`F_k` = recent motion features; the clip prevents physically implausible
stride.

------------------------------------------------------------------------

## 13. Walking heading

The walking direction is not the phone's screen-forward axis in general.
Two levels:

### 13.1 MVP: hand-held carry

Support one clearly stated carry mode first: **phone held in hand,
screen toward the user**. The "forward" direction is the more-horizontal
of two body axes: **+y_b** (phone held flat) or **−z_b** (phone upright,
camera direction). Add hysteresis so the choice does not flicker.

``` cpp
// Returns compass bearing (rad, clockwise from magnetic North) of the phone's forward direction.
double phoneForwardBearing(const Quaternion& q) {
    // columns of R(q) in earth (N, W, U) frame
    const double yN = 2*(q.x*q.y - q.w*q.z), yW = 1 - 2*(q.x*q.x + q.z*q.z);
    const double zN = 2*(q.x*q.z + q.w*q.y), zW = 2*(q.y*q.z - q.w*q.x);
    const double hA = std::hypot(yN, yW);   // +y_b horizontal extent
    const double hB = std::hypot(zN, zW);   // -z_b horizontal extent
    double fN, fW;
    if (hA >= hB) { fN =  yN; fW =  yW; }
    else          { fN = -zN; fW = -zW; }
    return std::atan2(-fW, fN);             // bearing = atan2(E, N), E = −W
}
```

### 13.2 Stretch: other carry modes (pocket, bag)

Estimate the horizontal walking axis from the principal component (PCA)
of earth-frame horizontal linear acceleration over several steps, and
store `ψ_offset = ψ_walk − ψ_phone`. **Unresolved:** PCA gives an axis,
not a direction (a ±180° ambiguity). It must be resolved by a
calibration walk, heel-strike acceleration asymmetry, or the AI
classifier. Do not claim pocket/bag support until this is validated.

------------------------------------------------------------------------

## 14. Pedestrian dead reckoning and vertical estimate

Per verified step `k`, with bearing ψ_k:

``` text
E_k = E_{k−1} + L_k · sin ψ_k
N_k = N_{k−1} + L_k · cos ψ_k
U_k = Û_k  (barometric, relative)
```

State: `x = [E, N, U, ψ, s]ᵀ` (s = stride scale), input
`u = [L, Δψ, ΔU]`, with `ψ_{k+1} = wrap(ψ_k + Δψ_k)`.

### 14.1 Barometric height (relative only)

``` text
Δh = 44330 · ( 1 − (P / P_ref)^0.1903 )
```

`P_ref` is the pressure at start; low-pass the signal; use short-window
differences because weather drifts pressure. Keep **vertical confidence
separate** from horizontal confidence; the barometer must never dominate
horizontal navigation.

------------------------------------------------------------------------

## 15. Confidence and uncertainty

### 15.1 Components

  ----------------------------------------------------------------------------
  Confidence                          Depends on
  ----------------------------------- ----------------------------------------
  **Heading**                         gyro quality/bias, `Cmag`, turn
                                      consistency, time since stationary
                                      recalibration

  **Horizontal**                      step confidence, stride stability,
                                      heading confidence, carry-mode
                                      confidence, recalibration history

  **Vertical**                        pressure stability and trend, motion
                                      consistency

  **Overall**                         `C_overall = min(C_H, C_V, C_heading)`
                                      (conservative; validate)
  ----------------------------------------------------------------------------

MVP heading confidence example (starting definition, calibrate):
`C_heading = smooth(Cmag) · exp(−t_since_recal / τ)`. MVP weighted
score: `C_H = w1·C_heading + w2·C_step + w3·C_stride + w4·C_mag`,
`Σ w = 1`, weights are calibration parameters, **not scientific
constants**.

### 15.2 Position uncertainty `σ_pos` (needed for gating and simplification)

Covariance form: `P_{k+1} = F_k P_k F_kᵀ + Q_k`, with Jacobian terms for
the bearing convention:

``` text
∂E/∂ψ =  L cos ψ     ∂N/∂ψ = −L sin ψ
∂E/∂L =  sin ψ       ∂N/∂L =  cos ψ
```

MVP scalar growth per step:

``` text
σ_ψ  = σ_ψ,min + (1 − C_heading)(σ_ψ,max − σ_ψ,min)
σ_pos² ← σ_pos² + (L·σ_ψ)² + (σ_L)²
```

Caveat: heading errors are *correlated* between steps (bias), so adding
them in quadrature underestimates growth; use it as a gate/normaliser,
not as a claim of statistical truth.

------------------------------------------------------------------------

## 16. Ordered trail memory

### 16.1 Node

``` text
TrailNode: id, sequenceIndex, segmentId, E, N, U, stepLength, heading,
           headingConfidence, horizontalConfidence, verticalConfidence,
           magneticReliability, timestamp
```

The trail is an **ordered sequence**, not a point cloud.

### 16.2 Segmentation

Start a new segment on: a major turn, a significant confidence-state
change, a stationary event, or a configurable spatial/temporal
condition.

### 16.3 Simplification

Ramer--Douglas--Peucker with an uncertainty-aware tolerance (a fixed ε
implies false precision):

``` text
ε = max(ε_min, k·σ_position)
```

### 16.4 Persistence

Never write every sensor sample to Room.

``` text
50 Hz sensors → real-time nav → step event → trail node → in-memory batch → periodic Room transaction
```

------------------------------------------------------------------------

## 17. Reverse navigation

### 17.1 Reverse trail

Forward trail `T = [p0 … pN]`; reverse polyline `R = [pN … p0]`. For a
segment traversed in travel order `[a, b]` on the reverse trail, the
tangent is `t = (b − a)/‖b − a‖`, equivalent to `−t_forward`. **Sign
inversion is critical.**

### 17.2 Why nearest-point matching is not enough

At self-crossings or near-parallel switchbacks, the closest segment can
be from a different part of the journey.

### 17.3 Practical approach: progress-pointer window (MVP)

Keep a monotonic **current segment** index. Search only in a window
around it (a few segments backward, a bounded number forward). This
alone resolves most self-crossings. The full cost below then ranks
candidates *inside* the window.

### 17.4 Candidate cost (symbols renamed to avoid clashes)

For candidate segment `i = [a_i, b_i]` and user position `p_u`:

``` text
u_i = clip( (p_u − a_i)·(b_i − a_i) / ‖b_i − a_i‖², 0, 1 )      (projection parameter)
q_i = a_i + u_i (b_i − a_i)                                     (projection point)
d_i = ‖p_u − q_i‖                                               (cross-track distance)
d̂_i = d_i / (σ_pos + d_0)                                       (uncertainty-normalised)

E_seq,i  = |i − j| / max(N−1, 1)           j = current segment
E_head,i = |wrap(ψ_u − ψ_i^r)| / π         ψ_i^r = bearing of reverse tangent
E_conf,i = 1 − C_i

J_i = λ_d·d̂_i + λ_s·E_seq,i + λ_h·E_head,i + λ_c·E_conf,i
```

All λ are calibration parameters.

**MVP progress constraint:** expected reverse progress is enforced by
the monotonic `current segment` pointer and bounded search window.
Therefore the MVP cost does not include a separate `E_prog` term. If a
future implementation replaces the progress window with global candidate
search, add an explicit progress term and re-validate the metric.

-   **Gating:** reject if `d_i > d_gate = max(d_min, k_g·σ_pos)`, unless
    the navigator is in OFF_TRAIL recovery.
-   **Hysteresis:** switch from current cost `J_c` to candidate `i` only
    if `J_i < J_c − ΔJ_switch`.

### 17.5 Reference implementation (C++)

``` cpp
#include <vector>
#include <cmath>
#include <algorithm>

struct P2 { double e, n; };
static inline P2 operator-(P2 a, P2 b){ return {a.e-b.e, a.n-b.n}; }
static inline P2 operator+(P2 a, P2 b){ return {a.e+b.e, a.n+b.n}; }
static inline P2 operator*(P2 a, double s){ return {a.e*s, a.n*s}; }
static inline double dot(P2 a, P2 b){ return a.e*b.e + a.n*b.n; }
static inline double norm(P2 a){ return std::sqrt(dot(a,a)); }
static inline double bearing(P2 v){ return std::atan2(v.e, v.n); }
static inline double wrapPi(double a){
    a = std::fmod(a + M_PI, 2*M_PI); if (a < 0) a += 2*M_PI; return a - M_PI;
}

struct GuidanceOut {
    double xte;            // signed cross-track error (m); + = trail is to the user's LEFT
    double steeringErr;    // rad, wrapped; + = turn RIGHT (clockwise)
    double remaining;      // metres of reverse path left
    int    segIndex;
    bool   offTrail;
};

class ReverseNavigator {
public:
    struct Cfg {
        int    backWin = 2, fwdWin = 12;                 // search window (segments)
        double lamD=1.0, lamS=0.3, lamH=0.4, lamC=0.3;   // calibration parameters
        double d0 = 1.0, dMin = 5.0, kGate = 3.0, dJswitch = 0.15;
        double lookAheadT = 2.0, dLmin = 2.0;            // s, m
        double ke = 0.25, keCap = 2.0;                   // lateral gain (1/m), cap on |ke·e⊥|
        bool   useBlend = false;                         // carrot only by default; A/B test in the field
        double piEps = 0.05;                             // 180° tie-break band (rad)
    };
    Cfg cfg;

    // fwd = recorded forward nodes (E,N) with per-node confidence
    void setTrail(const std::vector<P2>& fwd, const std::vector<float>& conf) {
        r_.assign(fwd.rbegin(), fwd.rend());
        c_.assign(conf.rbegin(), conf.rend());
        cum_.assign(r_.size(), 0.0);
        for (size_t i = 1; i < r_.size(); ++i) cum_[i] = cum_[i-1] + norm(r_[i]-r_[i-1]);
        cur_ = 0; prevTurn_ = +1;
    }

    GuidanceOut update(P2 pu, double psiU, double speed, double sigmaPos) {
        GuidanceOut out{};
        const int M = (int)r_.size();
        if (M < 2) { out.offTrail = true; return out; }

        const double gate = std::max(cfg.dMin, cfg.kGate * sigmaPos);
        int best = cur_; double bestJ = 1e18, curJ = 1e18; bool anyGated = false;

        const int lo = std::max(0, cur_ - cfg.backWin);
        const int hi = std::min(M - 2, cur_ + cfg.fwdWin);
        for (int i = lo; i <= hi; ++i) {
            const P2 a = r_[i], b = r_[i+1];
            const P2 ab = b - a; const double L2 = std::max(dot(ab,ab), 1e-9);
            const double u = std::clamp(dot(pu - a, ab) / L2, 0.0, 1.0);
            const P2 q = a + ab * u;
            const double d = norm(pu - q);
            if (d > gate && i != cur_) { continue; }
            if (d <= gate) anyGated = true;

            const double Eseq = double(std::abs(i - cur_)) / std::max(M - 2, 1);
            const double Ehead = std::abs(wrapPi(psiU - bearing(ab))) / M_PI;
            const double Econf = 1.0 - c_[i];
            const double J = cfg.lamD * (d / (sigmaPos + cfg.d0)) + cfg.lamS * Eseq
                           + cfg.lamH * Ehead + cfg.lamC * Econf;
            if (i == cur_) curJ = J;
            if (J < bestJ) { bestJ = J; best = i; }
        }
        if (best != cur_ && bestJ < curJ - cfg.dJswitch) cur_ = best;   // hysteresis
        out.offTrail = !anyGated;

        // projection onto the selected segment
        const P2 a = r_[cur_], b = r_[cur_+1];
        const P2 ab = b - a; const double segL = std::max(norm(ab), 1e-6);
        const P2 t = ab * (1.0 / segL);                      // reverse tangent
        const double u = std::clamp(dot(pu - a, ab) / (segL*segL), 0.0, 1.0);
        const P2 q = a + ab * u;
        const P2 nLeft{-t.n, t.e};
        const P2 e = q - pu;
        out.xte = dot(e, nLeft);
        out.segIndex = cur_;
        out.remaining = (cum_[M-1] - cum_[cur_]) - u * segL;

        // adaptive look-ahead carrot, carried across following segments
        double rem = std::max(cfg.dLmin, speed * cfg.lookAheadT);
        P2 c = q; int s = cur_;
        while (true) {
            const P2 end = r_[s+1];
            const double left = norm(end - c);
            if (rem <= left || s + 1 >= M - 1) {
                const P2 dir = (left > 1e-9) ? (end - c) * (1.0 / left) : t;
                c = c + dir * std::min(rem, left);
                break;
            }
            rem -= left; c = end; ++s;
        }

        // desired direction
        P2 dvec = c - pu;
        if (norm(dvec) < 1e-6) dvec = t;
        P2 g = dvec * (1.0 / norm(dvec));
        if (cfg.useBlend) {                                 // optional lateral blend (v2 §62)
            const double lat = std::clamp(cfg.ke * out.xte, -cfg.keCap, cfg.keCap);
            P2 gb = t + nLeft * lat;
            g = gb * (1.0 / std::max(norm(gb), 1e-9));
        }

        double err = wrapPi(bearing(g) - psiU);
        // deterministic 180° tie-break: use previous turn side, else RIGHT
        if (std::abs(std::abs(err) - M_PI) < cfg.piEps) err = (prevTurn_ >= 0 ? +M_PI : -M_PI);
        else if (std::abs(err) > 0.2) prevTurn_ = (err >= 0 ? +1 : -1);
        out.steeringErr = err;
        return out;
    }

private:
    std::vector<P2> r_;        // reverse polyline: r_[0] = last recorded node, r_[M-1] = start
    std::vector<float> c_;     // per-segment confidence (use confidence of node a)
    std::vector<double> cum_;  // cumulative arc length along reverse polyline
    int cur_ = 0, prevTurn_ = +1;
};
```

------------------------------------------------------------------------

## 18. Closed-loop guidance

### 18.1 The control loop

``` text
current position → trail projection → XTE → carrot (look-ahead) → desired bearing
→ steering error Δψ = wrap(ψ_d − ψ_u) → haptic/visual command → user motion → new position
```

### 18.2 Carrot point

`d_L = clip(v·T_L, d_min, 0.5·L_remaining)`; `p_c = q + d_L·t_r`,
carried over onto following segments if the look-ahead exceeds the
current segment (implemented above).

### 18.3 Cross-track blend (optional)

``` text
e⊥ = (q − p_u) · n_r,   n_r = [−t_n, t_e]  (left normal)
g  = normalize( t_r + clip(k_e·e⊥) · n_r ),   ψ_d = atan2(g_e, g_n)
```

Carrot pursuit alone is stable. Adding a strong lateral term on top can
double-count the correction and oscillate, so default to carrot-only and
A/B test the blend in the field. Confidence-aware gain:
`k_e = k_min + C_H·(k_max − k_min)`; at low confidence, steer gently and
prompt recalibration.

### 18.4 The 180° case

`atan2(sin, cos)` wraps to a unique value everywhere, **except exactly
±π, where left and right are mathematically equivalent**. Use a
deterministic tie-break: previous turn direction, otherwise RIGHT. Do
not pretend geometry contains information it does not. (Floating-point
may return +π or −π; normalise consistently.)

**Required test:** `ψ_u = 0`, `ψ_d = π` → \|Δψ\| ≈ π → tie-break policy
fires.

### 18.5 Guidance state machine

``` text
        ┌───────────┐  XTE increasing  ┌────────────┐
        │ ON_TRAIL  │ ───────────────► │ DEVIATING  │
        └─────▲─────┘                  └─────┬──────┘
              │ recovered                    │ XTE large
        ┌─────┴──────┐                 ┌─────▼──────┐
        │ RECOVERING │ ◄────────────── │ OFF_TRAIL  │
        └────────────┘                 └────────────┘
```

Rules: transitions need **persistence** (several consecutive updates),
not one noisy sample; `recoveryThreshold < deviationThreshold`
(hysteresis); thresholds scale with `σ_pos` (starting values are
placeholders to calibrate).

### 18.6 Arrival detection

Not just `‖p_u − p_0‖ < r`. Require: remaining reverse distance small,
sequence progress at the final segment, heading consistency, adequate
confidence, hysteresis radius `r_arrive = max(r_min, k·σ_pos)`, and
verification over several updates before "RETURN COMPLETE".

Note: heading for guidance comes from the phone's orientation, so the
guidance arrow also works while the user is **standing still and
turning**, which is handy for demos and debugging.

------------------------------------------------------------------------

## 19. Haptics and visual UI

### 19.1 Single-motor encoding (corrected)

A phone has one vibration motor, so left vs right must be encoded as
**patterns**. Starting proposal (tune with users):

  Meaning                Pattern
  ---------------------- -----------------------------------
  Turn left              1 short pulse
  Turn right             2 short pulses
  Turn around (\~180°)   3 quick pulses
  Off-trail warning      1 long pulse (\~600 ms)
  Low confidence         distinct slow low-frequency pulse
  Arrived                long-short-long

Avoid continuous vibration. Rate-limit and add hysteresis on the
steering-error thresholds (e.g., silent when \|Δψ\| is small,
single-side cue at moderate error, turn-around cue near π). Always pair
haptics with a visual arrow plus text, since vibration patterns are
ambiguous in noisy or gloved conditions.

### 19.2 Visual UI

Heading arrow to carrot, XTE bar, confidence meter (overall +
heading/horizontal/vertical), sensor-health dashboard, trail map
(relative, with uncertainty halo), state badge (ON_TRAIL...), step
count, and a RETURN TO START button.

------------------------------------------------------------------------

## 20. AI layer

### 20.1 What the AI is

A **sensor-intelligence layer**: it interprets motion context and adapts
bounded parameters. It is not a navigator.

### 20.2 Interface

``` cpp
struct AIContext {
    float motionWalkingProbability, motionRunningProbability;
    float carryHandProbability, carryPocketProbability, carryBagProbability;
    float turnLeftProbability, turnRightProbability;
    float accelReliability, gyroReliability, magReliability, baroReliability;
    float strideCorrection;     // δ in [−δmax, +δmax]
};
```

### 20.3 Hard safety constraints

The model must **not**: output coordinates, override sensor physics, set
impossible stride, force magnetic correction during a detected
disturbance, or suppress low-confidence states. Outputs pass through
deterministic clamps. The navigation must degrade gracefully if the
model is unavailable.

### 20.4 Model specification (proposal --- do not claim as implemented until trained and validated)

-   **Input:** sliding window of \~2 s at 50 Hz, 6 channels (accel xyz,
    gyro xyz), optionally gravity-aligned magnitude.
-   **Architecture:** small 1D-CNN or GRU, tens of thousands of
    parameters; multi-head outputs (motion, carry, turn, reliability).
-   **Deployment:** TFLite/LiteRT on CPU (int8 quantised if accuracy
    holds); inference at a low rate (e.g., 2--5 Hz), chosen by
    profiling.
-   **Data:** record labelled walks yourself (each carry mode, speeds,
    turns, several people if possible). Evaluate with
    **leave-one-person-out** splits, not random splits.
-   **Time-boxed fallback:** a rule-based carry classifier
    (gravity-vector orientation and signal variance) labelled honestly
    as a heuristic classifier.
-   **Do not claim** a specific NPU/Hexagon/GPU delegate unless verified
    on the exact device.

------------------------------------------------------------------------

## 21. Android implementation

### 21.1 Responsibility split

``` text
Android (Kotlin)                         Native (C++/NDK)
├── lifecycle, foreground service        ├── quaternion math / AHRS
├── sensor registration                  ├── magnetic reliability
├── permissions, notification            ├── step detector, PDR
├── haptics, UI                          ├── confidence / uncertainty
└── Room persistence                     └── reverse matching + guidance
```

**Recommendation (v2.1):** write the engine behind a `NavigationEngine`
interface and get Tier 0 working end-to-end first (even in Kotlin), then
move AHRS/PDR to C++ if time allows. The native split is for testability
and determinism, and a working demo must exist at every stage.

### 21.2 Manifest (Android 14+, sensor-only)

``` xml
<manifest xmlns:android="http://schemas.android.com/apk/res/android">
    <uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
    <uses-permission android:name="android.permission.FOREGROUND_SERVICE_SPECIAL_USE" />
    <uses-permission android:name="android.permission.POST_NOTIFICATIONS" />
    <uses-permission android:name="android.permission.WAKE_LOCK" />
    <uses-permission android:name="android.permission.VIBRATE" />

    <application android:label="Breadcrumb Trek">
        <service
            android:name=".NavigationForegroundService"
            android:exported="false"
            android:foregroundServiceType="specialUse">
            <property
                android:name="android.app.PROPERTY_SPECIAL_USE_FGS_SUBTYPE"
                android:value="Offline pedestrian navigation using onboard motion sensors, on-device PDR, and local trail backtracking." />
        </service>
    </application>
</manifest>
```

Do **not** declare `location` unless the app actually uses Android
location APIs. `specialUse` may be reviewed at Play submission (not an
issue for a sideloaded hackathon APK). Start order:

``` text
User presses START → visible Activity → permission/state checks → startForegroundService()
→ Service.onStartCommand() → ServiceCompat.startForeground(..., SPECIAL_USE)
→ acquire wake lock → register sensors → start navigation
```

### 21.3 Foreground service (per-sensor push, allocation-free callback)

``` kotlin
class NavigationForegroundService : Service(), SensorEventListener {

    private lateinit var sm: SensorManager
    private var wakeLock: PowerManager.WakeLock? = null
    private val nativeState = FloatArray(12)          // reused, no per-read allocation

    override fun onStartCommand(i: Intent?, flags: Int, id: Int): Int {
        ServiceCompat.startForeground(
            this, NOTIFICATION_ID, buildNotification(),
            ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE
        )
        wakeLock = (getSystemService(POWER_SERVICE) as PowerManager)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "breadcrumb:nav").apply { acquire() }
        sm = getSystemService(SENSOR_SERVICE) as SensorManager
        registerSensors()
        return START_STICKY
    }

    private fun registerSensors() {
        sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let { sm.registerListener(this, it, 20_000) }
        sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let     { sm.registerListener(this, it, 20_000) }
        sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)?.let{ sm.registerListener(this, it, 50_000) }
        sm.getDefaultSensor(Sensor.TYPE_PRESSURE)?.let      { sm.registerListener(this, it, 100_000) } // optional
    }

    override fun onSensorChanged(e: SensorEvent) {
        val v = e.values
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER  -> NavigationNative.pushAccel(v[0], v[1], v[2], e.timestamp)
            Sensor.TYPE_GYROSCOPE      -> NavigationNative.pushGyro(v[0], v[1], v[2], e.timestamp)
            Sensor.TYPE_MAGNETIC_FIELD -> NavigationNative.pushMag(v[0], v[1], v[2], e.timestamp)
            Sensor.TYPE_PRESSURE       -> NavigationNative.pushBaro(v[0], e.timestamp)
        }
    }

    override fun onAccuracyChanged(s: Sensor?, accuracy: Int) { /* update sensor-health state */ }
    override fun onBind(i: Intent?): IBinder? = null

    override fun onDestroy() {
        sm.unregisterListener(this)
        wakeLock?.release()
        super.onDestroy()
    }
}
```

UI/haptics sample state at 10--20 Hz via `nativeGetState(nativeState)`,
not per sensor event.

### 21.4 JNI boundary

``` kotlin
object NavigationNative {
    init { System.loadLibrary("breadcrumb_native") }
    external fun pushAccel(x: Float, y: Float, z: Float, tNs: Long)
    external fun pushGyro (x: Float, y: Float, z: Float, tNs: Long)   // triggers AHRS update
    external fun pushMag  (x: Float, y: Float, z: Float, tNs: Long)
    external fun pushBaro (hPa: Float, tNs: Long)
    external fun startReturn(): Int
    external fun nativeGetState(outBuffer: FloatArray): Int           // returns 12 on success
}
```

State layout (12 floats):
`0 E, 1 N, 2 U, 3 heading, 4 XTE, 5 steeringError, 6 overallConf, 7 headingConf, 8 horizontalConf, 9 verticalConf, 10 magReliability, 11 guidanceStateCode`.

``` cpp
static constexpr int STATE_SIZE = 12;

extern "C" JNIEXPORT jint JNICALL
Java_com_example_breadcrumb_NavigationNative_nativeGetState(
    JNIEnv* env, jobject, jfloatArray outBuffer) {
    if (outBuffer == nullptr || env->GetArrayLength(outBuffer) < STATE_SIZE) return -1;
    env->SetFloatArrayRegion(outBuffer, 0, STATE_SIZE, engine().stateBuffer());
    return STATE_SIZE;
}
```

### 21.5 Native engine skeleton

``` cpp
void NavigationEngine::pushAccel(const Vec3& a, uint64_t t) {
    accel_ = a; lastAccelT_ = t;
    const float aMag = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    const auto step = stepDet_.push(aMag, t);
    if (step.detected) onStep(step);
}

void NavigationEngine::pushGyro(const Vec3& g, uint64_t t) {
    if (!init_) { init_ = true; lastGyroT_ = t; return; }
    const double dt = double(t - lastGyroT_) * 1e-9; lastGyroT_ = t;
    if (dt <= 0.0 || dt > 0.25) return;                       // example guard

    const bool accelFresh = (t - lastAccelT_) < 100'000'000ULL;
    const bool magFresh   = (t - lastMagT_)   < 300'000'000ULL;
    const Vec3 a = accelFresh ? accel_ : Vec3{0,0,0};
    const Vec3 m = magFresh   ? mag_   : Vec3{0,0,0};
    const float Cmag = magFresh ? float(magRel_) : 0.f;

    const Vec3 gc{g.x - bias_.x, g.y - bias_.y, g.z - bias_.z};
    ahrs_.update(gc.x, gc.y, gc.z, a.x, a.y, a.z, m.x, m.y, m.z, Cmag, float(dt));
    updateStationary(g, a, t);                                // bias recalibration when still
    updateConfidence();
}

void NavigationEngine::onStep(const StepDetector::Step& s) {
    const double L   = strideScale_ * std::pow(std::max(0.f, s.aMax - s.aMin), 0.25);
    const double psi = phoneForwardBearing(ahrs_.q) + headingOffset_;   // bearing, rad
    pos_.e += L * std::sin(psi);
    pos_.n += L * std::cos(psi);
    if (baroValid_) pos_.u = relativeHeight_;                 // guarded; never 0.0f placeholder
    trail_.append(pos_, L, psi, confidences(), s.tNs);        // in-memory; batched to Room by Kotlin
    sigmaPos_ = growSigma(sigmaPos_, L, headingConf_);
}
```

### 21.6 Performance rules

No per-sample object creation, boxing, temporary lists, string
formatting, synchronous DB or UI work, or per-event logging in the
high-rate path. Use primitive arrays, reusable structs, fixed-size ring
buffers, preallocated native state (`std::array`, preallocated vectors;
avoid `new`/`push_back`/`std::string` in the update loop unless capacity
is guaranteed). Measure the real per-update time against the 20 ms
period, and do not quote latency you have not profiled.

### 21.7 OEM background behaviour (verify on the demo phone)

Some Android skins are commonly reported to restrict background services
aggressively. On the actual iQOO device: disable battery optimisation
for the app, allow auto-start/background activity in system settings,
keep the wake lock, and **test with the screen off for the full demo
duration**. Treat this as a test item, not an assumption.

------------------------------------------------------------------------

## 22. Validation strategy

### 22.1 Ground truth

Use an independent reference: tape-measured route, surveyed short route,
or high-quality GNSS in an open area (dev-time only). **Never use PDR as
its own ground truth.** GNSS is a validation reference, not a runtime
dependency.

### 22.2 Metrics

  -----------------------------------------------------------------------
  Area                                Metric
  ----------------------------------- -----------------------------------
  Step detection                      Precision `TP/(TP+FP)`, Recall
                                      `TP/(TP+FN)`, F1

  Step length                         `MAE_L = (1/N) Σ |L_i − L̂_i|`

  Heading                             wrapped error
                                      `wrap(ψ_est − ψ_ref)`: mean abs,
                                      median, max, drift per
                                      distance/time

  Position                            `e_p = ‖p_est − p_ref‖`: final,
                                      mean, error per distance travelled,
                                      per carry mode

  Reverse nav                         wrong-segment rate
                                      `N_wrong/N_decisions`,
                                      XTE-detection latency, recovery
                                      latency, **return endpoint error**
  -----------------------------------------------------------------------

### 22.3 Confidence calibration

Bin results into High / Medium / Low confidence and compare actual
errors. A calibrated system shows
`Error_High < Error_Medium < Error_Low` statistically; otherwise the
confidence number is decoration.

### 22.4 Suggested trials

Three routes (straight \~50 m, L-shape, loop with self-crossing), ≥10
repetitions each, for each carry mode you support; report return
endpoint error as median and worst case. Set your accuracy target from
the first measured results, not before.

------------------------------------------------------------------------

## 23. Testing

### 23.1 Record/replay harness (build early)

Log raw sensor data to CSV (`t_ns, sensor, x, y, z`). Compile the native
engine as a host executable (CMake, GoogleTest) and replay recorded
walks through the **same engine code**. Benefits: deterministic
regression tests, parameter tuning without walking, and a **fallback
demo** if live sensors misbehave on stage.

### 23.2 Unit tests

  -----------------------------------------------------------------------
  Module                              Tests
  ----------------------------------- -----------------------------------
  Quaternion                          identity, 90°, 180°, inverse,
                                      normalisation

  AHRS                                static orientation, known rotation,
                                      magnetic disturbance, variable dt,
                                      stale accel/mag

  Step detector                       walking, standing, phone shake,
                                      slow walk, running

  PDR                                 known step lengths, known turns,
                                      repeated straight walks, bearing
                                      convention (walk "north" ⇒ ΔN \> 0)

  Reverse nav                         straight line, U-turn, loop,
                                      self-crossing, switchback,
                                      near-parallel segments

  Guidance                            target ahead, behind, **exactly
                                      180°**, left/right, large XTE, low
                                      confidence
  -----------------------------------------------------------------------

### 23.3 Scenario matrix

  -----------------------------------------------------------------------------------------
  Scenario           Hand     Pocket\*    Bag\*      Turns         Mag       Recalibration
                                                               disturbance  
  --------------- ---------- ---------- ---------- ---------- ------------- ---------------
  Straight walk       ✓          ✓          ✓                                      ✓

  Multiple turns      ✓          ✓          ✓          ✓                           ✓

  Slow / fast         ✓          ✓          ✓                                      ✓
  walk                                                                      

  Magnetic            ✓          ✓          ✓                       ✓       
  disturbance                                                               

  Stop/start          ✓          ✓          ✓                                      ✓

  Self-crossing       ✓          ✓          ✓          ✓                           ✓
  route                                                                     

  Deliberate          ✓          ✓          ✓          ✓                           ✓
  deviation                                                                 
  -----------------------------------------------------------------------------------------

\* Only after the stretch heading-offset method (§13.2) is validated.

------------------------------------------------------------------------

## 24. Demo plan

### 24.1 Setup

Airplane mode **on**, Wi-Fi/Bluetooth off, **Location off** (airplane
mode alone does not guarantee GNSS is off; verify). Open the app, show
sensor health, perform stationary calibration (and the stride
calibration if not done earlier).

### 24.2 Stages

1.  **Start:** sensor health → stationary calibration → start recording.
2.  **Walk:** show step count, trail growth, confidence, heading, sensor
    health.
3.  **Return:** press **RETURN TO START**; show reversed path, carrot,
    XTE, direction arrow, haptic cues.
4.  **Deliberate deviation:**
    `ON_TRAIL → DEVIATING → OFF_TRAIL → RECOVERING → ON_TRAIL`.
5.  **Sensor disturbance:** bring a magnet or metal object near the
    phone (a controlled disturbance); show
    `Cmag ↓ → heading confidence ↓ → magnetic correction ↓`, then
    recovery.
6.  **Result slide:** your measured return-endpoint errors and
    confidence calibration chart.

### 24.3 Fallback

Keep a replay-mode demo ready (recorded walk through the same engine) in
case live conditions fail.

------------------------------------------------------------------------

## 25. iQOO device notes

Use only verified capabilities of the exact target device:

-   Confirm which sensors exist (accelerometer, gyroscope, magnetometer,
    **barometer**) via `SensorManager`, and record their real sampling
    rates/resolution.
-   Haptics: confirm vibration behaviour; patterns must be tuned by feel
    on this phone.
-   Test background/screen-off behaviour (§21.7) and thermal/battery
    impact over a realistic duration.
-   **Do not claim** a specific Snapdragon/Dimensity SoC, NPU, Hexagon,
    GPU, or sensor package unless you have checked it on the exact unit.

------------------------------------------------------------------------

## 26. What this system does not claim

-   exact global position,
-   centimetre-level positioning,
-   drift-free inertial navigation,
-   universal accuracy across users, phones, or carry modes,
-   universal stride constants,
-   perfect yaw without an external reference,
-   guaranteed performance on every Android device,
-   GPS replacement,
-   that AI eliminates sensor drift.

**Strongest defensible claim:** *confidence-aware local backtracking
from a recorded journey using on-device sensor fusion and AI-assisted
adaptation.*

------------------------------------------------------------------------

## 27. Technical differentiation

``` text
PDR alone                →  estimates movement.

Breadcrumb Trek          →  estimates movement
                            + adaptive, reliability-weighted sensor fusion
                            + AI sensor intelligence (bounded)
                            + confidence and uncertainty
                            + ordered journey memory
                            + sequence-aware reverse matching
                            + closed-loop carrot/XTE steering
                            + haptic and visual guidance
```

------------------------------------------------------------------------

## 28. Judge Q&A

**Q1. What exactly is the AI doing?** The navigation equations stay
deterministic. A lightweight on-device model estimates motion state,
carry mode, turn likelihood, and sensor reliability, and those outputs
adapt PDR and fusion parameters within hard bounds. AI is the
sensor-intelligence layer, not a black-box positioner.

**Q2. Why use PDR if it drifts?** We model that uncertainty explicitly:
confidence tracking, stationary recalibration, reduced trust in
disturbed sensors, and sequence-aware reverse matching instead of
treating the trail as ground truth. Constant heading/scale biases can
partially cancel during retracing because the same estimated frame is
used for recording and return, but drift, changing carry mode, and
transient disturbances do not necessarily cancel. Present measured
accuracy only after validation.

**Q3. What if the magnetometer is wrong?** We estimate magnetic
reliability continuously (strength, variance, dip angle). As disturbance
rises, magnetic correction is down-weighted and heading uncertainty
rises. We never blindly accept a bad heading.

**Q4. How do you handle a 180° turnaround?** We compute the bearing to a
look-ahead carrot and wrap the error with `atan2(sin Δψ, cos Δψ)` into
\[−π, π). Exactly 180° is inherently left/right ambiguous, so a
deterministic tie-breaker (previous turn side, else right) is used.

**Q5. Why not store GPS breadcrumbs?** If GNSS is unavailable or
unreliable, GPS breadcrumbs cannot give a dependable trail. Ours comes
from sensors on the phone and needs no live position solution.

**Q6. Is the NDK necessary for 50 Hz?** Not strictly; 50 Hz is not too
fast for Kotlin. We use native code to keep the numerical core
allocation-light, stateful, and independently testable, while Android
handles sensors, lifecycle, UI, and persistence.

**Q7. What happens when the user crosses their own path?** Nearest-point
matching can pick the wrong branch. We search a progress window and
combine distance, sequence continuity, expected progress, heading
compatibility, and confidence, with branch-switch hysteresis.

**Q8. Why not just use Android's rotation-vector sensor?** It is a
useful baseline and fallback for orientation, and we benchmark against
it. Our contribution is the reliability-weighted magnetic correction,
the confidence model, the journey memory, and the closed-loop return
guidance built on top.

**Q9. How accurate is it?** Quote only your measured numbers (median and
worst-case return endpoint error per route and carry mode), and state
the conditions.

------------------------------------------------------------------------

## 29. Hackathon build plan

### 29.1 Priority tiers

**Tier 0: must work end-to-end** 1. Sensor ingestion with proper
timestamps and staleness handling. 2. Madgwick AHRS with `Cmag`
weighting. 3. Step detection + PDR with a calibrated stride (hand-held
carry). 4. Trail recording → reverse trail. 5. Windowed reverse
matching. 6. Carrot guidance + XTE + steering error. 7. Haptic
patterns + visual arrow. 8. Confidence display. 9. Fully offline
operation.

**Tier 1: high value** 10. Magnetic reliability with dip-angle term. 11.
Stationary bias recalibration. 12. Barometer (if present). 13.
Sensor-health dashboard. 14. Rule-based carry-mode classifier. 15.
Record/replay harness and measured validation results.

**Tier 2: stretch** 16. TFLite carry/stride model. 17. Covariance-based
uncertainty. 18. Full EKF with true ZUPT. 19. Pocket/bag heading offset.
20. Automated confidence calibration.

### 29.2 Implementation order

1.  Sensor ingestion + CSV logging → 2. AHRS → 3. Step detector + stride
    calibration → 4. PDR (verify bearing convention: walk north ⇒ ΔN
    \> 0) → 5. Trail storage → 6. Reverse matching (windowed) → 7.
    Carrot guidance + XTE → 8. Haptics/UI → 9. Confidence → 10.
    Foreground service hardening + screen-off test → 11. AI adaptation
    → 12. Native/NDK optimisation and profiling → 13. Controlled
    validation and demo rehearsal.

### 29.3 Suggested repository layout

``` text
breadcrumb-trek/
├── app/                    (Kotlin: ui/, service/, sensors/, data/room/, haptics/, ml/)
├── native/                 (C++: ahrs/, pdr/, trail/, guidance/, jni/)
├── native-tests/           (GoogleTest + CMake host build, replay runner)
├── tools/                  (Python: log analysis, plots, calibration scripts)
├── data/                   (recorded walks: CSV + labels)
└── docs/                   (this spec, validation results, demo script)
```

### 29.4 Risk register

  -----------------------------------------------------------------------
  Risk                                Mitigation
  ----------------------------------- -----------------------------------
  Heading drift / magnetic            `Cmag` weighting, stationary
  disturbance during demo             recalibration, pre-demo mag
                                      calibration, disturbance shown as a
                                      *feature*

  Wrong stride on stage               per-user stride calibration walk
                                      right before demo

  OEM kills service / screen-off      wake lock, battery-optimisation
  stops sensors                       exemption, full-duration screen-off
                                      test

  Too many features, nothing finished Tier 0 first; one working loop
                                      beats a half-built stack

  Live demo failure                   replay-mode fallback demo

  Unverifiable claims                 use only measured numbers; follow
                                      §26
  -----------------------------------------------------------------------

------------------------------------------------------------------------

## 30. Agent Execution Contract

### 30.1 Start-from-scratch rule

Assume the repository is empty except for this specification. Do not
assume that any previous implementation, API, package name, native
library, ML model, dataset, or UI exists.

Before writing the full application:

1.  Create the Android project and verify it builds and installs on the
    target iQOO device.
2.  Create the native C++ library and verify Kotlin ↔ JNI loading works.
3.  Create the host-side C++ test target and verify it runs
    independently of Android.
4.  Create a minimal sensor-recording path and CSV export.
5.  Establish exact package/class/module names in code and keep them
    stable.
6.  Implement the Tier 0 loop before adding Tier 1 or Tier 2 features.

### 30.2 Required repository contract

Use this structure unless a build-system constraint requires a
documented change:

``` text
breadcrumb-trek/
├── app/
│   └── src/main/java/.../
│       ├── ui/
│       ├── service/
│       ├── sensors/
│       ├── haptics/
│       └── data/
├── native/
│   ├── ahrs/
│   ├── pdr/
│   ├── trail/
│   ├── guidance/
│   ├── confidence/
│   └── jni/
├── native-tests/
│   ├── tests/
│   └── replay/
├── tools/
│   ├── calibration/
│   └── analysis/
├── data/
│   └── README.md
└── docs/
    └── Breadcrumb_Trek_2.2.md
```

Keep Android I/O, lifecycle, permissions, notification, haptics, and
persistence outside the deterministic navigation core.

### 30.3 Tier 0 definition of done

Tier 0 is complete only when all of the following work on the target
device:

``` text
START
  ↓
stationary calibration
  ↓
record a handheld walk
  ↓
steps detected
  ↓
relative trail grows
  ↓
RETURN TO START
  ↓
reverse trail selected
  ↓
carrot / steering direction updates
  ↓
XTE updates
  ↓
visual guidance works
  ↓
single-motor haptic patterns work
  ↓
user deliberately leaves the trail
  ↓
OFF_TRAIL / RECOVERING state is reached
  ↓
user returns to trail
  ↓
RETURN COMPLETE is detected
```

A feature that exists only as a declaration, placeholder, mock, or
hard-coded demo value does not count as implemented.

### 30.4 Implementation priorities

Implement in this exact order unless a blocking technical issue is
documented:

1.  Android project + build/install.
2.  Sensor discovery and availability reporting.
3.  Timestamped CSV recorder.
4.  Native library + JNI loading.
5.  Sensor push APIs and timestamp/staleness handling.
6.  Quaternion/AHRS implementation.
7.  Static and known-rotation AHRS tests.
8.  Step detector.
9.  Per-user stride calibration.
10. PDR using the documented ENU/bearing convention.
11. Ordered trail storage.
12. Reverse trail generation.
13. Progress-window reverse matching.
14. Carrot guidance.
15. XTE and steering error.
16. Guidance state machine.
17. Visual UI.
18. Single-motor haptic patterns.
19. Confidence and uncertainty.
20. Foreground-service hardening and screen-off test.
21. Magnetic reliability and stationary bias recalibration.
22. Record/replay regression harness.
23. Controlled validation.
24. Only then add AI adaptation.
25. Only after profiling, optimise native/runtime performance.

### 30.5 Do not implement yet

Unless Tier 0 is already passing, do not spend implementation time on:

-   a trained deep-learning model,
-   arbitrary pocket/bag carry support,
-   full EKF/ZUPT,
-   cloud synchronization,
-   live GNSS integration,
-   chipset-specific acceleration,
-   NPU/GPU delegates,
-   centimetre-level localization,
-   or features not required by the return-to-start loop.

These are not part of the minimum working system.

### 30.6 Agent behavior rules

The implementation agent must:

-   prefer a working deterministic baseline over speculative complexity;
-   preserve all coordinate, sign, unit, and bearing conventions in this
    document;
-   write unit tests whenever a mathematical convention is introduced;
-   never silently turn a calibration parameter into a claimed physical
    constant;
-   never invent sensor availability on the target phone;
-   never claim measured accuracy before running the validation
    protocol;
-   never claim AI functionality before a model is actually trained,
    evaluated, and integrated;
-   keep replayable raw sensor logs so failures can be reproduced;
-   document deviations in `docs/implementation-deviations.md`;
-   keep navigation usable when the AI layer is disabled.

### 30.7 Required acceptance artifacts

Before calling the implementation complete, produce:

``` text
docs/
├── implementation-deviations.md
├── calibration.md
├── validation-results.md
└── demo-runbook.md

data/
└── README.md
```

`validation-results.md` must contain measured results, route conditions,
phone, carry mode, calibration procedure, number of trials, median
error, worst-case error, and known failure cases.

Do not fill result fields with expected, estimated, simulated, or
invented values.

### 30.8 Definition of "ready for hackathon demo"

The project is demo-ready only when:

-   the app builds from a clean checkout;
-   the app installs on the actual target iQOO phone;
-   sensors remain active for the full planned demo;
-   screen-off behavior has been tested if screen-off operation is
    claimed;
-   a complete return journey works without network connectivity;
-   deliberate deviation triggers the documented state transitions;
-   the self-crossing test does not immediately jump to the wrong
    branch;
-   the 180° guidance test passes;
-   UI confidence changes when sensor quality changes;
-   haptic patterns are understandable on the actual phone;
-   replay mode reproduces at least one recorded run;
-   measured validation results exist;
-   every presentation claim can be traced to this specification, a
    cited source, or a measured result.

## 31. Production readiness gates

**Sensor layer:** \[ \] timestamp monotonicity \[ \] stale-sample
handling \[ \] sensor availability \[ \] missing-sample handling
**AHRS:** \[ \] quaternion normalisation \[ \] dt validation \[ \]
magnetic reliability \[ \] static tests \[ \] rotation tests **PDR:** \[
\] step validation \[ \] stride calibration \[ \] carry-mode evaluation
\[ \] heading validation \[ \] bearing-convention test **Reverse
navigation:** \[ \] self-crossing tests \[ \] 180° test \[ \] XTE tests
\[ \] adaptive look-ahead tests **Runtime:** \[ \] no high-rate
allocations \[ \] database not in sensor callback \[ \] measured CPU
load \[ \] measured update latency \[ \] thermal behaviour \[ \] battery
impact \[ \] screen-off operation

------------------------------------------------------------------------

## 32. Final positioning

> **Breadcrumb Trek does not try to reconstruct the world from imperfect
> sensors. It remembers the user's own journey, continuously estimates
> how trustworthy that memory is, and closes the loop between the user's
> current motion and the path they need to retrace.**

The engineering contribution is the complete closed loop:

``` text
SENSE → INTERPRET → ESTIMATE → MEASURE CONFIDENCE → REMEMBER JOURNEY
→ MATCH CORRECT REVERSE SEGMENT → COMPUTE CARROT → STEER → MEASURE DEVIATION → CORRECT
```

Present it as **a confidence-aware local journey-memory system with
AI-assisted sensor interpretation and closed-loop reverse navigation**,
and **not** as a GPS replacement, a perfect inertial navigator, a
black-box AI positioner, or a claim of drift-free smartphone
localisation.

## 33. Final Agent Handoff

### Objective

Build Breadcrumb Trek from scratch as a working Android application for
the documented handheld-carry MVP.

### First milestone

Do not attempt to finish the entire specification in one pass. The first
milestone is:

``` text
Android app launches
→ sensors are discovered
→ raw samples are timestamped
→ samples can be recorded/replayed
→ native engine loads
→ AHRS unit tests pass
```

Then continue through the implementation order in Section 30.

### Non-negotiable technical invariants

``` text
Acceleration       m/s²
Gyroscope          rad/s
Magnetic field     µT
Pressure           hPa
Sensor timestamp   Android SensorEvent.timestamp / ns
Distance           m
Angles             rad
Trail frame        ENU
Heading            clockwise from North
PDR                ΔE=L·sinψ, ΔN=L·cosψ
```

Never mix the NWU AHRS frame directly with the ENU trail frame without
the explicit conversion:

``` text
E = −W
N =  N
U =  U
```

### Final product boundary

The finished MVP is an **offline, confidence-aware local journey-memory
and reverse-navigation system**. It is not a global localization system
and is not a GPS replacement.

The implementation should make that distinction visible in the product,
code comments, validation report, and hackathon presentation.
