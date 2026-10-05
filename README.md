# Breadcrumb Trek 2.x

> **Offline, confidence-aware pedestrian journey memory and reverse navigation system for Android.**

Breadcrumb Trek enables pedestrians to explore unfamiliar environments—indoors, underground, in dense urban canyons, or along unmarked trails—and reliably backtrack to their exact starting point without relying on GNSS, cellular network connectivity, or external map services.

---

## 1. Problem Statement

Modern mobile navigation fundamentally assumes continuous access to global satellite positioning (GPS/GNSS) and cloud routing services. In real-world pedestrian scenarios, these assumptions fail:

- **GNSS Denied / Degraded Environments**: Deep indoor facilities, basements, multi-level parking garages, underground transit tunnels, and dense urban canyons suffer severe multipath and signal blockage.
- **Privacy & Connectivity Constraints**: Emergency wilderness exploration, offline travel, or disaster zones require self-contained tracking without cellular data or map downloads.
- **Pedestrian Dead Reckoning (PDR) Drift**: Naive smartphone PDR systems conflate **phone forward orientation** with the **pedestrian's actual travel direction**. Minor wrist yaw, texting, glances at the screen, or arm swings introduce catastrophic cumulative orientation errors, pulling dead-reckoning trajectories tens of meters off course within seconds.

---

## 2. Solution Overview

Breadcrumb Trek 2.x introduces a deterministic, native C++ navigation engine that decouples chassis orientation from pedestrian motion:

1. **Origin Anchor**: Begins with a stationary calibration phase establishing local magnetic baseline and gyro bias.
2. **Constrained Travel-Course Observer (M3.8)**: Continuously observes pedestrian walking motion via Principal Component Analysis (PCA) of horizontal Earth-frame acceleration, determining the true motion axis independently of physical phone yaw.
3. **Dynamic Reversal Detection**: Distinguishes continuous 180° pedestrian walking U-turns from phone sign ambiguities using sustained step-averaged angular divergence.
4. **Monotonic Trail Memory**: Captures ordered breadcrumbs with non-decreasing distance guarantees, storing $(E, N)$ coordinates, travel heading, phone heading, and confidence metrics.
5. **Closed-Loop Reverse Guidance (M5 / M6)**: Projects the user's position onto the remembered path, generating adaptive carrot targets, steering instructions, cross-track error (XTE) deviation alerts, and automatic recovery vectors returning to origin.

---

## 3. Key Features

- **100% Offline & Deterministic**: Zero network requests, zero GNSS runtime dependencies, zero AI/black-box inference required for core navigation.
- **Chassis-Decoupled Travel Direction**: Walking heading is derived from gait dynamics rather than phone bearing, eliminating trajectory corruption from natural handheld yaw variations.
- **Dynamic 180° U-Turn Tracking**: Resolves PCA motion axis sign ambiguity and smoothly reverses course within 4 walking steps during intentional pedestrian turnaround.
- **Stationary Course Freezing**: Travel course is locked during pauses and stationary chassis rotation, preventing spurious dead-reckoning drift while stopped.
- **Confidence-Aware Sensor Fusion**: 9-DOF Madgwick AHRS with magnetic field disturbance rejection ($C_{\text{mag}}$) and online gyroscope bias drift compensation.
- **Three-Tier Reverse Guidance & Recovery**:
  - `ON_TRAIL` ($|XTE| \le 2.0\,\text{m}$): Tangential carrot lookahead guidance.
  - `DEVIATING` ($2.0\,\text{m} < |XTE| \le 5.0\,\text{m}$): Lateral blend steering steering user back to centerline.
  - `OFF_TRAIL` ($|XTE| > 5.0\,\text{m}$ for 3 updates): Direct orthogonal recovery vector to nearest trail segment, guarded by hysteresis.
  - `RETURN_COMPLETE`: Automatic arrival detection within $1.5\,\text{m}$ of starting anchor $(0,0)$.

---

## 4. Technical Architecture

```
                       SMARTPHONE SENSORS
            [Accelerometer]  [Gyroscope]  [Magnetometer]
                     │            │             │
                     └────────────┼─────────────┘
                                  ▼
                 ┌──────────────────────────────────┐
                 │       Native 9-DOF AHRS          │
                 │   (Madgwick Filter + Cmag Gate)  │
                 └────────────────┬─────────────────┘
                                  │
         ┌────────────────────────┴────────────────────────┐
         │ Attitude Quaternion (q)                         │
         │ Phone Heading (ψ_phone)                         │
         ▼                                                 ▼
┌──────────────────────────────┐              ┌─────────────────────────┐
│ Body-to-Earth Accel Rotation │              │ Dynamic Stride Estimator│
│  a_East, a_North (NWU Frame) │              │    (Weinberg Model)     │
└──────────────┬───────────────┘              └────────────┬────────────┘
               │                                           │
               ▼                                           │
┌─────────────────────────────────────────┐                │
│    ConstrainedCourseObserver (M3.8)     │                │
│  • Windowed Horizontal Accel PCA        │                │
│  • Confidence Gating (λ1, C_pca)        │                │
│  • Ambiguity Resolution                 │                │
│  • Dynamic 180° Reversal Detector       │                │
│  • Isolated Angular Transition Limiter  │                │
└──────────────────┬──────────────────────┘                │
                   │                                       │
                   ▼ Travel Heading (ψ_travel)             │
┌──────────────────────────────────────────────────────────┴────────┐
│                        PdrEngine Core                             │
│             ΔEast  = Stride * sin(ψ_travel)                       │
│             ΔNorth = Stride * cos(ψ_travel)                       │
└──────────────────────────────────┬────────────────────────────────┘
                                   │
                                   ▼ Relative Position (East, North)
┌───────────────────────────────────────────────────────────────────┐
│                     TrailMemory (M4 Engine)                       │
│   • Monotonic Ordered Breadcrumb Buffer                           │
│   • Bounding Box & Journey Telemetry                              │
│   • Frozen State Lifecycle (IDLE -> RECORDING -> RETURN_READY)    │
└──────────────────────────────────┬────────────────────────────────┘
                                   │
                                   ▼
┌───────────────────────────────────────────────────────────────────┐
│                  ReverseNavigator (M5 / M6)                       │
│   • Backward Trail Projection & Cross-Track Error (XTE)           │
│   • Lookahead Carrot Generator & Desired Heading                  │
│   • Multi-State Recovery Engine with Hysteresis                   │
└──────────────────────────────────┬────────────────────────────────┘
                                   │
                                   ▼ JNI Bridge (breadcrumb_native)
┌───────────────────────────────────────────────────────────────────┐
│                     Android Application Layer                     │
│   • SensorIngestionManager (Direct Hardware Listener)             │
│   • NativeNavState 40-float Zero-Allocation Direct Buffer         │
│   • TrailCanvasView (Relative 2D Journey & Guidance Display)     │
└───────────────────────────────────────────────────────────────────┘
```

---

## 5. M3.8 TravelHeading Innovation

In traditional smartphone PDR, dead-reckoning equations use the phone chassis bearing:

$$\Delta E = L \cdot \sin(\psi_{\text{phone}}), \quad \Delta N = L \cdot \cos(\psi_{\text{phone}})$$

If the user tilts the phone by 45° to text or view notifications, the computed trajectory veers 45° away from reality.

**Breadcrumb Trek M3.8 separates chassis bearing from pedestrian motion direction:**

$$\Delta E = L \cdot \sin(\psi_{\text{travel}}), \quad \Delta N = L \cdot \cos(\psi_{\text{travel}})$$

### Core Algorithmic Mechanics:
1. **Horizontal Acceleration Covariance**: Evaluates a 2.5-second sliding window of Earth-frame horizontal accelerations $(a_{\text{East}}, a_{\text{North}})$ using Principal Component Analysis (PCA).
2. **Confidence Gating**: The primary eigenvector indicates the motion axis. Updates are accepted only when:
   - Primary eigenvalue $\lambda_1 \ge 0.08\,\text{m}^2/\text{s}^4$
   - Anisotropy confidence $C_{\text{PCA}} = 1 - \frac{\lambda_2}{\lambda_1} \ge 0.20$
3. **Ambiguity Resolution**: PCA yields an undirected axis ($\pm \vec{v}_1$). The observer resolves sign relative to the established travel course.
4. **Dynamic Walking-Reversal Detector**: When a genuine 180° pedestrian U-turn occurs, sustained angular divergence between step phone heading and travel course ($|\text{wrapPi}(\psi_{\text{phone}} - \psi_{\text{travel}})| \ge 140^\circ$) across $\ge 3$ consecutive steps triggers controlled reversal mode, converging within 4 steps at $45^\circ/\text{step}$.
5. **Chassis Yaw Rejection**: Continuous handheld yaw oscillations up to $\pm 55^\circ$ while walking straight are suppressed; travel course remains within $< 3.8^\circ$ of truth.
6. **Stationary Rotation Protection**: When pedestrian motion ceases, the travel course is held locked, preventing chassis rotation from corrupting navigation state.

---

## 6. Project Structure

```
D:/IQOO Code/
├── app/                                # Android Application Module
│   ├── src/main/
│   │   ├── AndroidManifest.xml
│   │   ├── java/com/iqoo/breadcrumb/
│   │   │   ├── NavigationNative.kt     # JNI bindings and native state parsing
│   │   │   ├── sensors/                # Hardware sensor ingestion & CSV logging
│   │   │   └── ui/                     # MainActivity & TrailCanvasView
│   │   └── res/                        # UI layouts and vector drawables
│   └── build.gradle.kts
├── native/                             # Production C++17 Core Engine
│   ├── CMakeLists.txt                  # Android NDK CMake build script
│   ├── ahrs/                           # 9-DOF Madgwick AHRS & NavigationEngine
│   │   ├── MadgwickAHRS.hpp
│   │   ├── NavigationEngine.hpp
│   │   └── NavigationEngine.cpp
│   ├── pdr/                            # Pedestrian Dead Reckoning & Travel Course
│   │   ├── ConstrainedCourseObserver.hpp # M3.8 PCA motion-axis observer
│   │   ├── PdrEngine.hpp               # Step detector, Weinberg stride & PDR
│   │   └── StepDetector.hpp
│   ├── trail/                          # Monotonic Trail Memory
│   │   ├── TrailMemory.hpp
│   │   └── TrailPoint.hpp
│   ├── guidance/                       # Reverse Guidance & State Machine
│   │   ├── ReverseNavigator.hpp
│   │   └── ReverseNavigator.cpp
│   └── jni/                            # JNI boundary
│       └── breadcrumb_native.cpp
├── native-tests/                       # C++ Regression & Validation Test Suite
│   └── tests/                          # 8 Unit & Physical Replay Test Suites
├── data/                               # Controlled Physical Validation Datasets (CSV)
│   ├── m37_A_straight.csv              # Scenario A: Handheld straight walk
│   ├── m37_B_phone_yaw.csv             # Scenario B: Walking + ±55° chassis yaw
│   ├── m37_C_turn90.csv                # Scenario C: 90° walking turn
│   ├── m37_D_reverse180.csv            # Scenario D: Continuous 180° walking reversal
│   ├── m37_E_stop_rotate_resume.csv    # Scenario E: Stop -> rotate -> resume
│   ├── m37_F_phone_offset45.csv        # Scenario F1: Walking with fixed +45° yaw
│   ├── m37_F_phone_offset90.csv        # Scenario F2: Walking with fixed +90° yaw
│   ├── m37_G_varied_posture.csv        # Scenario G: Dangling arm carry
│   └── poco_walk.csv                   # Reference 87-step physical benchmark
├── docs/                               # Architectural documentation & screenshots
│   └── screenshots/                    # On-device capture records
├── tools/analysis/                     # Python diagnostic analysis scripts
├── build.gradle.kts                    # Root Gradle build script
└── settings.gradle.kts
```

---

## 7. Tech Stack

- **Native Core**: C++17 (Android NDK r26c / Clang 17)
- **Android Platform**: Kotlin 2.1.21, Android SDK (compileSdk 35, minSdk 26, targetSdk 35)
- **Build System**: Android Gradle Plugin 8.7.2, Gradle 8.9, CMake 3.22.1
- **Target Architectures**: `arm64-v8a`, `armeabi-v7a`, `x86`, `x86_64`
- **Validation Device**: POCO X6 Pro 5G (Android 14 / HyperOS, MediaTek Dimensity 8300-Ultra)

---

## 8. Verification & Test Results

The native C++ core is rigorously verified by a multi-milestone regression suite compiled for ARM64 and executed directly on the physical target device.

### Milestone Regression Summary (POCO X6 Pro 5G Target)

| Milestone Suite | Test Focus | Tests | Status | Verification Summary |
| :--- | :--- | :---: | :---: | :--- |
| **M2 AHRS** | Madgwick 9-DOF, Cmag gate, gyro calibration | 32 / 32 | **100% PASS** | Zero regressions across orientation & magnetic rejection |
| **M3 / M3.1 PDR** | Step detection, Weinberg stride, circular mean | 36 / 36 | **100% PASS** | Cadence gating, dynamic thresholds, displacement math |
| **M4 Trail Memory** | Breadcrumb buffer, monotonic distance | 46 / 46 | **100% PASS** | Ring buffer capacity, frozen lifecycle, bounding box |
| **M5 Reverse Matcher** | Carrot projection, backtrack tracking | 18 / 18 | **100% PASS** | Forward progress monotonicity, segment transitions |
| **M6 Guidance & Recovery** | Deviation detection, hysteresis recovery | 19 / 19 | **100% PASS** | ON_TRAIL, DEVIATING, OFF_TRAIL, RECOVERING, ARRIVED |
| **M3.6 Observer Synthetic** | PCA axis estimation, yaw suppression | 9 / 9 | **100% PASS** | ±60° yaw suppressed (<3.1° error), sign resolution |
| **M3.7 Controlled Validation** | 8 real-world physical walking scenarios | 8 / 8 | **100% PASS** | Validated across straight, yaw, turn, reverse, pause |
| **M3.8A Reversal Tests** | Dynamic U-turn convergence | 1 / 1 | **100% PASS** | Reversal at step 24; convergence in 4 steps; disp 8.38m |
| **End-to-End L-Route Run** | Integrated record and reverse backtrack | 6 / 6 | **100% PASS** | Complete journey from (0,0) to (10,15) and back |

### Mode A (Baseline) vs Mode B (Constrained Travel Course) Replay

Comparing identical physical sensor recordings on the production engine:

- **Straight Walk (Scenario A)**: Mode A net displacement 18.67m vs Mode B 18.68m (within 0.05m).
- **Walking with ±55° Phone Yaw (Scenario B)**: Mode A degraded to 15.35m with 17 course jumps $>30^\circ$. Mode B completely suppressed yaw swing (net displacement **18.67m**, **0 jumps $>30^\circ$**).
- **180° Walking U-Turn (Scenario D)**: Mode B detected reversal at step 24, converged within 4 steps, and tracked the return path to origin (**8.38m** net displacement).
- **Stop-Rotate-Resume (Scenario E)**: Mode A followed phone rotation while stationary, corrupting course (5.58m net disp). Mode B froze course during pause, resuming smoothly (**12.43m** net disp).
- **POCO 87-Step Walk Benchmark (`poco_walk.csv`)**: Mode A suffered 47 jumps $>30^\circ$ and 12 jumps $>60^\circ$ due to arm sway. Mode B achieved **0 jumps**, reduced course variation by 81.6%, and preserved path linearity.

---

## 9. Build & Setup Instructions

### Prerequisites
- Android Studio Ladybug or newer
- Android SDK Platform 35 & NDK 26.3.11579264
- CMake 3.22.1
- JDK 17 (e.g. Eclipse Adoptium OpenJDK 17)
- ADB enabled on an Android device (minSdk 26)

### Building the Android APK
```powershell
# From project root
./gradlew assembleDebug

# Install on connected device
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### Building & Running Native Tests on Device
```powershell
# Compile ARM64 test executable using Android NDK clang++
$NDK_BIN = "$env:LOCALAPPDATA/Android/Sdk/ndk/26.3.11579264/toolchains/llvm/prebuilt/windows-x86_64/bin"
& "$NDK_BIN/aarch64-linux-android26-clang++.cmd" -std=c++17 -O2 `
    -I"native" -I"native/ahrs" -I"native/pdr" -I"native/trail" -I"native/guidance" -I"native-tests/tests" `
    "native-tests/tests/M37ControlledValidation.cpp" "native/ahrs/NavigationEngine.cpp" "native/guidance/ReverseNavigator.cpp" `
    -llog -o "native-tests/m37_validation_arm64"

# Push and execute on device
adb push native-tests/m37_validation_arm64 /data/local/tmp/
adb shell "chmod 755 /data/local/tmp/m37_validation_arm64 && /data/local/tmp/m37_validation_arm64"
```

---

## 10. Development Device Notice & Limitations

### Development & Validation Device Notice
Physical validation, sensor characterization, and replay benchmarks for Milestones M1 through M3.8 were performed on a **POCO X6 Pro 5G** development device.
> **Note**: POCO hardware characteristics are specific to this test bench and may not represent the entire iQOO hardware lineup. Validation across diverse iQOO device sensor stacks (e.g., Bosch, InvenSense/TDK, STMicroelectronics IMUs) remains ongoing and planned for future milestones.

### Honest Limitations
1. **Not a GPS Replacement**: Breadcrumb Trek is an offline journey-memory and backtrack system. It does not provide global latitude/longitude or absolute geo-referencing.
2. **Carry Posture Envelope**: Validated for handheld texting, reading, waist-level holding, and controlled dangling arm carries. Arbitrary unconstrained pocket tumbling without orientation calibration is not yet supported.
3. **Severe Magnetic Anomalies**: Sustained exposure to massive ferromagnetic structures will trigger the $C_{\text{mag}}$ magnetic anomaly gate and temporarily rely on gyro integration, subject to uncorrected yaw drift.
4. **Zero AI Invariance**: The system operates deterministically on physical laws and classical signal processing. It does not perform deep-learning trajectory synthesis.

---

## 11. Future Scope

- **Milestone M3.9+**: Visualizing real-time `travelHeading` vs `phoneHeading` vectors and PCA confidence directly on the live UI canvas.
- **Barometric Multi-Floor Tracking**: Integrating barometric altimetry to track floor transitions in multi-story stairwells.
- **Broader Device Matrix**: Validating across the full range of iQOO smartphones to establish model-specific sensor noise baselines.
- **Map-Free Waypoint Tagging**: Allowing users to pin custom landmarks (e.g. trail junctions, parked car, tent) along the recorded breadcrumb memory.

---

## 12. License

This project is licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.
