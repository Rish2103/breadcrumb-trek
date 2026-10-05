# Implementation Deviations & Engineering Decisions

## Milestone 0 Decisions

1. **Toolchain Pinning**:
   - Gradle: `8.9`
   - Android Gradle Plugin (AGP): `8.7.2`
   - Kotlin: `2.1.21`
   - JDK Major Version: `17`
   - `compileSdk`: `35`
   - `targetSdk`: `35`
   - `minSdk`: `26`
   - NDK: `26.3.11579264`
   - CMake: `3.22.1`
   - Build-tools: Managed automatically by AGP 8.7.2 default compatibility rather than pinned.

2. **16 KB Native Page Size**:
   - NDK r26d (`26.3.11579264`) defaults to 4 KB page alignment. No automatic 16 KB alignment is claimed without experimental validation. If 16 KB alignment becomes required, explicit linker flags will be configured and verified using ELF inspection tools.

3. **Application Identity**:
   - Authoritative identity is declared via `namespace = "com.iqoo.breadcrumb"` and `applicationId = "com.iqoo.breadcrumb"` in `app/build.gradle.kts`. The deprecated manifest `package` attribute is omitted.

4. **Scope Control**:
   - No sensor, AHRS, PDR, confidence, trail memory, Room, reverse navigation, guidance, or ML implementations are present in Milestone 0.
