#include <jni.h>
#include "../ahrs/NavigationEngine.hpp"

extern "C" {

JNIEXPORT jint JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_nativeGetVersion(JNIEnv* /* env */, jobject /* thisObj */) {
    return 1;
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_pushAccel(
    JNIEnv* /* env */, jobject /* thisObj */, jfloat x, jfloat y, jfloat z, jlong tNs) {
    NavigationEngine::instance().pushAccel(x, y, z, static_cast<int64_t>(tNs));
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_pushGyro(
    JNIEnv* /* env */, jobject /* thisObj */, jfloat x, jfloat y, jfloat z, jlong tNs) {
    NavigationEngine::instance().pushGyro(x, y, z, static_cast<int64_t>(tNs));
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_pushMag(
    JNIEnv* /* env */, jobject /* thisObj */, jfloat x, jfloat y, jfloat z, jlong tNs) {
    NavigationEngine::instance().pushMag(x, y, z, static_cast<int64_t>(tNs));
}

JNIEXPORT jint JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_nativeGetState(
    JNIEnv* env, jobject /* thisObj */, jfloatArray outBuffer) {
    if (outBuffer == nullptr) return -1;
    const jsize len = env->GetArrayLength(outBuffer);
    if (len < 12) return -1;

    float temp[40];
    const int reqSize = (len >= 40) ? 40 : ((len >= 32) ? 32 : ((len >= 20) ? 20 : ((len >= 16) ? 16 : 12)));
    const int count = NavigationEngine::instance().getState(temp, reqSize);
    if (count > 0) {
        env->SetFloatArrayRegion(outBuffer, 0, count, temp);
    }
    return count;
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_nativeReset(
    JNIEnv* /* env */, jobject /* thisObj */) {
    NavigationEngine::instance().reset();
}

// Milestone 4: Trail Memory Controls
JNIEXPORT jboolean JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_startTrailRecording(
    JNIEnv* /* env */, jobject /* thisObj */) {
    return NavigationEngine::instance().startTrailRecording() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_stopTrailRecording(
    JNIEnv* /* env */, jobject /* thisObj */) {
    return NavigationEngine::instance().stopTrailRecording() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_resetTrail(
    JNIEnv* /* env */, jobject /* thisObj */) {
    NavigationEngine::instance().resetTrail();
}

JNIEXPORT jint JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_getTrailPoints(
    JNIEnv* env, jobject /* thisObj */, jfloatArray outFloats, jint maxPoints) {
    if (outFloats == nullptr || maxPoints <= 0) return 0;
    const jsize len = env->GetArrayLength(outFloats);
    if (len < maxPoints * 4) return 0;

    std::vector<float> temp(maxPoints * 4);
    const int pointsCopied = NavigationEngine::instance().getTrailPoints(temp.data(), maxPoints);
    if (pointsCopied > 0) {
        env->SetFloatArrayRegion(outFloats, 0, pointsCopied * 4, temp.data());
    }
    return pointsCopied;
}

// Milestone 5: Reverse Navigation Controls
JNIEXPORT jboolean JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_startReverseNavigation(
    JNIEnv* /* env */, jobject /* thisObj */) {
    return NavigationEngine::instance().startReverseNavigation() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_stopReverseNavigation(
    JNIEnv* /* env */, jobject /* thisObj */) {
    NavigationEngine::instance().stopReverseNavigation();
}

JNIEXPORT jboolean JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_isReverseNavigationActive(
    JNIEnv* /* env */, jobject /* thisObj */) {
    return NavigationEngine::instance().isReverseNavigationActive() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_setMadgwickVariant(
    JNIEnv* /* env */, jobject /* thisObj */, jint variant) {
    NavigationEngine::instance().setMadgwickVariant(variant);
}

JNIEXPORT jint JNICALL
Java_com_iqoo_breadcrumb_NavigationNative_getMadgwickVariant(
    JNIEnv* /* env */, jobject /* thisObj */) {
    return NavigationEngine::instance().getMadgwickVariant();
}

}

