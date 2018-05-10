#define LOG_TAG "org.visp.android.Utils"
#include "common.h"

#include "visp3/core/vpImage.h"
#include "visp3/imgproc/vpImgproc.h"

#ifdef __ANDROID__
#include <android/bitmap.h>

using namespace vp;

/*
 * Contains functions for conversion from Bitmap to vpImage and  vpImage to Bitmap
 * @author: AKS1996
 */


extern "C" {

/*
 * Class:     org_visp_android_Utils
 * Method:    void nBitmapToVpImage2(Bitmap b, long m_addr, boolean unPremultiplyAlpha)
 */

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nBitmapToVpImage2
  (JNIEnv * env, jclass, jobject bitmap, jlong m_addr, jboolean needUnPremultiplyAlpha);

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nBitmapToVpImage2
  (JNIEnv * env, jclass, jobject bitmap, jlong m_addr, jboolean needUnPremultiplyAlpha)
{
    AndroidBitmapInfo  info;
    void*              pixels = 0;
    vpImage&               dst = *((vpImage*)m_addr);

    try {
            LOGD("nBitmapToVpImage");
            assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
            assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                       info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
            assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
            assert( pixels );
            dst.create(info.height, info.width, CV_8UC4);
            if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
            {
                LOGD("nBitmapToVpImage: RGBA_8888 -> CV_8UC4");
                Mat tmp(info.height, info.width, CV_8UC4, pixels);
                if(needUnPremultiplyAlpha) cvtColor(tmp, dst, COLOR_mRGBA2RGBA);
                else tmp.copyTo(dst);
            } else {
                // info.format == ANDROID_BITMAP_FORMAT_RGB_565
                LOGD("nBitmapToVpImage: RGB_565 -> CV_8UC4");
                Mat tmp(info.height, info.width, CV_8UC2, pixels);
                cvtColor(tmp, dst, COLOR_BGR5652RGBA);
            }
            AndroidBitmap_unlockPixels(env, bitmap);
            return;
        } catch(const cv::Exception& e) {
            AndroidBitmap_unlockPixels(env, bitmap);
            LOGE("nBitmapToVpImage catched cv::Exception: %s", e.what());
            jclass je = env->FindClass("org/visp/core/VpException");
            if(!je) je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, e.what());
            return;
        } catch (...) {
            AndroidBitmap_unlockPixels(env, bitmap);
            LOGE("nBitmapToVpImage catched unknown exception (...)");
            jclass je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToVpImage}");
            return;
        }
}

// old signature is left for binary compatibility with 2.4.0 & 2.4.1, to removed in 2.5
JNIEXPORT void JNICALL Java_org_visp_android_Utils_nBitmapToVpImage
  (JNIEnv * env, jclass, jobject bitmap, jlong m_addr);

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nBitmapToVpImage
  (JNIEnv * env, jclass, jobject bitmap, jlong m_addr)
{
    Java_org_visp_android_Utils_nBitmapToVpImage2(env, 0, bitmap, m_addr, false);
}

/*
 * Class:     org_visp_android_Utils
 * Method:    void nVpImageToBitmap2(long m_addr, Bitmap b, boolean premultiplyAlpha)
 */

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nVpImageToBitmap2
  (JNIEnv * env, jclass, jlong m_addr, jobject bitmap, jboolean needPremultiplyAlpha);

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nVpImageToBitmap2
  (JNIEnv * env, jclass, jlong m_addr, jobject bitmap, jboolean needPremultiplyAlpha)
{
    AndroidBitmapInfo  info;
    void*              pixels = 0;
    vpImage&               src = *((vpImage*)m_addr);

    try {
            LOGD("nVpImageToBitmap");
            assert( AndroidBitmap_getInfo(env, bitmap, &info) >= 0 );
            assert( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                       info.format == ANDROID_BITMAP_FORMAT_RGB_565 );
            assert( src.dims == 2 && info.height == (uint32_t)src.rows && info.width == (uint32_t)src.cols );
            assert( src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4 );
            assert( AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0 );
            assert( pixels );
            if( info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 )
            {
                Mat tmp(info.height, info.width, CV_8UC4, pixels);
                if(src.type() == CV_8UC1)
                {
                    LOGD("nVpImageToBitmap: CV_8UC1 -> RGBA_8888");
                    cvtColor(src, tmp, COLOR_GRAY2RGBA);
                } else if(src.type() == CV_8UC3){
                    LOGD("nVpImageToBitmap: CV_8UC3 -> RGBA_8888");
                    cvtColor(src, tmp, COLOR_RGB2RGBA);
                } else if(src.type() == CV_8UC4){
                    LOGD("nVpImageToBitmap: CV_8UC4 -> RGBA_8888");
                    if(needPremultiplyAlpha) cvtColor(src, tmp, COLOR_RGBA2mRGBA);
                    else src.copyTo(tmp);
                }
            } else {
                // info.format == ANDROID_BITMAP_FORMAT_RGB_565
                Mat tmp(info.height, info.width, CV_8UC2, pixels);
                if(src.type() == CV_8UC1)
                {
                    LOGD("nVpImageToBitmap: CV_8UC1 -> RGB_565");
                    cvtColor(src, tmp, COLOR_GRAY2BGR565);
                } else if(src.type() == CV_8UC3){
                    LOGD("nVpImageToBitmap: CV_8UC3 -> RGB_565");
                    cvtColor(src, tmp, COLOR_RGB2BGR565);
                } else if(src.type() == CV_8UC4){
                    LOGD("nVpImageToBitmap: CV_8UC4 -> RGB_565");
                    cvtColor(src, tmp, COLOR_RGBA2BGR565);
                }
            }
            AndroidBitmap_unlockPixels(env, bitmap);
            return;
        } catch(const cv::Exception& e) {
            AndroidBitmap_unlockPixels(env, bitmap);
            LOGE("nVpImageToBitmap catched cv::Exception: %s", e.what());
            jclass je = env->FindClass("org/visp/core/VpException");
            if(!je) je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, e.what());
            return;
        } catch (...) {
            AndroidBitmap_unlockPixels(env, bitmap);
            LOGE("nVpImageToBitmap catched unknown exception (...)");
            jclass je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, "Unknown exception in JNI code {nVpImageToBitmap}");
            return;
        }
}

// old signature is left for binary compatibility with 2.4.0 & 2.4.1, to removed in 2.5
JNIEXPORT void JNICALL Java_org_visp_android_Utils_nVpImageToBitmap
  (JNIEnv * env, jclass, jlong m_addr, jobject bitmap);

JNIEXPORT void JNICALL Java_org_visp_android_Utils_nVpImageToBitmap
  (JNIEnv * env, jclass, jlong m_addr, jobject bitmap)
{
    Java_org_visp_android_Utils_nVpImageToBitmap2(env, 0, m_addr, bitmap, false);
}

} // extern "C"

#endif //__ANDROID__
