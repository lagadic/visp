#define LOG_TAG "org.visp.core.VpMatrix"
/*
 * This file contains the JNI equivalent member functions of C++ class `vpMatrix`
 * Refer http://visp-doc.inria.fr/doxygen/visp-daily/classvpMatrix.html for member functions of vpMatrix class

 * TODO The link contains some 120 member functions. Most of them are yet to be added. Refer OpenCV's equivalent files
 * author: AKS1996
 */
#include <stdexcept>
#include <string>

#include "common.h"
#include "visp3/core/vpMatrix.h"

/// throw java exception
static void throwJavaException(JNIEnv *env, const std::exception *e, const char *method) {
  std::string what = "unknown exception";
  jclass je = 0;

  if(e) {
    std::string exception_type = "std::exception";

      // In opencv its cv::Exception. Here its vpException
      if(dynamic_cast<const vpException*>(e)) {
          exception_type = "vpException";
          je = env->FindClass("org/visp/core/VpException");
      }

    what = exception_type + ": " + e->what();
  }

  if(!je) je = env->FindClass("java/lang/Exception");
  env->ThrowNew(je, what.c_str());

  LOGE("%s caught %s", method, what.c_str());
  (void)method;        // avoid "unused" warning
}

extern "C" {


//
//   vpMatrix::vpMatrix() - Basic constructor of a matrix of double
//


JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__
        (JNIEnv *, jclass);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__
        (JNIEnv *, jclass) {
    LOGD("VpMatrix::n_1VpMatrix__()");
    return (jlong)
    new vpMatrix(); // Returns the hex address of the object - like 0x55d3f2d9d890
}



//
//   vpMatrix::vpMatrix(int rows, int cols, double value)
//

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value) {
    static const char method_name[] = "VpMatrix::n_1VpMatrix__IID()";
    try {
        LOGD("%s", method_name);
        return (jlong)
        new vpMatrix(rows, cols, value);
    } catch (const std::exception &e) {
        throwJavaException(env, &e, method_name);
    } catch (...) {
        throwJavaException(env, 0, method_name);
    }

    return 0;
}


//
//   vpMatrix::vpMatrix(int rows, int cols) - Initialize with value 0
//

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__II
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1VpMatrix__II
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value) {
    static const char method_name[] = "VpMatrix::n_1VpMatrix__II()";
    try {
        LOGD("%s", method_name);
        return (jlong)
        new vpMatrix(rows, cols);
    } catch (const std::exception &e) {
        throwJavaException(env, &e, method_name);
    } catch (...) {
        throwJavaException(env, 0, method_name);
    }

    return 0;
}
}
