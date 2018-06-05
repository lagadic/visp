#define LOG_TAG "org.visp.core.VpMatrix"
/*
 * This file contains the JNI equivalent member functions of C++ class `vpMatrix`
 * Refer http://visp-doc.inria.fr/doxygen/visp-daily/classvpMatrix.html for member functions of vpMatrix class

 * TODO The link contains some 120 member functions. Most of them are yet to be added. Refer Open-CV's equivalent files
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

      // In open-cv its cv::Exception. Here its vpException
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
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__
        (JNIEnv *, jclass);

JNIEXPORT jlong
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__
        (JNIEnv *, jclass) {
    LOGD("vpMatrix::n_1vpMatrix__()");
    return (jlong)
    new vpMatrix(); // Returns the hex address of the object - like 0x55d3f2d9d890
}



//
//   vpMatrix::vpMatrix(int rows, int cols, double value)
//

JNIEXPORT jlong
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value);

JNIEXPORT jlong
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value) {
    static const char method_name[] = "vpMatrix::n_1vpMatrix__IID()";
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
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__II
        (JNIEnv *env, jclass, jint rows, jint cols);

JNIEXPORT jlong
JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__II
        (JNIEnv *env, jclass, jint rows, jint cols) {
    static const char method_name[] = "vpMatrix::n_1vpMatrix__II()";
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

//
// vpMatrix::vpMatrix(vpMatrix,int rows,int cols,int nrows, int ncols)
//

JNIEXPORT jlong JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__JIIII
  (JNIEnv *, jclass, jlong address, jint rows, jint cols, jint nrows, jint ncols);

JNIEXPORT jlong JNICALL Java_org_visp_core_vpMatrix_n_1vpMatrix__JIIII
  (JNIEnv *, jclass, jlong address, jint rows, jint cols, jint nrows, jint ncols){
	vpMatrix* me = (vpMatrix*) address; //TODO: check for NULL
	return (jlong) new vpMatrix(*me,rows,cols,nrows,ncols);
}

//
// vpMatrix::getCols()
//

JNIEXPORT jint JNICALL Java_org_visp_core_vpMatrix_n_1cols
  (JNIEnv *, jclass, jlong self);

JNIEXPORT jint JNICALL Java_org_visp_core_vpMatrix_n_1cols
  (JNIEnv *, jclass, jlong self){
	vpMatrix* me = (vpMatrix*) self; //TODO: check for NULL
	return me->getCols();
}

//
// vpMatrix::getRows()
//

JNIEXPORT jint JNICALL Java_org_visp_core_vpMatrix_n_1cols
  (JNIEnv *, jclass, jlong self);

JNIEXPORT jint JNICALL Java_org_visp_core_vpMatrix_n_1rows
  (JNIEnv *, jclass, jlong self){
	vpMatrix* me = (vpMatrix*) self; //TODO: check for NULL
	return me->getRows();
}

//
// void vpMatrix::transpose(vpMatrix&)
//

JNIEXPORT void JNICALL Java_org_visp_core_vpMatrix_n_1transpose
  (JNIEnv *, jclass, jlong self, jlong result){
	vpMatrix* me = (vpMatrix*) self;
	vpMatrix* other = (vpMatrix*) result;
	me->transpose(*other);
}

// Java Method:    dump()
JNIEXPORT jstring JNICALL Java_org_visp_core_vpMatrix_n_1dump
  (JNIEnv *env, jclass, jlong address){
		
	vpMatrix* me = (vpMatrix*) address; //TODO: check for NULL
	std::stringstream ss;
	ss << *me;
	return env->NewStringUTF(ss.str().c_str());
}
