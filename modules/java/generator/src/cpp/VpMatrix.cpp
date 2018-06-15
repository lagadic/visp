#include <jni.h>  // later add common.h
#include "visp3/core/vpMatrix.h"

extern "C" {

//
//   vpMatrix::vpMatrix(int rows, int cols, double value)
//

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__IID
        (JNIEnv *env, jclass, jint rows, jint cols, jdouble value) {
        return (jlong)
        new vpMatrix(rows, cols, value);
}


//
// vpMatrix::getCols()
//

JNIEXPORT jint JNICALL Java_org_visp_core_VpMatrix_n_1cols
  (JNIEnv *, jclass, jlong self);

JNIEXPORT jint JNICALL Java_org_visp_core_VpMatrix_n_1cols
  (JNIEnv *, jclass, jlong self){
	vpMatrix* me = (vpMatrix*) self; //TODO: check for NULL
	return me->getCols();
}

}
