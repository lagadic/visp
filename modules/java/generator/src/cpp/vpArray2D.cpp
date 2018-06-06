#include <jni.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include "VpArray2D.h"  // This include is native header file for current source file
#include <visp3/core/vpArray2D.h>


// Java Method:    VpArray2D(Class type)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpArray2D_n_1VpArray2D__
  (JNIEnv *env, jclass){
	return (jlong) new vpArray2D<double>();
}


// Java Method:    VpArray2D(Class type, int r, int c)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpArray2D_n_1VpArray2D__II
  (JNIEnv *env, jclass, jint r, jint c){
	return (jlong) new vpArray2D<double>(r,c);
}

// Java Method:    VpArray2D(Class type, int r, int c, double val)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpArray2D_n_1VpArray2D__IID
  (JNIEnv *env, jclass, jint r, jint c, jdouble value){
	return (jlong) new vpArray2D<double>(r,c, value);
}

// Java Method:    getCols(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_VpArray2D_n_1cols
  (JNIEnv *env, jclass, jlong address){
	vpArray2D<double>* me = (vpArray2D<double>*) address; //TODO: check for NULL
	return me->getCols();
}

// Java Method:    getRows(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_VpArray2D_n_1rows
  (JNIEnv *env, jclass, jlong address){
	vpArray2D<double>* me = (vpArray2D<double>*) address; //TODO: check for NULL
	return me->getRows();
}
