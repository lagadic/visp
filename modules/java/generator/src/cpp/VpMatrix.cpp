#include <visp3/core/vpMatrix.h>

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
#  define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>

} // extern "C"


//
//   vpMatrix::vpMatrix() - Basic constructor of a matrix of double
//
JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__
(JNIEnv *, jclass);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__
(JNIEnv *, jclass) {
  return (jlong)
      new vpMatrix(); // Returns the hex address of the object - like 0x55d3f2d9d890
}

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
//   vpMatrix::vpMatrix(int rows, int cols) - Initialize with value 0
//
JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__II
(JNIEnv *env, jclass, jint rows, jint cols);

JNIEXPORT jlong
JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__II
(JNIEnv *env, jclass, jint rows, jint cols) {
  return (jlong)
      new vpMatrix(rows, cols);
}

//
// vpMatrix::vpMatrix(vpMatrix,int rows,int cols,int nrows, int ncols)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__JIIII
(JNIEnv *, jclass, jlong address, jint rows, jint cols, jint nrows, jint ncols);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpMatrix_n_1vpMatrix__JIIII
(JNIEnv *, jclass, jlong address, jint rows, jint cols, jint nrows, jint ncols){
  vpMatrix* me = (vpMatrix*) address; //TODO: check for NULL
  return (jlong) new vpMatrix(*me,rows,cols,nrows,ncols);
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

//
// vpMatrix::getRows()
//
JNIEXPORT jint JNICALL Java_org_visp_core_VpMatrix_n_1cols
(JNIEnv *, jclass, jlong self);

JNIEXPORT jint JNICALL Java_org_visp_core_VpMatrix_n_1rows
(JNIEnv *, jclass, jlong self){
  vpMatrix* me = (vpMatrix*) self; //TODO: check for NULL
  return me->getRows();
}

//
// void vpMatrix::transpose(vpMatrix&)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpMatrix_n_1transpose
(JNIEnv *, jclass, jlong self, jlong result){
  vpMatrix* me = (vpMatrix*) self;
  vpMatrix* other = (vpMatrix*) result;
  me->transpose(*other);
}

// Java Method:    dump()
JNIEXPORT jstring JNICALL Java_org_visp_core_VpMatrix_n_1dump
(JNIEnv *env, jclass, jlong address){

  vpMatrix* me = (vpMatrix*) address; //TODO: check for NULL
  std::stringstream ss;
  ss << *me;
  return env->NewStringUTF(ss.str().c_str());
}
