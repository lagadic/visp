#include <cstring>
#include <sstream>
#include <visp3/core/vpImage.h>
typedef unsigned char u_char;

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
#  define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>


// Java Method:    VpImageUChar()
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageUChar_n_1VpImageUChar__
(JNIEnv *env, jclass, jstring type){
  (void)env;
  (void)type;
  return (jlong) new vpImage<u_char>();
}

// Java Method:    VpImageUChar(int r, int c)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageUChar_n_1VpImageUChar__II
(JNIEnv *env, jclass, jint r, jint c){
  (void)env;
  return (jlong) new vpImage<u_char>(r,c);
}

// Java Method:    VpImageUChar(int r, int c, byte val)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageUChar_n_1VpImageUChar__IIB
(JNIEnv *env, jclass, jint r, jint c, jbyte value){
  (void)env;
  return (jlong) new vpImage<u_char>(r,c, (u_char) value);
}

// Java Method:    VpImageUChar(byte[] array, int height, int width, boolean copyData)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageUChar_n_1VpImageUChar___3BIIZ
(JNIEnv *env, jclass, jbyteArray arr, jint h, jint w, jboolean copyData){
  jbyte *array = env->GetByteArrayElements(arr, NULL);

  return (jlong) new vpImage<u_char>((u_char *const) array, (const unsigned int) h, (const unsigned int) w, copyData);

  // be memory friendly
  env->ReleaseByteArrayElements(arr, array, 0);
}

// Java Method:    getCols()
JNIEXPORT jint JNICALL Java_org_visp_core_VpImageUChar_n_1cols
(JNIEnv *env, jclass, jlong address){
  (void)env;
  vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
  return me->getCols();
}

// Java Method:    getRows()
JNIEXPORT jint JNICALL Java_org_visp_core_VpImageUChar_n_1rows
(JNIEnv *env, jclass, jlong address){
  (void)env;
  vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
  return me->getRows();
}

// Java Method:    getPixel(int i, int j)
JNIEXPORT jint JNICALL Java_org_visp_core_VpImageUChar_n_1getPixel
(JNIEnv *env, jclass, jlong address, jint i, jint j){
  (void)env;
  vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
  return (*me)(i,j);
}

// Java Method:    getPixels()
JNIEXPORT jbyteArray JNICALL Java_org_visp_core_VpImageUChar_n_1getPixels
(JNIEnv *env, jclass, jlong address){
  vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
  jbyteArray ret = env->NewByteArray(me->getNumberOfPixel());
  env->SetByteArrayRegion (ret, 0, me->getNumberOfPixel(), (jbyte*) me->bitmap);
  return ret;
}

// Java Method:    dump()
JNIEXPORT jstring JNICALL Java_org_visp_core_VpImageUChar_n_1dump
(JNIEnv *env, jclass, jlong address){
  vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
  std::stringstream ss;
  ss << *me;
  return env->NewStringUTF(ss.str().c_str());
}

} // extern "C"
