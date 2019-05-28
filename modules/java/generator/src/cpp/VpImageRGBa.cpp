#include <cstring>
#include <sstream>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
#  define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>


// Java Method:    VpImageRGBa()
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageRGBa_n_1VpImageRGBa__
(JNIEnv *env, jclass, jstring type){
  (void)env;
  (void)type;
  return (jlong) new vpImage<vpRGBa>();
}

// Java Method:    VpImageRGBa(int r, int c)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageRGBa_n_1VpImageRGBa__II
(JNIEnv *env, jclass, jint r, jint c){
  (void)env;
  return (jlong) new vpImage<vpRGBa>(r,c);
}

// Java Method:    VpImageRGBa(int r, int c, byte val)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageRGBa_n_1VpImageRGBa__IICCCC
(JNIEnv *env, jclass, jint r, jint c, jchar R, jchar G, jchar B, jchar A){
  (void)env;
  vpRGBa val(R,G,B,A);
  return (jlong) new vpImage<vpRGBa>(r,c,val);
}

// Java Method:    VpImageRGBa(byte[] array, int height, int width, boolean copyData)
JNIEXPORT jlong JNICALL Java_org_visp_core_VpImageRGBa_n_1VpImageRGBa___3BIIZ
(JNIEnv *env, jclass, jbyteArray arr, jint h, jint w, jboolean copyData){
  jbyte *array = env->GetByteArrayElements(arr, NULL);

  return (jlong) new vpImage<vpRGBa>((vpRGBa *const) array, (const unsigned int) h, (const unsigned int) w, copyData);

  // be memory friendly
  env->ReleaseByteArrayElements(arr, array, 0);
}

// Java Method:    getCols()
JNIEXPORT jint JNICALL Java_org_visp_core_VpImageRGBa_n_1cols
(JNIEnv *env, jclass, jlong address){
  (void)env;
  vpImage<vpRGBa>* me = (vpImage<vpRGBa>*) address; //TODO: check for NULL
  return me->getCols();
}

// Java Method:    getRows()
JNIEXPORT jint JNICALL Java_org_visp_core_VpImageRGBa_n_1rows
(JNIEnv *env, jclass, jlong address){
  (void)env;
  vpImage<vpRGBa>* me = (vpImage<vpRGBa>*) address; //TODO: check for NULL
  return me->getRows();
}

// Java Method:    getPixel(int i, int j)
JNIEXPORT jbyteArray JNICALL Java_org_visp_core_VpImageRGBa_n_1getPixel
(JNIEnv *env, jclass, jlong address, jint i, jint j){
  vpImage<vpRGBa>* me = (vpImage<vpRGBa>*) address; //TODO: check for NULL
  vpRGBa val = (*me)(i,j);
  jbyteArray ret = env->NewByteArray(4);
  unsigned char temp[] = {val.R,val.G,val.B,val.A};
  env->SetByteArrayRegion (ret, 0, 4, (jbyte*) temp);
  return ret;
}

// Java Method:    getPixels()
JNIEXPORT jbyteArray JNICALL Java_org_visp_core_VpImageRGBa_n_1getPixels
(JNIEnv *env, jclass, jlong address){
  vpImage<vpRGBa>* me = (vpImage<vpRGBa>*) address; //TODO: check for NULL
  jbyteArray ret = env->NewByteArray(me->getNumberOfPixel()*4);
  env->SetByteArrayRegion (ret, 0, me->getNumberOfPixel()*4, (jbyte*) me->bitmap);
  return ret;
}

// Java Method:    dump()
JNIEXPORT jstring JNICALL Java_org_visp_core_VpImageRGBa_n_1dump
(JNIEnv *env, jclass, jlong address){
  vpImage<vpRGBa>* me = (vpImage<vpRGBa>*) address; //TODO: check for NULL
  std::stringstream ss;
  ss << *me;
  return env->NewStringUTF(ss.str().c_str());
}

} // extern "C"
