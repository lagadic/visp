// This file is part of ViSP project.
#define LOG_TAG "org.visp.utils.Converters"
#include "common.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, const std::vector<vpColVector>& V)
{
  jlongArray result = env->NewLongArray(V.size());
  jlong *body = env->GetLongArrayElements(result, 0);

  for(size_t i = 0; i < V.size(); i++) {
    body[i] = (jlong) new vpColVector(V[i]);
  }

  env->ReleaseLongArrayElements(result, body, 0);
  return result;
}

jlongArray vector_vpHomogeneousMatrix_to_List(JNIEnv* env, const std::vector<vpHomogeneousMatrix>& V)
{
  jlongArray result = env->NewLongArray(V.size());
  jlong *body = env->GetLongArrayElements(result, 0);

  for(size_t i = 0; i < V.size(); i++) {
    body[i] = (jlong) new vpHomogeneousMatrix(V[i]);
  }

  env->ReleaseLongArrayElements(result, body, 0);
  return result;
}

std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr)
{
  jlong *body = env->GetLongArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  std::vector<vpHomogeneousMatrix> V(len);
  for(int i = 0; i < len; i++) {
    vpHomogeneousMatrix *temp = (vpHomogeneousMatrix*) body[i];
    V[i] = *temp;
  }

  env->ReleaseLongArrayElements(arr, body, 0);
  return V;
}

std::vector<vpCameraParameters> List_to_vector_vpCameraParameters(JNIEnv* env, jlongArray arr)
{
  jlong *body = env->GetLongArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  std::vector<vpCameraParameters> V(len);
  for(int i = 0; i < len; i++) {
    vpCameraParameters *temp = (vpCameraParameters*) body[i];
    V[i] = *temp;
  }

  env->ReleaseLongArrayElements(arr, body, 0);
  return V;
}

std::vector<int> List_to_vector_int(JNIEnv* env, jintArray arr)
{
  jint *body = env->GetIntArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  std::vector<int> V(len);
  for(int i = 0; i < len; i++) {
    V[i] = body[i];
  }

  env->ReleaseIntArrayElements(arr, body, 0);
  return V;
}

std::vector<float> List_to_vector_float(JNIEnv* env, jfloatArray arr)
{
  jfloat *body = env->GetFloatArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  std::vector<float> V(len);
  for(int i = 0; i < len; i++) {
    V[i] = body[i];
  }

  env->ReleaseFloatArrayElements(arr, body, 0);
  return V;
}

std::vector<double> List_to_vector_double(JNIEnv* env, jdoubleArray arr)
{
  jdouble *body = env->GetDoubleArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  std::vector<double> V(len);
  for(int i = 0; i < len; i++) {
    V[i] = body[i];
  }

  env->ReleaseDoubleArrayElements(arr, body, 0);
  return V;
}

jobjectArray vector_vector_vpImagePoint_to_List(JNIEnv *env, const std::vector<std::vector<vpImagePoint> >& V) {
  if (V.empty()) {
    return NULL;
  }

  size_t outerSize = V.size();
  jobjectArray outerArray = env->NewObjectArray(outerSize, env->FindClass("java/lang/Object"), NULL);

  for (int i = 0; i < env->GetArrayLength(outerArray); i++) {
    size_t innerSize = V[i].size();
    jlongArray longArray = env->NewLongArray(innerSize);
    jlong *longArrayElements = env->GetLongArrayElements(longArray, 0);

    for (int j = 0; j < env->GetArrayLength(longArray); j++) {
      longArrayElements[j] = (jlong) new vpImagePoint(V[i][j]);
    }

    env->ReleaseLongArrayElements(longArray, longArrayElements, 0);
    env->SetObjectArrayElement(outerArray, i, longArray);
  }

  return outerArray;
}

jobjectArray vector_vector_double_to_List(JNIEnv *env, const std::vector<std::vector<double> >& V) {
  if (V.empty()) {
    return NULL;
  }

  size_t outerSize = V.size();
  jobjectArray outerArray = env->NewObjectArray(outerSize, env->FindClass("java/lang/Object"), NULL);

  for (int i = 0; i < env->GetArrayLength(outerArray); i++) {
    size_t innerSize = V[i].size();
    jdoubleArray doubleArray = env->NewDoubleArray(innerSize);
    jdouble *doubleArrayElements = env->GetDoubleArrayElements(doubleArray, 0);

    for (int j = 0; j < env->GetArrayLength(doubleArray); j++) {
      doubleArrayElements[j] = (jdouble) V[i][j];
    }

    env->ReleaseDoubleArrayElements(doubleArray, doubleArrayElements, 0);
    env->SetObjectArrayElement(outerArray, i, doubleArray);
  }

  return outerArray;
}

std::string convertTo(JNIEnv *env, jstring jstr) {
  const char *rawString = env->GetStringUTFChars(jstr, 0);
  std::string cppString(rawString ? rawString : "");
  env->ReleaseStringUTFChars(jstr, rawString);

  return cppString;
}

jstring convertTo(JNIEnv *env, const std::string& str) {
  return env->NewStringUTF(str.c_str());
}

jobjectArray map_string_vector_vector_double_to_array_native(JNIEnv *env, const std::map<std::string, std::vector<std::vector<double> > > &map) {
    if (map.empty()) {
    return NULL;
  }

  size_t mapSize = map.size();
  jobjectArray mapArray = env->NewObjectArray(mapSize, env->FindClass("java/lang/Object"), NULL);

  int idx = 0;
  for (std::map<std::string, std::vector<std::vector<double> > >::const_iterator it = map.begin(); it != map.end(); ++it, idx++) {
    size_t outerSize = it->second.size();
    jobjectArray outerArray = env->NewObjectArray(outerSize, env->FindClass("java/lang/Object"), NULL);

    for (int i = 0; i < env->GetArrayLength(outerArray); i++) {
      size_t innerSize = it->second[i].size();
      jdoubleArray doubleArray = env->NewDoubleArray(innerSize);
      jdouble *doubleArrayElements = env->GetDoubleArrayElements(doubleArray, 0);

      for (int j = 0; j < env->GetArrayLength(doubleArray); j++) {
        doubleArrayElements[j] = (jdouble) it->second[i][j];
      }

      env->ReleaseDoubleArrayElements(doubleArray, doubleArrayElements, 0);
      env->SetObjectArrayElement(outerArray, i, doubleArray);
    }

    env->SetObjectArrayElement(mapArray, idx, outerArray);
  }

  return mapArray;
}

jobjectArray vector_string_to_array_native(JNIEnv *env, const std::vector<std::string>& V) {
  if (V.empty()) {
    return NULL;
  }

  size_t vecSize = V.size();
  jobjectArray vec = env->NewObjectArray(vecSize, env->FindClass("java/lang/String"), env->NewStringUTF(""));
  for (size_t i = 0; i < V.size(); i++) {
    env->SetObjectArrayElement(vec, i, env->NewStringUTF(V[i].c_str()));
  }

  return vec;
}

std::vector<std::string> array_string_to_vector(JNIEnv *env, jobjectArray arr) {
  int size = env->GetArrayLength(arr);

  std::vector<std::string> vec(size);
  for (int i = 0; i < size; i++) {
    vec[i] = convertTo(env, (jstring)env->GetObjectArrayElement(arr, i));
  }

  return vec;
}
