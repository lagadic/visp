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

std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr)
{
  std::vector<vpHomogeneousMatrix> V;
  jlong *body = env->GetLongArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i = 0; i < len; i++) {
    vpHomogeneousMatrix *temp = (vpHomogeneousMatrix*) body[i];
    V.push_back(*temp);
  }

  env->ReleaseLongArrayElements(arr, body, 0);
  return V;
}

std::vector<float> List_to_vector_float(JNIEnv* env, jfloatArray arr)
{
  std::vector<float> V;
  jfloat *body = env->GetFloatArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i = 0; i < len; i++) {
    V.push_back(body[i]);
  }

  env->ReleaseFloatArrayElements(arr, body, 0);
  return V;
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

std::vector<double> List_to_vector_double(JNIEnv* env, jdoubleArray arr)
{
  std::vector<double> V;
  jdouble *body = env->GetDoubleArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i = 0; i < len; i++) {
    V.push_back(body[i]);
  }

  env->ReleaseDoubleArrayElements(arr, body, 0);
  return V;
}

std::vector<int> List_to_vector_int(JNIEnv* env, jintArray arr)
{
  std::vector<int> V;
  jint *body = env->GetIntArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i = 0; i < len; i++) {
    V.push_back(body[i]);
  }

  env->ReleaseIntArrayElements(arr, body, 0);
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
