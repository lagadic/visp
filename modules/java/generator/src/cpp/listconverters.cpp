// This file is part of ViSP project.
#define LOG_TAG "org.visp.utils.Converters"
#include "common.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, std::vector<vpColVector> V)
{
  jlongArray result;
  result = env->NewLongArray(V.size());
  jlong *body = env->GetLongArrayElements(result, 0);

  for(unsigned int i=0;i<V.size();++i)
    body[i] = (long) new vpColVector(V[i]);

  env->ReleaseLongArrayElements(result, body, 0);

  return result;
}

std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr)
{
  std::vector<vpHomogeneousMatrix> V;
  jlong *body = env->GetLongArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i=0;i<len;++i){
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

  for(int i=0;i<len;++i)
    V.push_back(body[i]);

  env->ReleaseFloatArrayElements(arr, body, 0);
  return V;
}

jlongArray vector_vpHomogeneousMatrix_to_List(JNIEnv* env, std::vector<vpHomogeneousMatrix> V)
{
  jlongArray result;
  result = env->NewLongArray(V.size());
  jlong *body = env->GetLongArrayElements(result, 0);

  for(unsigned int i=0;i<V.size();++i)
    body[i] = (long) new vpHomogeneousMatrix(V[i]);

  env->ReleaseLongArrayElements(result, body, 0);

  return result;
}

std::vector<double> List_to_vector_double(JNIEnv* env, jdoubleArray arr)
{
  std::vector<double> V;
  jdouble *body = env->GetDoubleArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i=0;i<len;++i)
    V.push_back(body[i]);

  env->ReleaseDoubleArrayElements(arr, body, 0);

  return V;
}

std::vector<int> List_to_vector_int(JNIEnv* env, jintArray arr)
{
  std::vector<int> V;
  jint *body = env->GetIntArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);

  for(int i=0;i<len;++i)
    V.push_back(body[i]);

  env->ReleaseIntArrayElements(arr, body, 0);

  return V;
}
