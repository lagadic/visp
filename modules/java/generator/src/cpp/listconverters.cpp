// This file is part of ViSP project.
#define LOG_TAG "org.visp.utils.Converters"
#include "common.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, std::vector<vpColVector> V)
{
  jlongArray result;
  result = env->NewLongArray(V.size());  
  jlong *body = env->GetLongArrayElements(result, 0);

  for(unsigned int i=0;i<V.size();++i)
    body[i] = reinterpret_cast<long>(&V[i]);

  env->ReleaseLongArrayElements(result, body, 0);
  
  return result;
}

std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr)
{
  std::vector<vpHomogeneousMatrix> V;
  jlong *body = env->GetLongArrayElements(arr, 0);
  int len = env->GetArrayLength(arr);
  vpHomogeneousMatrix* temp;

  for(unsigned int i=0;i<len;++i){
    temp = (vpHomogeneousMatrix*) body[i]; 
    V.push_back(*temp);
  }

  env->ReleaseLongArrayElements(arr, body, 0);
  
  return V;
}

jlongArray vector_vpHomogeneousMatrix_to_List(JNIEnv* env, std::vector<vpHomogeneousMatrix> V)
{
  jlongArray result;
  result = env->NewLongArray(V.size());  
  jlong *body = env->GetLongArrayElements(result, 0);

  for(unsigned int i=0;i<V.size();++i)
    body[i] = reinterpret_cast<long>(&V[i]);

  env->ReleaseLongArrayElements(result, body, 0);
  
  return result;
}

