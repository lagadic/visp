// This file is part of ViSP project.
#define LOG_TAG "org.visp.utils.Converters"
#include "common.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, std::vector<vpColVector> V)
{
  long arr[V.size()] = {0};
  for(int i=0;i<V.size();++i){
    arr[i] = reinterpret_cast<long>(&V[i]);
  }
   
  jlongArray result;
  result = env->NewLongArray(V.size());  
  env->SetLongArrayRegion(result, 0, V.size(), arr);

  return result;
}

/*
jobject vector_String_to_List(JNIEnv* env, std::vector<cv::String>& vs) {

    static jclass juArrayList   = ARRAYLIST(env);
    static jmethodID m_create   = CONSTRUCTOR(env, juArrayList);
    jmethodID m_add       = LIST_ADD(env, juArrayList);

    jobject result = env->NewObject(juArrayList, m_create, vs.size());
    for (std::vector<cv::String>::iterator it = vs.begin(); it != vs.end(); ++it) {
        jstring element = env->NewStringUTF((*it).c_str());
        env->CallBooleanMethod(result, m_add, element);
        env->DeleteLocalRef(element);
    }
    return result;
}

    public  void finalize(List<Mat> inputs, List<Mat> outputs)
    {
        Mat inputs_mat = Converters.vector_Mat_to_Mat(inputs);
        Mat outputs_mat = new Mat();
        finalize_1(nativeObj, inputs_mat.nativeObj, outputs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(outputs_mat, outputs);
        outputs_mat.release();
        return;
    }
*/
