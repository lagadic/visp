// This file is part of ViSP project
#ifndef LISTCONVERTERS_HPP
#define LISTCONVERTERS_HPP

#include "visp3/visp_modules.h"
#include "visp3/visp_core.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, const std::vector<vpColVector>&);

jlongArray vector_vpHomogeneousMatrix_to_List(JNIEnv* env, const std::vector<vpHomogeneousMatrix>&);

std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr);

std::vector<vpCameraParameters> List_to_vector_vpCameraParameters(JNIEnv* env, jlongArray arr);

std::vector<int> List_to_vector_int(JNIEnv* env, jintArray arr);

std::vector<float> List_to_vector_float(JNIEnv* env, jfloatArray arr);

std::vector<double> List_to_vector_double(JNIEnv* env, jdoubleArray arr);

jobjectArray vector_vector_vpImagePoint_to_List(JNIEnv *env, const std::vector<std::vector<vpImagePoint> >&);

jobjectArray vector_vector_double_to_List(JNIEnv *env, const std::vector<std::vector<double> >& V);

std::string convertTo(JNIEnv *env, jstring);

jstring convertTo(JNIEnv *env, const std::string& str);

jobjectArray map_string_vector_vector_double_to_array_native(JNIEnv *env, const std::map<std::string, std::vector<std::vector<double> > >& map);

jobjectArray vector_string_to_array_native(JNIEnv *env, const std::vector<std::string>& V);

std::vector<std::string> array_string_to_vector(JNIEnv *env, jobjectArray arr);

#endif  /* LISTCONVERTERS_HPP */
