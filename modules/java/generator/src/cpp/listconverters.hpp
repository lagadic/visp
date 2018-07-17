// This file is part of ViSP project
#ifndef LISTCONVERTERS_HPP
#define	LISTCONVERTERS_HPP

#include "visp3/visp_modules.h"
#include "visp3/visp_core.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, std::vector<vpColVector>);

std::vector<int> List_to_vector_int(JNIEnv* env, jintArray arr);

std::vector<float> List_to_vector_float(JNIEnv* env, jfloatArray arr);

std::vector<double> List_to_vector_double(JNIEnv* env, jdoubleArray arr);

#endif	/* LISTCONVERTERS_HPP */
