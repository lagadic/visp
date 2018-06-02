// This file is part of ViSP project.

#ifndef LISTCONVERTERS_HPP
#define	LISTCONVERTERS_HPP

#include "visp3/visp_modules.hpp"

jobject vector_String_to_List(JNIEnv* env, std::vector<cv::String>& vs);

std::vector<cv::String> List_to_vector_String(JNIEnv* env, jobject list);

void Copy_vector_String_to_List(JNIEnv* env, std::vector<cv::String>& vs, jobject list);

#endif	/* LISTCONVERTERS_HPP */
