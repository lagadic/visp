// This file is part of ViSP project
#ifndef LISTCONVERTERS_HPP
#define	LISTCONVERTERS_HPP

#include "visp3/visp_modules.h"
#include "visp3/visp_core.h"

jlongArray vector_vpColVector_to_List(JNIEnv* env, std::vector<vpColVector>);
std::vector<vpHomogeneousMatrix> List_to_vector_vpHomogeneousMatrix(JNIEnv* env, jlongArray arr);

#endif	/* LISTCONVERTERS_HPP */
