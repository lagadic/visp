// This file is part of ViSP project.
#ifndef __VISP_JAVA_COMMON_H__
#define __VISP_JAVA_COMMON_H__

#include <stdexcept>
#include <string>

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
# define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>

} // extern "C"

#include "visp_java.hpp"
//#include "visp/core/utility.hpp" TODO to be defined or remove

#include "converters.h"
#include "listconverters.hpp"

#ifdef _MSC_VER
#  pragma warning(disable:4800 4244)
#endif

#endif //__VISP_JAVA_COMMON_H__
