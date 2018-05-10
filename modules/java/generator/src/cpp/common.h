#ifndef __JAVA_COMMON_H__
#define __JAVA_COMMON_H__

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
# define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>

#include "visp/java.hpp"
//#include "visp/core/utility.hpp" TODO to be defined or remove

#include "converters.h"
#include "listconverters.hpp"

#ifdef _MSC_VER
#  pragma warning(disable:4800 4244)
#endif

#endif //__JAVA_COMMON_H__
