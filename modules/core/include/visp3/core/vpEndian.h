/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Determine machine endianness and define VISP_LITTLE_ENDIAN, VISP_BIG_ENDIAN and VISP_PDP_ENDIAN macros.
 */

/*!
  \file vpEndian.h
  \brief Determine machine endianness and define VISP_LITTLE_ENDIAN, VISP_BIG_ENDIAN and VISP_PDP_ENDIAN macros.
*/

#ifndef VP_ENDIAN_H
#define VP_ENDIAN_H

// Visual Studio 2010 or previous is missing inttypes.h
#if defined(_MSC_VER) && (_MSC_VER < 1700)
typedef unsigned short uint16_t;
#else
#include <inttypes.h>
#endif
#include <stdint.h> //for uint32_t related types ; works also with >= VS2010 / _MSC_VER >= 1600
#include <visp3/core/vpConfig.h>

// Detect endianness of the host machine
// Reference: http://www.boost.org/doc/libs/1_36_0/boost/detail/endian.hpp
#if defined(__GLIBC__) || (defined(__GNUC__) && !defined(__llvm__) && !defined(__MINGW32__) &&                         \
                           !defined(__FreeBSD__) && defined(__BYTE_ORDER__))
#include <endian.h>
#if (__BYTE_ORDER == __LITTLE_ENDIAN)
#define VISP_LITTLE_ENDIAN
#elif (__BYTE_ORDER == __BIG_ENDIAN)
#define VISP_BIG_ENDIAN
#elif (__BYTE_ORDER == __PDP_ENDIAN)
// Currently not supported when reading / writing binary file
#define VISP_PDP_ENDIAN
//#error PDP endian is not supported. //Uncomment if needed/happens
#else
#error Unknown machine endianness detected.
#endif
#elif defined(_BIG_ENDIAN) && !defined(_LITTLE_ENDIAN) || defined(__BIG_ENDIAN__) && !defined(__LITTLE_ENDIAN__)
#define VISP_BIG_ENDIAN
#elif defined(_LITTLE_ENDIAN) && !defined(_BIG_ENDIAN) || defined(__LITTLE_ENDIAN__) && !defined(__BIG_ENDIAN__)
#define VISP_LITTLE_ENDIAN
#elif defined(__sparc) || defined(__sparc__) || defined(_POWER) || defined(__powerpc__) || defined(__ppc__) ||         \
    defined(__hpux) || defined(_MIPSEB) || defined(_POWER) || defined(__s390__)

#define VISP_BIG_ENDIAN
#elif defined(__i386__) || defined(__alpha__) || defined(__ia64) || defined(__ia64__) || defined(_M_IX86) ||           \
    defined(_M_IA64) || defined(_M_ALPHA) || defined(__amd64) || defined(__amd64__) || defined(_M_AMD64) ||            \
    defined(__x86_64) || defined(__x86_64__) || defined(_M_X64) || defined(__ANDROID__)
// It appears that all Android systems are little endian.
// Refer https://stackoverflow.com/questions/6212951/endianness-of-android-ndk
#define VISP_LITTLE_ENDIAN
#elif defined(WINRT) // For UWP
// Refer
// https://social.msdn.microsoft.com/Forums/en-US/04c92ef9-e38e-415f-8958-ec9f7c196fd3/arm-endianess-under-windows-mobile?forum=windowsmobiledev
#define VISP_LITTLE_ENDIAN
#else
#error Cannot detect host machine endianness.
#endif

BEGIN_VISP_NAMESPACE
namespace vpEndian
{
VISP_EXPORT uint16_t swap16bits(uint16_t val);

VISP_EXPORT uint32_t swap32bits(uint32_t val);

VISP_EXPORT float swapFloat(float f);

VISP_EXPORT double swapDouble(double d);

VISP_EXPORT uint16_t reinterpret_cast_uchar_to_uint16_LE(unsigned char *const ptr);
} // namespace vpEndian
END_VISP_NAMESPACE
#endif
