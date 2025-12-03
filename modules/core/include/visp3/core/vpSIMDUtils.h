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
 * Rotation matrix.
 */

/*!
  \file vpSIMDUtils.h
  \brief Header that defines and includes useful SIMD routines and macros
*/
#include <visp3/core/vpConfig.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#include <immintrin.h>
#include <smmintrin.h>

#define VISP_HAVE_SSE2 1
#endif

#if defined __AVX2__
#define VISP_HAVE_AVX2 1
#endif

#if defined __AVX__
#define VISP_HAVE_AVX 1
#endif

// https://stackoverflow.com/a/40765925
#if !defined(__FMA__) && defined(__AVX2__)
#define __FMA__ 1
#endif


#if defined(__FMA__)
#define VISP_HAVE_FMA
#endif

#if defined _WIN32 && defined(_M_ARM64)
#define _ARM64_DISTINCT_NEON_TYPES
#include <Intrin.h>
#include <arm_neon.h>
#define VISP_HAVE_NEON 1
#elif (defined(__ARM_NEON__) || defined (__ARM_NEON)) && defined(__aarch64__)
#include <arm_neon.h>
#define VISP_HAVE_NEON 1
#else
#define VISP_HAVE_NEON 0
#endif


#if VISP_HAVE_SSE2 && USE_SIMD_CODE
#define USE_SSE 1
#else
#define USE_SSE 0
#endif

#if VISP_HAVE_NEON && USE_SIMD_CODE
#define USE_NEON 1
#else
#define USE_NEON 0
#endif
