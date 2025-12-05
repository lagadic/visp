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

#ifndef VP_SIMD_UTILS_H
#define VP_SIMD_UTILS_H
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

namespace vpSIMD
{
#if defined(VISP_HAVE_AVX2)
using Register = __m512d;

inline const int numLanes = 8;
inline Register add(const Register a, const Register b)
{
  return _mm512_add_pd(a, b);
}

inline Register sub(const Register a, const Register b)
{
  return _mm512_sub_pd(a, b);
}

inline Register mul(const Register a, const Register b)
{
  return _mm512_mul_pd(a, b);
}


inline Register fma(const Register a, const Register b, const Register c)
{
#if defined(VISP_HAVE_FMA)
  return _mm512_fmadd_pd(a, b, c);
#else
  return add(mul(a, b), c);
#endif
}

inline Register loadu(const double *const data)
{
  return _mm512_loadu_pd(data);
}


inline Register set1(double v)
{
  return _mm512_set1_pd(v);
}

inline void storeu(double *data, const Register a)
{
  _mm512_storeu_pd(data, a);
}

#elif defined(VISP_HAVE_AVX)
using Register = __m256d;
inline const int numLanes = 4;

inline Register add(const Register a, const Register b)
{
  return _mm256_add_pd(a, b);
}

inline Register sub(const Register a, const Register b)
{
  return _mm256_sub_pd(a, b);
}

inline Register mul(const Register a, const Register b)
{
  return _mm256_mul_pd(a, b);
}


inline Register fma(const Register a, const Register b, const Register c)
{
#if defined(VISP_HAVE_FMA)
  return _mm256_fmadd_pd(a, b, c);
#else
  return add(mul(a, b), c);
#endif
}

inline Register loadu(const double *const data)
{
  return _mm256_loadu_pd(data);
}


inline Register set1(double v)
{
  return _mm256_set1_pd(v);
}

inline void storeu(double *data, const Register a)
{
  _mm256_storeu_pd(data, a);
}

#elif VISP_HAVE_SSE2
using Register = __m128d;
inline const int numLanes = 2;

inline Register add(const Register a, const Register b)
{
  return _mm_add_pd(a, b);
}

inline Register sub(const Register a, const Register b)
{
  return _mm_sub_pd(a, b);
}

inline Register mul(const Register a, const Register b)
{
  return _mm_mul_pd(a, b);
}


inline Register fma(const Register a, const Register b, const Register c)
{
#if defined(VISP_HAVE_FMA)
  return _mm_fmadd_pd(a, b, c);
#else
  return add(mul(a, b), c);
#endif
}

inline Register loadu(const double *const data)
{
  return _mm_loadu_pd(data);
}

inline Register set1(double v)
{
  return _mm_set1_pd(v);
}

inline void storeu(double *data, const Register a)
{
  _mm_storeu_pd(data, a);
}

#endif

}

#endif // VP_SIMD_UTILS_H
