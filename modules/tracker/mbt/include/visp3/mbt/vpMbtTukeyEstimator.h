/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Tukey M-estimator.
 *
*****************************************************************************/

#ifndef _vpMbtTukeyEstimator_h_
#define _vpMbtTukeyEstimator_h_

#include <vector>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_VISP_NAMESPACE
template <typename T> class vpMbtTukeyEstimator
{
public:
  void MEstimator(const std::vector<T> &residues, std::vector<T> &weights, T NoiseThreshold);
  void MEstimator(const vpColVector &residues, vpColVector &weights, double NoiseThreshold);

private:
  T getMedian(std::vector<T> &vec);
  void MEstimator_impl(const std::vector<T> &residues, std::vector<T> &weights, T NoiseThreshold);
  void MEstimator_impl_simd(const std::vector<T> &residues, std::vector<T> &weights, T NoiseThreshold);
  void psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights);
  void psiTukey(const T sig, std::vector<T> &x, vpColVector &weights);

  std::vector<T> m_normres;
  std::vector<T> m_residues;
};
END_VISP_NAMESPACE
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 * The code bellow previously in vpMbtTuckeyEstimator.cpp produced
 * a link issue with MinGW-W64 x86_64-8.1.0-posix-seh-rt_v6-rev0 (g++ 8.1.0)
 * libvisp_mbt.so.3.1.0: undefined reference to
 * `vpMbtTukeyEstimator<double>::MEstimator(std::vector<double,
 * std::allocator<double> > const&, std::vector<double, std::allocator<double>
 * >&, double)'
 * Note that with the previous MinGW-W64 version x86_64-7.3.0-posix-seh-rt_v6-rev0 (g++ 7.3.0)
 * the build succeed.
 *
 * To remove this link issue the solution was to move the content of vpMbtTuckeyEstimator.cpp
 * before remove.
 */
#include <algorithm>
#include <cmath>
#include <iostream>

#include <visp3/core/vpCPUFeatures.h>

#define USE_TRANSFORM 1
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) && USE_TRANSFORM
#define HAVE_TRANSFORM 1
#include <functional>
#endif

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1

#if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <pmmintrin.h>
#define VISP_HAVE_SSE3 1
#endif
#if defined __SSSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <tmmintrin.h>
#define VISP_HAVE_SSSE3 1
#endif
#endif

#if defined _WIN32 && defined(_M_ARM64)
#   define _ARM64_DISTINCT_NEON_TYPES
#   include <Intrin.h>
#   include <arm_neon.h>
#   define VISP_HAVE_NEON 1
#elif (defined(__ARM_NEON__) || defined (__ARM_NEON)) && defined(__aarch64__)
#  include <arm_neon.h>
#  define VISP_HAVE_NEON 1
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if HAVE_TRANSFORM
  namespace
{
// Check if std:c++14 or higher
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
auto AbsDiff = [](const auto &a, const auto &b) { return std::fabs(a - b); };
#else
template <typename T> struct AbsDiff : public std::binary_function<T, T, T>
{
  T operator()(const T a, const T b) const { return std::fabs(a - b); }
};
#endif
} // namespace
#endif

BEGIN_VISP_NAMESPACE
template class vpMbtTukeyEstimator<float>;
template class vpMbtTukeyEstimator<double>;

#if VISP_HAVE_SSSE3
namespace
{
inline __m128 abs_ps(__m128 x)
{
  static const __m128 sign_mask = _mm_set1_ps(-0.f); // -0.f = 1 << 31
  return _mm_andnot_ps(sign_mask, x);
}
} // namespace
#endif

template <typename T> T vpMbtTukeyEstimator<T>::getMedian(std::vector<T> &vec)
{
  // Not the exact median when even number of elements
  int index = (int)(ceil(vec.size() / 2.0)) - 1;
  std::nth_element(vec.begin(), vec.begin() + index, vec.end());
  return vec[index];
}

// Without MEstimator_impl, error with g++4.6, ok with gcc 5.4.0
// Ubuntu-12.04-Linux-i386-g++4.6-Dyn-RelWithDebInfo-dc1394-v4l2-X11-OpenCV2.3.1-lapack-gsl-Coin-jpeg-png-xml-pthread-OpenMP-dmtx-zbar-Wov-Weq-Moment:
// libvisp_mbt.so.3.1.0: undefined reference to
// `vpMbtTukeyEstimator<double>::MEstimator(std::vector<double,
// std::allocator<double> > const&, std::vector<double, std::allocator<double>
// >&, double)'
template <typename T>
void vpMbtTukeyEstimator<T>::MEstimator_impl(const std::vector<T> &residues, std::vector<T> &weights,
                                             const T NoiseThreshold)
{
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  T med = getMedian(m_residues);
  m_normres.resize(residues.size());

#if HAVE_TRANSFORM
// Check if std:c++14 or higher
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
  std::transform(residues.begin(), residues.end(), m_normres.begin(), std::bind(AbsDiff, std::placeholders::_1, med));
#else
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<T>(), std::placeholders::_1, med));
#endif
#else
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i] - med));
  }
#endif

  m_residues = m_normres;
  T normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  T sigma = static_cast<T>(1.4826 * normmedian); // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}

template <>
inline void vpMbtTukeyEstimator<float>::MEstimator_impl_simd(const std::vector<float> &residues,
                                                             std::vector<float> &weights,
                                                             float NoiseThreshold)
{
#if VISP_HAVE_SSSE3 || VISP_HAVE_NEON
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  float med = getMedian(m_residues);
  m_normres.resize(residues.size());

  size_t i = 0;
#if VISP_HAVE_SSSE3
  __m128 med_128 = _mm_set_ps1(med);
#else
  float32x4_t med_128 = vdupq_n_f32(med);
#endif

  if (m_residues.size() >= 4) {
    for (i = 0; i <= m_residues.size() - 4; i += 4) {
#if VISP_HAVE_SSSE3
      __m128 residues_128 = _mm_loadu_ps(residues.data() + i);
      _mm_storeu_ps(m_normres.data() + i, abs_ps(_mm_sub_ps(residues_128, med_128)));
#else
      float32x4_t residues_128 = vld1q_f32(residues.data() + i);
      vst1q_f32(m_normres.data() + i, vabsq_f32(vsubq_f32(residues_128, med_128)));
#endif
    }
  }

  for (; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i] - med));
  }

  m_residues = m_normres;
  float normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  float sigma = 1.4826f * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
#else
  (void)residues;
  (void)weights;
  (void)NoiseThreshold;
#endif
}

/*!
 * \relates vpMbtTukeyEstimator
 */
template <>
inline void vpMbtTukeyEstimator<double>::MEstimator_impl_simd(const std::vector<double> &residues,
                                                              std::vector<double> &weights,
                                                              double NoiseThreshold)
{
#if VISP_HAVE_SSSE3 || VISP_HAVE_NEON
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  double med = getMedian(m_residues);
  m_normres.resize(residues.size());

#if HAVE_TRANSFORM
// Check if std:c++14 or higher
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
  std::transform(residues.begin(), residues.end(), m_normres.begin(), std::bind(AbsDiff, std::placeholders::_1, med));
#else
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<double>(), std::placeholders::_1, med));
#endif
#else
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (std::fabs(residues[i] - med));
  }
#endif

  m_residues = m_normres;
  double normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  double sigma = 1.4826 * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
#else
  (void)residues;
  (void)weights;
  (void)NoiseThreshold;
#endif
}

/*!
 * \relates vpMbtTukeyEstimator
 */
template <>
inline void vpMbtTukeyEstimator<float>::MEstimator(const std::vector<float> &residues, std::vector<float> &weights,
                                                   float NoiseThreshold)
{
#if defined(VISP_HAVE_SIMDLIB)
  bool checkSimd = vpCPUFeatures::checkSSSE3() || vpCPUFeatures::checkNeon();
#else
  bool checkSimd = vpCPUFeatures::checkSSSE3();
#endif
#if !VISP_HAVE_SSSE3 && !VISP_HAVE_NEON
  checkSimd = false;
#endif

  if (checkSimd)
    MEstimator_impl_simd(residues, weights, NoiseThreshold);
  else
    MEstimator_impl(residues, weights, NoiseThreshold);
}

/*!
 * \relates vpMbtTukeyEstimator
 */
template <>
inline void vpMbtTukeyEstimator<double>::MEstimator(const std::vector<double> &residues, std::vector<double> &weights,
                                                    double NoiseThreshold)
{
#if defined(VISP_HAVE_SIMDLIB)
  bool checkSimd = vpCPUFeatures::checkSSSE3() || vpCPUFeatures::checkNeon();
#else
  bool checkSimd = vpCPUFeatures::checkSSSE3();
#endif
#if !VISP_HAVE_SSSE3 && !VISP_HAVE_NEON
  checkSimd = false;
#endif

  if (checkSimd)
    MEstimator_impl_simd(residues, weights, NoiseThreshold);
  else
    MEstimator_impl(residues, weights, NoiseThreshold);
}

/*!
 * Consider Tukey influence function.
 */
template <typename T> void vpMbtTukeyEstimator<T>::psiTukey(const T sig, std::vector<T> &x, vpColVector &weights)
{
  double C = sig * 4.6851;

  // Here we consider that sig cannot be equal to 0
  for (unsigned int i = 0; i < (unsigned int)x.size(); i++) {
    double xi = x[i] / C;
    xi *= xi;

    if (xi > 1.) {
      weights[i] = 0;
    }
    else {
      xi = 1 - xi;
      xi *= xi;
      weights[i] = xi;
    }
  }
}

/*!
 * \relates vpMbtTukeyEstimator
 */
template <>
inline void vpMbtTukeyEstimator<double>::MEstimator(const vpColVector &residues, vpColVector &weights,
                                                    double NoiseThreshold)
{
  if (residues.size() == 0) {
    return;
  }

  m_residues.resize(0);
  m_residues.reserve(residues.size());
  m_residues.insert(m_residues.end(), &residues.data[0], &residues.data[residues.size()]);

  double med = getMedian(m_residues);

  m_normres.resize(residues.size());
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = std::fabs(residues[(unsigned int)i] - med);
  }

  m_residues = m_normres;
  double normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  double sigma = 1.4826 * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}

/*!
 * \relates vpMbtTukeyEstimator
 */
template <>
inline void vpMbtTukeyEstimator<float>::MEstimator(const vpColVector &residues, vpColVector &weights,
                                                   double NoiseThreshold)
{
  if (residues.size() == 0) {
    return;
  }

  m_residues.resize(0);
  m_residues.reserve(residues.size());
  for (unsigned int i = 0; i < residues.size(); i++) {
    m_residues.push_back((float)residues[i]);
  }

  float med = getMedian(m_residues);

  m_normres.resize(residues.size());
  for (size_t i = 0; i < m_residues.size(); i++) {
    m_normres[i] = (float)std::fabs(residues[(unsigned int)i] - med);
  }

  m_residues = m_normres;
  float normmedian = getMedian(m_residues);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  float sigma = 1.4826f * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = (float)NoiseThreshold;
  }

  psiTukey(sigma, m_normres, weights);
}

/*!
 * Consider Tukey influence function.
 */
template <class T> void vpMbtTukeyEstimator<T>::psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights)
{
  T C = static_cast<T>(4.6851) * sig;
  weights.resize(x.size());

  // Here we consider that sig cannot be equal to 0
  for (size_t i = 0; i < x.size(); i++) {
    T xi = x[i] / C;
    xi *= xi;

    if (xi > 1.) {
      weights[i] = 0;
    }
    else {
      xi = 1 - xi;
      xi *= xi;
      weights[i] = xi;
    }
  }
}
END_VISP_NAMESPACE
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS

#endif
