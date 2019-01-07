/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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

#include <algorithm>
#include <cmath>
#include <iostream>

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbtTukeyEstimator.h>

#define USE_TRANSFORM 1
#if defined(VISP_HAVE_CPP11_COMPATIBILITY) && USE_TRANSFORM
#define HAVE_TRANSFORM 1
#endif

#if HAVE_TRANSFORM
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

#define USE_ORIGINAL_TUKEY_CODE 1

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if HAVE_TRANSFORM
namespace
{
template <typename T> struct AbsDiff : public std::binary_function<T, T, T> {
  double operator()(const T a, const T b) const { return std::fabs(a - b); }
};
}
#endif

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
}
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
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<T>(), std::placeholders::_1, med));
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
void vpMbtTukeyEstimator<float>::MEstimator_impl_ssse3(const std::vector<float> &residues, std::vector<float> &weights,
                                                       const float NoiseThreshold)
{
#if VISP_HAVE_SSSE3
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  float med = getMedian(m_residues);
  m_normres.resize(residues.size());

  size_t i = 0;
  __m128 med_128 = _mm_set_ps1(med);

  if (m_residues.size() >= 4) {
    for (i = 0; i <= m_residues.size() - 4; i += 4) {
      __m128 residues_128 = _mm_loadu_ps(residues.data() + i);
      _mm_storeu_ps(m_normres.data() + i, abs_ps(_mm_sub_ps(residues_128, med_128)));
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

template <>
void vpMbtTukeyEstimator<double>::MEstimator_impl_ssse3(const std::vector<double> &residues,
                                                        std::vector<double> &weights, const double NoiseThreshold)
{
#if VISP_HAVE_SSSE3
  if (residues.empty()) {
    return;
  }

  m_residues = residues;

  double med = getMedian(m_residues);
  m_normres.resize(residues.size());

#if HAVE_TRANSFORM
  std::transform(residues.begin(), residues.end(), m_normres.begin(),
                 std::bind(AbsDiff<double>(), std::placeholders::_1, med));
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

template <>
void vpMbtTukeyEstimator<float>::MEstimator(const std::vector<float> &residues, std::vector<float> &weights,
                                            const float NoiseThreshold)
{
  bool checkSSSE3 = vpCPUFeatures::checkSSSE3();
#if !VISP_HAVE_SSSE3
  checkSSSE3 = false;
#endif

  if (checkSSSE3)
    MEstimator_impl_ssse3(residues, weights, NoiseThreshold);
  else
    MEstimator_impl(residues, weights, NoiseThreshold);
}

template <>
void vpMbtTukeyEstimator<double>::MEstimator(const std::vector<double> &residues, std::vector<double> &weights,
                                             const double NoiseThreshold)
{
  bool checkSSSE3 = vpCPUFeatures::checkSSSE3();
#if !VISP_HAVE_SSSE3
  checkSSSE3 = false;
#endif

  if (checkSSSE3)
    MEstimator_impl_ssse3(residues, weights, NoiseThreshold);
  else
    MEstimator_impl(residues, weights, NoiseThreshold);
}

template <typename T> void vpMbtTukeyEstimator<T>::psiTukey(const T sig, std::vector<T> &x, vpColVector &weights)
{
  T cst_const = static_cast<T>(4.6851);
  T inv_cst_const = 1 / cst_const;
  T inv_sig = 1 / sig;

  for (unsigned int i = 0; i < (unsigned int)x.size(); i++) {
#if USE_ORIGINAL_TUKEY_CODE
    if (std::fabs(sig) <= std::numeric_limits<T>::epsilon() &&
        std::fabs(weights[i]) > std::numeric_limits<double>::epsilon() // sig should be equal to 0
                                                                       // only if NoiseThreshold
                                                                       // ==
                                                                       // 0
    ) {
      weights[i] = 1;
      continue;
    }
#endif

    double xi_sig = x[(size_t)i] * inv_sig;

    if ((std::fabs(xi_sig) <= cst_const)
#if USE_ORIGINAL_TUKEY_CODE
        && std::fabs(weights[i]) > std::numeric_limits<double>::epsilon() // Consider the previous
                                                                          // weights here
#endif
    ) {
      weights[i] = (1 - (xi_sig * inv_cst_const) * (xi_sig * inv_cst_const)) *
                   (1 - (xi_sig * inv_cst_const) * (xi_sig * inv_cst_const)); // vpMath::sqr( 1 -
                                                                              // vpMath::sqr(xi_sig *
                                                                              // inv_cst_const) );
    } else {
      // Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}

template <>
void vpMbtTukeyEstimator<double>::MEstimator(const vpColVector &residues, vpColVector &weights,
                                             const double NoiseThreshold)
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

template <>
void vpMbtTukeyEstimator<float>::MEstimator(const vpColVector &residues, vpColVector &weights,
                                            const double NoiseThreshold)
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

template <class T> void vpMbtTukeyEstimator<T>::psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights)
{
  T cst_const = static_cast<T>(4.6851);
  T inv_cst_const = 1 / cst_const;
  T inv_sig = 1 / sig;

  for (size_t i = 0; i < x.size(); i++) {
#if USE_ORIGINAL_TUKEY_CODE
    if (std::fabs(sig) <= std::numeric_limits<T>::epsilon() &&
        std::fabs(weights[i]) > std::numeric_limits<T>::epsilon() // sig should be equal to 0 only
                                                                  // if NoiseThreshold == 0
    ) {
      weights[i] = 1;
      continue;
    }
#endif

    T xi_sig = x[i] * inv_sig;

    if ((std::fabs(xi_sig) <= cst_const)
#if USE_ORIGINAL_TUKEY_CODE
        && std::fabs(weights[i]) > std::numeric_limits<T>::epsilon() // Consider the previous
                                                                     // weights here
#endif
    ) {
      // vpMath::sqr( 1 - vpMath::sqr(xi_sig * inv_cst_const) );
      weights[i] = (1 - (xi_sig * inv_cst_const) * (xi_sig * inv_cst_const)) *
                   (1 - (xi_sig * inv_cst_const) * (xi_sig * inv_cst_const));
    } else {
      // Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS
