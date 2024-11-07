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
 * Gaussian filter class
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#include <visp3/core/vpGaussianFilter.h>
#include <visp3/core/vpImageConvert.h>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpGaussianFilter::Impl
{
public:
  Impl(unsigned int width, unsigned int height, float sigma, bool deinterleave)
    : m_funcPtrGray(nullptr), m_funcPtrRGBa(nullptr), m_deinterleave(deinterleave)
  {
    const float epsilon = 0.001f;

    const size_t channels_1 = 1;
    m_funcPtrGray = SimdGaussianBlurInit(width, height, channels_1, &sigma, &epsilon);

    const size_t channels_4 = 4;
    m_funcPtrRGBa = SimdGaussianBlurInit(width, height, channels_4, &sigma, &epsilon);

    if (m_deinterleave) {
      m_red.resize(height, width);
      m_green.resize(height, width);
      m_blue.resize(height, width);

      m_redBlurred.resize(height, width);
      m_greenBlurred.resize(height, width);
      m_blueBlurred.resize(height, width);
    }
  }

  ~Impl()
  {
    if (m_funcPtrGray) {
      SimdRelease(m_funcPtrGray);
    }

    if (m_funcPtrRGBa) {
      SimdRelease(m_funcPtrRGBa);
    }
  }

  void apply(const vpImage<unsigned char> &I, vpImage<unsigned char> &I_blur)
  {
    I_blur.resize(I.getHeight(), I.getWidth());
    SimdGaussianBlurRun(m_funcPtrGray, I.bitmap, I.getWidth(), I_blur.bitmap, I_blur.getWidth());
  }

  void apply(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &I_blur)
  {
    I_blur.resize(I.getHeight(), I.getWidth());
    if (!m_deinterleave) {
      const unsigned int rgba_size = 4;
      SimdGaussianBlurRun(m_funcPtrRGBa, reinterpret_cast<unsigned char *>(I.bitmap), I.getWidth() * rgba_size,
                          reinterpret_cast<unsigned char *>(I_blur.bitmap), I_blur.getWidth() * rgba_size);
    }
    else {
      vpImageConvert::split(I, &m_red, &m_green, &m_blue);
      SimdGaussianBlurRun(m_funcPtrGray, m_red.bitmap, m_red.getWidth(), m_redBlurred.bitmap, m_redBlurred.getWidth());
      SimdGaussianBlurRun(m_funcPtrGray, m_green.bitmap, m_green.getWidth(), m_greenBlurred.bitmap,
                          m_greenBlurred.getWidth());
      SimdGaussianBlurRun(m_funcPtrGray, m_blue.bitmap, m_blue.getWidth(), m_blueBlurred.bitmap,
                          m_blueBlurred.getWidth());

      vpImageConvert::merge(&m_redBlurred, &m_greenBlurred, &m_blueBlurred, nullptr, I_blur);
    }
  }

protected:
  void *m_funcPtrGray;
  void *m_funcPtrRGBa;
  bool m_deinterleave;
  vpImage<unsigned char> m_red;
  vpImage<unsigned char> m_green;
  vpImage<unsigned char> m_blue;
  vpImage<unsigned char> m_redBlurred;
  vpImage<unsigned char> m_greenBlurred;
  vpImage<unsigned char> m_blueBlurred;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

/*!
  Gaussian filter constructor.

  \param[in] width : image width.
  \param[in] height : image height.
  \param[in] sigma : Standard deviation for Gaussian kernel.
  \param[in] deinterleave : if true, deinterleave R, G, B channels and perform Gaussian filter on each individual
  channel. It can be faster to deinterleave when repeatedly calling Gaussian filter with the same sigma and the same
  image resolution.
*/
vpGaussianFilter::vpGaussianFilter(unsigned int width, unsigned int height, float sigma, bool deinterleave)
  : m_impl(new Impl(width, height, sigma, deinterleave))
{ }

vpGaussianFilter::~vpGaussianFilter() { delete m_impl; }

/*!
  Apply Gaussian filter on grayscale image.

  \param[in] I : input grayscale image.
  \param[out] I_blur : output blurred grayscale image.
*/
void vpGaussianFilter::apply(const vpImage<unsigned char> &I, vpImage<unsigned char> &I_blur)
{
  m_impl->apply(I, I_blur);
}

/*!
  Apply Gaussian filter on color image.

  \param[in] I : input color image.
  \param[out] I_blur : output blurred color image.
*/
void vpGaussianFilter::apply(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &I_blur) { m_impl->apply(I, I_blur); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
 // Work around to avoid warning: libvisp_core.a(vpGaussianFilter.cpp.o) has no symbols
void dummy_vpGaussianFilter() { };

#endif
