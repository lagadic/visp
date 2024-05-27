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
 * Bayer conversion tools.
 *
*****************************************************************************/

/*!
  \file vpBayerConversion.h
  \brief Bayer conversion tools
*/

#ifndef vpBAYERCONVERSION_H
#define vpBAYERCONVERSION_H

#include <visp3/core/vpConfig.h>

#ifndef VISP_SKIP_BAYER_CONVERSION
#include <cassert>


#include <visp3/core/vpMath.h>

// Workaround to avoid warning: "left operand of comma operator has no effect" when compiled in g++ with
// "-Wunused-value"
#define m_assert(msg, expr) assert(((void)(msg), (expr)))

// Bilinear
template <typename T> T demosaicPhiBilinear(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return static_cast<T>(0.5f * bayer[(i - 1) * width + j] + 0.5f * bayer[(i + 1) * width + j]);
}

template <typename T> T demosaicThetaBilinear(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return static_cast<T>(0.5f * bayer[i * width + j - 1] + 0.5f * bayer[i * width + j + 1]);
}

template <typename T> T demosaicCheckerBilinear(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return static_cast<T>(0.25f * bayer[(i - 1) * width + j - 1] + 0.25f * bayer[(i - 1) * width + j + 1] +
                        0.25f * bayer[(i + 1) * width + j - 1] + 0.25f * bayer[(i + 1) * width + j + 1]);
}

template <typename T> T demosaicCrossBilinear(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return static_cast<T>(0.25f * bayer[(i - 1) * width + j] + 0.25f * bayer[i * width + j - 1] +
                        0.25f * bayer[i * width + j + 1] + 0.25f * bayer[(i + 1) * width + j]);
}

// Malvar
template <typename T> T demosaicPhiMalvar(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return VISP_NAMESPACE_ADDRESSING vpMath::saturate<T>(
      (-bayer[(i - 2) * width + j] - bayer[(i - 1) * width + j - 1] + 4 * bayer[(i - 1) * width + j] -
       bayer[(i - 1) * width + j + 1] + 0.5f * bayer[i * width + j - 2] + 5 * bayer[i * width + j] +
       0.5f * bayer[i * width + j + 2] - bayer[(i + 1) * width + j - 1] + 4 * bayer[(i + 1) * width + j] -
       bayer[(i + 1) * width + j + 1] - bayer[(i + 2) * width + j]) *
      0.125f);
}

template <typename T> T demosaicThetaMalvar(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return VISP_NAMESPACE_ADDRESSING vpMath::saturate<T>((0.5f * bayer[(i - 2) * width + j] - bayer[(i - 1) * width + j - 1] -
                                                        bayer[(i - 1) * width + j + 1] - bayer[i * width + j - 2] + 4 * bayer[i * width + j - 1] +
                                                        5 * bayer[i * width + j] + 4 * bayer[i * width + j + 1] - bayer[i * width + j + 2] -
                                                        bayer[(i + 1) * width + j - 1] - bayer[(i + 1) * width + j + 1] +
                                                        0.5f * bayer[(i + 2) * width + j]) *
                             0.125f);
}

template <typename T> T demosaicCheckerMalvar(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return VISP_NAMESPACE_ADDRESSING vpMath::saturate<T>(
      (-1.5f * bayer[(i - 2) * width + j] + 2 * bayer[(i - 1) * width + j - 1] + 2 * bayer[(i - 1) * width + j + 1] -
       1.5f * bayer[i * width + j - 2] + 6 * bayer[i * width + j] - 1.5f * bayer[i * width + j + 2] +
       2 * bayer[(i + 1) * width + j - 1] + 2 * bayer[(i + 1) * width + j + 1] - 1.5f * bayer[(i + 2) * width + j]) *
      0.125f);
}

template <typename T> T demosaicCrossMalvar(const T *bayer, unsigned int width, unsigned int i, unsigned int j)
{
  return VISP_NAMESPACE_ADDRESSING vpMath::saturate<T>((-bayer[(i - 2) * width + j] + 2 * bayer[(i - 1) * width + j] - bayer[i * width + j - 2] +
                                                        2 * bayer[i * width + j - 1] + 4 * bayer[i * width + j] + 2 * bayer[i * width + j + 1] -
                                                        bayer[i * width + j + 2] + 2 * bayer[(i + 1) * width + j] - bayer[(i + 2) * width + j]) *
                             0.125f);
}

template <typename T>
void demosaicBGGRToRGBaBilinearTpl(const T *bggr, T *rgba, unsigned int width, unsigned int height,
                                   unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = bggr[width + 1];
  rgba[1] = bggr[1];
  rgba[2] = bggr[0];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = bggr[2 * width - 1];
  rgba[(width - 1) * 4 + 1] = bggr[width - 1];
  rgba[(width - 1) * 4 + 2] = bggr[width - 2];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = bggr[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 1] = bggr[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 2] = bggr[(height - 2) * width];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = bggr[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = bggr[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = bggr[(height - 1) * width - 2];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * bggr[width + j - 1] + 0.5f * bggr[width + j + 1]);
      rgba[j * 4 + 1] = static_cast<T>(0.5f * bggr[j - 1] + 0.5f * bggr[j + 1]);
      rgba[j * 4 + 2] = bggr[j];
    }
    else {
      rgba[j * 4 + 0] = bggr[width + j];
      rgba[j * 4 + 1] = bggr[j];
      rgba[j * 4 + 2] = static_cast<T>(0.5f * bggr[j - 1] + 0.5f * bggr[j + 1]);
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * bggr[(i - 1) * width + 1] + 0.5f * bggr[(i + 1) * width + 1]);
      rgba[i * width * 4 + 1] = bggr[i * width + 1];
      rgba[i * width * 4 + 2] = bggr[i * width];
    }
    else {
      rgba[i * width * 4 + 0] = bggr[i * width + 1];
      rgba[i * width * 4 + 1] = bggr[i * width];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * bggr[(i - 1) * width] + 0.5f * bggr[(i + 1) * width]);
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * bggr[i * width - 1] + 0.5f * bggr[(i + 2) * width - 1]);
      rgba[(i * width + width - 1) * 4 + 1] = bggr[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] = bggr[(i + 1) * width - 2];
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] = bggr[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 1] = bggr[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * bggr[i * width - 2] + 0.5f * bggr[(i + 2) * width - 2]);
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * bggr[(height - 1) * width + j - 1] + 0.5f * bggr[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] = bggr[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] = bggr[(height - 2) * width + j];
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] = bggr[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * bggr[(height - 1) * width + j - 1] + 0.5f * bggr[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * bggr[(height - 2) * width + j - 1] + 0.5f * bggr[(height - 2) * width + j + 1]);
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 1; i < height - 1; ++i) {
    for (unsigned int j = 1; j < width - 1; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerBilinear(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 2] = bggr[i * width + j];
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiBilinear(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaBilinear(bggr, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaBilinear(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiBilinear(bggr, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerBilinear(bggr, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicGBRGToRGBaBilinearTpl(const T *gbrg, T *rgba, unsigned int width, unsigned int height,
                                   unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = gbrg[width];
  rgba[1] = gbrg[0];
  rgba[2] = gbrg[1];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = gbrg[2 * width - 2];
  rgba[(width - 1) * 4 + 1] = gbrg[width - 2];
  rgba[(width - 1) * 4 + 2] = gbrg[width - 1];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = gbrg[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 1] = gbrg[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 2] = gbrg[(height - 2) * width + 1];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = gbrg[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = gbrg[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = gbrg[(height - 1) * width - 1];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = gbrg[width + j];
      rgba[j * 4 + 1] = gbrg[j];
      rgba[j * 4 + 2] = static_cast<T>(0.5f * gbrg[j - 1] + 0.5f * gbrg[j + 1]);
    }
    else {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * gbrg[width + j - 1] + 0.5f * gbrg[width + j + 1]);
      rgba[j * 4 + 1] = static_cast<T>(0.5f * gbrg[j - 1] + 0.5f * gbrg[j + 1]);
      rgba[j * 4 + 2] = gbrg[j];
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * gbrg[(i - 1) * width] + 0.5f * gbrg[(i + 1) * width]);
      rgba[i * width * 4 + 1] = gbrg[i * width];
      rgba[i * width * 4 + 2] = gbrg[i * width + 1];
    }
    else {
      rgba[i * width * 4 + 0] = gbrg[i * width];
      rgba[i * width * 4 + 1] = static_cast<T>(0.5f * gbrg[(i - 1) * width] + 0.5f * gbrg[(i + 1) * width]);
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * gbrg[(i - 1) * width + 1] + 0.5f * gbrg[(i + 1) * width + 1]);
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[i * width - 2] + 0.5f * gbrg[(i + 2) * width - 2]);
      rgba[(i * width + width - 1) * 4 + 1] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] = gbrg[(i + 1) * width - 1];
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 1] = gbrg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[i * width - 1] + 0.5f * gbrg[(i + 2) * width - 1]);
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] = gbrg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * gbrg[(height - 1) * width + j - 1] + 0.5f * gbrg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[(height - 2) * width + j - 1] + 0.5f * gbrg[(height - 2) * width + j + 1]);
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[(height - 1) * width + j - 1] + 0.5f * gbrg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] = gbrg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] = gbrg[(height - 2) * width + j];
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 1; i < height - 1; ++i) {
    for (unsigned int j = 1; j < width - 1; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiBilinear(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaBilinear(gbrg, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerBilinear(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = gbrg[i * width + j];
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerBilinear(gbrg, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaBilinear(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiBilinear(gbrg, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicGRBGToRGBaBilinearTpl(const T *grbg, T *rgba, unsigned int width, unsigned int height,
                                   unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = grbg[1];
  rgba[1] = grbg[0];
  rgba[2] = grbg[width];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = grbg[width - 1];
  rgba[(width - 1) * 4 + 1] = grbg[width - 2];
  rgba[(width - 1) * 4 + 2] = grbg[2 * width - 2];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = grbg[(height - 2) * width + 1];
  rgba[((height - 1) * width) * 4 + 1] = grbg[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 2] = grbg[(height - 1) * width];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = grbg[(height - 1) * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = grbg[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = grbg[height * width - 2];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * grbg[j - 1] + 0.5f * grbg[j + 1]);
      rgba[j * 4 + 1] = grbg[j];
      rgba[j * 4 + 2] = grbg[width + j];
    }
    else {
      rgba[j * 4 + 0] = grbg[j];
      rgba[j * 4 + 1] = static_cast<T>(0.5f * grbg[j - 1] + 0.5f * grbg[j + 1]);
      rgba[j * 4 + 2] = static_cast<T>(0.5f * grbg[width + j - 1] + 0.5f * grbg[width + j + 1]);
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = grbg[i * width + 1];
      rgba[i * width * 4 + 1] = grbg[i * width];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * grbg[(i - 1) * width] + 0.5f * grbg[(i + 1) * width]);
    }
    else {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * grbg[(i - 1) * width + 1] + 0.5f * grbg[(i + 1) * width + 1]);
      rgba[i * width * 4 + 1] = grbg[i * width + 1];
      rgba[i * width * 4 + 2] = grbg[i * width];
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] = grbg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 1] = grbg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * grbg[i * width - 2] + 0.5f * grbg[(i + 2) * width - 2]);
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * grbg[i * width - 1] + 0.5f * grbg[(i + 2) * width - 1]);
      rgba[(i * width + width - 1) * 4 + 1] = grbg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] = grbg[(i + 1) * width - 2];
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * grbg[(height - 2) * width + j - 1] + 0.5f * grbg[(height - 2) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * grbg[(height - 1) * width + j - 1] + 0.5f * grbg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] = grbg[(height - 1) * width + j];
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] = grbg[(height - 2) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] = grbg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * grbg[(height - 1) * width + j - 1] + 0.5f * grbg[(height - 1) * width + j + 1]);
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 1; i < height - 1; ++i) {
    for (unsigned int j = 1; j < width - 1; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaBilinear(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiBilinear(grbg, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerBilinear(grbg, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerBilinear(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = grbg[i * width + j];
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiBilinear(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaBilinear(grbg, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicRGGBToRGBaBilinearTpl(const T *rggb, T *rgba, unsigned int width, unsigned int height,
                                   unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = rggb[0];
  rgba[1] = rggb[1];
  rgba[2] = rggb[width + 1];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = rggb[width - 2];
  rgba[(width - 1) * 4 + 1] = rggb[width - 1];
  rgba[(width - 1) * 4 + 2] = rggb[2 * width - 1];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = rggb[(height - 2) * width];
  rgba[((height - 1) * width) * 4 + 1] = rggb[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 2] = rggb[(height - 1) * width + 1];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = rggb[(height - 1) * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = rggb[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = rggb[height * width - 1];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = rggb[j];
      rgba[j * 4 + 1] = static_cast<T>(0.5f * rggb[j - 1] + 0.5f * rggb[j + 1]);
      rgba[j * 4 + 2] = static_cast<T>(0.5f * rggb[width + j - 1] + 0.5f * rggb[width + j + 1]);
    }
    else {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * rggb[j - 1] + 0.5f * rggb[j + 1]);
      rgba[j * 4 + 1] = rggb[j];
      rgba[j * 4 + 2] = rggb[width + j];
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = rggb[i * width];
      rgba[i * width * 4 + 1] = rggb[i * width + 1];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * rggb[(i - 1) * width + 1] + 0.5f * rggb[(i + 1) * width + 1]);
    }
    else {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * rggb[(i - 1) * width] + 0.5f * rggb[(i + 1) * width]);
      rgba[i * width * 4 + 1] = rggb[i * width];
      rgba[i * width * 4 + 2] = rggb[i * width + 1];
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 1] = rggb[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * rggb[i * width - 1] + 0.5f * rggb[(i + 2) * width - 1]);
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * rggb[i * width - 2] + 0.5f * rggb[(i + 2) * width - 2]);
      rgba[(i * width + width - 1) * 4 + 1] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] = rggb[(i + 1) * width - 1];
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] = rggb[(height - 2) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] = rggb[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * rggb[(height - 1) * width + j - 1] + 0.5f * rggb[(height - 1) * width + j + 1]);
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * rggb[(height - 2) * width + j - 1] + 0.5f * rggb[(height - 2) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * rggb[(height - 1) * width + j - 1] + 0.5f * rggb[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] = rggb[(height - 1) * width + j];
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 1; i < height - 1; ++i) {
    for (unsigned int j = 1; j < width - 1; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerBilinear(rggb, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaBilinear(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiBilinear(rggb, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiBilinear(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaBilinear(rggb, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerBilinear(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossBilinear(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 2] = rggb[i * width + j];
      }
    }
  }
}

// Malvar

template <typename T>
void demosaicBGGRToRGBaMalvarTpl(const T *bggr, T *rgba, unsigned int width, unsigned int height, unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = bggr[width + 1];
  rgba[1] = bggr[1];
  rgba[2] = bggr[0];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = bggr[2 * width - 1];
  rgba[(width - 1) * 4 + 1] = bggr[width - 1];
  rgba[(width - 1) * 4 + 2] = bggr[width - 2];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = bggr[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 1] = bggr[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 2] = bggr[(height - 2) * width];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = bggr[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = bggr[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = bggr[(height - 1) * width - 2];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * bggr[width + j - 1] + 0.5f * bggr[width + j + 1]);
      rgba[j * 4 + 1] = static_cast<T>(0.5f * bggr[j - 1] + 0.5f * bggr[j + 1]);
      rgba[j * 4 + 2] = bggr[j];
    }
    else {
      rgba[j * 4 + 0] = bggr[width + j];
      rgba[j * 4 + 1] = bggr[j];
      rgba[j * 4 + 2] = static_cast<T>(0.5f * bggr[j - 1] + 0.5f * bggr[j + 1]);
    }
  }

  // i == 1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.5f * bggr[width + j - 1] + 0.5f * bggr[width + j + 1]);
      rgba[(width + j) * 4 + 1] = bggr[width + j];
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.5f * bggr[j] + 0.5f * bggr[2 * width + j]);
    }
    else {
      rgba[(width + j) * 4 + 0] = bggr[width + j];
      rgba[(width + j) * 4 + 1] = static_cast<T>(0.25f * bggr[j] + 0.25f * bggr[width + j - 1] +
                                                 0.25f * bggr[width + j + 1] + 0.25f * bggr[2 * width + j]);
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.25f * bggr[j - 1] + 0.25f * bggr[j + 1] +
                                                 0.25f * bggr[2 * width + j - 1] + 0.25f * bggr[2 * width + j + 1]);
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * bggr[(i - 1) * width + 1] + 0.5f * bggr[(i + 1) * width + 1]);
      rgba[i * width * 4 + 1] = bggr[i * width + 1];
      rgba[i * width * 4 + 2] = bggr[i * width];
    }
    else {
      rgba[i * width * 4 + 0] = bggr[i * width + 1];
      rgba[i * width * 4 + 1] = bggr[i * width];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * bggr[(i - 1) * width] + 0.5f * bggr[(i + 1) * width]);
    }
  }

  // j == 1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + 1) * 4 + 0] =
        static_cast<T>(0.5f * bggr[(i - 1) * width + 1] + 0.5f * bggr[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 1] = bggr[i * width + 1];
      rgba[(i * width + 1) * 4 + 2] = static_cast<T>(0.5f * bggr[i * width] + 0.5f * bggr[i * width + 2]);
    }
    else {
      rgba[(i * width + 1) * 4 + 0] = bggr[i * width + 1];
      rgba[(i * width + 1) * 4 + 1] = static_cast<T>(0.25f * bggr[(i - 1) * width + 1] + 0.25f * bggr[i * width] +
                                                     0.25f * bggr[i * width + 2] + 0.25f * bggr[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 2] = static_cast<T>(0.25f * bggr[(i - 1) * width] + 0.25f * bggr[(i - 1) * width + 2] +
                                                     0.25f * bggr[(i + 1) * width] + 0.25f * bggr[(i + 1) * width + 2]);
    }
  }

  // j == width-2
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.25f * bggr[i * width - 3] + 0.25f * bggr[i * width - 1] + 0.25f * bggr[(i + 2) * width - 3] +
                       0.25f * bggr[(i + 2) * width - 1]);
      rgba[(i * width + width - 2) * 4 + 1] =
        static_cast<T>(0.25f * bggr[i * width - 2] + 0.25f * bggr[(i + 1) * width - 3] +
                       0.25f * bggr[(i + 1) * width - 1] + 0.25f * bggr[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 2] = bggr[(i + 1) * width - 2];
    }
    else {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.5f * bggr[(i + 1) * width - 3] + 0.5f * bggr[(i + 1) * width - 1]);
      rgba[(i * width + width - 2) * 4 + 1] = bggr[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.5f * bggr[i * width - 2] + 0.5f * bggr[(i + 2) * width - 2]);
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * bggr[i * width - 1] + 0.5f * bggr[(i + 2) * width - 1]);
      rgba[(i * width + width - 1) * 4 + 1] = bggr[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] = bggr[(i + 1) * width - 2];
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] = bggr[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 1] = bggr[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * bggr[i * width - 2] + 0.5f * bggr[(i + 2) * width - 2]);
    }
  }

  // i == height-2
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.25f * bggr[(height - 3) * width + j - 1] + 0.25f * bggr[(height - 3) * width + j + 1] +
                       0.25f * bggr[(height - 1) * width + j - 1] + 0.25f * bggr[(height - 1) * width + j + 1]);
      rgba[((height - 2) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * bggr[(height - 2) * width + j - 1] + 0.5f * bggr[(height - 2) * width + j + 1]);
      rgba[((height - 2) * width + j) * 4 + 2] = bggr[(height - 2) * width + j];
    }
    else {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * bggr[(height - 3) * width + j] + 0.5f * bggr[(height - 1) * width + j]);
      rgba[((height - 2) * width + j) * 4 + 1] = bggr[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * bggr[(height - 2) * width + j - 1] + 0.5f * bggr[(height - 2) * width + j + 1]);
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * bggr[(height - 1) * width + j - 1] + 0.5f * bggr[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] = bggr[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] = bggr[(height - 2) * width + j];
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] = bggr[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * bggr[(height - 1) * width + j - 1] + 0.5f * bggr[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * bggr[(height - 2) * width + j - 1] + 0.5f * bggr[(height - 2) * width + j + 1]);
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 2; i < height - 2; ++i) {
    for (unsigned int j = 2; j < width - 2; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerMalvar(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 2] = bggr[i * width + j];
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiMalvar(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaMalvar(bggr, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaMalvar(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 1] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiMalvar(bggr, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = bggr[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(bggr, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerMalvar(bggr, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicGBRGToRGBaMalvarTpl(const T *gbrg, T *rgba, unsigned int width, unsigned int height, unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = gbrg[width];
  rgba[1] = gbrg[0];
  rgba[2] = gbrg[1];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = gbrg[2 * width - 2];
  rgba[(width - 1) * 4 + 1] = gbrg[width - 2];
  rgba[(width - 1) * 4 + 2] = gbrg[width - 1];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = gbrg[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 1] = gbrg[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 2] = gbrg[(height - 2) * width + 1];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = gbrg[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = gbrg[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = gbrg[(height - 1) * width - 1];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = gbrg[width + j];
      rgba[j * 4 + 1] = gbrg[j];
      rgba[j * 4 + 2] = static_cast<T>(0.5f * gbrg[j - 1] + 0.5f * gbrg[j + 1]);
    }
    else {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * gbrg[width + j - 1] + 0.5f * gbrg[width + j + 1]);
      rgba[j * 4 + 1] = static_cast<T>(0.5f * gbrg[j - 1] + 0.5f * gbrg[j + 1]);
      rgba[j * 4 + 2] = gbrg[j];
    }
  }

  // i == 1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[(width + j) * 4 + 0] = gbrg[width + j];
      rgba[(width + j) * 4 + 1] = static_cast<T>(0.25f * gbrg[j] + 0.25f * gbrg[width + j - 1] +
                                                 0.25f * gbrg[width + j + 1] + 0.25f * gbrg[2 * width + j]);
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.25f * gbrg[j - 1] + 0.25f * gbrg[j + 1] +
                                                 0.25f * gbrg[2 * width + j - 1] + 0.25f * gbrg[2 * width + j + 1]);
    }
    else {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.5f * gbrg[width + j - 1] + 0.5f * gbrg[width + j + 1]);
      rgba[(width + j) * 4 + 1] = gbrg[width + j];
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.5f * gbrg[j] + 0.5f * gbrg[2 * width + j]);
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * gbrg[(i - 1) * width] + 0.5f * gbrg[(i + 1) * width]);
      rgba[i * width * 4 + 1] = gbrg[i * width];
      rgba[i * width * 4 + 2] = gbrg[i * width + 1];
    }
    else {
      rgba[i * width * 4 + 0] = gbrg[i * width];
      rgba[i * width * 4 + 1] = static_cast<T>(0.5f * gbrg[(i - 1) * width] + 0.5f * gbrg[(i + 1) * width]);
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * gbrg[(i - 1) * width + 1] + 0.5f * gbrg[(i + 1) * width + 1]);
    }
  }

  // j == 1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + 1) * 4 + 0] = static_cast<T>(0.25f * gbrg[(i - 1) * width] + 0.25f * gbrg[(i - 1) * width + 2] +
                                                     0.25f * gbrg[(i + 1) * width] + 0.5f * gbrg[(i + 1) * width + 2]);
      rgba[(i * width + 1) * 4 + 1] = static_cast<T>(0.25f * gbrg[(i - 1) * width + 1] + 0.25f * gbrg[i * width] +
                                                     0.25f * gbrg[i * width + 2] + 0.5f * gbrg[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 2] = gbrg[i * width + 1];
    }
    else {
      rgba[(i * width + 1) * 4 + 0] = static_cast<T>(0.5f * gbrg[i * width] + 0.5f * gbrg[i * width + 2]);
      rgba[(i * width + 1) * 4 + 1] = gbrg[i * width + 1];
      rgba[(i * width + 1) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[(i - 1) * width + 1] + 0.5f * gbrg[(i + 1) * width + 1]);
    }
  }

  // j == width-2
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[i * width - 2] + 0.5f * gbrg[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 1] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[(i + 1) * width - 3] + 0.5f * gbrg[(i + 1) * width - 1]);
    }
    else {
      rgba[(i * width + width - 2) * 4 + 0] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 1] =
        static_cast<T>(0.25f * gbrg[i * width - 2] + 0.25f * gbrg[(i + 1) * width - 3] +
                       0.25f * gbrg[(i + 1) * width - 1] + 0.25f * gbrg[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.25f * gbrg[i * width - 3] + 0.25f * gbrg[i * width - 1] + 0.25f * gbrg[(i + 2) * width - 3] +
                       0.25f * gbrg[(i + 2) * width - 1]);
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[i * width - 2] + 0.5f * gbrg[(i + 2) * width - 2]);
      rgba[(i * width + width - 1) * 4 + 1] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] = gbrg[(i + 1) * width - 1];
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] = gbrg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 1] = gbrg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[i * width - 1] + 0.5f * gbrg[(i + 2) * width - 1]);
    }
  }

  // i == height-2
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[(height - 3) * width + j] + 0.5f * gbrg[(height - 1) * width + j]);
      rgba[((height - 2) * width + j) * 4 + 1] = gbrg[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[(height - 2) * width + j - 1] + 0.5f * gbrg[(height - 2) * width + j + 1]);
    }
    else {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.25f * gbrg[(height - 3) * width + j - 1] + 0.25f * gbrg[(height - 3) * width + j + 1] +
                       0.25f * gbrg[(height - 1) * width + j - 1] + 0.25f * gbrg[(height - 1) * width + j + 1]);
      rgba[((height - 2) * width + j) * 4 + 1] =
        static_cast<T>(0.25f * gbrg[(height - 3) * width + j] + 0.25f * gbrg[(height - 2) * width + j - 1] +
                       0.25f * gbrg[(height - 2) * width + j + 1] + 0.25f * gbrg[(height - 1) * width + j]);
      rgba[((height - 2) * width + j) * 4 + 2] = gbrg[(height - 2) * width + j];
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] = gbrg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * gbrg[(height - 1) * width + j - 1] + 0.5f * gbrg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * gbrg[(height - 2) * width + j - 1] + 0.5f * gbrg[(height - 2) * width + j + 1]);
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * gbrg[(height - 1) * width + j - 1] + 0.5f * gbrg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] = gbrg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] = gbrg[(height - 2) * width + j];
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 2; i < height - 2; ++i) {
    for (unsigned int j = 2; j < width - 2; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiMalvar(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaMalvar(gbrg, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerMalvar(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = gbrg[i * width + j];
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerMalvar(gbrg, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaMalvar(gbrg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = gbrg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiMalvar(gbrg, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicGRBGToRGBaMalvarTpl(const T *grbg, T *rgba, unsigned int width, unsigned int height, unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = grbg[1];
  rgba[1] = grbg[0];
  rgba[2] = grbg[width];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = grbg[width - 1];
  rgba[(width - 1) * 4 + 1] = grbg[width - 2];
  rgba[(width - 1) * 4 + 2] = grbg[2 * width - 2];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = grbg[(height - 2) * width + 1];
  rgba[((height - 1) * width) * 4 + 1] = grbg[(height - 1) * width + 1];
  rgba[((height - 1) * width) * 4 + 2] = grbg[(height - 1) * width];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = grbg[(height - 1) * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = grbg[height * width - 1];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = grbg[height * width - 2];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * grbg[j - 1] + 0.5f * grbg[j + 1]);
      rgba[j * 4 + 1] = grbg[j];
      rgba[j * 4 + 2] = grbg[width + j];
    }
    else {
      rgba[j * 4 + 0] = grbg[j];
      rgba[j * 4 + 1] = static_cast<T>(0.5f * grbg[j - 1] + 0.5f * grbg[j + 1]);
      rgba[j * 4 + 2] = static_cast<T>(0.5f * grbg[width + j - 1] + 0.5f * grbg[width + j + 1]);
    }
  }

  // i == 1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.25f * grbg[j - 1] + 0.25f * grbg[j + 1] +
                                                 0.25f * grbg[2 * width + j - 1] + 0.25f * grbg[2 * width + j + 1]);
      rgba[(width + j) * 4 + 1] = static_cast<T>(0.25f * grbg[j] + 0.25f * grbg[width + j - 1] +
                                                 0.25f * grbg[width + j + 1] + 0.25f * grbg[2 * width + j]);
      rgba[(width + j) * 4 + 2] = grbg[width + j];
    }
    else {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.5f * grbg[j] + 0.5f * grbg[2 * width + j]);
      rgba[(width + j) * 4 + 1] = grbg[width + j];
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.5f * grbg[width + j - 1] + 0.5f * grbg[width + j + 1]);
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = grbg[i * width + 1];
      rgba[i * width * 4 + 1] = grbg[i * width];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * grbg[(i - 1) * width] + 0.5f * grbg[(i + 1) * width]);
    }
    else {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * grbg[(i - 1) * width + 1] + 0.5f * grbg[(i + 1) * width + 1]);
      rgba[i * width * 4 + 1] = grbg[i * width + 1];
      rgba[i * width * 4 + 2] = grbg[i * width];
    }
  }

  // j == 1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + 1) * 4 + 0] = grbg[i * width + 1];
      rgba[(i * width + 1) * 4 + 1] = static_cast<T>(0.25f * grbg[(i - 1) * width + 1] + 0.25f * grbg[i * width] +
                                                     0.25f * grbg[i * width + 2] + 0.25f * grbg[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 2] = static_cast<T>(0.25f * grbg[(i - 1) * width] + 0.25f * grbg[(i - 1) * width + 2] +
                                                     0.25f * grbg[(i + 1) * width] + 0.25f * grbg[(i + 1) * width + 2]);
    }
    else {
      rgba[(i * width + 1) * 4 + 0] =
        static_cast<T>(0.5f * grbg[(i - 1) * width + 1] + 0.5f * grbg[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 1] = grbg[i * width + 1];
      rgba[(i * width + 1) * 4 + 2] = static_cast<T>(0.5f * grbg[i * width] + 0.5f * grbg[i * width + 2]);
    }
  }

  // j == width-2
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.5f * grbg[(i + 1) * width - 3] + 0.5f * grbg[(i + 1) * width - 1]);
      rgba[(i * width + width - 2) * 4 + 1] = grbg[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.5f * grbg[i * width - 2] + 0.5f * grbg[(i + 2) * width - 2]);
    }
    else {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.25f * grbg[i * width - 3] + 0.25f * grbg[i * width - 1] + 0.25f * grbg[(i + 2) * width - 3] +
                       0.25f * grbg[(i + 2) * width - 1]);
      rgba[(i * width + width - 2) * 4 + 1] =
        static_cast<T>(0.25f * grbg[i * width - 2] + 0.25f * grbg[(i + 1) * width - 3] +
                       0.25f * grbg[(i + 1) * width - 1] + 0.25f * grbg[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 2] = grbg[(i + 1) * width - 2];
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] = grbg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 1] = grbg[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * grbg[i * width - 2] + 0.5f * grbg[(i + 2) * width - 2]);
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * grbg[i * width - 1] + 0.5f * grbg[(i + 2) * width - 1]);
      rgba[(i * width + width - 1) * 4 + 1] = grbg[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] = grbg[(i + 1) * width - 2];
    }
  }

  // i == height-2
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * grbg[(height - 2) * width + j - 1] + 0.5f * grbg[(height - 2) * width + j + 1]);
      rgba[((height - 2) * width + j) * 4 + 1] = grbg[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * grbg[(height - 3) * width + j] + 0.5f * grbg[(height - 1) * width + j]);
    }
    else {
      rgba[((height - 2) * width + j) * 4 + 0] = grbg[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 1] =
        static_cast<T>(0.25f * grbg[(height - 3) * width + j] + 0.25f * grbg[(height - 2) * width + j - 1] +
                       0.25f * grbg[(height - 2) * width + j + 1] + 0.25f * grbg[(height - 1) * width + j]);
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.25f * grbg[(height - 3) * width + j - 1] + 0.25f * grbg[(height - 3) * width + j + 1] +
                       0.25f * grbg[(height - 1) * width + j - 1] + 0.25f * grbg[(height - 1) * width + j + 1]);
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * grbg[(height - 2) * width + j - 1] + 0.5f * grbg[(height - 2) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * grbg[(height - 1) * width + j - 1] + 0.5f * grbg[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] = grbg[(height - 1) * width + j];
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] = grbg[(height - 2) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] = grbg[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * grbg[(height - 1) * width + j - 1] + 0.5f * grbg[(height - 1) * width + j + 1]);
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 2; i < height - 2; ++i) {
    for (unsigned int j = 2; j < width - 2; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaMalvar(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiMalvar(grbg, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerMalvar(grbg, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerMalvar(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 2] = grbg[i * width + j];
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiMalvar(grbg, width, i, j);
        rgba[(i * width + j) * 4 + 1] = grbg[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaMalvar(grbg, width, i, j);
      }
    }
  }
}

template <typename T>
void demosaicRGGBToRGBaMalvarTpl(const T *rggb, T *rgba, unsigned int width, unsigned int height, unsigned int nThreads)
{
  m_assert("width must be >= 4", width >= 4);
  m_assert("height must be >= 4", height >= 4);
  m_assert("width must be a multiple of 2", width % 2 == 0);
  m_assert("height must be a multiple of 2", height % 2 == 0);

  // (0,0)
  rgba[0] = rggb[0];
  rgba[1] = rggb[1];
  rgba[2] = rggb[width + 1];

  // (0,w-1)
  rgba[(width - 1) * 4 + 0] = rggb[width - 2];
  rgba[(width - 1) * 4 + 1] = rggb[width - 1];
  rgba[(width - 1) * 4 + 2] = rggb[2 * width - 1];

  // (h-1,0)
  rgba[((height - 1) * width) * 4 + 0] = rggb[(height - 2) * width];
  rgba[((height - 1) * width) * 4 + 1] = rggb[(height - 1) * width];
  rgba[((height - 1) * width) * 4 + 2] = rggb[(height - 1) * width + 1];

  // (h-1,w-1)
  rgba[((height - 1) * width + width - 1) * 4 + 0] = rggb[(height - 1) * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 1] = rggb[height * width - 2];
  rgba[((height - 1) * width + width - 1) * 4 + 2] = rggb[height * width - 1];

  // i == 0
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[j * 4 + 0] = rggb[j];
      rgba[j * 4 + 1] = static_cast<T>(0.5f * rggb[j - 1] + 0.5f * rggb[j + 1]);
      rgba[j * 4 + 2] = static_cast<T>(0.5f * rggb[width + j - 1] + 0.5f * rggb[width + j + 1]);
    }
    else {
      rgba[j * 4 + 0] = static_cast<T>(0.5f * rggb[j - 1] + 0.5f * rggb[j + 1]);
      rgba[j * 4 + 1] = rggb[j];
      rgba[j * 4 + 2] = rggb[width + j];
    }
  }

  // i == 1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.5f * rggb[j] + 0.5f * rggb[2 * width + j]);
      rgba[(width + j) * 4 + 1] = rggb[width + j];
      rgba[(width + j) * 4 + 2] = static_cast<T>(0.5f * rggb[width + j - 1] + 0.5f * rggb[width + j + 1]);
    }
    else {
      rgba[(width + j) * 4 + 0] = static_cast<T>(0.25f * rggb[j - 1] + 0.25f * rggb[j + 1] +
                                                 0.25f * rggb[2 * width + j - 1] + 0.25f * rggb[2 * width + j + 1]);
      rgba[(width + j) * 4 + 1] = static_cast<T>(0.25f * rggb[j] + 0.25f * rggb[width + j - 1] +
                                                 0.25f * rggb[width + j + 1] + 0.25f * rggb[2 * width + j]);
      rgba[(width + j) * 4 + 2] = rggb[width + j];
    }
  }

  // j == 0
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[i * width * 4 + 0] = rggb[i * width];
      rgba[i * width * 4 + 1] = rggb[i * width + 1];
      rgba[i * width * 4 + 2] = static_cast<T>(0.5f * rggb[(i - 1) * width + 1] + 0.5f * rggb[(i + 1) * width + 1]);
    }
    else {
      rgba[i * width * 4 + 0] = static_cast<T>(0.5f * rggb[(i - 1) * width] + 0.5f * rggb[(i + 1) * width]);
      rgba[i * width * 4 + 1] = rggb[i * width];
      rgba[i * width * 4 + 2] = rggb[i * width + 1];
    }
  }

  // j == 1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + 1) * 4 + 0] = static_cast<T>(0.5f * rggb[i * width] + 0.5f * rggb[i * width + 2]);
      rgba[(i * width + 1) * 4 + 1] = rggb[i * width + 1];
      rgba[(i * width + 1) * 4 + 2] =
        static_cast<T>(0.5f * rggb[(i - 1) * width + 1] + 0.5f * rggb[(i + 1) * width + 1]);
    }
    else {
      rgba[(i * width + 1) * 4 + 0] = static_cast<T>(0.25f * rggb[(i - 1) * width] + 0.25f * rggb[(i - 1) * width + 2] +
                                                     0.25f * rggb[(i + 1) * width] + 0.25f * rggb[(i + 1) * width + 2]);
      rgba[(i * width + 1) * 4 + 1] = static_cast<T>(0.25f * rggb[(i - 1) * width + 1] + 0.25f * rggb[i * width] +
                                                     0.25f * rggb[i * width + 2] + 0.25f * rggb[(i + 1) * width + 1]);
      rgba[(i * width + 1) * 4 + 2] = rggb[i * width + 1];
    }
  }

  // j == width-2
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 2) * 4 + 0] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 1] =
        static_cast<T>(0.25f * rggb[i * width - 2] + 0.25f * rggb[(i + 1) * width - 3] +
                       0.25f * rggb[(i + 1) * width - 1] + 0.25f * rggb[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.25f * rggb[i * width - 3] + 0.25f * rggb[i * width - 1] + 0.25f * rggb[(i + 2) * width - 3] +
                       0.25f * rggb[(i + 2) * width - 1]);
    }
    else {
      rgba[(i * width + width - 2) * 4 + 0] =
        static_cast<T>(0.5f * rggb[i * width - 2] + 0.5f * rggb[(i + 2) * width - 2]);
      rgba[(i * width + width - 2) * 4 + 1] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 2) * 4 + 2] =
        static_cast<T>(0.5f * rggb[(i + 1) * width - 3] + 0.5f * rggb[(i + 1) * width - 1]);
    }
  }

  // j == width-1
  for (unsigned int i = 1; i < height - 1; ++i) {
    if (i % 2 == 0) {
      rgba[(i * width + width - 1) * 4 + 0] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 1] = rggb[(i + 1) * width - 1];
      rgba[(i * width + width - 1) * 4 + 2] =
        static_cast<T>(0.5f * rggb[i * width - 1] + 0.5f * rggb[(i + 2) * width - 1]);
    }
    else {
      rgba[(i * width + width - 1) * 4 + 0] =
        static_cast<T>(0.5f * rggb[i * width - 2] + 0.5f * rggb[(i + 2) * width - 2]);
      rgba[(i * width + width - 1) * 4 + 1] = rggb[(i + 1) * width - 2];
      rgba[(i * width + width - 1) * 4 + 2] = rggb[(i + 1) * width - 1];
    }
  }

  // i == height-2
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 2) * width + j) * 4 + 0] = rggb[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 1] =
        static_cast<T>(0.25f * rggb[(height - 3) * width + j] + 0.25f * rggb[(height - 2) * width + j - 1] +
                       0.25f * rggb[(height - 2) * width + j + 1] + 0.25f * rggb[(height - 1) * width + j]);
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.25f * rggb[(height - 3) * width + j - 1] + 0.25f * rggb[(height - 3) * width + j + 1] +
                       0.25f * rggb[(height - 1) * width + j - 1] + 0.25f * rggb[(height - 1) * width + j + 1]);
    }
    else {
      rgba[((height - 2) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * rggb[(height - 2) * width + j - 1] + 0.5f * rggb[(height - 2) * width + j + 1]);
      rgba[((height - 2) * width + j) * 4 + 1] = rggb[(height - 2) * width + j];
      rgba[((height - 2) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * rggb[(height - 3) * width + j] + 0.5f * rggb[(height - 1) * width + j]);
    }
  }

  // i == height-1
  for (unsigned int j = 1; j < width - 1; ++j) {
    if (j % 2 == 0) {
      rgba[((height - 1) * width + j) * 4 + 0] = rggb[(height - 2) * width + j];
      rgba[((height - 1) * width + j) * 4 + 1] = rggb[(height - 1) * width + j];
      rgba[((height - 1) * width + j) * 4 + 2] =
        static_cast<T>(0.5f * rggb[(height - 1) * width + j - 1] + 0.5f * rggb[(height - 1) * width + j + 1]);
    }
    else {
      rgba[((height - 1) * width + j) * 4 + 0] =
        static_cast<T>(0.5f * rggb[(height - 2) * width + j - 1] + 0.5f * rggb[(height - 2) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 1] =
        static_cast<T>(0.5f * rggb[(height - 1) * width + j - 1] + 0.5f * rggb[(height - 1) * width + j + 1]);
      rgba[((height - 1) * width + j) * 4 + 2] = rggb[(height - 1) * width + j];
    }
  }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
  if (nThreads > 0) {
    omp_set_num_threads(static_cast<int>(nThreads));
  }
#pragma omp parallel for schedule(dynamic)
#else
  (void)nThreads;
#endif
  for (unsigned int i = 2; i < height - 2; ++i) {
    for (unsigned int j = 2; j < width - 2; ++j) {
      if (i % 2 == 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 2] = demosaicCheckerMalvar(rggb, width, i, j);
      }
      else if (i % 2 == 0 && j % 2 != 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicThetaMalvar(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicPhiMalvar(rggb, width, i, j);
      }
      else if (i % 2 != 0 && j % 2 == 0) {
        rgba[(i * width + j) * 4 + 0] = demosaicPhiMalvar(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = rggb[i * width + j];
        rgba[(i * width + j) * 4 + 2] = demosaicThetaMalvar(rggb, width, i, j);
      }
      else {
        rgba[(i * width + j) * 4 + 0] = demosaicCheckerMalvar(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 1] = demosaicCrossMalvar(rggb, width, i, j);
        rgba[(i * width + j) * 4 + 2] = rggb[i * width + j];
      }
    }
  }
}

#endif
#endif
