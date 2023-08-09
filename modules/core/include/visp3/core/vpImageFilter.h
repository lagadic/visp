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
 * Various image tools, convolution, ...
 *
*****************************************************************************/

#ifndef _vpImageFilter_h_
#define _vpImageFilter_h_

/*!
  \file vpImageFilter.h
  \brief  Various image filter, convolution, etc...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#endif

/*!
  \class vpImageFilter

  \ingroup group_core_image

  \brief  Various image filter, convolution, etc...

*/
class VISP_EXPORT vpImageFilter
{
public:
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
  static double median(const cv::Mat &cv_I);
  static double median(const vpImage<unsigned char> &Isrc);
  static std::vector<double> median(const vpImage<vpRGBa> &Isrc);
  static void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, unsigned int gaussianFilterSize,
                    double thresholdCanny, unsigned int apertureSobel);
#endif

  /*!
   Apply a 1x3 derivative filter to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template <class T> static double derivativeFilterX(const vpImage<T> &I, unsigned int r, unsigned int c)
  {
    return (2047.0 * (I[r][c + 1] - I[r][c - 1]) + 913.0 * (I[r][c + 2] - I[r][c - 2]) +
            112.0 * (I[r][c + 3] - I[r][c - 3])) /
      8418.0;
  }

  /*!
   Apply a 3x1 derivative filter to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template <class T> static double derivativeFilterY(const vpImage<T> &I, unsigned int r, unsigned int c)
  {
    return (2047.0 * (I[r + 1][c] - I[r - 1][c]) + 913.0 * (I[r + 2][c] - I[r - 2][c]) +
            112.0 * (I[r + 3][c] - I[r - 3][c])) /
      8418.0;
  }

  /*!
   Apply a 1 x size Derivative Filter in X to an image pixel.

   \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using
   vpImageFilter::getGaussianDerivativeKernel(). \param size : size of the
   filter

   \sa vpImageFilter::getGaussianDerivativeKernel()
   */

  template <class T, typename FilterType>
  static FilterType derivativeFilterX(const vpImage<T> &I, unsigned int r, unsigned int c, const FilterType *filter,
                                  unsigned int size)
  {
    unsigned int i;
    FilterType result;

    result = 0;

    for (i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] - I[r][c - i]);
    }
    return result;
  }

  /*!
   Apply a size x 1 Derivative Filter in Y to an image pixel.

   \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using
  vpImageFilter::getGaussianDerivativeKernel(). \param size : size of the
  filter

  \sa vpImageFilter::getGaussianDerivativeKernel()
   */
  template <class T, typename FilterType>
  static FilterType derivativeFilterY(const vpImage<T> &I, unsigned int r, unsigned int c, const FilterType *filter,
                                  unsigned int size)
  {
    unsigned int i;
    FilterType result;

    result = 0;

    for (i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] - I[r - i][c]);
    }
    return result;
  }

  /*!
  Apply a filter to an image.
  \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
  \param I : Image to filter
  \param If : Filtered image.
  \param M : Filter kernel.
  \param convolve : If true, perform a convolution otherwise a correlation.

  \note By default it performs a correlation:
  \f[
    \textbf{I\_filtered} \left( u,v \right) =
    \sum_{y=0}^{\textbf{kernel\_h}}
    \sum_{x=0}^{\textbf{kernel\_w}}
    \textbf{M} \left( x,y \right ) \times
    \textbf{I} \left(
  u-\frac{\textbf{kernel\_w}}{2}+x,v-\frac{\textbf{kernel\_h}}{2}+y \right)
  \f]
  The convolution is almost the same operation:
  \f[
    \textbf{I\_filtered} \left( u,v \right) =
    \sum_{y=0}^{\textbf{kernel\_h}}
    \sum_{x=0}^{\textbf{kernel\_w}}
    \textbf{M} \left( x,y \right ) \times
    \textbf{I} \left(
  u+\frac{\textbf{kernel\_w}}{2}-x,v+\frac{\textbf{kernel\_h}}{2}-y \right)
  \f]
  Only pixels in the input image fully covered by the kernel are considered.
  */
  template <typename FilterType>
  static void filter(const vpImage<unsigned char> &I, vpImage<FilterType> &If, const vpArray2D<FilterType> &M, bool convolve = false)
  {
    unsigned int size_y = M.getRows(), size_x = M.getCols();
    unsigned int half_size_y = size_y / 2, half_size_x = size_x / 2;

    If.resize(I.getHeight(), I.getWidth(), 0.0);

    if (convolve) {
      for (unsigned int i = half_size_y; i < I.getHeight() - half_size_y; i++) {
        for (unsigned int j = half_size_x; j < I.getWidth() - half_size_x; j++) {
          FilterType conv = 0;

          for (unsigned int a = 0; a < size_y; a++) {
            for (unsigned int b = 0; b < size_x; b++) {
              FilterType val = I[i + half_size_y - a][j + half_size_x - b]; // Convolution
              conv += M[a][b] * val;
            }
          }
          If[i][j] = conv;
        }
      }
    }
    else {
      for (unsigned int i = half_size_y; i < I.getHeight() - half_size_y; i++) {
        for (unsigned int j = half_size_x; j < I.getWidth() - half_size_x; j++) {
          FilterType corr = 0;

          for (unsigned int a = 0; a < size_y; a++) {
            for (unsigned int b = 0; b < size_x; b++) {
              FilterType val = I[i - half_size_y + a][j - half_size_x + b]; // Correlation
              corr += M[a][b] * val;
            }
          }
          If[i][j] = corr;
        }
      }
    }
  }

  /*!
  Apply a filter to an image:
  \f[
    \textbf{I}_u = \textbf{M} \ast \textbf{I} \textbf{ and } \textbf{I}_v =
  \textbf{M}^t \ast \textbf{I} \f]
  \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
  \param I : Image to filter
  \param Iu : Filtered image along the horizontal axis (u = columns).
  \param Iv : Filtered image along the vertical axis (v = rows).
  \param M : Filter kernel.
  \param convolve : If true, perform a convolution otherwise a correlation.
  */
  template <typename FilterType>
  static void filter(const vpImage<FilterType> &I, vpImage<FilterType> &Iu, vpImage<FilterType> &Iv, const vpArray2D<FilterType> &M,
    bool convolve = false)
  {
    unsigned int size = M.getRows();
    unsigned int half_size = size / 2;

    Iu.resize(I.getHeight(), I.getWidth(), 0.0);
    Iv.resize(I.getHeight(), I.getWidth(), 0.0);

    if (convolve) {
      for (unsigned int v = half_size; v < I.getHeight() - half_size; v++) {
        for (unsigned int u = half_size; u < I.getWidth() - half_size; u++) {
          FilterType conv_u = 0;
          FilterType conv_v = 0;

          for (unsigned int a = 0; a < size; a++) {
            for (unsigned int b = 0; b < size; b++) {
              FilterType val = I[v + half_size - a][u + half_size - b]; // Convolution
              conv_u += M[a][b] * val;
              conv_v += M[b][a] * val;
            }
          }
          Iu[v][u] = conv_u;
          Iv[v][u] = conv_v;
        }
      }
    }
    else {
      for (unsigned int v = half_size; v < I.getHeight() - half_size; v++) {
        for (unsigned int u = half_size; u < I.getWidth() - half_size; u++) {
          FilterType conv_u = 0;
          FilterType conv_v = 0;

          for (unsigned int a = 0; a < size; a++) {
            for (unsigned int b = 0; b < size; b++) {
              FilterType val = I[v - half_size + a][u - half_size + b]; // Correlation
              conv_u += M[a][b] * val;
              conv_v += M[b][a] * val;
            }
          }
          Iu[v][u] = conv_u;
          Iv[v][u] = conv_v;
        }
      }
    }
  }

  static void sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
                        const vpColVector &kernelV);

  /*!
    Apply a separable filter.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I: The original image.
    \param GI: The filtered image.
    \param filter: The separable filter.
    \param size: The size of the filter.
  */
  template <typename FilterType>
  static void filter(const vpImage<unsigned char> &I, vpImage<FilterType> &GI, const FilterType *filter,
    unsigned int size)
  {
    vpImage<FilterType> GIx;
    filterX<FilterType>(I, GIx, filter, size);
    filterY<FilterType>(GIx, GI, filter, size);
    GIx.destroy();
  }

  /*!
    Apply a separable filter.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I: The original image.
    \param GI: The filtered image.
    \param filter: The separable filter.
    \param size: The size of the filter.
  */
  template <typename FilterType>
  static void filter(const vpImage<FilterType> &I, vpImage<FilterType> &GI, const FilterType *filter, unsigned int size)
  {
    vpImage<FilterType> GIx;
    filterX<FilterType>(I, GIx, filter, size);
    filterY<FilterType>(GIx, GI, filter, size);
    GIx.destroy();
  }

  static inline unsigned char filterGaussXPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
  {
    return (unsigned char)((1. * I[i][j - 2] + 4. * I[i][j - 1] + 6. * I[i][j] + 4. * I[i][j + 1] + 1. * I[i][j + 2]) /
                           16.);
  }
  static inline unsigned char filterGaussYPyramidal(const vpImage<unsigned char> &I, unsigned int i, unsigned int j)
  {
    return (unsigned char)((1. * I[i - 2][j] + 4. * I[i - 1][j] + 6. * I[i][j] + 4. * I[i + 1][j] + 1. * I[i + 2][j]) /
                           16.);
  }

  template <typename FilterType>
  static void filterX(const vpImage<unsigned char> &I, vpImage<FilterType> &dIx, const FilterType *filter,
  unsigned int size)
  {
    dIx.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < (size - 1) / 2; j++) {
        dIx[i][j] = vpImageFilter::filterXLeftBorder<FilterType>(I, i, j, filter, size);
      }
      for (unsigned int j = (size - 1) / 2; j < I.getWidth() - (size - 1) / 2; j++) {
        dIx[i][j] = vpImageFilter::filterX<FilterType>(I, i, j, filter, size);
      }
      for (unsigned int j = I.getWidth() - (size - 1) / 2; j < I.getWidth(); j++) {
        dIx[i][j] = vpImageFilter::filterXRightBorder<FilterType>(I, i, j, filter, size);
      }
    }
  }

  template<typename FilterType>
  static void filterX(const vpImage<FilterType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size)
  {
    dIx.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < (size - 1) / 2; j++) {
        dIx[i][j] = vpImageFilter::filterXLeftBorder<FilterType>(I, i, j, filter, size);
      }
      for (unsigned int j = (size - 1) / 2; j < I.getWidth() - (size - 1) / 2; j++) {
        dIx[i][j] = vpImageFilter::filterX<FilterType>(I, i, j, filter, size);
      }
      for (unsigned int j = I.getWidth() - (size - 1) / 2; j < I.getWidth(); j++) {
        dIx[i][j] = vpImageFilter::filterXRightBorder<FilterType>(I, i, j, filter, size);
      }
    }
  }

  static void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterXR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterXG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterXB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

  template<typename FilterType>
  static inline FilterType filterX(const vpImage<unsigned char> &I, unsigned int r, unsigned int c, const FilterType *filter,
                               unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i].R + I[r][c - i].R);
    }
    return result + filter[0] * I[r][c].R;
  }

  static inline double filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i].G + I[r][c - i].G);
    }
    return result + filter[0] * I[r][c].G;
  }

  static inline double filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i].B + I[r][c - i].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template <typename FilterType>
  static inline FilterType filterXLeftBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                         const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][c + i] + I[r][i - c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i].R + I[r][c - i].R);
      else
        result += filter[i] * (I[r][c + i].R + I[r][i - c].R);
    }
    return result + filter[0] * I[r][c].R;
  }

  static inline double filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i].G + I[r][c - i].G);
      else
        result += filter[i] * (I[r][c + i].G + I[r][i - c].G);
    }
    return result + filter[0] * I[r][c].G;
  }

  static inline double filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i].B + I[r][c - i].B);
      else
        result += filter[i] * (I[r][c + i].B + I[r][i - c].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template <typename FilterType>
  static inline FilterType filterXRightBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                          const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                           const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i].R + I[r][c - i].R);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1].R + I[r][c - i].R);
    }
    return result + filter[0] * I[r][c].R;
  }

  static inline double filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                           const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i].G + I[r][c - i].G);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1].G + I[r][c - i].G);
    }
    return result + filter[0] * I[r][c].G;
  }

  static inline double filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                           const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i].B + I[r][c - i].B);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1].B + I[r][c - i].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template <typename FilterType>
  static inline FilterType filterX(const vpImage<FilterType> &I, unsigned int r, unsigned int c, const FilterType *filter,
                               unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  template <typename FilterType>
  static inline FilterType filterXLeftBorder(const vpImage<FilterType> &I, unsigned int r, unsigned int c, const FilterType *filter,
                                         unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][c + i] + I[r][i - c]);
    }
    return result + filter[0] * I[r][c];
  }

  template <typename FilterType>
  static inline FilterType filterXRightBorder(const vpImage<FilterType> &I, unsigned int r, unsigned int c,
                                          const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  template <typename FilterType>
  static void filterY(const vpImage<unsigned char> &I, vpImage<FilterType> &dIy, const FilterType *filter,
  unsigned int size)
  {
    dIy.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterYTopBorder<FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int i = (size - 1) / 2; i < I.getHeight() - (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterY<FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int i = I.getHeight() - (size - 1) / 2; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterYBottomBorder<FilterType>(I, i, j, filter, size);
      }
    }
  }

  static void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterYR(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterYG(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);
  static void filterYB(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size);

  template<typename FilterType>
  static void filterY(const vpImage<FilterType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size)
  {
    dIy.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterYTopBorder<FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int i = (size - 1) / 2; i < I.getHeight() - (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterY<FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int i = I.getHeight() - (size - 1) / 2; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::filterYBottomBorder<FilterType>(I, i, j, filter, size);
      }
    }
  }

  template<typename FilterType>
  static inline FilterType filterY(const vpImage<unsigned char> &I, unsigned int r, unsigned int c, const FilterType *filter,
                               unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c].R + I[r - i][c].R);
    }
    return result + filter[0] * I[r][c].R;
  }
  static inline double filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c].G + I[r - i][c].G);
    }
    return result + filter[0] * I[r][c].G;
  }
  static inline double filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c].B + I[r - i][c].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template<typename FilterType>
  static inline FilterType filterYTopBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                        const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[r + i][c] + I[i - r][c]);
    }
    return result + filter[0] * I[r][c];
  }

  double static inline filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                         unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c].R + I[r - i][c].R);
      else
        result += filter[i] * (I[r + i][c].R + I[i - r][c].R);
    }
    return result + filter[0] * I[r][c].R;
  }

  double static inline filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                         unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c].G + I[r - i][c].G);
      else
        result += filter[i] * (I[r + i][c].G + I[i - r][c].G);
    }
    return result + filter[0] * I[r][c].G;
  }

  double static inline filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter,
                                         unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c].B + I[r - i][c].B);
      else
        result += filter[i] * (I[r + i][c].B + I[i - r][c].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template<typename FilterType>
  static inline FilterType filterYBottomBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                           const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  double static inline filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                            const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c].R + I[r - i][c].R);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c].R + I[r - i][c].R);
    }
    return result + filter[0] * I[r][c].R;
  }

  double static inline filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                            const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c].G + I[r - i][c].G);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c].G + I[r - i][c].G);
    }
    return result + filter[0] * I[r][c].G;
  }

  double static inline filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                            const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c].B + I[r - i][c].B);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c].B + I[r - i][c].B);
    }
    return result + filter[0] * I[r][c].B;
  }

  template<typename FilterType>
  static inline FilterType filterYTopBorder(const vpImage<double> &I, unsigned int r, unsigned int c, const FilterType *filter,
                                        unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[r + i][c] + I[i - r][c]);
    }
    return result + filter[0] * I[r][c];
  }

  template<typename FilterType>
  static inline FilterType filterYBottomBorder(const vpImage<double> &I, unsigned int r, unsigned int c,
                                           const FilterType *filter, unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  template<typename FilterType>
  static inline FilterType filterY(const vpImage<double> &I, unsigned int r, unsigned int c, const FilterType *filter,
                               unsigned int size)
  {
    FilterType result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  /*!
    Apply a Gaussian blur to an image.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I : Input image.
    \param GI : Filtered image.
    \param size : Filter size. This value should be odd.
    \param sigma : Gaussian standard deviation. If it is equal to zero or
    negative, it is computed from filter size as sigma = (size-1)/6.
    \param normalize : Flag indicating whether to normalize the filter coefficients or
    not.

    \sa getGaussianKernel() to know which kernel is used.
  */
  template <typename FilterType>
  static void gaussianBlur(const vpImage<unsigned char> &I, vpImage<FilterType> &GI, unsigned int size = 7, FilterType sigma = 0.,
    bool normalize = true)
  {
    FilterType *fg = new FilterType[(size + 1) / 2];
    vpImageFilter::getGaussianKernel<FilterType>(fg, size, sigma, normalize);
    vpImage<FilterType> GIx;
    vpImageFilter::filterX<FilterType>(I, GIx, fg, size);
    vpImageFilter::filterY<FilterType>(GIx, GI, fg, size);
    GIx.destroy();
    delete[] fg;
  }

  static void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size = 7, double sigma = 0.,
                           bool normalize = true);

  /*!
    Apply a Gaussian blur to a double image.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I : Input double image.
    \param GI : Filtered image.
    \param size : Filter size. This value should be odd.
    \param sigma : Gaussian standard deviation. If it is equal to zero or
    negative, it is computed from filter size as sigma = (size-1)/6.
    \param normalize : Flag indicating whether to normalize the filter coefficients or not.

    \sa getGaussianKernel() to know which kernel is used.
  */
  template <typename FilterType>
  static void gaussianBlur(const vpImage<FilterType> &I, vpImage<FilterType> &GI, unsigned int size = 7, FilterType sigma = 0.,
    bool normalize = true)
  {
    FilterType *fg = new FilterType[(size + 1) / 2];
    vpImageFilter::getGaussianKernel<FilterType>(fg, size, sigma, normalize);
    vpImage<FilterType> GIx;
    vpImageFilter::filterX<FilterType>(I, GIx, fg, size);
    vpImageFilter::filterY<FilterType>(GIx, GI, fg, size);
    GIx.destroy();
    delete[] fg;
  }

  /*!
   Apply a 5x5 Gaussian filter to an image pixel.

   \param fr : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template <class T> static double gaussianFilter(const vpImage<T> &fr, unsigned int r, unsigned int c)
  {
    // filter Gaussien
    return (15.0 * fr[r][c] + 12.0 * (fr[r - 1][c] + fr[r][c - 1] + fr[r + 1][c] + fr[r][c + 1]) +
            9.0 * (fr[r - 1][c - 1] + fr[r + 1][c - 1] + fr[r - 1][c + 1] + fr[r + 1][c + 1]) +
            5.0 * (fr[r - 2][c] + fr[r][c - 2] + fr[r + 2][c] + fr[r][c + 2]) +
            4.0 * (fr[r - 2][c + 1] + fr[r - 2][c - 1] + fr[r - 1][c - 2] + fr[r + 1][c - 2] + fr[r + 2][c - 1] +
                   fr[r + 2][c + 1] + fr[r - 1][c + 2] + fr[r + 1][c + 2]) +
            2.0 * (fr[r - 2][c - 2] + fr[r + 2][c - 2] + fr[r - 2][c + 2] + fr[r + 2][c + 2])) /
      159.0;
  }
  // Gaussain pyramid operation
  static void getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);

  /*!
    Return the coefficients \f$G_i\f$ of a Gaussian filter.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param[out] filter : Pointer to the half size filter kernel that should refer to a
    (size+1)/2 array. The first value refers to the central coefficient, the
    next one to the right coefficients. Left coefficients could be deduced by
    symmetry.
    \param[in] size : Filter size. This value should be odd and positive.
    \param[in] sigma : Gaussian standard deviation \f$ \sigma \f$. If it is equal to zero or negative, it is
    computed from filter size as sigma = (size-1)/6.
    \param[in] normalize : Flag indicating whether to normalize the filter coefficients or not. In that case \f$\Sigma G_i
    = 1 \f$.

    The function computes the \e (size+1)/2 values of the Gaussian filter cooefficients \f$ G_i \f$ as:
    \f[ G_i = \frac{1}{\sigma  \sqrt{2 \pi}} \exp{(-i^2 / (2. * \sigma^2))}\f]
  */
  template<typename FilterType>
  static void getGaussianKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
  {
    if (size % 2 != 1)
      throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));

    if (sigma <= 0)
      sigma = (size - 1) / 6.0;

    int middle = (int)(size - 1) / 2;
    FilterType sigma2 = vpMath::sqr(sigma);
    for (int i = 0; i <= middle; i++) {
      filter[i] = (1. / (sigma * sqrt(2. * M_PI))) * exp(-(i * i) / (2. * sigma2));
    }
    if (normalize) {
      // renormalization
      FilterType sum = 0;
      for (int i = 1; i <= middle; i++) {
        sum += 2 * filter[i];
      }
      sum += filter[0];

      for (int i = 0; i <= middle; i++) {
        filter[i] = filter[i] / sum;
      }
    }
  }

  /*!
    Return the coefficients of a Gaussian derivative filter that may be used to
    compute spatial image derivatives after applying a Gaussian blur.

    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param filter : Pointer to the filter kernel that should refer to a
    (size+1)/2 array. The first value refers to the central coefficient, the
    next one to the right coefficients. Left coefficients could be deduced by
    symmetry.
    \param size : Filter size. This value should be odd.
    \param sigma : Gaussian standard deviation. If it is equal to zero or negative, it is
    computed from filter size as sigma = (size-1)/6.
    \param normalize : Flag indicating whether to normalize the filter coefficients or not.
  */
  template <typename FilterType>
  static void getGaussianDerivativeKernel(FilterType *filter, unsigned int size, FilterType sigma = 0., bool normalize = true)
  {
    if (size % 2 != 1)
      throw(vpImageException(vpImageException::incorrectInitializationError, "Bad Gaussian filter size"));

    if (sigma <= 0)
      sigma = (size - 1) / 6.0;

    int middle = (int)(size - 1) / 2;
    FilterType sigma2 = vpMath::sqr(sigma);
    filter[0] = 0.;
    for (int i = 1; i <= middle; i++) {
      filter[i] = -(1. / (sigma * sqrt(2. * M_PI))) *
        (exp(-((i + 1) * (i + 1)) / (2. * sigma2)) - exp(-((i - 1) * (i - 1)) / (2. * sigma2))) / 2.;
    }

    if (normalize) {
      FilterType sum = 0;
      for (int i = 1; i <= middle; i++) {
        sum += 2. * (1. / (sigma * sqrt(2. * M_PI))) * exp(-(i * i) / (2. * sigma2));
      }
      sum += (1. / (sigma * sqrt(2. * M_PI)));

      for (int i = 1; i <= middle; i++) {
        filter[i] = filter[i] / sum;
      }
    }
  }

  // Gradient along X
  template<typename FilterType>
  static void getGradX(const vpImage<unsigned char> &I, vpImage<FilterType> &dIx)
  {
    dIx.resize(I.getHeight(), I.getWidth());
    // dIx=0;
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < 3; j++) {
        dIx[i][j] = 0;
      }
      for (unsigned int j = 3; j < I.getWidth() - 3; j++) {
        dIx[i][j] = vpImageFilter::derivativeFilterX(I, i, j);
      }
      for (unsigned int j = I.getWidth() - 3; j < I.getWidth(); j++) {
        dIx[i][j] = 0;
      }
    }
  }

  template <typename ImageType, typename FilterType>
  static void getGradX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter,
  unsigned int size)
  {
    dIx.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < (size - 1) / 2; j++) {
        dIx[i][j] = 0;
      }
      for (unsigned int j = (size - 1) / 2; j < I.getWidth() - (size - 1) / 2; j++) {
        dIx[i][j] = vpImageFilter::derivativeFilterX<ImageType, FilterType>(I, i, j, filter, size);
      }
      for (unsigned int j = I.getWidth() - (size - 1) / 2; j < I.getWidth(); j++) {
        dIx[i][j] = 0;
      }
    }
  }

  /*!
    Compute the gradient along X after applying a gaussian filter along Y.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I : Input image
    \param dIx : Gradient along X.
    \param gaussianKernel : Gaussian kernel which values should be computed using vpImageFilter::getGaussianKernel().
    \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
    vpImageFilter::getGaussianDerivativeKernel().
    \param size : Size of the Gaussian and Gaussian derivative kernels.
  */
  template <typename ImageType, typename FilterType>
  static void getGradXGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *gaussianKernel,
    const FilterType *gaussianDerivativeKernel, unsigned int size)
  {
    vpImage<FilterType> GIy;
    vpImageFilter::filterY<FilterType>(I, GIy, gaussianKernel, size);
    vpImageFilter::getGradX<FilterType, FilterType>(GIy, dIx, gaussianDerivativeKernel, size);
  }

  // Gradient along Y
  template <typename FilterType>
  static void getGradY(const vpImage<unsigned char> &I, vpImage<FilterType> &dIy)
  {
    dIy.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = 0;
      }
    }
    for (unsigned int i = 3; i < I.getHeight() - 3; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::derivativeFilterY(I, i, j);
      }
    }
    for (unsigned int i = I.getHeight() - 3; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = 0;
      }
    }
  }

  template <typename ImageType, typename FilterType>
  static void getGradY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size)
  {
    dIy.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = 0;
      }
    }
    for (unsigned int i = (size - 1) / 2; i < I.getHeight() - (size - 1) / 2; i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = vpImageFilter::derivativeFilterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int i = I.getHeight() - (size - 1) / 2; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        dIy[i][j] = 0;
      }
    }
  }

  /*!
    Compute the gradient along Y after applying a gaussian filter along X.
    \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
    \param I : Input image
    \param dIy : Gradient along Y.
    \param gaussianKernel : Gaussian kernel which values should be computed  using vpImageFilter::getGaussianKernel().
    \param gaussianDerivativeKernel : Gaussian derivative kernel which values should be computed using
    vpImageFilter::getGaussianDerivativeKernel().
    \param size : Size of the Gaussian and Gaussian derivative kernels.
  */
  template <typename ImageType, typename FilterType>
  static void getGradYGauss2D(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *gaussianKernel,
    const FilterType *gaussianDerivativeKernel, unsigned int size)
  {
    vpImage<FilterType> GIx;
    vpImageFilter::filterX<FilterType>(I, GIx, gaussianKernel, size);
    vpImageFilter::getGradY<FilterType, FilterType>(GIx, dIy, gaussianDerivativeKernel, size);
  }

  /*!
  Get Sobel kernel for X-direction.
  \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
  \param filter : Pointer to a double array already allocated.
  \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
  \return Scaling factor.
  */
  template <typename FilterType>
  inline static FilterType getSobelKernelX(FilterType *filter, unsigned int size)
  {
    if (size == 0)
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    if (size > 20)
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");

    vpArray2D<FilterType> SobelY(size * 2 + 1, size * 2 + 1);
    FilterType norm = getSobelKernelY<FilterType>(SobelY.data, size);
    memcpy(filter, SobelY.t().data, SobelY.getRows() * SobelY.getCols() * sizeof(FilterType));
    return norm;
  }

  /*!
  Get Sobel kernel for Y-direction.
  \tparam FilterType: Either float, to accelerate the computation time, or double, to have greater precision.
  \param filter : Pointer to a double array already allocated.
  \param size : Kernel size computed as: kernel_size = size*2 + 1 (max size is 20).
  \return Scaling factor.
 */
  template <typename FilterType>
  inline static FilterType getSobelKernelY(FilterType *filter, unsigned int size)
  {
    // Sobel kernel pre-computed for the usual size
    static const FilterType SobelY3x3[9] = { -1.0, -2.0, -1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0 };
    static const FilterType SobelY5x5[25] = { -1.0, -4.0, -6.0, -4.0, -1.0, -2.0, -8.0, -12.0, -8.0, -2.0, 0.0, 0.0, 0.0,
                                        0.0,  0.0,  2.0,  8.0,  12.0, 8.0,  2.0,  1.0,   4.0,  6.0,  4.0, 1.0 };
    static const FilterType SobelY7x7[49] = { -1,   -6,  -15, -20, -15, -6, -1, -4, -24, -60, -80, -60, -24, -4, -5,  -30, -75,
                                        -100, -75, -30, -5,  0,   0,  0,  0,  0,   0,   0,   5,   30,  75, 100, 75,  30,
                                        5,    4,   24,  60,  80,  60, 24, 4,  1,   6,   15,  20,  15,  6,  1 };
    const vpArray2D<FilterType> smoothingKernel(3, 3);
    smoothingKernel[0][0] = 1.0;
    smoothingKernel[0][1] = 2.0;
    smoothingKernel[0][2] = 1.0;
    smoothingKernel[1][0] = 2.0;
    smoothingKernel[1][1] = 4.0;
    smoothingKernel[1][2] = 2.0;
    smoothingKernel[2][0] = 1.0;
    smoothingKernel[2][1] = 2.0;
    smoothingKernel[2][2] = 1.0;

    if (size == 0)
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size 0!");
    if (size > 20)
      throw vpException(vpException::dimensionError, "Cannot get Sobel kernel of size > 20!");

    const unsigned int kernel_size = size * 2 + 1;
    if (kernel_size == 3) {
      memcpy(filter, SobelY3x3, kernel_size * kernel_size * sizeof(FilterType));
      return 1 / 8.0;
    }
    if (kernel_size == 5) {
      memcpy(filter, SobelY5x5, kernel_size * kernel_size * sizeof(FilterType));
      return 1 / 16.0;
    }
    if (kernel_size == 7) {
      memcpy(filter, SobelY7x7, kernel_size * kernel_size * sizeof(FilterType));
      return 1 / 16.0;
    }

    vpArray2D<FilterType> sobelY(7, 7);
    memcpy(sobelY.data, SobelY7x7, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));
    for (unsigned int i = 4; i <= size; i++) {
      sobelY = vpArray2D<FilterType>::conv2(sobelY, smoothingKernel, "full");
    }

    memcpy(filter, sobelY.data, sobelY.getRows() * sobelY.getCols() * sizeof(FilterType));

    return 1 / 16.0;
  }
};

#endif
