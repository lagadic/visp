/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Various image tools, convolution, ...
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpImageFilter_H
#define vpImageFilter_H

/*!
  \file vpImageFilter.h
  \brief  Various image filter, convolution, etc...

*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

/*!
  \class vpImageFilter

  \ingroup group_core_image

  \brief  Various image filter, convolution, etc...

*/
class VISP_EXPORT vpImageFilter
{
public:
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  static void canny(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ic, const unsigned int gaussianFilterSize,
                    const double thresholdCanny, const unsigned int apertureSobel);
#endif

  /*!
   Apply a 1x3 derivative filter to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template <class T> static double derivativeFilterX(const vpImage<T> &I, const unsigned int r, const unsigned int c)
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
  template <class T> static double derivativeFilterY(const vpImage<T> &I, const unsigned int r, const unsigned int c)
  {
    return (2047.0 * (I[r + 1][c] - I[r - 1][c]) + 913.0 * (I[r + 2][c] - I[r - 2][c]) +
            112.0 * (I[r + 3][c] - I[r - 3][c])) /
           8418.0;
  }

  /*!
   Apply a 1 x size Derivative Filter in X to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using
   vpImageFilter::getGaussianDerivativeKernel(). \param size : size of the
   filter

   \sa vpImageFilter::getGaussianDerivativeKernel()
   */

  template <class T>
  static double derivativeFilterX(const vpImage<T> &I, const unsigned int r, const unsigned int c, const double *filter,
                                  const unsigned int size)
  {
    unsigned int i;
    double result;

    result = 0;

    for (i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] - I[r][c - i]);
    }
    return result;
  }

  /*!
   Apply a size x 1 Derivative Filter in Y to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using
  vpImageFilter::getGaussianDerivativeKernel(). \param size : size of the
  filter

  \sa vpImageFilter::getGaussianDerivativeKernel()
   */
  template <class T>
  static double derivativeFilterY(const vpImage<T> &I, const unsigned int r, const unsigned int c, const double *filter,
                                  const unsigned int size)
  {
    unsigned int i;
    double result;

    result = 0;

    for (i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] - I[r - i][c]);
    }
    return result;
  }

  static void filter(const vpImage<double> &I, vpImage<double> &Iu, vpImage<double> &Iv, const vpMatrix &M,
                     const bool convolve = false);

  static void filter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpMatrix &M,
                     const bool convolve = false);

  static void sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
                        const vpColVector &kernelV);

  static void filter(const vpImage<unsigned char> &I, vpImage<double> &GI, const double *filter, unsigned int size);
  static void filter(const vpImage<double> &I, vpImage<double> &GI, const double *filter, unsigned int size);

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

  static void filterX(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter, unsigned int size);
  static void filterX(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size);

  static inline double filterX(const vpImage<unsigned char> &I, unsigned int r, unsigned int c, const double *filter,
                               unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXLeftBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][c + i] + I[r][i - c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXRightBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterX(const vpImage<double> &I, unsigned int r, unsigned int c, const double *filter,
                               unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r][c + i] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXLeftBorder(const vpImage<double> &I, unsigned int r, unsigned int c, const double *filter,
                                         unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c > i)
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][c + i] + I[r][i - c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterXRightBorder(const vpImage<double> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (c + i < I.getWidth())
        result += filter[i] * (I[r][c + i] + I[r][c - i]);
      else
        result += filter[i] * (I[r][2 * I.getWidth() - c - i - 1] + I[r][c - i]);
    }
    return result + filter[0] * I[r][c];
  }

  static void filterY(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter, unsigned int size);
  static void filterY(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size);
  static inline double filterY(const vpImage<unsigned char> &I, unsigned int r, unsigned int c, const double *filter,
                               unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  double static inline filterYTopBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[r + i][c] + I[i - r][c]);
    }
    return result + filter[0] * I[r][c];
  }

  double static inline filterYBottomBorder(const vpImage<unsigned char> &I, unsigned int r, unsigned int c,
                                           const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterYTopBorder(const vpImage<double> &I, unsigned int r, unsigned int c, const double *filter,
                                        unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r > i)
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[r + i][c] + I[i - r][c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterYBottomBorder(const vpImage<double> &I, unsigned int r, unsigned int c,
                                           const double *filter, unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      if (r + i < I.getHeight())
        result += filter[i] * (I[r + i][c] + I[r - i][c]);
      else
        result += filter[i] * (I[2 * I.getHeight() - r - i - 1][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  static inline double filterY(const vpImage<double> &I, unsigned int r, unsigned int c, const double *filter,
                               unsigned int size)
  {
    double result;

    result = 0;

    for (unsigned int i = 1; i <= (size - 1) / 2; i++) {
      result += filter[i] * (I[r + i][c] + I[r - i][c]);
    }
    return result + filter[0] * I[r][c];
  }

  static void gaussianBlur(const vpImage<unsigned char> &I, vpImage<double> &GI, unsigned int size = 7,
                           double sigma = 0., bool normalize = true);
  static void gaussianBlur(const vpImage<double> &I, vpImage<double> &GI, unsigned int size = 7, double sigma = 0.,
                           bool normalize = true);
  /*!
   Apply a 5x5 Gaussian filter to an image pixel.

   \param fr : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template <class T> static double gaussianFilter(const vpImage<T> &fr, const unsigned int r, const unsigned int c)
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
  // operation pour pyramide gaussienne
  static void getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);
  static void getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI);

  static void getGaussianKernel(double *filter, unsigned int size, double sigma = 0., bool normalize = true);
  static void getGaussianDerivativeKernel(double *filter, unsigned int size, double sigma = 0., bool normalize = true);

  // fonction renvoyant le gradient en X de l'image I pour traitement
  // pyramidal => dimension /2
  static void getGradX(const vpImage<unsigned char> &I, vpImage<double> &dIx);
  static void getGradX(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter, unsigned int size);
  static void getGradX(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size);
  static void getGradXGauss2D(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *gaussianKernel,
                              const double *gaussianDerivativeKernel, unsigned int size);

  // fonction renvoyant le gradient en Y de l'image I
  static void getGradY(const vpImage<unsigned char> &I, vpImage<double> &dIy);
  static void getGradY(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *filter, unsigned int size);
  static void getGradY(const vpImage<double> &I, vpImage<double> &dIy, const double *filter, unsigned int size);
  static void getGradYGauss2D(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *gaussianKernel,
                              const double *gaussianDerivativeKernel, unsigned int size);
};

#endif
