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

#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpRGBa.h>

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
void vpImageFilter::filter<float>(const vpImage<unsigned char> &I, vpImage<float> &If, const vpArray2D<float> &M, bool convolve);

template<>
void vpImageFilter::filter<double>(const vpImage<unsigned char> &I, vpImage<double> &If, const vpArray2D<double> &M, bool convolve);

template <>
void vpImageFilter::filter<float>(const vpImage<float> &I, vpImage<float> &Iu, vpImage<float> &Iv, const vpArray2D<float> &M,
  bool convolve);

template <>
void vpImageFilter::filter<double>(const vpImage<double> &I, vpImage<double> &Iu, vpImage<double> &Iv, const vpArray2D<double> &M,
  bool convolve);
/**
 * \endcond
 */

 /*!
   Apply a filter to an image using two separable kernels. For instance,
   the Sobel kernel can be decomposed to:
   \f[
     \left [
     \begin{matrix}
     1 & 0 & -1 \\
     2 & 0 & -2 \\
     1 & 0 & -1
     \end{matrix}
     \right ] =
     \left [
     \begin{matrix}
     1 \\
     2 \\
     1
     \end{matrix}
     \right ] \ast
     \left [
     \begin{matrix}
     1 && 0 && -1
     \end{matrix}
     \right ]
   \f]
   Thus, the convolution operation can be performed as:
   \f[
     G_x =
     \left [
     \begin{matrix}
     1 \\
     2 \\
     1
     \end{matrix}
     \right ] \ast
     \left (
     \left [
     \begin{matrix}
     1 && 0 && -1
     \end{matrix}
     \right ] \ast I
     \right )
   \f]
   Using two separable kernels reduce the number of operations and can be
   faster for large kernels.

   \param I : Image to filter
   \param If : Filtered image.
   \param kernelH : Separable kernel (performed first).
   \param kernelV : Separable kernel (performed last).
   \note Only pixels in the input image fully covered by the kernel are
   considered.
 */
void vpImageFilter::sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
  const vpColVector &kernelV)
{
  unsigned int size = kernelH.size();
  unsigned int half_size = size / 2;

  If.resize(I.getHeight(), I.getWidth(), 0.0);
  vpImage<double> I_filter(I.getHeight(), I.getWidth(), 0.0);

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = half_size; j < I.getWidth() - half_size; j++) {
      double conv = 0.0;
      for (unsigned int a = 0; a < kernelH.size(); a++) {
        conv += kernelH[a] * I[i][j + half_size - a];
      }

      I_filter[i][j] = conv;
    }
  }

  for (unsigned int i = half_size; i < I.getHeight() - half_size; i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      double conv = 0.0;
      for (unsigned int a = 0; a < kernelV.size(); a++) {
        conv += kernelV[a] * I_filter[i + half_size - a][j];
      }

      If[i][j] = conv;
    }
  }
}

/*!
  Apply the Canny edge operator on the image \e Isrc and return the resulting
  image \e Ires.

  The following example shows how to use the method:

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>

int main()
{
  // Constants for the Canny operator.
  const unsigned int gaussianFilterSize = 5;
  const double thresholdCanny = 15;
  const unsigned int apertureSobel = 3;

  // Image for the Canny edge operator
  vpImage<unsigned char> Isrc;
  vpImage<unsigned char> Icanny;

  // First grab the source image Isrc.

  // Apply the Canny edge operator and set the Icanny image.
  vpImageFilter::canny(Isrc, Icanny, gaussianFilterSize, thresholdCanny, apertureSobel);
  return (0);
}
  \endcode

  \param Isrc : Image to apply the Canny edge detector to.
  \param Ires : Filtered image (255 means an edge, 0 otherwise).
  \param gaussianFilterSize : The size of the mask of the Gaussian filter to
  apply (an odd number).
  \param thresholdCanny : The threshold for the Canny operator. Only value
  greater than this value are marked as an edge).
  \param apertureSobel : Size of the mask for the Sobel operator (odd number).
*/
void vpImageFilter::canny(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Ires,
  unsigned int gaussianFilterSize, double thresholdCanny, unsigned int apertureSobel)
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
  cv::Mat img_cvmat, edges_cvmat;
  vpImageConvert::convert(Isrc, img_cvmat);
  cv::GaussianBlur(img_cvmat, img_cvmat, cv::Size((int)gaussianFilterSize, (int)gaussianFilterSize), 0, 0);
  cv::Canny(img_cvmat, edges_cvmat, thresholdCanny, thresholdCanny, (int)apertureSobel);
  vpImageConvert::convert(edges_cvmat, Ires);
#else
  (void)apertureSobel;
  vpCannyEdgeDetection edgeDetector(gaussianFilterSize, 0.1, thresholdCanny * 0.5, thresholdCanny);
  Ires = edgeDetector.detect(Isrc);
#endif
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
void vpImageFilter::filter<float>(const vpImage<unsigned char> &I, vpImage<float> &GI, const float *filter,
  unsigned int size);

template<>
void vpImageFilter::filter<double>(const vpImage<unsigned char> &I, vpImage<double> &GI, const double *filter,
  unsigned int size);

template<>
void vpImageFilter::filter<float>(const vpImage<float> &I, vpImage<float> &GI, const float *filter, unsigned int size);

template<>
void vpImageFilter::filter<double>(const vpImage<double> &I, vpImage<double> &GI, const double *filter, unsigned int size);

template<>
void vpImageFilter::filterX<float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *filter,
  unsigned int size);

template<>
void vpImageFilter::filterX<double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter,
  unsigned int size);
/**
 * \endcond
 */

void vpImageFilter::filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size)
{
  dIx.resize(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < (size - 1) / 2; j++) {
      dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderR(I, i, j, filter, size));
      dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderG(I, i, j, filter, size));
      dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderB(I, i, j, filter, size));
    }
    for (unsigned int j = (size - 1) / 2; j < I.getWidth() - (size - 1) / 2; j++) {
      dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXR(I, i, j, filter, size));
      dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXG(I, i, j, filter, size));
      dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXB(I, i, j, filter, size));
    }
    for (unsigned int j = I.getWidth() - (size - 1) / 2; j < I.getWidth(); j++) {
      dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXRightBorderR(I, i, j, filter, size));
      dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXRightBorderG(I, i, j, filter, size));
      dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXRightBorderB(I, i, j, filter, size));
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
void vpImageFilter::filterX<float>(const vpImage<float> &I, vpImage<float> &dIx, const float *filter, unsigned int size);

template<>
void vpImageFilter::filterX<double>(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size);

template<>
void vpImageFilter::filterY<float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *filter,
  unsigned int size);

template<>
void vpImageFilter::filterY<double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *filter,
  unsigned int size);
/**
 * \endcond
 */

void vpImageFilter::filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIy, const double *filter, unsigned int size)
{
  dIy.resize(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < (size - 1) / 2; i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYTopBorderR(I, i, j, filter, size));
      dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYTopBorderG(I, i, j, filter, size));
      dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYTopBorderB(I, i, j, filter, size));
    }
  }
  for (unsigned int i = (size - 1) / 2; i < I.getHeight() - (size - 1) / 2; i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYR(I, i, j, filter, size));
      dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYG(I, i, j, filter, size));
      dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYB(I, i, j, filter, size));
    }
  }
  for (unsigned int i = I.getHeight() - (size - 1) / 2; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderR(I, i, j, filter, size));
      dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderG(I, i, j, filter, size));
      dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderB(I, i, j, filter, size));
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
void vpImageFilter::filterY<float>(const vpImage<float> &I, vpImage<float> &dIy, const float *filter, unsigned int size);

template<>
void vpImageFilter::filterY<double>(const vpImage<double> &I, vpImage<double> &dIy, const double *filter, unsigned int size)
{
  dIy.resize(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < (size - 1) / 2; i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j] = vpImageFilter::filterYTopBorder(I, i, j, filter, size);
    }
  }
  for (unsigned int i = (size - 1) / 2; i < I.getHeight() - (size - 1) / 2; i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j] = vpImageFilter::filterY(I, i, j, filter, size);
    }
  }
  for (unsigned int i = I.getHeight() - (size - 1) / 2; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      dIy[i][j] = vpImageFilter::filterYBottomBorder(I, i, j, filter, size);
    }
  }
}

template<>
void vpImageFilter::gaussianBlur<float>(const vpImage<unsigned char> &I, vpImage<float> &GI, unsigned int size, float sigma,
  bool normalize);

template<>
void vpImageFilter::gaussianBlur<double>(const vpImage<unsigned char> &I, vpImage<double> &GI, unsigned int size, double sigma,
  bool normalize);
/**
 * \endcond
 */

 /*!
   Apply a Gaussian blur to RGB color image.
   \param I : Input image.
   \param GI : Filtered image.
   \param size : Filter size. This value should be odd.
   \param sigma : Gaussian standard deviation. If it is equal to zero or
   negative, it is computed from filter size as sigma = (size-1)/6.
   \param normalize : Flag indicating whether to normalize the filter coefficients or not.

   \sa getGaussianKernel() to know which kernel is used.
  */
void vpImageFilter::gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size, double sigma,
  bool normalize)
{
  double *fg = new double[(size + 1) / 2];
  vpImageFilter::getGaussianKernel(fg, size, sigma, normalize);
  vpImage<vpRGBa> GIx;
  vpImageFilter::filterX(I, GIx, fg, size);
  vpImageFilter::filterY(GIx, GI, fg, size);
  GIx.destroy();
  delete[] fg;
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
void vpImageFilter::gaussianBlur<float>(const vpImage<float> &I, vpImage<float> &GI, unsigned int size, float sigma,
  bool normalize);

template<>
void vpImageFilter::gaussianBlur<double>(const vpImage<double> &I, vpImage<double> &GI, unsigned int size, double sigma,
  bool normalize);

template<>
void vpImageFilter::getGaussianKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template <>
void vpImageFilter::getGaussianDerivativeKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template <>
void vpImageFilter::getGaussianDerivativeKernel<double>(double *filter, unsigned int size, double sigma, bool normalize);

template<>
void vpImageFilter::getGradX<float>(const vpImage<unsigned char> &I, vpImage<float> &dIx);

template<>
void vpImageFilter::getGradX<double>(const vpImage<unsigned char> &I, vpImage<double> &dIx);

template<>
void vpImageFilter::getGradY<float>(const vpImage<unsigned char> &I, vpImage<float> &dIy);

template<>
void vpImageFilter::getGradY<double>(const vpImage<unsigned char> &I, vpImage<double> &dIy);

template<>
void vpImageFilter::getGradX<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *filter, unsigned int size);

template<>
void vpImageFilter::getGradX<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter, unsigned int size);

template<>
void vpImageFilter::getGradX<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *filter, unsigned int size);

template<>
void vpImageFilter::getGradX<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size);

template<>
void vpImageFilter::getGradY<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *filter, unsigned int size);

template<>
void vpImageFilter::getGradY<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *filter, unsigned int size);

template<>
void vpImageFilter::getGradY<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *filter, unsigned int size);

template<>
void vpImageFilter::getGradY<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *filter, unsigned int size);

template<>
void vpImageFilter::getGradXGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *gaussianKernel,
  const float *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradXGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *gaussianKernel,
  const double *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradXGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *gaussianKernel,
  const float *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradXGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *gaussianKernel,
  const double *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradYGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *gaussianKernel,
  const float *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradYGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *gaussianKernel,
  const double *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradYGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *gaussianKernel,
  const float *gaussianDerivativeKernel, unsigned int size);

template<>
void vpImageFilter::getGradYGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *gaussianKernel,
  const double *gaussianDerivativeKernel, unsigned int size);
/**
 * \endcond
 */

 // operation pour pyramide gaussienne
void vpImageFilter::getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
  vpImage<unsigned char> GIx;
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat imgsrc, imgdest;
  vpImageConvert::convert(I, imgsrc);
  cv::pyrDown(imgsrc, imgdest, cv::Size((int)I.getWidth() / 2, (int)I.getHeight() / 2));
  vpImageConvert::convert(imgdest, GI);
#else
  cv::Mat imgsrc, imgdest;
  vpImageConvert::convert(I, imgsrc);
  cv::pyrDown(imgsrc, imgdest, cvSize((int)I.getWidth() / 2, (int)I.getHeight() / 2));
  vpImageConvert::convert(imgdest, GI);
#endif
#else
  vpImageFilter::getGaussXPyramidal(I, GIx);
  vpImageFilter::getGaussYPyramidal(GIx, GI);
#endif
}

void vpImageFilter::getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
#if 0
  GI.resize(I.getHeight(), (int)((I.getWidth() + 1.) / 2.));
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    GI[i][0] = I[i][0];
    for (unsigned int j = 1; j < ((I.getWidth() + 1.) / 2.) - 1; j++) {
      GI[i][j] = vpImageFilter::filterGaussXPyramidal(I, i, 2 * j);
    }
    GI[i][(int)((I.getWidth() + 1.) / 2.) - 1] = I[i][2 * ((int)((I.getWidth() + 1.) / 2.) - 1)];
  }
#else
  unsigned int w = I.getWidth() / 2;

  GI.resize(I.getHeight(), w);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    GI[i][0] = I[i][0];
    for (unsigned int j = 1; j < w - 1; j++) {
      GI[i][j] = vpImageFilter::filterGaussXPyramidal(I, i, 2 * j);
    }
    GI[i][w - 1] = I[i][2 * w - 1];
  }

#endif
}
void vpImageFilter::getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{

#ifdef ORIG
  GI.resize((int)((I.getHeight() + 1.) / 2.), I.getWidth());
  for (unsigned int j = 0; j < I.getWidth(); j++) {
    GI[0][j] = I[0][j];
    for (unsigned int i = 1; i < ((I.getHeight() + 1.) / 2.) - 1; i++) {
      GI[i][j] = vpImageFilter::filterGaussYPyramidal(I, 2 * i, j);
    }
    GI[(int)((I.getHeight() + 1.) / 2.) - 1][j] = I[2 * ((int)((I.getHeight() + 1.) / 2.) - 1)][j];
  }

#else
  unsigned int h = I.getHeight() / 2;

  GI.resize(h, I.getWidth());
  for (unsigned int j = 0; j < I.getWidth(); j++) {
    GI[0][j] = I[0][j];
    for (unsigned int i = 1; i < h - 1; i++) {
      GI[i][j] = vpImageFilter::filterGaussYPyramidal(I, 2 * i, j);
    }
    GI[h - 1][j] = I[2 * h - 1][j];
  }
#endif
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template<>
double vpImageFilter::getSobelKernelX<double>(double *filter, unsigned int size);

template<>
float vpImageFilter::getSobelKernelX<float>(float *filter, unsigned int size);

template<>
double vpImageFilter::getSobelKernelY<double>(double *filter, unsigned int size);

template<>
float vpImageFilter::getSobelKernelY<float>(float *filter, unsigned int size);
/**
 * \endcond
 */