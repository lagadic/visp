/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 */

#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRGBa.h>

BEGIN_VISP_NAMESPACE
namespace vpImageFilter
{
/**
* \cond DO_NOT_DOCUMENT
*/
template
void filter<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &If,
                                                 const vpArray2D<float> &M, bool convolve, const vpImage<bool> *p_mask);

template
void filter<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &If,
                                                  const vpArray2D<double> &M, bool convolve, const vpImage<bool> *p_mask);

template
void filter<float, float>(const vpImage<float> &I, vpImage<float> &Iu, vpImage<float> &Iv,
                                         const vpArray2D<float> &M, bool convolve, const vpImage<bool> *p_mask);

template
void filter<double, double>(const vpImage<double> &I, vpImage<double> &Iu, vpImage<double> &Iv,
                                           const vpArray2D<double> &M, bool convolve, const vpImage<bool> *p_mask);

template
void separableFilter<unsigned char, float, float>(const vpImage<unsigned char> &I, vpImage<float> &GI, const vpArray2D<float> &filter,
                                                 const vpImage<bool> *p_mask);

template
void separableFilter<unsigned char, double, double>(const vpImage<unsigned char> &I, vpImage<double> &GI, const vpArray2D<double> &filter,
                                                  const vpImage<bool> *p_mask);

template
void separableFilter<float, float, float>(const vpImage<float> &I, vpImage<float> &GI, const vpArray2D<float> &filter,
                                         const vpImage<bool> *p_mask);

template
void separableFilter<double, double, double>(const vpImage<double> &I, vpImage<double> &GI, const vpArray2D<double> &filter,
                                           const vpImage<bool> *p_mask);
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
    1 & 0 & -1
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
    1 & 0 & -1
    \end{matrix}
    \right ] \ast I
    \right )
  \f]
  Using two separable kernels reduce the number of operations and can be
  faster for large kernels.

  \param[in] I : Image to filter
  \param[out] If : Filtered image.
  \param[in] kernelH : Separable kernel (performed first).
  \param[in] kernelV : Separable kernel (performed last).
  \note Only pixels in the input image fully covered by the kernel are considered.
*/
void separableFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
                     const vpColVector &kernelV)
{
  const unsigned int size = kernelH.size(), sizeV = kernelV.size();
  const unsigned int widthI = I.getWidth(), heightI = I.getHeight();
  const unsigned int half_size = size / 2;

  If.resize(heightI, widthI, 0.0);
  vpImage<double> I_filter(heightI, widthI, 0.0);

  for (unsigned int i = 0; i < heightI; ++i) {
    for (unsigned int j = half_size; j < (widthI - half_size); ++j) {
      double conv = 0.0;
      for (unsigned int a = 0; a < size; ++a) {
        conv += kernelH[a] * static_cast<double>(I[i][(j + half_size) - a]);
      }

      I_filter[i][j] = conv;
    }
  }

  for (unsigned int i = half_size; i < (heightI - half_size); ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      double conv = 0.0;
      for (unsigned int a = 0; a < sizeV; ++a) {
        conv += kernelV[a] * I_filter[(i + half_size) - a][j];
      }

      If[i][j] = conv;
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void filterX<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const vpArray2D<float> &filter,
                                                  const vpImage<bool> *p_mask);

template
void filterX<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const vpArray2D<double> &filter,
                                                   const vpImage<bool> *p_mask);

template
void filterX<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void filterX<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);
/**
 * \endcond
 */

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const vpArray2D<double> &filter,
                            const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1J = filter.getCols();
  const unsigned int stop2J = widthI - filter.getCols();
  detail::resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIx);

  for (unsigned int i = 0; i < heightI; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(detail::filterXLeftBorderR(I, i, j, filter));
        dIx[i][j].G = static_cast<unsigned char>(detail::filterXLeftBorderG(I, i, j, filter));
        dIx[i][j].B = static_cast<unsigned char>(detail::filterXLeftBorderB(I, i, j, filter));
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(detail::filterXR(I, i, j, filter));
        dIx[i][j].G = static_cast<unsigned char>(detail::filterXG(I, i, j, filter));
        dIx[i][j].B = static_cast<unsigned char>(detail::filterXB(I, i, j, filter));
      }
    }
    for (unsigned int j = stop2J; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(detail::filterXRightBorderR(I, i, j, filter));
        dIx[i][j].G = static_cast<unsigned char>(detail::filterXRightBorderG(I, i, j, filter));
        dIx[i][j].B = static_cast<unsigned char>(detail::filterXRightBorderB(I, i, j, filter));
      }
    }
  }
}
#endif

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void filterY<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const vpArray2D<float> &filter,
                                                  const vpImage<bool> *p_mask);

template
void filterY<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const vpArray2D<double> &filter,
                                                   const vpImage<bool> *p_mask);

template
void filterY<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void filterY<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);
/**
 * \endcond
 */

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIy, const vpArray2D<double> &filter,
                            const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1I = filter.getCols();
  const unsigned int stop2I = heightI - filter.getCols();
  detail::resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(detail::filterYTopBorderR(I, i, j, filter));
        dIy[i][j].G = static_cast<unsigned char>(detail::filterYTopBorderG(I, i, j, filter));
        dIy[i][j].B = static_cast<unsigned char>(detail::filterYTopBorderB(I, i, j, filter));
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(detail::filterYR(I, i, j, filter));
        dIy[i][j].G = static_cast<unsigned char>(detail::filterYG(I, i, j, filter));
        dIy[i][j].B = static_cast<unsigned char>(detail::filterYB(I, i, j, filter));
      }
    }
  }
  for (unsigned int i = stop2I; i < heightI; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
            // pixels for which the mask is true otherwise
      bool computeVal = detail::checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(detail::filterYBottomBorderR(I, i, j, filter));
        dIy[i][j].G = static_cast<unsigned char>(detail::filterYBottomBorderG(I, i, j, filter));
        dIy[i][j].B = static_cast<unsigned char>(detail::filterYBottomBorderB(I, i, j, filter));
      }
    }
  }
}
#endif

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void gaussianBlur<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &GI, unsigned int size, float sigma, bool normalize, const vpImage<bool> *p_mask);

template
void gaussianBlur<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &GI, unsigned int size, double sigma, bool normalize, const vpImage<bool> *p_mask);

template
void gaussianBlur<float, float>(const vpImage<float> &I, vpImage<float> &GI, unsigned int size, float sigma, bool normalize, const vpImage<bool> *p_mask);

template
void gaussianBlur<double, double>(const vpImage<double> &I, vpImage<double> &GI, unsigned int size, double sigma, bool normalize, const vpImage<bool> *p_mask);
/**
 * \endcond
 */

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
/*!
  Apply a Gaussian blur to RGB color image.
  \param[in] I : Input image.
  \param[out] GI : Filtered image.
  \param[in] size : Filter size. This value should be odd.
  \param[in] sigma : Gaussian standard deviation. If it is equal to zero or
  negative, it is computed from filter size as sigma = (size-1)/6.
  \param[in] normalize : Flag indicating whether to normalize the filter coefficients or not.
  \param[in] p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).

  \sa getGaussianKernel() to know which kernel is used.
 */
void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size, double sigma, bool normalize,
                                 const vpImage<bool> *p_mask)
{
  if (size == 0 || size-1 > I.getWidth() || size-1 > I.getHeight()) {
    std::ostringstream oss;
    oss << "Image size (" << I.getWidth() << "x" << I.getHeight() << ") is too small for the Gaussian kernel ("
      << "size=" << size << "), min size is " << (size-1);
    throw vpException(vpException::dimensionError, oss.str());
  }

  vpArray2D<double> fg(1, (size + 1) / 2);
  getGaussianKernel(fg, size, sigma, normalize);
  vpImage<vpRGBa> GIx;
  filterX(I, GIx, fg, p_mask);
  filterY(GIx, GI, fg, p_mask);
  GIx.destroy();
}
#endif

/**
 * \cond DO_NOT_DOCUMENT
 */
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
template
void getGaussianKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template
void getGaussianDerivativeKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template
void getGaussianDerivativeKernel<double>(double *filter, unsigned int size, double sigma, bool normalize);

template
void getGradXGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *gaussianKernel,
                                                          const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradXGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *gaussianKernel,
                                                           const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradXGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *gaussianKernel,
                                                  const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradXGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *gaussianKernel,
                                                    const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradYGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *gaussianKernel,
                                                          const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradYGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *gaussianKernel,
                                                           const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradYGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *gaussianKernel,
                                                  const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void getGradYGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *gaussianKernel,
                                                    const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);
#endif

template
void getGradX<float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const vpImage<bool> *p_mask);

template
void getGradX<double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const vpImage<bool> *p_mask);

template
void getGradY<float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const vpImage<bool> *p_mask);

template
void getGradY<double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const vpImage<bool> *p_mask);

template
void getGradX<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void getGradX<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);

template
void getGradX<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void getGradX<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);

template
void getGradY<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void getGradY<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);

template
void getGradY<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const vpArray2D<float> &filter, const vpImage<bool> *p_mask);

template
void getGradY<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const vpArray2D<double> &filter, const vpImage<bool> *p_mask);

/**
 * \endcond
 */

 // Operation for Gaussian pyramid
void getGaussPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
  vpImage<unsigned char> GIx;
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat imgsrc, imgdest;
  vpImageConvert::convert(I, imgsrc);
  cv::pyrDown(imgsrc, imgdest, cv::Size(static_cast<int>(I.getWidth()) / 2, static_cast<int>(I.getHeight()) / 2));
  vpImageConvert::convert(imgdest, GI);
#else
  cv::Mat imgsrc, imgdest;
  vpImageConvert::convert(I, imgsrc);
  cv::pyrDown(imgsrc, imgdest, cvSize(static_cast<int>(I.getWidth()) / 2, static_cast<int>(I.getHeight()) / 2));
  vpImageConvert::convert(imgdest, GI);
#endif
#else
  getGaussXPyramidal(I, GIx);
  getGaussYPyramidal(GIx, GI);
#endif
}

void getGaussXPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
  const unsigned int w = I.getWidth() / 2;
  const unsigned int height = I.getHeight();
  const unsigned int val_2 = 2;

  GI.resize(height, w);
  for (unsigned int i = 0; i < height; ++i) {
    GI[i][0] = I[i][0];
    for (unsigned int j = 1; j < (w - 1); ++j) {
      GI[i][j] = detail::filterGaussXPyramidal(I, i, val_2 * j);
    }
    GI[i][w - 1] = I[i][(val_2 * w) - 1];
  }
}

void getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
  const unsigned int h = I.getHeight() / 2;
  const unsigned int width = I.getWidth();
  const unsigned int val_2 = 2;

  GI.resize(h, width);
  for (unsigned int j = 0; j < width; ++j) {
    GI[0][j] = I[0][j];
    for (unsigned int i = 1; i < (h - 1); ++i) {
      GI[i][j] = detail::filterGaussYPyramidal(I, val_2 * i, j);
    }
    GI[h - 1][j] = I[(val_2 * h) - 1][j];
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
template
double getSobelKernelX<double>(double *filter, unsigned int size);

template
float getSobelKernelX<float>(float *filter, unsigned int size);

template
double getSobelKernelY<double>(double *filter, unsigned int size);

template
float getSobelKernelY<float>(float *filter, unsigned int size);
#endif
/**
 * \endcond
 */


#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
 /**
  * \brief Calculates the median value of a single channel.
  * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
  * \param[in] channel : Single channel image in OpenCV format.
  */
float median(const cv::Mat &channel)
{
  float m = (channel.rows * channel.cols) / 2.f;
  int bin = 0;
  float med = -1.0f;

  int histSize = 256;
  float range[] = { 0, 256 };
  const float *histRange = { range };
  bool uniform = true;
  bool accumulate = false;
  cv::Mat hist;
  cv::calcHist(&channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

  int i = 0;
  while ((i < histSize) && (med < 0.0f)) {
    bin += cvRound(hist.at<float>(i));
    if ((bin > m) && (med < 0.0f)) {
      med = static_cast<float>(i);
    }
    ++i;
  }

  return med;
}

/**
 * \brief Calculates the median value of a single channel.
 * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
 * \param[in] Isrc : Gray-level image in ViSP format.
 * \return Gray level image median value.
 * \sa \ref median() "median(const cv::Mat)"
 */
float median(const vpImage<unsigned char> &Isrc)
{
  cv::Mat cv_I;
  vpImageConvert::convert(Isrc, cv_I);
  return median(cv_I);
}

/**
 * \brief Calculates the median value of a vpRGBa image.
 * The result is ordered in RGB format.
 * \param[in] Isrc : RGB image in ViSP format. Alpha channel is ignored.
 * \return std::vector<float> meds such as meds[0] = red-channel-median, meds[1] = green-channel-median
 * and meds[2] = blue-channel-median.
 * \sa \ref median() "median(const cv::Mat)"
 */
std::vector<float> median(const vpImage<vpRGBa> &Isrc)
{
  cv::Mat cv_I_bgr;
  vpImageConvert::convert(Isrc, cv_I_bgr);
  std::vector<cv::Mat> channels;
  cv::split(cv_I_bgr, channels);
  std::vector<float> meds(3);
  const int orderMeds[] = { 2, 1, 0 }; // To keep the order R, G, B
  const int orderCvChannels[] = { 0, 1, 2 }; // Because the order of the cv::Mat is B, G, R
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    meds[static_cast<unsigned int>(orderMeds[static_cast<unsigned int>(i)])] =
      median(channels[static_cast<unsigned int>(orderCvChannels[static_cast<unsigned int>(i)])]);
  }
  return meds;
}


/**
 * \brief Compute the partial derivatives (i.e. horizontal and vertical gradients) of the input image.
 *
 * \param[in] cv_I The input image we want the partial derivatives.
 * \param[out] cv_dIx The horizontal partial derivative, i.e. horizontal gradient.
 * \param[out] cv_dIy The vertical partial derivative, i.e. vertical gradient.
 * \param[in] computeDx Indicate if we must compute the horizontal gradient.
 * \param[in] computeDy Indicate if we must compute  the vertical gradient.
 * \param[in] normalize Indicate if we must normalize the gradient filters.
 * \param[in] gaussianKernelSize The size of the kernel of the Gaussian filter used to blur the image.
 * \param[in] gaussianStdev The standard deviation of the Gaussian filter used to blur the image.
 * If it is non-positive, it is computed from kernel size (`gaussianKernelSize` parameter) as
 * \f$\sigma = 0.3*((gaussianKernelSize-1)*0.5 - 1) + 0.8\f$.
 * \param[in] apertureGradient The size of the kernel of the gradient filter.
 * \param[in] filteringType The type of filters to apply to compute the gradients.
 */
void computePartialDerivatives(const cv::Mat &cv_I,
                                              cv::Mat &cv_dIx, cv::Mat &cv_dIy,
                                              const bool &computeDx, const bool &computeDy, const bool &normalize,
                                              const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                              const unsigned int &apertureGradient,
                                              const vpCannyFilteringAndGradientType &filteringType)
{
  if ((filteringType == CANNY_GBLUR_SCHARR_FILTERING)
      || (filteringType == CANNY_GBLUR_SOBEL_FILTERING)) {
    cv::Mat img_blur;
    // Apply Gaussian blur to the image
    cv::Size gsz(static_cast<int>(gaussianKernelSize), static_cast<int>(gaussianKernelSize));
    cv::GaussianBlur(cv_I, img_blur, gsz, gaussianStdev);

    // Compute the gradient of the blurred image
    if (filteringType == CANNY_GBLUR_SOBEL_FILTERING) {
      double scale = 1.;
      if (normalize) {
        scale = 1. / 8.;
        if (apertureGradient > 3) {
          scale *= std::pow(1. / 2., (static_cast<double>(apertureGradient) * 2. - 3.)); // 1 / 2^(2 x ksize - dx - dy -2) with ksize =apertureGradient and dx xor dy = 1
        }
      }
      if (computeDx) {
        cv::Sobel(img_blur, cv_dIx, CV_16S, 1, 0, static_cast<int>(apertureGradient), scale, 0., cv::BORDER_REPLICATE);
      }
      if (computeDy) {
        cv::Sobel(img_blur, cv_dIy, CV_16S, 0, 1, static_cast<int>(apertureGradient), scale, 0., cv::BORDER_REPLICATE);
      }
    }
    else if (filteringType == CANNY_GBLUR_SCHARR_FILTERING) {
      double scale = 1.;
      if (normalize) {
        scale = 1. / 32.;
      }
      if (computeDx) {
        cv::Scharr(img_blur, cv_dIx, CV_16S, 1, 0, scale);
      }
      if (computeDy) {
        cv::Scharr(img_blur, cv_dIy, CV_16S, 0, 1, scale);
      }
    }
  }
}
#endif

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void computePartialDerivatives<unsigned char, float>(const vpImage<unsigned char> &I,
                                                                    vpImage<float> &dIx, vpImage<float> &dIy,
                                                                    const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                                    const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                                    const unsigned int &apertureGradient,
                                                                    const vpCannyFilteringAndGradientType &filteringType,
                                                                    const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void computePartialDerivatives<unsigned char, double>(const vpImage<unsigned char> &I,
                                                                     vpImage<double> &dIx, vpImage<double> &dIy,
                                                                     const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                                     const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                                     const unsigned int &apertureGradient,
                                                                     const vpCannyFilteringAndGradientType &filteringType,
                                                                     const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void computePartialDerivatives<float, float>(const vpImage<float> &I,
                                                            vpImage<float> &dIx, vpImage<float> &dIy,
                                                            const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                            const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                            const unsigned int &apertureGradient,
                                                            const vpCannyFilteringAndGradientType &filteringType,
                                                            const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void computePartialDerivatives<float, double>(const vpImage<float> &I,
                                                             vpImage<double> &dIx, vpImage<double> &dIy,
                                                             const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                             const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                             const unsigned int &apertureGradient,
                                                             const vpCannyFilteringAndGradientType &filteringType,
                                                             const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void computePartialDerivatives<double, float>(const vpImage<double> &I,
                                                             vpImage<float> &dIx, vpImage<float> &dIy,
                                                             const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                             const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                             const unsigned int &apertureGradient,
                                                             const vpCannyFilteringAndGradientType &filteringType,
                                                             const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void computePartialDerivatives<double, double>(const vpImage<double> &I,
                                                              vpImage<double> &dIx, vpImage<double> &dIy,
                                                              const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                              const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                              const unsigned int &apertureGradient,
                                                              const vpCannyFilteringAndGradientType &filteringType,
                                                              const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
float computeCannyThreshold<double>(const vpImage<unsigned char> &I, float &lowerThresh,
                                                   const vpImage<double> *p_dIx, const vpImage<double> *p_dIy,
                                                   const unsigned int &gaussianKernelSize,
                                                   const double &gaussianStdev, const unsigned int &apertureGradient,
                                                   const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                                   const vpCannyFilteringAndGradientType &filteringType,
                                                   const vpImage<bool> *p_mask);

template
float computeCannyThreshold<float>(const vpImage<unsigned char> &I, float &lowerThresh,
                                                  const vpImage<float> *p_dIx, const vpImage<float> *p_dIy,
                                                  const unsigned int &gaussianKernelSize,
                                                  const float &gaussianStdev, const unsigned int &apertureGradient,
                                                  const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                                  const vpCannyFilteringAndGradientType &filteringType,
                                                  const vpImage<bool> *p_mask);
/**
 * \endcond
*/
}
END_VISP_NAMESPACE
