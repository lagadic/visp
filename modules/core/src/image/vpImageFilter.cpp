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

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
/**
 * \brief Calculates the median value of a single channel
 * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
 * \param[in] channel Single channel image in OpenCV format.
 */
double vpImageFilter::median(const cv::Mat &channel)
{
  double m = (channel.rows * channel.cols) / 2;
  int bin = 0;
  double med = -1.0;

  int histSize = 256;
  float range[] = { 0, 256 };
  const float *histRange = { range };
  bool uniform = true;
  bool accumulate = false;
  cv::Mat hist;
  cv::calcHist(&channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

  for (int i = 0; i < histSize && med < 0.0; ++i) {
    bin += cvRound(hist.at<float>(i));
    if (bin > m && med < 0.0)
      med = i;
  }

  return med;
}

/**
 * \brief Calculates the median value of a single channel
 * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
 * \param[in] Isrc Gray-level image in ViSP format.
 * \sa \ref vpImageFilter::median() "vpImageFilter::median(const cv::Mat)"
 */
double vpImageFilter::median(const vpImage<unsigned char> &Isrc)
{
  cv::Mat cv_I;
  vpImageConvert::convert(Isrc, cv_I);
  return median(cv_I);
}

/**
 * \brief Calculates the median value of a single channel
 * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
 * \param[in] Isrc RGB image in ViSP format. Alpha channel is ignored.
 * \sa \ref vpImageFilter::median() "vpImageFilter::median(const cv::Mat)"
 */
std::vector<double> vpImageFilter::median(const vpImage<vpRGBa> &Isrc)
{
  cv::Mat cv_I;
  vpImageConvert::convert(Isrc, cv_I);
  std::vector<cv::Mat> channels;
  cv::split(cv_I, channels);
  std::vector<double> meds;
  for (unsigned char i = 0; i < 3; i++) {
    meds.push_back(median(channels[i]));
  }
  return meds;
}

/**
 * \brief Compute the upper Canny edge filter threshold.
 *
 * \param[in] cv_I The image, in cv format.
 * \param[in] p_cv_blur If different from nullptr, must contain a blurred version of cv_I.
 * \param[out] lowerThresh The lower threshold for the Canny edge filter.
 * \return double The upper Canny edge filter threshold.
 */
double computeCannyThreshold(const cv::Mat &cv_I, const cv::Mat *p_cv_blur, double &lowerThresh)
{
  cv::Mat cv_I_blur;
  if (p_cv_blur != nullptr) {
    cv_I_blur = *p_cv_blur;
  }
  else {
    cv::GaussianBlur(cv_I, cv_I_blur, cv::Size(9, 9), 2, 2);
  }

  // Subsample image to reach a 256 x 256 size
  int req_size = 256;
  int orig_size = std::min(static_cast<int>(cv_I.rows), static_cast<int>(cv_I.cols));
  int scale_down = std::max(1, static_cast<int>(orig_size / req_size));
  cv::Mat cv_I_scaled_down;
  resize(cv_I_blur, cv_I_scaled_down, cv::Size(), scale_down, scale_down, cv::INTER_NEAREST);

  double median_pix = vpImageFilter::median(cv_I_scaled_down);
  double lower = std::max(0., 0.7 * median_pix);
  double upper = std::min(255., 1.3 * median_pix);
  upper = std::max(1., upper);
  lowerThresh = lower;
  return upper;
}

/**
 * \brief Compute the upper Canny edge filter threshold.
 *
 * \param[in] I The gray-scale image, in ViSP format.
 * \return double The upper Canny edge filter threshold.
 */
double computeCannyThreshold(const vpImage<unsigned char> &I, double &lowerThresh)
{
  cv::Mat cv_I;
  vpImageConvert::convert(I, cv_I);
  return computeCannyThreshold(cv_I, nullptr, lowerThresh);
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
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
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
#endif
  return (0);
}
  \endcode

  \param Isrc : Image to apply the Canny edge detector to.
  \param Ires : Filtered image (255 means an edge, 0 otherwise).
  \param gaussianFilterSize : The size of the mask of the Gaussian filter to
  apply (an odd number).
  \param thresholdCanny : The upper threshold for the Canny operator. Only value
  greater than this value are marked as an edge. If negative, it will be automatically
  computed, along with the lower threshold. Otherwise, the lower threshold will be set to one third
  of the thresholdCanny .
  \param apertureSobel : Size of the mask for the Sobel operator (odd number).
*/
void vpImageFilter::canny(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Ires,
  unsigned int gaussianFilterSize, double thresholdCanny, unsigned int apertureSobel)
{
  cv::Mat img_cvmat, cv_I_blur, edges_cvmat;
  vpImageConvert::convert(Isrc, img_cvmat);
  cv::GaussianBlur(img_cvmat, cv_I_blur, cv::Size((int)gaussianFilterSize, (int)gaussianFilterSize), 0, 0);
  double upperCannyThresh = thresholdCanny;
  double lowerCannyThresh = thresholdCanny / 3.;
  if (upperCannyThresh < 0) {
    upperCannyThresh = computeCannyThreshold(img_cvmat, &cv_I_blur, lowerCannyThresh);
  }
  cv::Canny(cv_I_blur, edges_cvmat, lowerCannyThresh, upperCannyThresh, (int)apertureSobel);
  vpImageConvert::convert(edges_cvmat, Ires);
}
#endif

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