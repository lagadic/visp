/*
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
 */

#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRGBa.h>

 /**
  * \brief Get the list of available vpCannyBackendType.
  *
  * \param[in] pref The prefix of the list.
  * \param[in] sep The separator between two elements of the list.
  * \param[in] suf The suffix of the list.
  * \return std::string The list of available items.
  */
std::string vpImageFilter::vpCannyBackendTypeList(const std::string &pref, const std::string &sep,
                                                  const std::string &suf)
{
  std::string list(pref);
  for (unsigned int i = 0; i < (vpCannyBackendType::CANNY_COUNT_BACKEND - 1); ++i) {
    vpCannyBackendType type = static_cast<vpCannyBackendType>(i);
    list += vpCannyBackendTypeToString(type);
    list += sep;
  }
  vpCannyBackendType type = static_cast<vpCannyBackendType>(vpCannyBackendType::CANNY_COUNT_BACKEND - 1);
  list += vpCannyBackendTypeToString(type);
  list += suf;
  return list;
}

/**
 * \brief Cast a \b vpImageFilter::vpCannyBackendTypeToString into a string, to know its name.
 *
 * \param[in] type The type that must be casted into a string.
 * \return std::string The corresponding name.
 */
std::string vpImageFilter::vpCannyBackendTypeToString(const vpImageFilter::vpCannyBackendType &type)
{
  std::string name;
  switch (type) {
  case vpCannyBackendType::CANNY_OPENCV_BACKEND:
    name = "opencv-backend";
    break;
  case vpCannyBackendType::CANNY_VISP_BACKEND:
    name = "visp-backend";
    break;
  case vpCannyBackendType::CANNY_COUNT_BACKEND:
  default:
    return "unknown-backend";
  }
  return name;
}

/**
 * \brief Cast a string into a \b vpImageFilter::vpCannyBackendTypeToString.
 *
 * \param[in] name The name of the backend.
 * \return vpImageFilter::vpCannyBackendTypeToString The corresponding enumeration value.
 */
vpImageFilter::vpCannyBackendType vpImageFilter::vpCannyBackendTypeFromString(const std::string &name)
{
  vpCannyBackendType type(vpCannyBackendType::CANNY_COUNT_BACKEND);
  std::string nameLowerCase = vpIoTools::toLowerCase(name);
  unsigned int count = static_cast<unsigned int>(vpCannyBackendType::CANNY_COUNT_BACKEND);
  bool notFound = true;
  unsigned int i = 0;
  while ((i < count) && notFound) {
    vpCannyBackendType temp = static_cast<vpCannyBackendType>(i);
    if (nameLowerCase == vpCannyBackendTypeToString(temp)) {
      type = temp;
      notFound = false;
    }
    ++i;
  }
  return type;
}

/**
 * \brief Get the list of available vpCannyFilteringAndGradientType.
 *
 * \param[in] pref The prefix of the list.
 * \param[in] sep The separator between two elements of the list.
 * \param[in] suf The suffix of the list.
 * \return std::string The list of available items.
 */
std::string vpImageFilter::vpCannyFilteringAndGradientTypeList(const std::string &pref, const std::string &sep,
                                                               const std::string &suf)
{
  std::string list(pref);
  for (unsigned int i = 0; i < (vpCannyFilteringAndGradientType::CANNY_COUNT_FILTERING - 1); ++i) {
    vpCannyFilteringAndGradientType type = static_cast<vpCannyFilteringAndGradientType>(i);
    list += vpCannyFilteringAndGradientTypeToString(type);
    list += sep;
  }
  vpCannyFilteringAndGradientType type = static_cast<vpCannyFilteringAndGradientType>(CANNY_COUNT_FILTERING - 1);
  list += vpCannyFilteringAndGradientTypeToString(type);
  list += suf;
  return list;
}

/**
 * \brief Cast a \b vpImageFilter::vpCannyFilteringAndGradientType into a string, to know its name.
 *
 * \param[in] type The type that must be casted into a string.
 * \return std::string The corresponding name.
 */
std::string vpImageFilter::vpCannyFilteringAndGradientTypeToString(const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  std::string name;
  switch (type) {
  case vpCannyFilteringAndGradientType::CANNY_GBLUR_SOBEL_FILTERING:
    name = "gaussianblur+sobel-filtering";
    break;
  case vpCannyFilteringAndGradientType::CANNY_GBLUR_SCHARR_FILTERING:
    name = "gaussianblur+scharr-filtering";
    break;
  case vpCannyFilteringAndGradientType::CANNY_COUNT_FILTERING:
  default:
    return "unknown-filtering";
  }
  return name;
}

/**
 * \brief Cast a string into a \b vpImageFilter::vpCannyFilteringAndGradientType.
 *
 * \param[in] name The name of the backend.
 * \return vpImageFilter::vpCannyFilteringAndGradientType The corresponding enumeration value.
 */
vpImageFilter::vpCannyFilteringAndGradientType vpImageFilter::vpCannyFilteringAndGradientTypeFromString(const std::string &name)
{
  vpCannyFilteringAndGradientType type(vpCannyFilteringAndGradientType::CANNY_COUNT_FILTERING);
  std::string nameLowerCase = vpIoTools::toLowerCase(name);
  unsigned int count = static_cast<unsigned int>(vpCannyFilteringAndGradientType::CANNY_COUNT_FILTERING);
  bool notFound = true;
  unsigned int i = 0;
  while ((i < count) && notFound) {
    vpCannyFilteringAndGradientType temp = static_cast<vpCannyFilteringAndGradientType>(i);
    if (nameLowerCase == vpCannyFilteringAndGradientTypeToString(temp)) {
      type = temp;
      notFound = false;
    }
    ++i;
  }
  return type;
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void vpImageFilter::filter<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &If, const vpArray2D<float> &M, bool convolve, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &If, const vpArray2D<double> &M, bool convolve, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<float, float>(const vpImage<float> &I, vpImage<float> &Iu, vpImage<float> &Iv, const vpArray2D<float> &M,
                                         bool convolve, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<double, double>(const vpImage<double> &I, vpImage<double> &Iu, vpImage<double> &Iv, const vpArray2D<double> &M,
                                           bool convolve, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &GI, const float *filter,
                                                 unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &GI, const double *filter,
                                                  unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<float, float>(const vpImage<float> &I, vpImage<float> &GI, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filter<double, double>(const vpImage<double> &I, vpImage<double> &GI, const double *filter, unsigned int size, const vpImage<bool> *p_mask);
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
void vpImageFilter::sepFilter(const vpImage<unsigned char> &I, vpImage<double> &If, const vpColVector &kernelH,
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
        conv += kernelH[a] * static_cast<double>(I[i][j + half_size - a]);
      }

      I_filter[i][j] = conv;
    }
  }

  for (unsigned int i = half_size; i < (heightI - half_size); ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      double conv = 0.0;
      for (unsigned int a = 0; a < sizeV; ++a) {
        conv += kernelV[a] * I_filter[i + half_size - a][j];
      }

      If[i][j] = conv;
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void vpImageFilter::filterX<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *filter,
                                                  unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterX<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter,
                                                   unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterX<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterX<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask);
/**
 * \endcond
 */

void vpImageFilter::filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size,
                            const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = widthI - (size - 1) / 2;
  resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIx);

  for (unsigned int i = 0; i < heightI; i++) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderR(I, i, j, filter, size));
        dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderG(I, i, j, filter, size));
        dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXLeftBorderB(I, i, j, filter, size));
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXR(I, i, j, filter, size));
        dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXG(I, i, j, filter, size));
        dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXB(I, i, j, filter, size));
      }
    }
    for (unsigned int j = stop2J; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIx[i][j].R = static_cast<unsigned char>(vpImageFilter::filterXRightBorderR(I, i, j, filter, size));
        dIx[i][j].G = static_cast<unsigned char>(vpImageFilter::filterXRightBorderG(I, i, j, filter, size));
        dIx[i][j].B = static_cast<unsigned char>(vpImageFilter::filterXRightBorderB(I, i, j, filter, size));
      }
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void vpImageFilter::filterY<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *filter,
                                                  unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterY<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *filter,
                                                   unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterY<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::filterY<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *filter, unsigned int size, const vpImage<bool> *p_mask);
/**
 * \endcond
 */

void vpImageFilter::filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIy, const double *filter, unsigned int size,
                            const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = heightI - (size - 1) / 2;
  resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYTopBorderR(I, i, j, filter, size));
        dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYTopBorderG(I, i, j, filter, size));
        dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYTopBorderB(I, i, j, filter, size));
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYR(I, i, j, filter, size));
        dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYG(I, i, j, filter, size));
        dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYB(I, i, j, filter, size));
      }
    }
  }
  for (unsigned int i = stop2I; i < heightI; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = ((p_mask == nullptr) ? true : (*p_mask)[i][j]);
      if (computeVal) {
        dIy[i][j].R = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderR(I, i, j, filter, size));
        dIy[i][j].G = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderG(I, i, j, filter, size));
        dIy[i][j].B = static_cast<unsigned char>(vpImageFilter::filterYBottomBorderB(I, i, j, filter, size));
      }
    }
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void vpImageFilter::gaussianBlur<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &GI, unsigned int size, float sigma, bool normalize, const vpImage<bool> *p_mask);

template
void vpImageFilter::gaussianBlur<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &GI, unsigned int size, double sigma, bool normalize, const vpImage<bool> *p_mask);

template
void vpImageFilter::gaussianBlur<float, float>(const vpImage<float> &I, vpImage<float> &GI, unsigned int size, float sigma, bool normalize, const vpImage<bool> *p_mask);

template
void vpImageFilter::gaussianBlur<double, double>(const vpImage<double> &I, vpImage<double> &GI, unsigned int size, double sigma, bool normalize, const vpImage<bool> *p_mask);
/**
 * \endcond
*/

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
void vpImageFilter::gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size, double sigma, bool normalize,
                                 const vpImage<bool> *p_mask)
{
  double *fg = new double[(size + 1) / 2];
  vpImageFilter::getGaussianKernel(fg, size, sigma, normalize);
  vpImage<vpRGBa> GIx;
  vpImageFilter::filterX(I, GIx, fg, size, p_mask);
  vpImageFilter::filterY(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
void vpImageFilter::getGaussianKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template
void vpImageFilter::getGaussianDerivativeKernel<float>(float *filter, unsigned int size, float sigma, bool normalize);

template
void vpImageFilter::getGaussianDerivativeKernel<double>(double *filter, unsigned int size, double sigma, bool normalize);

template
void vpImageFilter::getGradX<float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradX<double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradX<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradX<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradX<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradX<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradY<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *filter, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradXGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIx, const float *gaussianKernel,
                                                          const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradXGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIx, const double *gaussianKernel,
                                                           const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradXGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIx, const float *gaussianKernel,
                                                  const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradXGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIx, const double *gaussianKernel,
                                                    const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradYGauss2D<unsigned char, float>(const vpImage<unsigned char> &I, vpImage<float> &dIy, const float *gaussianKernel,
                                                          const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradYGauss2D<unsigned char, double>(const vpImage<unsigned char> &I, vpImage<double> &dIy, const double *gaussianKernel,
                                                           const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradYGauss2D<float, float>(const vpImage<float> &I, vpImage<float> &dIy, const float *gaussianKernel,
                                                  const float *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);

template
void vpImageFilter::getGradYGauss2D<double, double>(const vpImage<double> &I, vpImage<double> &dIy, const double *gaussianKernel,
                                                    const double *gaussianDerivativeKernel, unsigned int size, const vpImage<bool> *p_mask);
/**
 * \endcond
*/

// Operation for Gaussian pyramid
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
  const unsigned int w = I.getWidth() / 2;
  const unsigned int height = I.getHeight();

  GI.resize(height, w);
  for (unsigned int i = 0; i < height; ++i) {
    GI[i][0] = I[i][0];
    for (unsigned int j = 1; j < (w - 1); ++j) {
      GI[i][j] = vpImageFilter::filterGaussXPyramidal(I, i, 2 * j);
    }
    GI[i][w - 1] = I[i][2 * w - 1];
  }
}

void vpImageFilter::getGaussYPyramidal(const vpImage<unsigned char> &I, vpImage<unsigned char> &GI)
{
  const unsigned int h = I.getHeight() / 2;
  const unsigned int width = I.getWidth();

  GI.resize(h, width);
  for (unsigned int j = 0; j < width; ++j) {
    GI[0][j] = I[0][j];
    for (unsigned int i = 1; i < (h - 1); ++i) {
      GI[i][j] = vpImageFilter::filterGaussYPyramidal(I, 2 * i, j);
    }
    GI[h - 1][j] = I[2 * h - 1][j];
  }
}

/**
 * \cond DO_NOT_DOCUMENT
 */
template
double vpImageFilter::getSobelKernelX<double>(double *filter, unsigned int size);

template
float vpImageFilter::getSobelKernelX<float>(float *filter, unsigned int size);

template
double vpImageFilter::getSobelKernelY<double>(double *filter, unsigned int size);

template
float vpImageFilter::getSobelKernelY<float>(float *filter, unsigned int size);
/**
 * \endcond
 */


#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
 /**
  * \brief Calculates the median value of a single channel.
  * The algorithm is based on based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
  * \param[in] channel : Single channel image in OpenCV format.
  */
float vpImageFilter::median(const cv::Mat &channel)
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
 * \sa \ref vpImageFilter::median() "vpImageFilter::median(const cv::Mat)"
 */
float vpImageFilter::median(const vpImage<unsigned char> &Isrc)
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
 * \sa \ref vpImageFilter::median() "vpImageFilter::median(const cv::Mat)"
 */
std::vector<float> vpImageFilter::median(const vpImage<vpRGBa> &Isrc)
{
  cv::Mat cv_I_bgr;
  vpImageConvert::convert(Isrc, cv_I_bgr);
  std::vector<cv::Mat> channels;
  cv::split(cv_I_bgr, channels);
  std::vector<float> meds(3);
  const int orderMeds[] = { 2, 1, 0 }; // To keep the order R, G, B
  const int orderCvChannels[] = { 0, 1, 2 }; // Because the order of the cv::Mat is B, G, R
  for (unsigned int i = 0; i < 3; ++i) {
    meds[orderMeds[i]] = median(channels[orderCvChannels[i]]);
  }
  return meds;
}

/**
 * \brief Compute the upper Canny edge filter threshold, using Gaussian blur + Sobel or + Scharr operators to compute
 * the gradient of the image.
 *
 * \param[in] cv_I : The image, in cv format.
 * \param[in] p_cv_dIx : If different from nullptr, the gradient of cv_I with regard to the horizontal axis.
 * \param[in] p_cv_dIy : If different from nullptr, the gradient of cv_I with regard to the vertical axis.
 * \param[out] lowerThresh : The lower threshold for the Canny edge filter.
 * \param[in] gaussianKernelSize : The size of the mask of the Gaussian filter to apply (an odd number).
 * \param[in] gaussianStdev : The standard deviation of the Gaussian filter to apply.
 * \param[in] apertureGradient : Size of the mask for the Sobel operator (odd number).
 * \param[in] lowerThresholdRatio : The ratio of the upper threshold the lower threshold must be equal to.
 * \param[in] upperThresholdRatio : The ratio of pixels whose absolute gradient Gabs is lower or equal to to define
 * \param[in] filteringType : The gradient filter to apply to compute the gradient, if \b p_cv_dIx and \b p_cv_dIy are
 * nullptr.
 * the upper threshold.
 * \return The upper Canny edge filter threshold.
 */
float vpImageFilter::computeCannyThreshold(const cv::Mat &cv_I, const cv::Mat *p_cv_dIx, const cv::Mat *p_cv_dIy,
                                           float &lowerThresh, const unsigned int &gaussianKernelSize,
                                           const float &gaussianStdev, const unsigned int &apertureGradient,
                                           const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                           const vpImageFilter::vpCannyFilteringAndGradientType &filteringType)
{
  double w = cv_I.cols;
  double h = cv_I.rows;
  int bins = 256;
  cv::Mat dI, dIx, dIy, dIx_abs, dIy_abs;

  if ((p_cv_dIx == nullptr) || (p_cv_dIy == nullptr)) {
    computePartialDerivatives(cv_I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev, apertureGradient,
                              filteringType);
  }
  else {
    dIx = *p_cv_dIx;
    dIy = *p_cv_dIy;
  }

  // Compute the absolute gradient of the blurred image G = |dIx| + |dIy|
  cv::convertScaleAbs(dIx, dIx_abs);
  cv::convertScaleAbs(dIy, dIy_abs);
  cv::addWeighted(dIx_abs, 1, dIy_abs, 1, 0, dI);
  dI.convertTo(dI, CV_8U);

  // Compute the upper threshold from the equalized histogram
  cv::Mat hist;
  const float range[] = { 0.f, 256.f }; // The upper boundary is exclusive
  const float *ranges[] = { range };
  int channels[] = { 0 };
  int dims = 1; // The number of dimensions of the histogram
  int histSize[] = { bins };
  bool uniform = true;
  bool accumulate = false; // Clear the histogram at the beginning of calcHist if false, does not clear it otherwise
  cv::calcHist(&dI, 1, channels, cv::Mat(), hist, dims, histSize, ranges, uniform, accumulate);
  float accu = 0;
  float t = static_cast<float>(upperThresholdRatio * w * h);
  float bon = 0;
  for (int i = 0; i < bins; ++i) {
    float tf = hist.at<float>(i);
    accu = accu + tf;
    if (accu > t) {
      bon = static_cast<float>(i);
      break;
    }
  }
  float upperThresh = std::max(bon, 1.f);
  lowerThresh = lowerThresholdRatio * bon;
  return upperThresh;
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
void vpImageFilter::computePartialDerivatives(const cv::Mat &cv_I,
                                              cv::Mat &cv_dIx, cv::Mat &cv_dIy,
                                              const bool &computeDx, const bool &computeDy, const bool &normalize,
                                              const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                              const unsigned int &apertureGradient,
                                              const vpImageFilter::vpCannyFilteringAndGradientType &filteringType)
{
  if ((filteringType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING)
      || (filteringType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING)) {
    cv::Mat img_blur;
    // Apply Gaussian blur to the image
    cv::Size gsz(gaussianKernelSize, gaussianKernelSize);
    cv::GaussianBlur(cv_I, img_blur, gsz, gaussianStdev);

    // Compute the gradient of the blurred image
    if (filteringType == vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING) {
      double scale = 1.;
      if (normalize) {
        scale = 1. / 8.;
        if (apertureGradient > 3) {
          scale *= std::pow(1. / 2., (static_cast<double>(apertureGradient) * 2. - 3.)); // 1 / 2^(2 x ksize - dx - dy -2) with ksize =apertureGradient and dx xor dy = 1
        }
      }
      if (computeDx) {
        cv::Sobel(img_blur, cv_dIx, CV_16S, 1, 0, apertureGradient, scale, 0., cv::BORDER_REPLICATE);
      }
      if (computeDy) {
        cv::Sobel(img_blur, cv_dIy, CV_16S, 0, 1, apertureGradient, scale, 0., cv::BORDER_REPLICATE);
      }
    }
    else if (filteringType == vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING) {
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
void vpImageFilter::computePartialDerivatives<unsigned char, float>(const vpImage<unsigned char> &I,
                                                                    vpImage<float> &dIx, vpImage<float> &dIy,
                                                                    const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                                    const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                                    const unsigned int &apertureGradient,
                                                                    const vpCannyFilteringAndGradientType &filteringType,
                                                                    const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void vpImageFilter::computePartialDerivatives<unsigned char, double>(const vpImage<unsigned char> &I,
                                                                     vpImage<double> &dIx, vpImage<double> &dIy,
                                                                     const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                                     const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                                     const unsigned int &apertureGradient,
                                                                     const vpCannyFilteringAndGradientType &filteringType,
                                                                     const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void vpImageFilter::computePartialDerivatives<float, float>(const vpImage<float> &I,
                                                            vpImage<float> &dIx, vpImage<float> &dIy,
                                                            const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                            const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                            const unsigned int &apertureGradient,
                                                            const vpCannyFilteringAndGradientType &filteringType,
                                                            const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void vpImageFilter::computePartialDerivatives<float, double>(const vpImage<float> &I,
                                                             vpImage<double> &dIx, vpImage<double> &dIy,
                                                             const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                             const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                             const unsigned int &apertureGradient,
                                                             const vpCannyFilteringAndGradientType &filteringType,
                                                             const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void vpImageFilter::computePartialDerivatives<double, float>(const vpImage<double> &I,
                                                             vpImage<float> &dIx, vpImage<float> &dIy,
                                                             const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                             const unsigned int &gaussianKernelSize, const float &gaussianStdev,
                                                             const unsigned int &apertureGradient,
                                                             const vpCannyFilteringAndGradientType &filteringType,
                                                             const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
void vpImageFilter::computePartialDerivatives<double, double>(const vpImage<double> &I,
                                                              vpImage<double> &dIx, vpImage<double> &dIy,
                                                              const bool &computeDx, const bool &computeDy, const bool &normalize,
                                                              const unsigned int &gaussianKernelSize, const double &gaussianStdev,
                                                              const unsigned int &apertureGradient,
                                                              const vpCannyFilteringAndGradientType &filteringType,
                                                              const vpCannyBackendType &backend, const vpImage<bool> *p_mask);

template
float vpImageFilter::computeCannyThreshold<double>(const vpImage<unsigned char> &I, float &lowerThresh,
                                                   const vpImage<double> *p_dIx, const vpImage<double> *p_dIy,
                                                   const unsigned int &gaussianKernelSize,
                                                   const double &gaussianStdev, const unsigned int &apertureGradient,
                                                   const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                                   const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
                                                   const vpImage<bool> *p_mask);

template
float vpImageFilter::computeCannyThreshold<float>(const vpImage<unsigned char> &I, float &lowerThresh,
                                                  const vpImage<float> *p_dIx, const vpImage<float> *p_dIy,
                                                  const unsigned int &gaussianKernelSize,
                                                  const float &gaussianStdev, const unsigned int &apertureGradient,
                                                  const float &lowerThresholdRatio, const float &upperThresholdRatio,
                                                  const vpImageFilter::vpCannyFilteringAndGradientType &filteringType,
                                                  const vpImage<bool> *p_mask);
/**
 * \endcond
*/

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

  \param[in] Isrc : Image to apply the Canny edge detector to.
  \param[out] Ires : Filtered image (255 means an edge, 0 otherwise).
  \param[in] gaussianFilterSize : The size of the mask of the Gaussian filter to
  apply (an odd number).
  \param[in] thresholdCanny : The upper threshold for the Canny operator. Only value
  greater than this value are marked as an edge. If negative, it will be automatically
  computed, along with the lower threshold. Otherwise, the lower threshold will be set to one third
  of the thresholdCanny .
  \param[in] apertureSobel : Size of the mask for the Sobel operator (odd number).
*/
void vpImageFilter::canny(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Ires,
                          const unsigned int &gaussianFilterSize, const float &thresholdCanny,
                          const unsigned int &apertureSobel)
{
  vpImageFilter::canny(Isrc, Ires, gaussianFilterSize, thresholdCanny / 3.f, thresholdCanny, apertureSobel);
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
  const float upperThresholdCanny = 15;
  const float lowerThresholdCanny = 5;
  const unsigned int apertureSobel = 3;

  // Image for the Canny edge operator
  vpImage<unsigned char> Isrc;
  vpImage<unsigned char> Icanny;

  // First grab the source image Isrc.

  // Apply the Canny edge operator and set the Icanny image.
  vpImageFilter::canny(Isrc, Icanny, gaussianFilterSize, lowerThresholdCanny, upperThresholdCanny, apertureSobel);
  return (0);
}
  \endcode

  \param[in] Isrc : Image to apply the Canny edge detector to.
  \param[out] Ires : Filtered image (255 means an edge, 0 otherwise).
  \param[in] gaussianFilterSize : The size of the mask of the Gaussian filter to
  apply (an odd number).
  \param[in] lowerThreshold : The lower threshold for the Canny operator. Values lower
  than this value are rejected. If negative, it will be set to one third
  of the thresholdCanny .
  \param[in] upperThreshold : The upper threshold for the Canny operator. Only value
  greater than this value are marked as an edge. If negative, it will be automatically
  computed, along with the lower threshold. Otherwise, the lower threshold will be set to one third
  of the thresholdCanny .
  \param[in] apertureSobel : Size of the mask for the Sobel operator (odd number).
*/
void vpImageFilter::canny(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Ires,
                          const unsigned int &gaussianFilterSize,
                          const float &lowerThreshold, const float &upperThreshold,
                          const unsigned int &apertureSobel)
{
  const float gaussianStdev = 2.f;
  const float upperThresholdRatio = 0.8f;
  const float lowerThresholdRatio = 0.6f;
#if defined(HAVE_OPENCV_IMGPROC)
  const vpCannyBackendType cannyBackend = CANNY_OPENCV_BACKEND;
#else
  const vpCannyBackendType cannyBackend = CANNY_VISP_BACKEND;
#endif
  const vpCannyFilteringAndGradientType cannyFilteringSteps = CANNY_GBLUR_SOBEL_FILTERING;
  canny(Isrc, Ires, gaussianFilterSize, lowerThreshold, upperThreshold, apertureSobel,
        gaussianStdev, lowerThresholdRatio, upperThresholdRatio, false, cannyBackend, cannyFilteringSteps);
}

/*!
 * Apply the Canny edge operator on the image \e Isrc and return the resulting
 * image \e Ires.
 *
 * The following example shows how to use the method:
 *
 * \code
 * #include <visp3/core/vpImage.h>
 * #include <visp3/core/vpImageFilter.h>
 *
 * int main()
 * {
 *   // Constants for the Canny operator.
 *   const unsigned int gaussianFilterSize = 5;
 *   const float gaussianStdev = 2.0f;
 *   const float upperThresholdCanny = 15.f;
 *   const float lowerThresholdCanny = 5.f;
 *   const float upperThresholdRatio = 0.8f;
 *   const float lowerThresholdRatio = 0.6f;
 *   const unsigned int apertureSobel = 3;
 *   const bool normalizeGradient = true;
 *   const vpCannyBackendType cannyBackend = CANNY_OPENCV_BACKEND; // or CANNY_VISP_BACKEND;
 *   const vpCannyFilteringAndGradientType filteringType = CANNY_GBLUR_SOBEL_FILTERING; // or CANNY_GBLUR_SCHARR_FILTERING
 *
 *   // Image for the Canny edge operator
 *   vpImage<unsigned char> Isrc;
 *   vpImage<unsigned char> Icanny;
 *
 *   // First grab the source image Isrc.
 *
 *   // Apply the Canny edge operator and set the Icanny image.
 *   vpImageFilter::canny(Isrc, Icanny, gaussianFilterSize, lowerThresholdCanny, upperThresholdCanny, apertureSobel,
 *                       gaussianStdev, lowerThresholdRatio, upperThresholdRatio, normalizeGradient,
 *                       cannyBackend, filteringType);
 *   return (0);
 * }
 * \endcode
 *
 * \param[in] Isrc : Image to apply the Canny edge detector to.
 * \param[out] Ires : Filtered image (255 means an edge, 0 otherwise).
 * \param[in] gaussianFilterSize : The size of the mask of the Gaussian filter to
 * apply (an odd number).
 * \param[in] lowerThreshold : The lower threshold for the Canny operator. Values lower
 * than this value are rejected. If negative, it will be set to one third
 * of the thresholdCanny.
 * \param[in] upperThreshold : The upper threshold for the Canny operator. Only value
 * greater than this value are marked as an edge. If negative, it will be automatically
 * computed, along with the lower threshold. Otherwise, the lower threshold will be set to one third
 * of the upper threshold.
 * \param[in] apertureGradient : Size of the mask for the gradient (Sobel or Scharr) operator (odd number).
 * \param[in] gaussianStdev : The standard deviation of the Gaussian filter to apply.
 * If it is non-positive, it is computed from kernel size (`gaussianKernelSize` parameter) as
 * \f$\sigma = 0.3*((gaussianKernelSize-1)*0.5 - 1) + 0.8\f$.
 * \param[in] lowerThresholdRatio : The ratio of the upper threshold the lower threshold must be equal to.
 * It is used only if the user asks to compute the Canny thresholds.
 * \param[in] upperThresholdRatio : The ratio of pixels whose absolute gradient is lower or equal to define
 * the upper threshold. It is used only if the user asks to compute the Canny thresholds.
 * \param[in] normalizeGradients : Needs to be true if asking to compute the \b upperThreshold, otherwise it depends on
 * the user application and user-defined thresholds.
 * \param[in] cannyBackend : The backend to use to perform the Canny edge filtering.
 * \param[in] cannyFilteringSteps : The filtering + gradient operators to apply to compute the gradient in the early
 * stage of the Canny algorithm.
 * \param[in] p_mask : Optional image mask that indicates where Canny edge detection has to be computed.
 */
void vpImageFilter::canny(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Ires,
                          const unsigned int &gaussianFilterSize,
                          const float &lowerThreshold, const float &upperThreshold, const unsigned int &apertureGradient,
                          const float &gaussianStdev, const float &lowerThresholdRatio, const float &upperThresholdRatio,
                          const bool &normalizeGradients,
                          const vpCannyBackendType &cannyBackend, const vpCannyFilteringAndGradientType &cannyFilteringSteps,
                          const vpImage<bool> *p_mask)
{
  if (cannyBackend == CANNY_OPENCV_BACKEND) {
#if defined(HAVE_OPENCV_IMGPROC)
    cv::Mat img_cvmat, cv_dx, cv_dy, edges_cvmat;
    vpImageConvert::convert(Isrc, img_cvmat);
    computePartialDerivatives(img_cvmat, cv_dx, cv_dy, true, true, normalizeGradients, gaussianFilterSize,
                              gaussianStdev, apertureGradient, cannyFilteringSteps);
    float upperCannyThresh = upperThreshold;
    float lowerCannyThresh = lowerThreshold;
    if (upperCannyThresh < 0.f) {
      upperCannyThresh = computeCannyThreshold(img_cvmat, &cv_dx, &cv_dy, lowerCannyThresh, gaussianFilterSize,
                                               gaussianStdev, apertureGradient, lowerThresholdRatio, upperThresholdRatio,
                                               cannyFilteringSteps);
    }
    else if (lowerCannyThresh < 0.f) {
      lowerCannyThresh = upperCannyThresh / 3.f;
    }
#if (VISP_HAVE_OPENCV_VERSION >= 0x030200)
    cv::Canny(cv_dx, cv_dy, edges_cvmat, lowerCannyThresh, upperCannyThresh, false);
#else
    cv::GaussianBlur(img_cvmat, img_cvmat, cv::Size((int)gaussianFilterSize, (int)gaussianFilterSize),
                     gaussianStdev, gaussianStdev);
    cv::Canny(img_cvmat, edges_cvmat, lowerCannyThresh, upperCannyThresh);
#endif
    vpImageConvert::convert(edges_cvmat, Ires);
#else
    std::string errMsg("[vpImageFilter::canny]You asked for CANNY_OPENCV_BACKEND but ViSP has not been compiled with OpenCV");
    throw(vpException(vpException::badValue, errMsg));
#endif
  }
  else if (cannyBackend == CANNY_VISP_BACKEND) {
    float upperCannyThresh = upperThreshold;
    float lowerCannyThresh = lowerThreshold;

    vpImage<float> dIx, dIy;
    computePartialDerivatives(Isrc, dIx, dIy, true, true, normalizeGradients, gaussianFilterSize,
                              gaussianStdev, apertureGradient, cannyFilteringSteps, cannyBackend, p_mask);

    if (upperCannyThresh < 0.f) {
      upperCannyThresh = computeCannyThreshold(Isrc, lowerCannyThresh, &dIx, &dIy, gaussianFilterSize, gaussianStdev,
                                               apertureGradient, lowerThresholdRatio, upperThresholdRatio,
                                               cannyFilteringSteps, p_mask);
    }
    else if (lowerCannyThresh < 0.f) {
      lowerCannyThresh = upperCannyThresh / 3.;
    }
    vpCannyEdgeDetection edgeDetector(gaussianFilterSize, gaussianStdev, apertureGradient, lowerCannyThresh, upperCannyThresh,
                                      lowerThresholdRatio, upperThresholdRatio, cannyFilteringSteps);
    edgeDetector.setGradients(dIx, dIy);
    edgeDetector.setMask(p_mask);
    Ires = edgeDetector.detect(Isrc);
  }
}
