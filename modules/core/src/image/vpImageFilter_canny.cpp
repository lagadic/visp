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
 * Image Canny filtering.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpCannyEdgeDetection.h>

BEGIN_VISP_NAMESPACE

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
  for (unsigned int i = 0; i < (CANNY_COUNT_BACKEND - 1); ++i) {
    vpCannyBackendType type = static_cast<vpCannyBackendType>(i);
    list += vpCannyBackendTypeToString(type);
    list += sep;
  }
  vpCannyBackendType type = static_cast<vpCannyBackendType>(CANNY_COUNT_BACKEND - 1);
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
  case CANNY_OPENCV_BACKEND:
    name = "opencv-backend";
    break;
  case CANNY_VISP_BACKEND:
    name = "visp-backend";
    break;
  case CANNY_COUNT_BACKEND:
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
  vpCannyBackendType type(CANNY_COUNT_BACKEND);
  std::string nameLowerCase = vpIoTools::toLowerCase(name);
  unsigned int count = static_cast<unsigned int>(CANNY_COUNT_BACKEND);
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
std::string vpImageFilter::vpGetCannyFiltAndGradTypes(const std::string &pref, const std::string &sep,
                                                               const std::string &suf)
{
  std::string list(pref);
  for (unsigned int i = 0; i < (CANNY_COUNT_FILTERING - 1); ++i) {
    vpCannyFilteringAndGradientType type = static_cast<vpCannyFilteringAndGradientType>(i);
    list += vpCannyFiltAndGradTypeToStr(type);
    list += sep;
  }
  vpCannyFilteringAndGradientType type = static_cast<vpCannyFilteringAndGradientType>(CANNY_COUNT_FILTERING - 1);
  list += vpCannyFiltAndGradTypeToStr(type);
  list += suf;
  return list;
}

/**
 * \brief Cast a \b vpImageFilter::vpCannyFilteringAndGradientType into a string, to know its name.
 *
 * \param[in] type The type that must be casted into a string.
 * \return std::string The corresponding name.
 */
std::string vpImageFilter::vpCannyFiltAndGradTypeToStr(const vpImageFilter::vpCannyFilteringAndGradientType &type)
{
  std::string name;
  switch (type) {
  case CANNY_GBLUR_SOBEL_FILTERING:
    name = "gaussianblur+sobel-filtering";
    break;
  case CANNY_GBLUR_SCHARR_FILTERING:
    name = "gaussianblur+scharr-filtering";
    break;
  case CANNY_COUNT_FILTERING:
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
vpImageFilter::vpCannyFilteringAndGradientType vpImageFilter::vpCannyFiltAndGradTypeFromStr(const std::string &name)
{
  vpCannyFilteringAndGradientType type(CANNY_COUNT_FILTERING);
  std::string nameLowerCase = vpIoTools::toLowerCase(name);
  unsigned int count = static_cast<unsigned int>(CANNY_COUNT_FILTERING);
  bool notFound = true;
  unsigned int i = 0;
  while ((i < count) && notFound) {
    vpCannyFilteringAndGradientType temp = static_cast<vpCannyFilteringAndGradientType>(i);
    if (nameLowerCase == vpCannyFiltAndGradTypeToStr(temp)) {
      type = temp;
      notFound = false;
    }
    ++i;
  }
  return type;
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
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
  float upperThresh = std::max<float>(bon, 1.f);
  lowerThresh = lowerThresholdRatio * bon;
  return upperThresh;
}
#endif

/*!
  Apply the Canny edge operator on the image \e Isrc and return the resulting
  image \e Ires.

  The following example shows how to use the method:

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageFilter.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
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
      lowerCannyThresh = upperCannyThresh / 3.f;
    }
    vpCannyEdgeDetection edgeDetector(gaussianFilterSize, gaussianStdev, apertureGradient, lowerCannyThresh, upperCannyThresh,
                                      lowerThresholdRatio, upperThresholdRatio, cannyFilteringSteps);
    edgeDetector.setGradients(dIx, dIy);
    edgeDetector.setMask(p_mask);
    Ires = edgeDetector.detect(Isrc);
  }
}


END_VISP_NAMESPACE
