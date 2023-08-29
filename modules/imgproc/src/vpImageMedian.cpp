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
*****************************************************************************/

#include <visp3/core/vpImageConvert.h>

#if defined(HAVE_OPENCV_IMGPROC)

#include <visp3/imgproc/vpImageMedian.h>

namespace vp
{
/**
 * \ingroup group_image_median
 * Calculates the median value of a single channel
 * based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
 *
 * \param[in] channel : OpenCV input image.
 * \return The median value of the input image.
 */
double median(const cv::Mat &channel)
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
 * \ingroup group_image_median
 * \brief Compute the upper Canny edge filter threshold based on image median.
 *
 * \param[in] cv_I : The image, in cv format.
 * \return The upper Canny edge filter threshold.
 * \sa median()
 */
double computeCannyThreshold(const cv::Mat &cv_I)
{
  cv::Mat cv_I_blur;
  cv::GaussianBlur(cv_I, cv_I_blur, cv::Size(9, 9), 2, 2);

  // Subsample image to reach a 256 x 256 size
  int req_size = 256;
  int orig_size = std::min(static_cast<int>(cv_I.rows), static_cast<int>(cv_I.cols));
  int scale_down = std::max(1, static_cast<int>(orig_size / req_size));
  cv::Mat cv_I_scaled_down;
  resize(cv_I_blur, cv_I_scaled_down, cv::Size(), scale_down, scale_down, cv::INTER_NEAREST);

  double median_pix = vp::median(cv_I_scaled_down);
  // double lower = std::max(0., 0.7 * median_pix); // Unused, but to know the formula exists
  double upper = std::min(255., 1.3 * median_pix);
  upper = std::max(1., upper);
  return upper;
}

/**
 * \ingroup group_image_median
 * \brief Compute the upper Canny edge filter threshold based on image median.
 *
 * \param[in] I : The gray-scale image, in ViSP format.
 * \return The upper Canny edge filter threshold.
 * \sa median()
 */
double computeCannyThreshold(const vpImage<unsigned char> &I)
{
  cv::Mat cv_I;
  vpImageConvert::convert(I, cv_I);
  return computeCannyThreshold(cv_I);
}
} // namespace ImageFilter

#endif
