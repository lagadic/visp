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

#ifndef _vpImageMedian_h_
#define _vpImageMedian_h_

#include <visp3/core/vpImageFilter.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020408)
#include <opencv2/imgproc/imgproc.hpp>
#elif defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020101)
#include <opencv2/imgproc/imgproc_c.h>
#elif defined(VISP_HAVE_OPENCV)
#include <cv.h>
#endif

namespace ImageFilter
{
// calculates the median value of a single channel
// based on https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
double median(const cv::Mat &channel);

/**
 * \brief Compute the upper Canny edge filter threshold.
 * 
 * \param[in] cv_I The image, in cv format.
 * \return double The upper Canny edge filter threshold.
 */
double computeCannyThreshold(const cv::Mat &cv_I);

/**
 * \brief Compute the upper Canny edge filter threshold.
 * 
 * \param[in] I The gray-scale image, in ViSP format.
 * \return double The upper Canny edge filter threshold.
 */
double computeCannyThreshold(const vpImage<unsigned char> &I);
} // namespace ImageFilter

#endif
