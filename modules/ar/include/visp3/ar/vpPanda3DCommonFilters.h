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
 */

#ifndef VP_PANDA3D_COMMON_FILTERS_H
#define VP_PANDA3D_COMMON_FILTERS_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DPostProcessFilter.h>

BEGIN_VISP_NAMESPACE
class vpPanda3DRGBRenderer;

/**
 * \ingroup group_ar_renderer_panda3d_filters
 * \brief Class that implements an RGB to grayscale conversion.
 *
 */
class VISP_EXPORT vpPanda3DLuminanceFilter : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DLuminanceFilter(const std::string &name, std::shared_ptr<vpPanda3DRGBRenderer> inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const VP_OVERRIDE;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const char *FRAGMENT_SHADER;
};

/**
 *
 * \ingroup group_ar_renderer_panda3d_filters
 * \brief Class that implements a gaussian filter on a grayscale image.
 * The grayscale image should be contained in the blue channel of the image.
 *
 */
class VISP_EXPORT vpPanda3DGaussianBlur : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DGaussianBlur(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const VP_OVERRIDE;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const char *FRAGMENT_SHADER;
};

/**
 * \ingroup group_ar_renderer_panda3d_filters
 * \brief Implementation of canny filtering, using Sobel kernels.
 *
 * The results of the canny are filtered based on a threshold value (defined between 0 and 255), checking whether there is enough gradient information.
 * The output of this image is a floating RGB image containing:
 * - In the red channel, the value of the convolution with the sobel horizontal kernel
 * - In the green channel, the value of the convolution with the sobel vertical kernel
 * - In the blue channel, the angle (in radians) of the edge normal.
 */
class VISP_EXPORT vpPanda3DCanny : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DCanny(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, float edgeThreshold);
  FrameBufferProperties getBufferProperties() const VP_OVERRIDE;
  void getRender(vpImage<vpRGBf> &I) const;
  void setEdgeThreshold(float edgeThreshold);

protected:
  void setupScene() VP_OVERRIDE;

private:
  static const char *FRAGMENT_SHADER;
  float m_edgeThreshold;
};

END_VISP_NAMESPACE
#endif
#endif
