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

/*!
  \file vpPanda3DDepthFilters.h
  \brief Custom shaders for depth silhouette extraction
*/
#ifndef VP_PANDA3D_DEPTH_FILTERS_H
#define VP_PANDA3D_DEPTH_FILTERS_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpRect.h>
#include <visp3/ar/vpPanda3DPostProcessFilter.h>

BEGIN_VISP_NAMESPACE
/**
 *
 * \ingroup group_rbt_rendering
 * \brief
*/
class VISP_EXPORT vpPanda3DDepthGaussianBlur : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DDepthGaussianBlur(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput);
  FrameBufferProperties getBufferProperties() const VP_OVERRIDE;
  void getRender(vpImage<unsigned char> &I) const;

private:
  static const std::string FRAGMENT_SHADER;
};

/**
 * \ingroup group_rbt_rendering
 * \brief Implementation of canny filtering, using Sobel kernels.
 *
 * The results of the canny are filtered based on a threshold value (defined between 0 and 255), checking whether there is enough gradient information.
 * The output of this image is a floating RGB image containing:
 * - In the red channel, the value of the convolution with the Sobel horizontal kernel
 * - In the green channel, the value of the convolution with the Sobel vertical kernel
 * - In the blue channel, the angle (in radians) of the edge normal.
 */
class VISP_EXPORT vpPanda3DDepthCannyFilter : public vpPanda3DPostProcessFilter
{
public:
  vpPanda3DDepthCannyFilter(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, float edgeThreshold);
  FrameBufferProperties getBufferProperties() const VP_OVERRIDE;
  PointerTo<Texture> setupTexture(const FrameBufferProperties &fbp) const VP_OVERRIDE;
  void getRender(vpImage<float> &I, vpImage<unsigned char> &valid) const;
  void getRender(vpImage<float> &I, vpImage<unsigned char> &valid, const vpRect &bb, unsigned int h, unsigned w) const;

  void setEdgeThreshold(float edgeThreshold);

protected:
  void setupScene() VP_OVERRIDE;

private:
  static const std::string FRAGMENT_SHADER;
  float m_edgeThreshold;
};

END_VISP_NAMESPACE

#endif
#endif
