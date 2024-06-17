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

#ifndef VP_PANDA3D_RENDER_PARAMETERS_H
#define VP_PANDA3D_RENDER_PARAMETERS_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>

class Camera;

BEGIN_VISP_NAMESPACE
/**
 * @brief Rendering parameters for a panda3D simulation
 *
 * includes:
 *  - Camera intrinsics
 *  - Image resolution
 *  - Clipping parameters
*/
class VISP_EXPORT vpPanda3DRenderParameters
{
public:
  vpPanda3DRenderParameters() : m_cam(), m_height(0), m_width(0), m_clipNear(0.001), m_clipFar(10.0) { }
  vpPanda3DRenderParameters(const vpCameraParameters &cam, unsigned int h, unsigned int w,
                            double clipNear, double clipFar)
    : m_cam(cam), m_height(h), m_width(w), m_clipNear(clipNear), m_clipFar(clipFar)
  { }

  /**
   * @brief Retrieve camera intrinsics.
   *
   * @return const vpCameraParameters&
   */
  const vpCameraParameters &getCameraIntrinsics() const { return m_cam; }
  /**
   * @brief set camera intrinsics. Only camera intrinsics for a lens without distortion are supported.
   * \throws if camera intrinsics have a distortion model.
   */
  void setCameraIntrinsics(const vpCameraParameters &cam)
  {
    if (cam.get_projModel() != vpCameraParameters::perspectiveProjWithoutDistortion) {
      throw vpException(vpException::badValue, "Panda3D renderer: only lenses with no distortion are supported");
    }
    m_cam = cam;
  }

  double getNearClippingDistance() const { return m_clipNear; }
  double getFarClippingDistance() const { return m_clipFar; }

  /**
   * @brief Set the clipping distance. When a panda camera uses these render parameters, objects that are closer than "near" or further than "far" will be clipped.
   *
   * @param nearV near clipping distance
   * @param farV far clipping distance
   */
  void setClippingDistance(double nearV, double farV)
  {
    if (farV < nearV) {
      std::swap(nearV, farV);
    }
    m_clipNear = nearV;
    m_clipFar = farV;
  }

  unsigned int getImageWidth() const { return m_width; }
  unsigned int getImageHeight() const { return m_height; }

  /**
   * @brief Set the image resolution.
   * When this object is given to a vpPanda3DBaseRenderer,
   * this will be the resolution of the renderer's output images.
   *
   * @param height vertical image resolution
   * @param width horizontal image resolution
   */
  void setImageResolution(unsigned int height, unsigned int width)
  {
    m_height = height;
    m_width = width;
  }

  /**
   * @brief Update a Panda3D camera object to use this objects's parameters.
   *
   * @param camera the camera for which to update the rendering parameters
   *
   * \throws if getImageWidth() or getImageHeight() are equal to 0.
   */
  void setupPandaCamera(Camera *camera);

private:
  vpCameraParameters m_cam;
  unsigned int m_height, m_width;
  double m_clipNear, m_clipFar;
};

END_VISP_NAMESPACE
#endif
#endif
