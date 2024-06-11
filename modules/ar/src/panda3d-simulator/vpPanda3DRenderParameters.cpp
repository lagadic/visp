
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

#include <visp3/ar/vpPanda3DRenderParameters.h>
#if defined(VISP_HAVE_PANDA3D)

#include <matrixLens.h>
#include <camera.h>

BEGIN_VISP_NAMESPACE
void vpPanda3DRenderParameters::setupPandaCamera(Camera *camera)
{
  // Adapted from Megapose code (https://github.com/megapose6d/megapose6d/blob/master/src/megapose/panda3d_renderer/types.py#L59),
  // which was itself inspired by https://discourse.panda3d.org/t/lens-camera-for-opencv-style-camera-parameterisation/15413
  // And http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl

  if (m_width == 0 || m_height == 0) {
    throw vpException(vpException::dimensionError, "Cannot create a projection matrix when the image width or height is 0");
  }

  PT(MatrixLens) lens = new MatrixLens();
  const double A = (m_clipFar + m_clipNear) / (m_clipFar - m_clipNear);
  const double B = -2.0 * (m_clipFar * m_clipNear) / (m_clipFar - m_clipNear);

  const double cx = m_cam.get_u0();
  const double cy = m_height - m_cam.get_v0();

  lens->set_near_far(m_clipNear, m_clipFar);
  lens->set_user_mat(LMatrix4(
    m_cam.get_px(), 0, 0, 0,
    0, 0, A, 1,
    0, m_cam.get_py(), 0, 0,
    0, 0, B, 0
  ));
  lens->set_film_size(m_width, m_height);
  lens->set_film_offset(m_width * 0.5 - cx, m_height * 0.5 - cy);
  camera->set_lens(lens);
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpPanda3DRenderParameters.cpp.o) has no symbols
void dummy_vpPanda3DRenderParameters() { };

#endif
