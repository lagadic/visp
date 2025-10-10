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

#ifndef VP_PANDA3D_GEOMETRY_RENDERER_H
#define VP_PANDA3D_GEOMETRY_RENDERER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRect.h>
#include <visp3/ar/vpPanda3DBaseRenderer.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d_3d
 *
 * @brief Renderer that outputs object geometric information.
 *
 * This information may contain, depending on requested render type:
 *
 * - Normals in the world frame or in the camera frame.
 * - Depth information
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you are interested in using Panda3D in Augmented Reality applications, you may have a look at:</span><br>
 *
 * - \ref tutorial-panda3d
*/
class VISP_EXPORT vpPanda3DGeometryRenderer : public vpPanda3DBaseRenderer
{
public:

  enum vpRenderType
  {
    OBJECT_NORMALS, //! Surface normals in the object frame.
    CAMERA_NORMALS, //! Surface normals in the frame of the camera. Z points towards the camera and y is up.
  };

  vpPanda3DGeometryRenderer(vpRenderType renderType);


  /**
   * @brief Get render results into ViSP readable structures
   *
   *
   * @param colorData Depending on the vpRenderType, normals in the world or camera frame may be stored in this image.
   * @param depth Image used to store depth
   */
  void getRender(vpImage<vpRGBf> &colorData, vpImage<float> &depth) const;

  void getRender(vpImage<vpRGBf> &normals, vpImage<float> &depth, const vpRect &bb, unsigned int h, unsigned w) const;

  /**
   * @brief Get render results into ViSP readable structures. This version only retrieves the normal data
   * @param colorData Depending on the vpRenderType, normals in the world or camera frame may be stored in this image.
   */
  void getRender(vpImage<vpRGBf> &colorData) const;
  /**
   * @brief Get render results into ViSP readable structures. This version only retrieves the depth data.
   * @param depth Depending on the vpRenderType, rendered depth may be stored in this image.
   */
  void getRender(vpImage<float> &depth) const;

  PointerTo<GraphicsOutput> getMainOutputBuffer() VP_OVERRIDE { return m_normalDepthBuffer; }


protected:
  void setupScene() VP_OVERRIDE;
  void setupRenderTarget() VP_OVERRIDE;

private:
  vpRenderType m_renderType;
  PointerTo<Texture> m_normalDepthTexture;
  PointerTo<GraphicsOutput> m_normalDepthBuffer;
};
END_VISP_NAMESPACE
#endif //VISP_HAVE_PANDA3D
#endif
