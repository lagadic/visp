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
 */

#ifndef vpPanda3DRGBRenderer_h
#define vpPanda3DRGBRenderer_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DLight.h>
#include <visp3/core/vpImage.h>

class VISP_EXPORT vpPanda3DRGBRenderer : public vpPanda3DBaseRenderer, public vpPanda3DLightableScene
{
public:
  vpPanda3DRGBRenderer() : vpPanda3DBaseRenderer("RGB") { }

  /**
   * @brief Store the render resulting from calling renderFrame() into a vpImage.
   *
   * If the image does not have the correct dimensions, it is resized.
   *
   * @param I The image in which to store the render.
   */
  void getRender(vpImage<vpRGBa> &I) const;

protected:
  void setupScene() vp_override;
  void setupRenderTarget() vp_override;

private:
  Texture *m_colorTexture;
  GraphicsOutput *m_colorBuffer;

};

#endif //VISP_HAVE_PANDA3D
#endif
