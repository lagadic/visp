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

#ifndef VP_PANDA3D_POST_PROCESS_FILTER_H
#define VP_PANDA3D_POST_PROCESS_FILTER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include "cardMaker.h"
#include "orthographicLens.h"

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d_filters
 * \brief Base class for postprocessing filters that map the result of a vpPanda3DBaseRenderer to a new image.
 *
 * Unlike 3D renderers, implementations of this class do not have access to 3D information
 * (except if it is the result of the processed image).
 *
 * Implementation wise, the process is the following:
 *
 * - The output texture (retrieved through vpPanda3DBaseRenderer::getMainOutputBuffer) is blitted on a quad,
 * that is placed perfectly in front of the camera.
 * - A shader (given as an argument to the constructor) is applied to this quad.
 * - The result is copied back to ram if required.
*/
class VISP_EXPORT vpPanda3DPostProcessFilter : public vpPanda3DBaseRenderer
{
public:
  vpPanda3DPostProcessFilter(const std::string &name, std::shared_ptr<vpPanda3DBaseRenderer> inputRenderer, bool isOutput, std::string fragmentShader)
    : vpPanda3DBaseRenderer(name), m_inputRenderer(inputRenderer), m_isOutput(isOutput), m_fragmentShader(fragmentShader)
  {
    m_renderOrder = m_inputRenderer->getRenderOrder() + 1;
  }

  bool isRendering3DScene() const VP_OVERRIDE
  {
    return false;
  }

  GraphicsOutput *getMainOutputBuffer() VP_OVERRIDE { return m_buffer; }

protected:
  virtual void setupScene() VP_OVERRIDE;

  void setupCamera() VP_OVERRIDE;

  void setupRenderTarget() VP_OVERRIDE;

  void setRenderParameters(const vpPanda3DRenderParameters &params) VP_OVERRIDE;

  void getRenderBasic(vpImage<unsigned char> &I) const;
  void getRenderBasic(vpImage<vpRGBf> &I) const;


  virtual FrameBufferProperties getBufferProperties() const = 0;

  std::shared_ptr<vpPanda3DBaseRenderer> m_inputRenderer;
  bool m_isOutput; //! Whether this filter is an output to be used and should be copied to ram
  std::string m_fragmentShader;
  PointerTo<Shader> m_shader;
  Texture *m_texture;
  GraphicsOutput *m_buffer;

  static const char *FILTER_VERTEX_SHADER;
};
END_VISP_NAMESPACE
#endif
#endif
