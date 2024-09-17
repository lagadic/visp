/****************************************************************************
 *
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
*****************************************************************************/

/*!
  \file vpObjectCentricRenderer.h
  \brief Single object focused renderer
*/
#ifndef VP_OBJECT_CENTRIC_RENDERER_H
#define VP_OBJECT_CENTRIC_RENDERER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/core/vpRect.h>

class VISP_EXPORT vpObjectCentricRenderer : public vpPanda3DRendererSet
{
public:
  vpObjectCentricRenderer(const vpPanda3DRenderParameters &renderParameters);

  virtual void setRenderParameters(const vpPanda3DRenderParameters &params) VP_OVERRIDE
  {
    vpPanda3DRendererSet::setRenderParameters(params);
  }

  vpRect getBoundingBox() const { return m_bb; }

  void setFocusedObject(const std::string &focused)
  {
    m_focusedObject = focused;
    m_shouldComputeBBPoints = true;
  }

  void beforeFrameRendered() VP_OVERRIDE;

  void computeBoundingBox3DPoints();
  void computeClipping(float &nearV, float &farV);

  std::vector<vpColVector> getBoundingBox3D()
  {
    if (m_shouldComputeBBPoints) {
      computeBoundingBox3DPoints();
      m_shouldComputeBBPoints = false;
    }
    return m_bb3DPoints;
  }

  vpRect computeBoundingBox();


  template <typename T>
  void placeRenderInto(const vpImage<T> &render, vpImage<T> &target, const T &clearValue)
  {
    if (!m_enableCrop) {
      target = render;
    }
    else {
      const unsigned h = m_renderParameters.getImageHeight();
      const unsigned w = m_renderParameters.getImageWidth();
      const unsigned top = static_cast<unsigned int>(std::max(0.0, m_bb.getTop()));
      const unsigned left = static_cast<unsigned int>(std::max(0.0, m_bb.getLeft()));
      const unsigned bottom = static_cast<unsigned int>(std::min(static_cast<double>(h), m_bb.getBottom()));
      const unsigned right = static_cast<unsigned int>(std::min(static_cast<double>(w), m_bb.getRight()));

      target.resize(h, w, clearValue);
      for (unsigned int i = top; i < bottom; ++i) {
        memcpy(target.bitmap + i * w + left, render[i - top], (right - left) * sizeof(T));
        // for (unsigned int j = left; j < right; ++j) {
        //   target[i][j] = render[i - unsigned(m_bb.getTop())][j - unsigned(m_bb.getLeft())];
        // }
      }

    }
  }

private:
  bool m_enableCrop;
  std::string m_focusedObject;
  vpRect m_bb;
  std::vector<vpColVector> m_bb3DPoints;
  bool m_shouldComputeBBPoints;
  vpPanda3DRenderParameters m_subRenderParams;

};

#endif
#endif
