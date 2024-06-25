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

#ifndef VP_PANDA3D_RGB_RENDERER_H
#define VP_PANDA3D_RGB_RENDERER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DLight.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d_3d
 * \brief Implementation of a traditional RGB renderer in Panda3D
 *
 * The lighting model follows a Cook-torrance BRDF.
 *
 * For each object, a specific Version of the cook-torrance shader is compiled: diffuse textures are supported, but normal/bump/roughness maps are not.
 * This class will try to automatically detect whether an object has RGB textures.
 *
 * Specular highlights and reflections can be ignored, depending on the value of isShowingSpeculars.
 *
 * \warning if an object is detected as having image textures but it actually doesn't have any, the object may appear washed out.
 *
 * \note Most of the tested objects were in BAM format, Panda3D's own format.
 * The following pipeline was used:
 *  - Export to GLTF with Blender
 *  - In a Python environment, install gltf2bam with `pip install panda3d-gltf`
 *  - run gltf2bam path/to/yourObject.gltf path/to/yourObject.bam
 *  - then, in the code, use `renderer.addNodeToScene("/path/to/yourObject.bam");`
 *
*/
class VISP_EXPORT vpPanda3DRGBRenderer : public vpPanda3DBaseRenderer, public vpPanda3DLightableScene
{
public:
  /**
   * \brief Default constructor. Initialize an RGB renderer with the normal rendering behavior showing speculars
   *
   */
  vpPanda3DRGBRenderer() : vpPanda3DBaseRenderer("RGB"), m_showSpeculars(true), m_display2d(nullptr), m_backgroundTexture(nullptr) { }
  /**
   * \brief RGB renderer constructor allowing to specify
   * whether specular highlights should be rendered or
   * if only ambient/diffuse lighting should be considered.
   *
   * \param showSpeculars whether to render speculars
   */
  vpPanda3DRGBRenderer(bool showSpeculars) : vpPanda3DBaseRenderer(showSpeculars ? "RGB" : "RGB-diffuse"), m_showSpeculars(showSpeculars) { }


  /**
   * @brief Store the render resulting from calling renderFrame() into a vpImage.
   *
   * If the image does not have the correct dimensions, it is resized.
   *
   * @param I The image in which to store the render.
   */
  void getRender(vpImage<vpRGBa> &I) const;

  void addNodeToScene(const NodePath &object) VP_OVERRIDE;

  void setBackgroundImage(const vpImage<vpRGBa> &background);

  GraphicsOutput *getMainOutputBuffer() VP_OVERRIDE { return m_colorBuffer; }

  bool isShowingSpeculars() const { return m_showSpeculars; }


protected:
  void setupScene() VP_OVERRIDE;
  void setupRenderTarget() VP_OVERRIDE;
  virtual std::string makeFragmentShader(bool hasTexture, bool specular);

private:
  bool m_showSpeculars;
  Texture *m_colorTexture;
  GraphicsOutput *m_colorBuffer;
  static const char *COOK_TORRANCE_VERT;
  static const char *COOK_TORRANCE_FRAG;

  NodePath m_backgroundImage;
  DisplayRegion *m_display2d;
  Texture *m_backgroundTexture;

};

END_VISP_NAMESPACE
#endif //VISP_HAVE_PANDA3D
#endif
