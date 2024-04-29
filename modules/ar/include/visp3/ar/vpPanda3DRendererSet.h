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

#ifndef vpPanda3DRendererSet_h
#define vpPanda3DRendererSet_h
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <vector>

#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DLight.h>

class VISP_EXPORT vpPanda3DRendererSet : public vpPanda3DBaseRenderer, public vpPanda3DLightable
{
public:
  vpPanda3DRendererSet(const vpPanda3DRenderParameters &renderParameters);
  virtual ~vpPanda3DRendererSet();

  /**
   * @brief Initialize the framework and propagate the created panda3D framework to the subrenderers.
   *
   * The subrenderers will be initialized in the order of their priority as defined by vpPanda3DBaseRenderer::getRenderOrder
   * Thus, if a renderer B depends on A for its render, and if B.getRenderOrder() > A.getRenderOrder() it can rely on A being initialized when B.initFromParent is called (along with the setupCamera, setupRenderTarget).
   */
  void initFramework() vp_override;

  /**
   * @brief Set the pose of the camera, using the ViSP convention. This change is propagated to all subrenderers
   *
   * @param wTc Pose of the camera
   */
  void setCameraPose(const vpHomogeneousMatrix &wTc) vp_override;

  /**
   * @brief Retrieve the pose of the camera. As this renderer contains multiple other renderers.
   *
   * \warning It is assumed that all the sub renderers are synchronized (i.e., the setCameraPose of this renderer was called before calling this method).
   * Otherwise, you may get incoherent results.
   *
   * @return the pose of the camera using the ViSP convention
   */
  vpHomogeneousMatrix getCameraPose() vp_override;

  /**
   * @brief Set the pose of an object for all the subrenderers. The pose is specified using the ViSP convention
   * This method may fail if a subrenderer does not have a node with the given name.
   *
   * \warning This method may fail if a subrenderer does not have a node with the given name. It is assumed that the scenes are synchronized
   *
   * @param name
   * @param wTo
   */
  void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo) vp_override;

  /**
   * @brief This method is not supported for this renderer type. Use the std::string version
   *
   * \throws vpException, as this method is not supported
   * @param object
   * @param wTo
   */
  void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo) vp_override;

  /**
   * @brief Retrieve the pose of a scene node. The pose is in the world frame, using a ViSP convention.
   *
   * \see the base method
   *
   * \warning It is assumed that all the sub renderers are synchronized (i.e., the setNodePose of this renderer was called before calling this method).
   * Otherwise, you may get incoherent results.
   * @param name name of the node
   * @return vpHomogeneousMatrix the pose of the node in the world frame
   */
  vpHomogeneousMatrix getNodePose(const std::string &name) vp_override;

  /**
   * @brief This method is not supported for this renderer type. Use the std::string version
   *
   * \throws vpException, as this method is not supported
   * @param object
   * @param wTo
   */
  vpHomogeneousMatrix getNodePose(NodePath &object) vp_override;

  /**
   * \warn this method is not supported and will throw
   */
  void addNodeToScene(const NodePath &object) vp_override;

  void setRenderParameters(const vpPanda3DRenderParameters &params) vp_override;

  void addLight(const vpPanda3DLight &light) vp_override;

  /**
   * @brief Add a new subrenderer
   *
   * @param renderer
   */
  void addSubRenderer(std::shared_ptr<vpPanda3DBaseRenderer> renderer);

  template<typename RendererType>
  std::shared_ptr<RendererType> getRenderer()
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      std::shared_ptr<RendererType> rendererCast = std::dynamic_pointer_cast<RendererType>(renderer);
      if (rendererCast != nullptr) {
        return rendererCast;
      }
    }
    return nullptr;
  }

  template<typename RendererType>
  std::shared_ptr<RendererType> getRenderer(const std::string &name)
  {
    for (std::shared_ptr<vpPanda3DBaseRenderer> &renderer: m_subRenderers) {
      if (renderer->getName() == name) {
        std::shared_ptr<RendererType> rendererCast = std::dynamic_pointer_cast<RendererType>(renderer);
        if (rendererCast != nullptr) {
          return rendererCast;
        }
      }
    }
    return nullptr;
  }

protected:
  void setupScene() vp_override { }

  void setupCamera() vp_override { }

private:
  std::vector<std::shared_ptr<vpPanda3DBaseRenderer>> m_subRenderers;
};

#endif
#endif
