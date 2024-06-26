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

#ifndef VP_PANDA3D_RENDERER_SET_H
#define VP_PANDA3D_RENDERER_SET_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <vector>

#include <visp3/ar/vpPanda3DBaseRenderer.h>
#include <visp3/ar/vpPanda3DLight.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d
 *
 * @brief Class that rendering multiple datatypes, in a single pass. A RendererSet contains multiple subrenderers, all inheriting from vpPanda3DBaseRenderer.
 * The renderer set synchronizes all scene properties for the different subrenderers. This includes:
 *  * The camera properties (intrinsics, resolution) and extrinsics
 *  * The pose and properties of every object in the scene
 *  * The pose and properties of lights, for subrenderers that are defined as lightable.
 *
 * The overall usage workflow is the following:
 *  1. Create vpPanda3DRendererSet instance
 *  2. Create the subrenderers (e.g, vpPanda3DGeometryRenderer)
 *  3. Add the subrenderers to the set with addSubRenderer
 *  4. Call renderFrame() on the rendererSet. Each subrenderer now has its output computed and ready to be retrieved
 *  5. Retrieve relevant outputs in ViSP format with something similar to `rendererSet.getRenderer<RendererType>("MyRendererName").getRender(I)` where RendererType is the relevant subclass of vpPanda3DBaseRenderer and "MyRendererName" its name (see vpPanda3DBaseRenderer::getName)
*/
class VISP_EXPORT vpPanda3DRendererSet : public vpPanda3DBaseRenderer, public vpPanda3DLightable
{
public:
  vpPanda3DRendererSet(const vpPanda3DRenderParameters &renderParameters);

  virtual ~vpPanda3DRendererSet() = default;

  /**
   * @brief Initialize the framework and propagate the created panda3D framework to the subrenderers.
   *
   * The subrenderers will be initialized in the order of their priority as defined by vpPanda3DBaseRenderer::getRenderOrder
   * Thus, if a renderer B depends on A for its render, and if B.getRenderOrder() > A.getRenderOrder() it can rely on A being initialized when B.initFromParent is called (along with the setupCamera, setupRenderTarget).
   */
  void initFramework() VP_OVERRIDE;

  /**
   * @brief Set the pose of the camera, using the ViSP convention. This change is propagated to all subrenderers
   *
   * @param wTc Pose of the camera
   */
  void setCameraPose(const vpHomogeneousMatrix &wTc) VP_OVERRIDE;

  /**
   * @brief Retrieve the pose of the camera. As this renderer contains multiple other renderers.
   *
   * \warning It is assumed that all the sub renderers are synchronized (i.e., the setCameraPose of this renderer was called before calling this method).
   * Otherwise, you may get incoherent results.
   *
   * @return the pose of the camera using the ViSP convention
   */
  vpHomogeneousMatrix getCameraPose() VP_OVERRIDE;

  /**
   * @brief Set the pose of an object for all the subrenderers. The pose is specified using the ViSP convention
   * This method may fail if a subrenderer does not have a node with the given name.
   *
   * \warning This method may fail if a subrenderer does not have a node with the given name. It is assumed that the scenes are synchronized
   *
   * @param name
   * @param wTo
   */
  void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo) VP_OVERRIDE;

  /**
   * @brief This method is not supported for this renderer type. Use the std::string version
   *
   * \throws vpException, as this method is not supported
   * @param object
   * @param wTo
   */
  void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo) VP_OVERRIDE;

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
  vpHomogeneousMatrix getNodePose(const std::string &name) VP_OVERRIDE;

  /**
   * @brief This method is not supported for this renderer type. Use the std::string version
   *
   * \throws vpException, as this method is not supported
   * @param object
   */
  vpHomogeneousMatrix getNodePose(NodePath &object) VP_OVERRIDE;

  /**
   * \warning This method is not supported and will throw
   */
  void addNodeToScene(const NodePath &object) VP_OVERRIDE;

  void setRenderParameters(const vpPanda3DRenderParameters &params) VP_OVERRIDE;

  void addLight(const vpPanda3DLight &light) VP_OVERRIDE;

  /**
   * @brief Add a new subrenderer: This subrenderer should have a unique name, not present in the set.
   *
   * \throws if the subrenderer's name is already present in the set.
   *
   * @param renderer the renderer to add
   */
  void addSubRenderer(std::shared_ptr<vpPanda3DBaseRenderer> renderer);

  /**
   * @brief Retrieve the first subrenderer with the specified template type.
   *
   * @tparam RendererType  The type of the renderer to find
   * @return std::shared_ptr<RendererType> Pointer to the first renderer match, nullptr if none is found.
   */
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
  /**
   * @brief Retrieve the subrenderer with the specified template type and the given name.
   *
   * @param name the name of the subrenderer to find
   * @tparam RendererType  The type of the renderer to find
   * @return std::shared_ptr<RendererType> Pointer to the renderer, nullptr if none is found.
   */
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
  void setupScene() VP_OVERRIDE { }

  void setupCamera() VP_OVERRIDE { }

private:
  std::vector<std::shared_ptr<vpPanda3DBaseRenderer>> m_subRenderers;
};
END_VISP_NAMESPACE
#endif
#endif
