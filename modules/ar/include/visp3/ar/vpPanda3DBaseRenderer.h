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

#ifndef VP_PANDA3D_BASE_RENDERER_H
#define VP_PANDA3D_BASE_RENDERER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/ar/vpPanda3DRenderParameters.h>

#include <pandaFramework.h>
#include <pandaSystem.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_ar_renderer_panda3d
 *
 * \brief Base class for a panda3D renderer. This class handles basic functionalities,
 * such as loading object, changing camera parameters.
 *
 * For a subclass to have a novel behaviour (e.g, display something else) These methods should be overriden:
 *
 * - setupScene: This is where you should apply your shaders.
 * - setupCamera: This is where cameras are created and intrinsics parameters are applied
 * - setupRenderTarget: This is where you should create the texture buffers, where the render results should be stored.
*/
class VISP_EXPORT vpPanda3DBaseRenderer
{
public:
  vpPanda3DBaseRenderer(const std::string &rendererName)
    : m_name(rendererName), m_renderOrder(-100), m_framework(nullptr), m_window(nullptr), m_camera(nullptr)
  {
    setVerticalSyncEnabled(false);
  }

  virtual ~vpPanda3DBaseRenderer() = default;

  /**
   * @brief Initialize the whole Panda3D framework. Create a new PandaFramework object and a new window.
   *
   * Will also perform the renderer setup (scene, camera and render targets)
   */
  virtual void initFramework();

  /**
   * @brief
   *
   * @param framework
   * @param window
   */
  void initFromParent(std::shared_ptr<PandaFramework> framework, std::shared_ptr<WindowFramework> window);


  virtual void renderFrame();

  /**
   * @brief Get the name of the renderer
   *
   * @return const std::string&
   */
  const std::string &getName() const { return m_name; }

  /**
   * @brief Get the scene root
   *
   */
  NodePath &getRenderRoot() { return m_renderRoot; }

  /**
   * @brief Set new rendering parameters. If the scene has already been initialized, the renderer camera is updated.
   *
   * @param params the new rendering parameters
   */
  virtual void setRenderParameters(const vpPanda3DRenderParameters &params)
  {
    unsigned int previousH = m_renderParameters.getImageHeight(), previousW = m_renderParameters.getImageWidth();
    bool resize = previousH != params.getImageHeight() || previousW != params.getImageWidth();

    m_renderParameters = params;

    if (resize) {
      for (GraphicsOutput *buffer: m_buffers) {
        //buffer->get_type().is_derived_from()
        GraphicsBuffer *buf = dynamic_cast<GraphicsBuffer *>(buffer);
        if (buf == nullptr) {
          throw vpException(vpException::fatalError, "Panda3D: could not cast to GraphicsBuffer when rendering.");
        }
        else {
          buf->set_size(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight());
        }
      }
    }

    // If renderer is already initialized, modify camera properties
    if (m_camera != nullptr) {
      m_renderParameters.setupPandaCamera(m_camera);
    }
  }

  /**
   * @brief Returns true if this renderer process 3D data and its scene root can be interacted with.
   *
   * This value could be false, if for instance it is redefined in a subclass that performs postprocessing on a texture.
   */
  virtual bool isRendering3DScene() const { return true; }

  /**
   * @brief Get the rendering order of this renderer.
   * If a renderer A has a lower order value than B, it will be rendered before B.
   * This is useful, if for instance, B is a postprocessing filter that depends on the result of B.
   *
   * @return int
   */
  int getRenderOrder() const { return m_renderOrder; }

  /**
   * @brief Set the camera's pose.
   * The pose is specified using the ViSP convention (Y-down right handed).
   *
   * @param wTc the new pose of the camera, in world frame
   */
  virtual void setCameraPose(const vpHomogeneousMatrix &wTc);

  /**
   * @brief Retrieve the camera's pose, in the world frame.
   * The pose is specified using the ViSP convention (Y-down right handed).
   */
  virtual vpHomogeneousMatrix getCameraPose();

  /**
   * @brief Set the pose of a node. This node can be any Panda object (light, mesh, camera).
   * The pose is specified using the ViSP convention (Y-down right handed).
   *
   * @param name Node path to search for, from the render root. This is the object that will be modified See https://docs.panda3d.org/1.10/python/programming/scene-graph/searching-scene-graph
   * @param wTo Pose of the object in the world frame
   *
   * \throws if the corresponding node cannot be found.
   */
  virtual void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo);

  /**
   * @brief Set the pose of a node.
   * The pose is specified using the ViSP convention (Y-down right handed).
   * This node can be any Panda object (light, mesh, camera).
   *
   * @param object The object for which to set the pose
   * @param wTo Pose of the object in the world frame
   */
  virtual void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo);

  /**
   * @brief Get the pose of a Panda node, in world frame in the ViSP convention (Y-down right handed).
   *
   * @param name Node path to search for. \see setNodePose(const std::string &, const vpHomogeneousMatrix &) for more info
   * @return wTo, the pose of the object in world frame
   * \throws if no node can be found from the given path.
   */
  virtual vpHomogeneousMatrix getNodePose(const std::string &name);

  /**
   * @brief Get the pose of a Panda node, in world frame in the ViSP convention (Y-down right handed). This version of the method directly uses the Panda Nodepath.
   */
  virtual vpHomogeneousMatrix getNodePose(NodePath &object);

  /**
   * @brief Compute the near and far planes for the camera at the current pose, given a certain node/part of the graph.
   *
   * The near clipping value will be set to the distance to the closest point of the object.
   * The far clipping value will be set to the distance to farthest vertex of the object.
   *
   * \warning Depending on geometry complexity, this may be an expensive operation.
   * \warning if the object lies partly behind the camera, the near plane value will be zero.
   *  If it fully behind, the far plane will also be zero. If these near/far values are used to update the
   *  rendering parameters of the camera, this may result in an invalid projection matrix.
   *
   * @param name name of the node that should be used to compute near and far values.
   * @param near resulting near clipping plane distance
   * @param far resulting far clipping plane distance
   */
  void computeNearAndFarPlanesFromNode(const std::string &name, float &near, float &far);

  /**
   * @brief Load a 3D object. To load an .obj file, Panda3D must be compiled with assimp support.
   *
   * Once loaded, the object will not be visible, it should be added to the scene.
   *
   * @param nodeName the name that will be used when inserting the node in the scene graph
   * @param modelPath  Path to the model file
   * @return NodePath The NodePath containing the 3D model, which can now be added to the scene graph.
   */
  NodePath loadObject(const std::string &nodeName, const std::string &modelPath);

  /**
   * @brief Add a node to the scene. Its pose is set as the identity matrix
   *
   * @param object
   */
  virtual void addNodeToScene(const NodePath &object);

  /**
   * @brief set whether vertical sync is enabled.
   * When vertical sync is enabled, render speed will be limited by the display's refresh rate
   *
   * @param useVsync Whether to use vsync or not
   */
  void setVerticalSyncEnabled(bool useVsync);
  /**
   * @brief Set the behaviour when a Panda3D assertion fails. If abort is true, the program will stop.
   * Otherwise, an error will be displayed in the console.
   *
   * @param abort whether to abort (true) or display a message when an assertion fails.
   */
  void setAbortOnPandaError(bool abort);
  void enableDebugLog();

  static vpColVector vispPointToPanda(const vpColVector &point);
  static vpColVector vispVectorToPanda(const vpColVector &vec);

  void printStructure();

  virtual GraphicsOutput *getMainOutputBuffer() { return nullptr; }

protected:

  /**
   * @brief Initialize the scene for this specific renderer.
   *
   * Creates a root scene for this node and applies shaders. that will be used for rendering
   *
   */
  virtual void setupScene();

  /**
   * @brief Initialize camera. Should be called when the scene root of this render has already been created.
   *
   */
  virtual void setupCamera();

  /**
   * @brief Initialize buffers and other objects that are required to save the render.
   *
   */
  virtual void setupRenderTarget() { }


  const static vpHomogeneousMatrix VISP_T_PANDA; //! Homogeneous transformation matrix to convert from the Panda coordinate system (right-handed Z-up) to the ViSP coordinate system (right-handed Y-Down)
  const static vpHomogeneousMatrix PANDA_T_VISP; //! Inverse of VISP_T_PANDA


protected:
  const std::string m_name; //! name of the renderer
  int m_renderOrder; //! Rendering priority for this renderer and its buffers. A lower value will be rendered first. Should be used when calling make_output in setupRenderTarget()
  std::shared_ptr<PandaFramework> m_framework; //! Pointer to the active panda framework
  std::shared_ptr<WindowFramework> m_window; //! Pointer to owning window, which can create buffers etc. It is not necessarily visible.
  vpPanda3DRenderParameters m_renderParameters; //! Rendering parameters
  NodePath m_renderRoot; //! Node containing all the objects and the camera for this renderer
  PointerTo<Camera> m_camera;
  NodePath m_cameraPath; //! NodePath of the camera
  std::vector<GraphicsOutput *> m_buffers; //! Set of buffers that this renderer uses. This storage contains weak refs to those buffers and should not deallocate them.
};

END_VISP_NAMESPACE
#endif //VISP_HAVE_PANDA3D
#endif
