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

#include <visp3/ar/vpPanda3DBaseRenderer.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/core/vpMath.h>

#include "load_prc_file.h"
#include <antialiasAttrib.h>

BEGIN_VISP_NAMESPACE
const vpHomogeneousMatrix vpPanda3DBaseRenderer::VISP_T_PANDA({
  1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, -1., 0.0,
  0.0, 1.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 1.0
});
const vpHomogeneousMatrix vpPanda3DBaseRenderer::PANDA_T_VISP(vpPanda3DBaseRenderer::VISP_T_PANDA.inverse());

void vpPanda3DBaseRenderer::initFramework()
{
  if (m_framework.use_count() > 0) {
    throw vpException(vpException::notImplementedError,
    "Panda3D renderer: Reinitializing is not supported!");
  }
  m_framework = std::shared_ptr<PandaFramework>(new PandaFramework());
  m_framework->open_framework();
  WindowProperties winProps;
  winProps.set_size(LVecBase2i(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight()));
  int flags = GraphicsPipe::BF_refuse_window;
  m_window = std::shared_ptr<WindowFramework>(m_framework->open_window(winProps, flags));
  // try and reopen with visible window
  if (m_window == nullptr) {
    winProps.set_minimized(true);
    m_window = std::shared_ptr<WindowFramework>(m_framework->open_window(winProps, 0));
  }
  if (m_window == nullptr) {
    throw vpException(vpException::notInitialized,
    "Panda3D renderer: Could not create the requested window when performing initialization.");
  }
  m_window->set_background_type(WindowFramework::BackgroundType::BT_black);
  setupScene();
  setupCamera();
  setupRenderTarget();
  //m_window->get_display_region_3d()->set_camera(m_cameraPath);
}

void vpPanda3DBaseRenderer::initFromParent(std::shared_ptr<PandaFramework> framework, std::shared_ptr<WindowFramework> window)
{
  m_framework = framework;
  m_window = window;
  setupScene();
  setupCamera();
  setupRenderTarget();
}

void vpPanda3DBaseRenderer::setupScene()
{
  m_renderRoot = m_window->get_render().attach_new_node(m_name);
  //m_renderRoot.set_antialias(AntialiasAttrib::M_none);
}

void vpPanda3DBaseRenderer::setupCamera()
{
  m_cameraPath = m_window->make_camera();
  m_camera = (Camera *)m_cameraPath.node();
  // m_camera = m_window->get_camera(0);
  m_cameraPath = m_renderRoot.attach_new_node(m_camera);
  m_renderParameters.setupPandaCamera(m_camera);
  m_camera->set_scene(m_renderRoot);
}

void vpPanda3DBaseRenderer::renderFrame()
{
  m_framework->get_graphics_engine()->render_frame();
}

void vpPanda3DBaseRenderer::setCameraPose(const vpHomogeneousMatrix &wTc)
{
  if (m_camera.is_null() || m_cameraPath.is_empty()) {
    throw vpException(vpException::notInitialized, "Camera was not initialized before trying to set its pose");
  }
  setNodePose(m_cameraPath, wTc);
}

vpHomogeneousMatrix vpPanda3DBaseRenderer::getCameraPose()
{
  if (m_camera.is_null()) {
    throw vpException(vpException::notInitialized, "Camera was not initialized before trying to get its pose");
  }
  return getNodePose(m_cameraPath);
}

void vpPanda3DBaseRenderer::setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo)
{
  NodePath object = m_renderRoot.find(name);
  setNodePose(object, wTo);
}

void vpPanda3DBaseRenderer::setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo)
{
  const vpHomogeneousMatrix wpTo = wTo * VISP_T_PANDA;
  vpTranslationVector t = wpTo.getTranslationVector();
  vpQuaternionVector q(wpTo.getRotationMatrix());
  object.set_pos(t[0], t[1], t[2]);
  object.set_quat(LQuaternion(q.w(), q.x(), q.y(), q.z()));
}

vpHomogeneousMatrix vpPanda3DBaseRenderer::getNodePose(const std::string &name)
{
  NodePath object = m_renderRoot.find(name);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", name.c_str());
  }
  return getNodePose(object);
}

vpHomogeneousMatrix vpPanda3DBaseRenderer::getNodePose(NodePath &object)
{
  const LPoint3 pos = object.get_pos();
  const LQuaternion quat = object.get_quat();
  const vpTranslationVector t(pos[0], pos[1], pos[2]);
  const vpQuaternionVector q(quat.get_i(), quat.get_j(), quat.get_k(), quat.get_r());
  return vpHomogeneousMatrix(t, q) * PANDA_T_VISP;
}

void vpPanda3DBaseRenderer::computeNearAndFarPlanesFromNode(const std::string &name, float &nearV, float &farV)
{
  if (m_camera == nullptr) {
    throw vpException(vpException::notInitialized, "Cannot compute planes when the camera is not initialized");
  }
  NodePath object = m_renderRoot.find(name);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", name.c_str());
  }
  LPoint3 minP, maxP;
  object.calc_tight_bounds(minP, maxP, m_cameraPath);
  nearV = vpMath::maximum<float>(0.f, minP.get_y());
  farV = vpMath::maximum<float>(nearV, maxP.get_y());
}

NodePath vpPanda3DBaseRenderer::loadObject(const std::string &nodeName, const std::string &modelPath)
{
  NodePath model = m_window->load_model(m_framework->get_models(), modelPath);
  std::cout << "After loading model" << std::endl;
  model.detach_node();
  model.set_name(nodeName);
  return model;
}

void vpPanda3DBaseRenderer::addNodeToScene(const NodePath &object)
{
  NodePath objectInScene = object.copy_to(m_renderRoot);
  objectInScene.set_name(object.get_name());
  setNodePose(objectInScene, vpHomogeneousMatrix());
}

void vpPanda3DBaseRenderer::setVerticalSyncEnabled(bool useVsync)
{
  if (useVsync) {
    load_prc_file_data("", "sync-video true");
  }
  else {
    load_prc_file_data("", "sync-video false");
  }
}
void vpPanda3DBaseRenderer::setAbortOnPandaError(bool abort)
{
  if (abort) {
    load_prc_file_data("", "assert-abort 1");
  }
  else {
    load_prc_file_data("", "assert-abort 0");
  }
}

void vpPanda3DBaseRenderer::enableDebugLog()
{
  load_prc_file_data("", "gl-debug 1");
  load_prc_file_data("", "notify-level-display spam");
}

vpColVector vpPanda3DBaseRenderer::vispPointToPanda(const vpColVector &point)
{
  vpColVector pandaPos = PANDA_T_VISP * point;
  pandaPos /= pandaPos[3];
  return pandaPos;
}
vpColVector vpPanda3DBaseRenderer::vispVectorToPanda(const vpColVector &point)
{
  vpColVector pandaPos = PANDA_T_VISP.getRotationMatrix() * point;
  return pandaPos;
}

void vpPanda3DBaseRenderer::printStructure()
{
  m_renderRoot.ls();
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_ar.a(vpPanda3DBaseRenderer.cpp.o) has no symbols
void dummy_vpPanda3DBaseRenderer() { };

#endif
