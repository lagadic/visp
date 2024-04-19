#include <visp3/ar/vpPanda3DBaseRenderer.h>

#if defined(VISP_HAVE_PANDA3D)

#include "load_prc_file.h"

void vpPanda3DBaseRenderer::initFramework(bool showWindow)
{
  if (m_framework.use_count() > 0) {
    throw vpException(vpException::notImplementedError,
    "Panda3D renderer: Reinitializing is not supported!");
  }
  m_framework = std::shared_ptr<PandaFramework>(new PandaFramework());
  m_framework->open_framework();
  WindowProperties winProps;
  winProps.set_size(LVecBase2i(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight()));
  int flags = showWindow ? 0 : GraphicsPipe::BF_refuse_window;
  m_window = m_framework->open_window(winProps, flags);
  if (m_window == nullptr) {
    throw vpException(vpException::notInitialized,
    "Panda3D renderer: Could not create the requested window when performing initialization.");
  }
  m_window->set_background_type(WindowFramework::BackgroundType::BT_black);
  setupScene();
  setupCamera();
  setupRenderTarget();
  m_window->get_display_region_3d()->set_camera(m_cameraPath);
}


void vpPanda3DBaseRenderer::initFromParent(std::shared_ptr<PandaFramework> framework, PT(WindowFramework) window)
{
  m_framework = framework;
  m_window = window;
  setupScene();
  setupCamera();
  setupRenderTarget();
  //m_window->get_display_region_3d()->set_camera(m_cameraPath);
}

void vpPanda3DBaseRenderer::setupScene()
{
  m_renderRoot = m_window->get_render().attach_new_node(m_name);
  m_renderRoot.set_shader_auto();
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
  m_framework->get_graphics_engine()->sync_frame();
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
  vpTranslationVector t = wTo.getTranslationVector();
  vpQuaternionVector q(wTo.getRotationMatrix());
  object.set_pos(t[0], t[1], t[2]);
  object.set_quat(LQuaternion(q.w(), q.x(), q.y(), q.z()));
}

vpHomogeneousMatrix vpPanda3DBaseRenderer::getNodePose(const std::string &name)
{
  NodePath object = m_renderRoot.find(name);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", name);
  }
  return getNodePose(object);
}

vpHomogeneousMatrix vpPanda3DBaseRenderer::getNodePose(NodePath &object)
{
  const LPoint3 pos = object.get_pos();
  const LQuaternion quat = object.get_quat();
  const vpTranslationVector t(pos[0], pos[1], pos[2]);
  const vpQuaternionVector q(quat.get_i(), quat.get_j(), quat.get_k(), quat.get_r());
  return vpHomogeneousMatrix(t, q);
}

void vpPanda3DBaseRenderer::computeNearAndFarPlanesFromNode(const std::string &name, float &near, float &far)
{
  if (m_camera == nullptr) {
    throw vpException(vpException::notInitialized, "Cannot compute planes when the camera is not initialized");
  }
  NodePath object = m_renderRoot.find(name);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", name);
  }
  LPoint3 minP, maxP;
  object.calc_tight_bounds(minP, maxP, m_cameraPath);
  near = std::max(0.f, minP.get_y());
  far = std::max(near, maxP.get_y());
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

#endif
