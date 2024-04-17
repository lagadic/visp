#ifndef vpPanda3DBaseRenderer_h
#define vpPanda3DBaseRenderer_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>

#include <pandaFramework.h>
#include <pandaSystem.h>
#include <matrixLens.h>



/**
 * @brief Rendering parameters for a panda3D simulation
 *
 * includes:
 *  - Camera intrinsics
 *  - Image resolution
 *  - Clipping parameters
 */
class VISP_EXPORT vpPanda3DRenderParameters
{
public:
  vpPanda3DRenderParameters() : m_cam(), m_height(0), m_width(0), m_clipNear(0.001), m_clipFar(10.0) { }
  vpPanda3DRenderParameters(const vpCameraParameters &cam, unsigned int h, unsigned int w,
                            double clipNear, double clipFar)
    : m_cam(cam), m_height(h), m_width(w), m_clipNear(clipNear), m_clipFar(clipFar)
  { }

  /**
   * @brief Retrieve camera intrinsics.
   *
   * @return const vpCameraParameters&
   */
  const vpCameraParameters &getCameraIntrinsics() const { return m_cam; }
  /**
   * @brief set camera intrinsics. Only camera intrinsics for a lens without distortion are supported.
   * \throws if camera intrinsics have a distortion model.
   */
  void setCameraIntrinsics(const vpCameraParameters &cam)
  {
    if (cam.get_projModel() != vpCameraParameters::perspectiveProjWithoutDistortion) {
      throw vpException(vpException::badValue, "Panda3D renderer: only lenses with no distortion are supported");
    }
    m_cam = cam;
  }

  double getNearClippingDistance() const { return m_clipNear; }
  double getFarClippingDistance() const { return m_clipFar; }

  void setClippingDistance(double near, double far)
  {
    if (far < near) {
      std::swap(near, far);
    }
    m_clipNear = near;
    m_clipFar = far;
  }

  unsigned int getImageWidth() const { return m_width; }
  unsigned int getImageHeight() const { return m_height; }

  void setImageResolution(unsigned int height, unsigned int width)
  {
    m_height = height;
    m_width = width;
  }

  void setupPandaCamera(Camera *camera)
  {
    // Adapted from Megapose code (https://github.com/megapose6d/megapose6d/blob/master/src/megapose/panda3d_renderer/types.py#L59),
    // which was itself inspired by https://discourse.panda3d.org/t/lens-camera-for-opencv-style-camera-parameterisation/15413
    // And http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl
    std::cout << "Calling setup panda camera" << std::endl;
    if (lens == nullptr) {
      lens = new MatrixLens();
      const double A = (m_clipFar + m_clipNear) / (m_clipFar - m_clipNear);
      const double B = -2.0 * (m_clipFar * m_clipNear) / (m_clipFar - m_clipNear);

      const double cx = m_cam.get_u0();
      const double cy = m_height - m_cam.get_v0();

      lens->set_near_far(m_clipNear, m_clipFar);
      lens->set_user_mat(LMatrix4(
        m_cam.get_px(), 0, 0, 0,
        0, 0, A, 1,
        0, m_cam.get_py(), 0, 0,
        0, 0, B, 0
      ));
      lens->set_film_size(m_width, m_height);
      lens->set_film_offset(m_width * 0.5 - cx, m_height * 0.5 - cy);
    }

    camera->set_lens(lens);
    // camera->set_lens_active(0, true);
  }

private:
  vpCameraParameters m_cam;
  unsigned int m_height, m_width;
  double m_clipNear, m_clipFar;
  PT(MatrixLens) lens;
};


/**
 * @brief Base class for a panda3D renderer. This class handles basic functionalities,
 * such as loading object, changing camera parameters etc.
 *
 */
class VISP_EXPORT vpPanda3DBaseRenderer
{
public:
  vpPanda3DBaseRenderer(const std::string &rendererName) : m_name(rendererName), m_framework(nullptr), m_window(nullptr) { }

  virtual ~vpPanda3DBaseRenderer() = default;

  virtual void initFramework(bool showWindow)
  {
    if (m_framework.use_count() > 0) {
      throw vpException(vpException::notImplementedError, "Panda3D renderer: Reinitializing is not supported!");
    }
    m_framework = std::shared_ptr<PandaFramework>(new PandaFramework());
    m_framework->open_framework();
    WindowProperties winProps;
    winProps.set_size(LVecBase2i(m_renderParameters.getImageWidth(), m_renderParameters.getImageHeight()));
    int flags = showWindow ? 0 : GraphicsPipe::BF_refuse_window;
    m_window = m_framework->open_window(winProps, flags);
    m_window->set_background_type(WindowFramework::BackgroundType::BT_black);
    setupScene();
    setupCamera();
    m_window->get_display_region_3d()->set_camera(m_cameraPath);
  }


  void initFromParent(std::shared_ptr<PandaFramework> framework, PT(WindowFramework) window)
  {
    m_framework = framework;
    m_window = window;
    setupScene();
    setupCamera();
    setupRenderTarget();
    m_window->get_display_region_3d()->set_camera(m_cameraPath);
  }

  virtual void setupScene()
  {
    m_renderRoot = m_window->get_render().attach_new_node(m_name);
    m_renderRoot.set_shader_auto();
  }

  virtual void setupRenderTarget() { }

  virtual void renderFrame()
  {
    m_framework->get_graphics_engine()->render_frame();
    m_framework->get_graphics_engine()->sync_frame();

  }


  const std::string &getName() const { return m_name; }
  NodePath getRenderRoot() { return m_renderRoot; }

  virtual void setCameraPose(const vpHomogeneousMatrix &wTc)
  {
    if (m_camera.is_null() || m_cameraPath.is_empty()) {
      throw vpException(vpException::notInitialized, "Camera was not initialized before trying to set its pose");
    }
    setNodePose(m_cameraPath, wTc);
  }
  virtual vpHomogeneousMatrix getCameraPose()
  {
    if (m_camera.is_null()) {
      throw vpException(vpException::notInitialized, "Camera was not initialized before trying to get its pose");
    }
    return getNodePose(m_cameraPath);
  }
  /**
   * @brief Set the pose of a node.
   *
   * @param name Node path to search for, from the render root. See https://docs.panda3d.org/1.10/python/programming/scene-graph/searching-scene-graph
   * @param wTo Pose of the object in the world frame
   */
  virtual void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo)
  {
    NodePath object = m_renderRoot.find(name);
    setNodePose(object, wTo);
  }

  virtual void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo)
  {
    vpTranslationVector t = wTo.getTranslationVector();
    vpQuaternionVector q(wTo.getRotationMatrix());
    object.set_pos(t[0], t[1], t[2]);
    object.set_quat(LQuaternion(q.w(), q.x(), q.y(), q.z()));
  }
  virtual vpHomogeneousMatrix getNodePose(const std::string &name)
  {
    NodePath object = m_renderRoot.find(name);
    if (object.is_empty()) {
      throw vpException(vpException::badValue, "Node %s was not found", name);
    }
    return getNodePose(object);
  }
  virtual vpHomogeneousMatrix getNodePose(NodePath &object)
  {
    const LPoint3 pos = object.get_pos();
    const LQuaternion quat = object.get_quat();
    const vpTranslationVector t(pos[0], pos[1], pos[2]);
    const vpQuaternionVector q(quat.get_i(), quat.get_j(), quat.get_k(), quat.get_r());
    return vpHomogeneousMatrix(t, q);
  }


  /**
   * @brief Initialize camera. Should be called when the scene root of this render has already been created.
   *
   */
  virtual void setupCamera()
  {

    m_camera = m_window->get_camera(0);
    //m_camera = (Camera *)m_cameraPath.node();
    // m_camera = m_window->get_camera(0);
    m_cameraPath = m_renderRoot.attach_new_node(m_camera);
    m_renderParameters.setupPandaCamera(m_camera);
  }

  NodePath loadObject(const std::string &nodeName, const std::string &modelPath)
  {
    NodePath model = m_window->load_model(m_framework->get_models(), modelPath);
    std::cout << "After loading model" << std::endl;
    model.detach_node();
    model.set_name(nodeName);
    return model;
  }

  virtual void addNodeToScene(const NodePath &object)
  {
    NodePath objectInScene = m_renderRoot.attach_new_node(object.node());
    //objectInScene.set_shader_auto();
    objectInScene.set_name(object.get_name());
    std::cout << objectInScene.get_mat() << std::endl;
  }


  virtual void setRenderParameters(const vpPanda3DRenderParameters &params) { m_renderParameters = params; }


protected:
  const std::string m_name; //! name of the renderer
  std::shared_ptr<PandaFramework> m_framework; //! Pointer to the active panda framework
  PT(WindowFramework) m_window; //! Pointer to owning window, which can create buffers etc. It is not necessarily visible.
  vpPanda3DRenderParameters m_renderParameters; //! Rendering parameters
  NodePath m_renderRoot; //! Node containing all the objects and the camera for this renderer
  PT(Camera) m_camera;
  NodePath m_cameraPath; //! NodePath of the camera

};


#endif //VISP_HAVE_PANDA3D
#endif
