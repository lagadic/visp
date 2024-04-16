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
    // near, far = self.z_near, self.z_far
    //   lens.set_far(far)
    //   lens.set_near(near)

    //   h, w = self.resolution
    //   fx = self.K[0, 0]
    //   fy = self.K[1, 1]
    //   cx = self.K[0, 2]
    //   cy = h - self.K[1, 2]
    //   A = (far + near) / (far - near)
    //   B = -2 * (far * near) / (far - near)
    //   user_mat = np.array(
    //       [
    //         [fx, 0, 0, 0],
    //           [0, 0, A, 1],
    //           [0, fy, 0, 0],
    //           [0, 0, B, 0],
    //       ]
    //   )

    //   lens.setFilmSize(w, h)
    //   lens.setUserMat(p3d.core.LMatrix4f(*user_mat.flatten().tolist()))
    //   lens.setFilmOffset(w * 0.5 - cx, h * 0.5 - cy)
    //   return
    PT(MatrixLens) lens = new MatrixLens();
    lens->set_near_far(m_clipNear, m_clipFar);
    const double A = (m_clipFar + m_clipNear) / (m_clipFar - m_clipNear);
    const double B = -2 * (m_clipFar * m_clipNear) / (m_clipFar - m_clipNear);

    const double cx = m_cam.get_u0();
    const double cy = m_height - m_cam.get_v0();
    lens->set_film_size(m_width, m_height);
    lens->set_film_offset(m_width * 0.5 - cx, m_height * 0.5 - cy);
    lens->set_user_mat(LMatrix4(
      m_cam.get_px(), 0, 0, 0,
      0, 0, A, 1,
      0, m_cam.get_py(), 0, 0,
      0, 0, B, 0
    ));

    camera->set_lens(lens);
  }

private:
  vpCameraParameters m_cam;
  unsigned int m_height, m_width;
  double m_clipNear, m_clipFar;
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
    m_renderRoot = m_window->get_render().attach_new_node(m_name);
    m_renderRoot.set_shader_auto();
    initCamera();
  }

  virtual void renderFrame()
  {
    m_framework->get_graphics_engine()->render_frame();
  }

  virtual void setCameraPose(const vpHomogeneousMatrix &wTc)
  {
    if (m_camera.is_null()) {
      throw vpException(vpException::notInitialized, "Camera was not initialized before trying to set its pose");
    }
    setNodePose(m_cameraPath, wTc);
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

  /**
   * @brief Initialize camera. Should be called when the scene root of this render has already been created.
   *
   */
  virtual void initCamera()
  {
    m_camera = new Camera("camera");
    m_renderRoot.attach_new_node(m_camera);
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

  virtual void addNodeToScene(NodePath &object)
  {
    NodePath objectInScene = m_renderRoot.attach_new_node(object.node());
    objectInScene.set_name(object.get_name());
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
