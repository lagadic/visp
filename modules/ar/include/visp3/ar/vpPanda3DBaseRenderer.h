#ifndef vpPanda3DBaseRenderer_h
#define vpPanda3DBaseRenderer_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/ar/vpPanda3DRenderParameters.h>

#include <pandaFramework.h>
#include <pandaSystem.h>


/**
 * @brief Base class for a panda3D renderer. This class handles basic functionalities,
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
  vpPanda3DBaseRenderer(const std::string &rendererName) : m_name(rendererName), m_framework(nullptr), m_window(nullptr) { }

  virtual ~vpPanda3DBaseRenderer() = default;

  /**
   * @brief Initialize the whole Panda3D framework. Create a new PandaFramework object and a new window.
   *
   * Will also perform the renderer setup (scene, camera and render targets)
   *
   * @param showWindow whether the created window should be visible
   */
  virtual void initFramework(bool showWindow);

  /**
   * @brief
   *
   * @param framework
   */
  void initFromParent(std::shared_ptr<PandaFramework> framework, PT(WindowFramework) window);

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

  virtual void setupRenderTarget() { }

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
    m_renderParameters = params;
    if (m_camera != nullptr) {
      m_renderParameters.setupPandaCamera(m_camera);
    }
  }

  /**
   * @brief Set the camera's pose.
   *
   * @param wTc the new pose of the camera, in world frame
   */
  virtual void setCameraPose(const vpHomogeneousMatrix &wTc);

  /**
   * @brief Retrieve the camera's pose, in the world frame.
   */
  virtual vpHomogeneousMatrix getCameraPose();
  /**
   * @brief Set the pose of a node. This node can be any Panda object (light, mesh, camera).
   *
   * @param name Node path to search for, from the render root. This is the object that will be modified See https://docs.panda3d.org/1.10/python/programming/scene-graph/searching-scene-graph
   * @param wTo Pose of the object in the world frame
   *
   * \throws if the corresponding node cannot be found.
   */
  virtual void setNodePose(const std::string &name, const vpHomogeneousMatrix &wTo);

  /**
   * @brief Set the pose of a node. This node can be any Panda object (light, mesh, camera).
   *
   * @param object The object for which to set the pose
   * @param wTo Pose of the object in the world frame
   */
  virtual void setNodePose(NodePath &object, const vpHomogeneousMatrix &wTo);
  /**
   * @brief Get the pose of a Panda node, in world frame.
   *
   * @param name Node path to search for. \see setNodePose(const std::string &, const vpHomogeneousMatrix &) for more info
   * @return wTo, the pose of the object in world frame
   * \throws if no node can be found from the given path.
   */
  virtual vpHomogeneousMatrix getNodePose(const std::string &name);
  /**
   * @brief Get the pose of a Panda node, in world frame. This version of the method directly uses the Panda Nodepath.
   */
  virtual vpHomogeneousMatrix getNodePose(NodePath &object);

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
   * @brief Add a node to the scene
   *
   * @param object
   */
  virtual void addNodeToScene(const NodePath &object);

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
