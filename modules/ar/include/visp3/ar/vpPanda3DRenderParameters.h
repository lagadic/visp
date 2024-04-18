#ifndef vpPanda3DRenderParameters_h
#define vpPanda3DRenderParameters_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)
#include <visp3/core/vpCameraParameters.h>


class Camera;

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

  /**
   * @brief Set the clipping distance. When a panda camera uses these render parameters, objects that are closer than "near" or further than "far" will be clipped.
   *
   * @param near near clipping distance
   * @param far far clipping distance
   */
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


  /**
   * @brief Set the image resolution.
   * When this object is given to a vpPanda3DBaseRenderer,
   * this will be the resolution of the renderer's output images.
   *
   * @param height vertical image resolution
   * @param width horizontal image resolution
   */
  void setImageResolution(unsigned int height, unsigned int width)
  {
    m_height = height;
    m_width = width;
  }

  /**
   * @brief Update a Panda3D camera object to use this objects's parameters.
   *
   * @param camera the camera for which to update the rendering parameters
   *
   * \throws if getImageWidth() or getImageHeight() are equal to 0.
   */
  void setupPandaCamera(Camera *camera);

private:
  vpCameraParameters m_cam;
  unsigned int m_height, m_width;
  double m_clipNear, m_clipFar;
};

#endif
#endif
