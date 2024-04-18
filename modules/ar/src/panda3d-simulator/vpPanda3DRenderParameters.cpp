
#include <visp3/ar/vpPanda3DRenderParameters.h>
#if defined(VISP_HAVE_PANDA3D)


#include <matrixLens.h>
#include <camera.h>

void vpPanda3DRenderParameters::setupPandaCamera(Camera *camera)
{
  // Adapted from Megapose code (https://github.com/megapose6d/megapose6d/blob/master/src/megapose/panda3d_renderer/types.py#L59),
  // which was itself inspired by https://discourse.panda3d.org/t/lens-camera-for-opencv-style-camera-parameterisation/15413
  // And http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl

  if (m_width == 0 || m_height == 0) {
    throw vpException(vpException::dimensionError, "Cannot create a projection matrix when the image width or height is 0");
  }

  PT(MatrixLens) lens = new MatrixLens();
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
  camera->set_lens(lens);

}
#endif
