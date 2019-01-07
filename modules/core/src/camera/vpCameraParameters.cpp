/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 * Description:
 * Camera intrinsic parameters.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpCameraParameters.cpp
  \brief Definition of the vpCameraParameters class member functions.
  Class vpCameraParameters define the camera intrinsic parameters

*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpRotationMatrix.h>

const double vpCameraParameters::DEFAULT_PX_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_PY_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_U0_PARAMETER = 192.0;
const double vpCameraParameters::DEFAULT_V0_PARAMETER = 144.0;
const double vpCameraParameters::DEFAULT_KUD_PARAMETER = 0.0;
const double vpCameraParameters::DEFAULT_KDU_PARAMETER = 0.0;
const vpCameraParameters::vpCameraParametersProjType vpCameraParameters::DEFAULT_PROJ_TYPE =
    vpCameraParameters::perspectiveProjWithoutDistortion;

/*!
  Default constructor.
  By default, a perspective projection without distortion model is set.

  \sa init()
*/
vpCameraParameters::vpCameraParameters()
  : px(DEFAULT_PX_PARAMETER), py(DEFAULT_PY_PARAMETER), u0(DEFAULT_U0_PARAMETER), v0(DEFAULT_V0_PARAMETER),
    kud(DEFAULT_KUD_PARAMETER), kdu(DEFAULT_KDU_PARAMETER), width(0), height(0), isFov(false), m_hFovAngle(0),
    m_vFovAngle(0), fovNormals(), inv_px(1. / DEFAULT_PX_PARAMETER), inv_py(1. / DEFAULT_PY_PARAMETER),
    projModel(DEFAULT_PROJ_TYPE)
{
  init();
}

/*!
  Copy constructor
 */
vpCameraParameters::vpCameraParameters(const vpCameraParameters &c)
  : px(DEFAULT_PX_PARAMETER), py(DEFAULT_PY_PARAMETER), u0(DEFAULT_U0_PARAMETER), v0(DEFAULT_V0_PARAMETER),
    kud(DEFAULT_KUD_PARAMETER), kdu(DEFAULT_KDU_PARAMETER), width(0), height(0), isFov(false), m_hFovAngle(0),
    m_vFovAngle(0), fovNormals(), inv_px(1. / DEFAULT_PX_PARAMETER), inv_py(1. / DEFAULT_PY_PARAMETER),
    projModel(DEFAULT_PROJ_TYPE)
{
  init(c);
}

/*!
  Constructor for perspective projection without distortion model

  \param cam_px,cam_py : pixel size
  \param cam_u0,cam_v0 : principal points

 */
vpCameraParameters::vpCameraParameters(const double cam_px, const double cam_py, const double cam_u0,
                                       const double cam_v0)
  : px(DEFAULT_PX_PARAMETER), py(DEFAULT_PY_PARAMETER), u0(DEFAULT_U0_PARAMETER), v0(DEFAULT_V0_PARAMETER),
    kud(DEFAULT_KUD_PARAMETER), kdu(DEFAULT_KDU_PARAMETER), width(0), height(0), isFov(false), m_hFovAngle(0),
    m_vFovAngle(0), fovNormals(), inv_px(1. / DEFAULT_PX_PARAMETER), inv_py(1. / DEFAULT_PY_PARAMETER),
    projModel(DEFAULT_PROJ_TYPE)
{
  initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0);
}

/*!
  Constructor for perspective projection with distortion model

  \param cam_px,cam_py : pixel size
  \param cam_u0,cam_v0 : principal points
  \param cam_kud : undistorted to distorted radial distortion
  \param cam_kdu : distorted to undistorted radial distortion

 */
vpCameraParameters::vpCameraParameters(const double cam_px, const double cam_py, const double cam_u0,
                                       const double cam_v0, const double cam_kud, const double cam_kdu)
  : px(DEFAULT_PX_PARAMETER), py(DEFAULT_PY_PARAMETER), u0(DEFAULT_U0_PARAMETER), v0(DEFAULT_V0_PARAMETER),
    kud(DEFAULT_KUD_PARAMETER), kdu(DEFAULT_KDU_PARAMETER), width(0), height(0), isFov(false), m_hFovAngle(0),
    m_vFovAngle(0), fovNormals(), inv_px(1. / DEFAULT_PX_PARAMETER), inv_py(1. / DEFAULT_PY_PARAMETER),
    projModel(DEFAULT_PROJ_TYPE)
{
  initPersProjWithDistortion(cam_px, cam_py, cam_u0, cam_v0, cam_kud, cam_kdu);
}

/*!
  \brief basic initialization with the default parameters
*/
void vpCameraParameters::init()
{
  if (fabs(this->px) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(this->py) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->inv_px = 1. / this->px;
  this->inv_py = 1. / this->py;
}

/*!
  Initialization with specific parameters using perpective projection without
  distortion model.
  \param cam_px,cam_py : the ratio between the focal length and the size of a
pixel. \param cam_u0,cam_v0 : principal point coordinates in pixels.

   The following sample code shows how to use this function:
   \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<unsigned char> I(480, 640);
  double u0 = I.getWidth()  / 2.;
  double v0 = I.getHeight() / 2.;
  double px = 600;
  double py = 600;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);
  cam.computeFov(I.getWidth(), I.getHeight());
  std::cout << cam << std::endl;
  std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
            << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle())
            << " degrees)" << std::endl;
}
   \endcode
   It produces the following output:
   \code
Camera parameters for perspective projection without distortion:
  px = 600	 py = 600
  u0 = 320	 v0 = 240

Field of view (horizontal: 56.145 and vertical: 43.6028 degrees)
   \endcode

 */
void vpCameraParameters::initPersProjWithoutDistortion(const double cam_px, const double cam_py, const double cam_u0,
                                                       const double cam_v0)
{
  this->projModel = vpCameraParameters::perspectiveProjWithoutDistortion;

  this->px = cam_px;
  this->py = cam_py;
  this->u0 = cam_u0;
  this->v0 = cam_v0;
  this->kud = 0;
  this->kdu = 0;

  if (fabs(px) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(py) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->inv_px = 1. / px;
  this->inv_py = 1. / py;
}

/*!
  Initialization with specific parameters using perpective projection with
  distortion model.
  \param cam_px,cam_py : the ratio between the focal length and the size of a pixel.
  \param cam_u0,cam_v0 : principal points coordinates in pixels.
  \param cam_kud : undistorted to distorted radial distortion.
  \param cam_kdu : distorted to undistorted radial distortion.

   The following sample code shows how to use this function:
   \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<unsigned char> I(480, 640);
  double u0 = I.getWidth()  / 2.;
  double v0 = I.getHeight() / 2.;
  double px = 600;
  double py = 600;
  double kud = -0.19;
  double kdu = 0.20;
  vpCameraParameters cam;
  cam.initPersProjWithDistortion(px, py, u0, v0, kud, kdu);
  cam.computeFov(I.getWidth(), I.getHeight());
  std::cout << cam << std::endl;
  std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
            << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle())
            << " degrees)" << std::endl;
}
   \endcode
   It produces the following output:
   \code
Camera parameters for perspective projection with distortion:
  px = 600	 py = 600
  u0 = 320	 v0 = 240
  kud = -0.19
  kdu = 0.2

Field of view (horizontal: 56.14497387 and vertical: 43.60281897 degrees)
\endcode
*/
void vpCameraParameters::initPersProjWithDistortion(const double cam_px, const double cam_py, const double cam_u0,
                                                    const double cam_v0, const double cam_kud, const double cam_kdu)
{
  this->projModel = vpCameraParameters::perspectiveProjWithDistortion;

  this->px = cam_px;
  this->py = cam_py;
  this->u0 = cam_u0;
  this->v0 = cam_v0;
  this->kud = cam_kud;
  this->kdu = cam_kdu;

  if (fabs(px) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(py) < 1e-6) {
    vpERROR_TRACE("Camera parameter px = 0");
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->inv_px = 1. / px;
  this->inv_py = 1. / py;
}

/*!
  destructor

  nothing much to destroy...
*/
vpCameraParameters::~vpCameraParameters() {}

/*!
  initialization from another vpCameraParameters object
*/
void vpCameraParameters::init(const vpCameraParameters &c) { *this = c; }

/*!
  initialise the camera from a calibration matrix.
  Using a calibration matrix leads to a camera without distortion

  The K matrix in parameters must be like:

  \f$ K = \left(\begin{array}{ccc}
  p_x & 0 & u_0 \\
  0 & p_y & v_0  \\
  0 & 0 & 1
  \end{array} \right) \f$

  \param _K : the 3by3 calibration matrix
*/
void vpCameraParameters::initFromCalibrationMatrix(const vpMatrix &_K)
{
  if (_K.getRows() != 3 || _K.getCols() != 3) {
    throw vpException(vpException::dimensionError, "bad size for calibration matrix");
  }
  if (std::fabs(_K[2][2] - 1.0) > std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::badValue, "bad value: K[2][2] must be equal to 1");
  }
  initPersProjWithoutDistortion(_K[0][0], _K[1][1], _K[0][2], _K[1][2]);
}

/*!
   Initialize the camera model without distorsion from the image dimension and
the camera field of view. \param w : Image width. \param h : Image height.
   \param hfov : Camera horizontal field of view angle expressed in radians.
   \param vfov : Camera vertical field of view angle expressed in radians.

   The following sample code shows how to use this function:
   \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<unsigned char> I(480, 640);
  vpCameraParameters cam;
  double hfov = vpMath::rad(56);
  double vfov = vpMath::rad(43);
  cam.initFromFov(I.getWidth(), I.getHeight(), hfov, vfov);

  std::cout << cam << std::endl;
  std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
            << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle()) << " degrees)" << std::endl;
}
   \endcode
   It produces the following output:
   \code
Camera parameters for perspective projection without distortion:
  px = 601.832	 py = 609.275
  u0 = 320	 v0 = 240

Field of view (horizontal: 56 and vertical: 43 degrees)
   \endcode
 */
void vpCameraParameters::initFromFov(const unsigned int &w, const unsigned int &h, const double &hfov,
                                     const double &vfov)
{
  projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
  u0 = (double)w / 2.;
  v0 = (double)h / 2.;
  px = u0 / tan(hfov / 2);
  py = v0 / tan(vfov / 2);
  kud = 0;
  kdu = 0;
  inv_px = 1. / px;
  inv_py = 1. / py;
  computeFov(w, h);
}

/*!
  copy operator
 */
vpCameraParameters &vpCameraParameters::operator=(const vpCameraParameters &cam)
{
  projModel = cam.projModel;
  px = cam.px;
  py = cam.py;
  u0 = cam.u0;
  v0 = cam.v0;
  kud = cam.kud;
  kdu = cam.kdu;

  inv_px = cam.inv_px;
  inv_py = cam.inv_py;

  isFov = cam.isFov;
  m_hFovAngle = cam.m_hFovAngle;
  m_vFovAngle = cam.m_vFovAngle;
  width = cam.width;
  height = cam.height;
  fovNormals = cam.fovNormals;

  return *this;
}

/*!
  True if the two objects are absolutely identical.
 */
bool vpCameraParameters::operator==(const vpCameraParameters &c) const {
  if (projModel != c.projModel)
    return false;

  if (!vpMath::equal(px, c.px, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(py, c.py, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(u0, c.u0, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(v0, c.v0, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(kud, c.kud, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(kdu, c.kdu, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(inv_px, c.inv_px, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(inv_py, c.inv_py, std::numeric_limits<double>::epsilon()))
    return false;

  if (isFov != c.isFov ||
      !vpMath::equal(m_hFovAngle, c.m_hFovAngle, std::numeric_limits<double>::epsilon()) ||
      !vpMath::equal(m_vFovAngle, c.m_vFovAngle, std::numeric_limits<double>::epsilon()) ||
      width != c.width || height != c.height)
    return false;

  if (fovNormals.size() != c.fovNormals.size())
    return false;

  std::vector<vpColVector>::const_iterator it1 = fovNormals.begin();
  std::vector<vpColVector>::const_iterator it2 = c.fovNormals.begin();
  for (; it1 != fovNormals.end() && it2 != c.fovNormals.end(); ++it1, ++it2) {
    if (*it1 != *it2)
      return false;
  }

  return true;
}

/*!
  False if the two objects are absolutely identical.
 */
bool vpCameraParameters::operator!=(const vpCameraParameters &c) const {
  return !(*this == c);
}

/*!
  Compute angles and normals of the FOV.

  \param w : Width of the image
  \param h : Height of the image.
*/
void vpCameraParameters::computeFov(const unsigned int &w, const unsigned int &h)
{
  if ((!isFov || w != width || h != height) && w != 0 && h != 0) {
    fovNormals = std::vector<vpColVector>(4);

    isFov = true;

    double hFovAngle = atan(((double)w - u0) * (1.0 / px));
    double vFovAngle = atan((v0) * (1.0 / py));
    double minushFovAngle = atan((u0) * (1.0 / px));
    double minusvFovAngle = atan(((double)h - v0) * (1.0 / py));

    width = w;
    height = h;

    vpColVector n(3);
    n = 0;
    n[0] = 1.0;

    vpRotationMatrix Rleft(0, -minushFovAngle, 0);
    vpRotationMatrix Rright(0, hFovAngle, 0);

    vpColVector nLeft, nRight;

    nLeft = Rleft * (-n);
    fovNormals[0] = nLeft.normalize();

    nRight = Rright * n;
    fovNormals[1] = nRight.normalize();

    n = 0;
    n[1] = 1.0;

    vpRotationMatrix Rup(vFovAngle, 0, 0);
    vpRotationMatrix Rdown(-minusvFovAngle, 0, 0);

    vpColVector nUp, nDown;

    nUp = Rup * (-n);
    fovNormals[2] = nUp.normalize();

    nDown = Rdown * n;
    fovNormals[3] = nDown.normalize();

    m_hFovAngle = hFovAngle + minushFovAngle;
    m_vFovAngle = vFovAngle + minusvFovAngle;
  }
}

/*!
  Return the camera matrix \f$K\f$ given by:

  \f$ K = \left[\begin{array}{ccc}
  p_x & 0 & u_0 \\
  0 & p_y & v_0  \\
  0 & 0 & 1
  \end{array} \right] \f$

  \sa get_K_inverse()
*/
vpMatrix vpCameraParameters::get_K() const
{
  vpMatrix K(3, 3, 0.);
  K[0][0] = px;
  K[1][1] = py;
  K[0][2] = u0;
  K[1][2] = v0;
  K[2][2] = 1.0;

  return K;
}
/*!
  Return the inverted camera matrix \f$K^{-1}\f$ given by:

  \f$ K^{-1} = \left[\begin{array}{ccc}
  1/p_x & 0 & -u_0/p_x \\
  0 & 1/p_y & -v_0/p_y  \\
  0 & 0 & 1
  \end{array} \right] \f$

  \sa get_K()
*/
vpMatrix vpCameraParameters::get_K_inverse() const
{
  vpMatrix K_inv(3, 3, 0.);
  K_inv[0][0] = inv_px;
  K_inv[1][1] = inv_py;
  K_inv[0][2] = -u0 * inv_px;
  K_inv[1][2] = -v0 * inv_py;
  K_inv[2][2] = 1.0;

  return K_inv;
}

/*!
  Print the camera parameters on the standard output

  \sa operator<<(std::ostream &, const vpCameraParameters &)
*/
void vpCameraParameters::printParameters()
{
  std::ios::fmtflags original_flags(std::cout.flags());
  switch (projModel) {
  case vpCameraParameters::perspectiveProjWithoutDistortion:
    std::cout.precision(10);
    std::cout << "Camera parameters for perspective projection without distortion:" << std::endl;
    std::cout << "  px = " << px << "\t py = " << py << std::endl;
    std::cout << "  u0 = " << u0 << "\t v0 = " << v0 << std::endl;
    break;
  case vpCameraParameters::perspectiveProjWithDistortion:
    std::cout.precision(10);
    std::cout << "Camera parameters for perspective projection with distortion:" << std::endl;
    std::cout << "  px = " << px << "\t py = " << py << std::endl;
    std::cout << "  u0 = " << u0 << "\t v0 = " << v0 << std::endl;
    std::cout << "  kud = " << kud << std::endl;
    std::cout << "  kdu = " << kdu << std::endl;
    break;
  }
  // Restore ostream format
  std::cout.flags(original_flags);
}
/*!

  Print on the output stream \e os the camera parameters.

  \param os : Output stream.
  \param cam : Camera parameters.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpCameraParameters &cam)
{
  switch (cam.get_projModel()) {
  case vpCameraParameters::perspectiveProjWithoutDistortion:
    os << "Camera parameters for perspective projection without distortion:" << std::endl;
    os << "  px = " << cam.get_px() << "\t py = " << cam.get_py() << std::endl;
    os << "  u0 = " << cam.get_u0() << "\t v0 = " << cam.get_v0() << std::endl;
    break;
  case vpCameraParameters::perspectiveProjWithDistortion:
    std::ios_base::fmtflags original_flags = os.flags();
    os.precision(10);
    os << "Camera parameters for perspective projection with distortion:" << std::endl;
    os << "  px = " << cam.get_px() << "\t py = " << cam.get_py() << std::endl;
    os << "  u0 = " << cam.get_u0() << "\t v0 = " << cam.get_v0() << std::endl;
    os << "  kud = " << cam.get_kud() << std::endl;
    os << "  kdu = " << cam.get_kdu() << std::endl;

    os.flags(original_flags); // restore os to standard state
    break;
  }
  return os;
}
