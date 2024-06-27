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
 *
 * Description:
 * Camera intrinsic parameters.
 */

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
#include <visp3/core/vpException.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpMath.h>

BEGIN_VISP_NAMESPACE

const double vpCameraParameters::DEFAULT_PX_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_PY_PARAMETER = 600.0;
const double vpCameraParameters::DEFAULT_U0_PARAMETER = 192.0;
const double vpCameraParameters::DEFAULT_V0_PARAMETER = 144.0;
const double vpCameraParameters::DEFAULT_KUD_PARAMETER = 0.0;
const double vpCameraParameters::DEFAULT_KDU_PARAMETER = 0.0;
const vpCameraParameters::vpCameraParametersProjType vpCameraParameters::DEFAULT_PROJ_TYPE =
vpCameraParameters::perspectiveProjWithoutDistortion;

/*!
 * Default constructor.
 * By default, a perspective projection without distortion model is set.
 *
 * \sa init()
 */
vpCameraParameters::vpCameraParameters()
  : m_px(DEFAULT_PX_PARAMETER), m_py(DEFAULT_PY_PARAMETER), m_u0(DEFAULT_U0_PARAMETER), m_v0(DEFAULT_V0_PARAMETER),
  m_kud(DEFAULT_KUD_PARAMETER), m_kdu(DEFAULT_KDU_PARAMETER), m_dist_coefs(), m_width(0), m_height(0), m_isFov(false),
  m_hFovAngle(0), m_vFovAngle(0), m_fovNormals(), m_inv_px(1. / DEFAULT_PX_PARAMETER), m_inv_py(1. / DEFAULT_PY_PARAMETER),
  m_projModel(DEFAULT_PROJ_TYPE)
{
  init();
}

/*!
 * Copy constructor
 */
vpCameraParameters::vpCameraParameters(const vpCameraParameters &c)
  : m_px(DEFAULT_PX_PARAMETER), m_py(DEFAULT_PY_PARAMETER), m_u0(DEFAULT_U0_PARAMETER), m_v0(DEFAULT_V0_PARAMETER),
  m_kud(DEFAULT_KUD_PARAMETER), m_kdu(DEFAULT_KDU_PARAMETER), m_dist_coefs(), m_width(0), m_height(0), m_isFov(false),
  m_hFovAngle(0), m_vFovAngle(0), m_fovNormals(), m_inv_px(1. / DEFAULT_PX_PARAMETER), m_inv_py(1. / DEFAULT_PY_PARAMETER),
  m_projModel(DEFAULT_PROJ_TYPE)
{
  init(c);
}

/*!
 * Constructor for perspective projection without distortion model
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 */
vpCameraParameters::vpCameraParameters(double cam_px, double cam_py, double cam_u0, double cam_v0)
  : m_px(DEFAULT_PX_PARAMETER), m_py(DEFAULT_PY_PARAMETER), m_u0(DEFAULT_U0_PARAMETER), m_v0(DEFAULT_V0_PARAMETER),
  m_kud(DEFAULT_KUD_PARAMETER), m_kdu(DEFAULT_KDU_PARAMETER), m_dist_coefs(), m_width(0), m_height(0), m_isFov(false),
  m_hFovAngle(0), m_vFovAngle(0), m_fovNormals(), m_inv_px(1. / DEFAULT_PX_PARAMETER), m_inv_py(1. / DEFAULT_PY_PARAMETER),
  m_projModel(DEFAULT_PROJ_TYPE)
{
  initPersProjWithoutDistortion(cam_px, cam_py, cam_u0, cam_v0);
}

/*!
 * Constructor for perspective projection with distortion model
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 * \param cam_kud : Undistorted to distorted radial distortion.
 * \param cam_kdu : Distorted to undistorted radial distortion.
 */
vpCameraParameters::vpCameraParameters(double cam_px, double cam_py, double cam_u0, double cam_v0, double cam_kud,
                                       double cam_kdu)
  : m_px(DEFAULT_PX_PARAMETER), m_py(DEFAULT_PY_PARAMETER), m_u0(DEFAULT_U0_PARAMETER), m_v0(DEFAULT_V0_PARAMETER),
  m_kud(DEFAULT_KUD_PARAMETER), m_kdu(DEFAULT_KDU_PARAMETER), m_dist_coefs(), m_width(0), m_height(0), m_isFov(false),
  m_hFovAngle(0), m_vFovAngle(0), m_fovNormals(), m_inv_px(1. / DEFAULT_PX_PARAMETER), m_inv_py(1. / DEFAULT_PY_PARAMETER),
  m_projModel(DEFAULT_PROJ_TYPE)
{
  initPersProjWithDistortion(cam_px, cam_py, cam_u0, cam_v0, cam_kud, cam_kdu);
}

/*!
 * Constructor for projection with Kannala-Brandt distortion model
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 * \param coefficients  : distortion model coefficients
 */
vpCameraParameters::vpCameraParameters(double cam_px, double cam_py, double cam_u0, double cam_v0,
                                       const std::vector<double> &coefficients)
  : m_px(DEFAULT_PX_PARAMETER), m_py(DEFAULT_PY_PARAMETER), m_u0(DEFAULT_U0_PARAMETER), m_v0(DEFAULT_V0_PARAMETER),
  m_kud(DEFAULT_KUD_PARAMETER), m_kdu(DEFAULT_KDU_PARAMETER), m_dist_coefs(), m_width(0), m_height(0), m_isFov(false),
  m_hFovAngle(0), m_vFovAngle(0), m_fovNormals(), m_inv_px(1. / DEFAULT_PX_PARAMETER), m_inv_py(1. / DEFAULT_PY_PARAMETER),
  m_projModel(DEFAULT_PROJ_TYPE)
{
  initProjWithKannalaBrandtDistortion(cam_px, cam_py, cam_u0, cam_v0, coefficients);
}

/*!
 * \brief Basic initialization with the default parameters.
 */
void vpCameraParameters::init()
{
  if (fabs(this->m_px) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(this->m_py) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->m_inv_px = 1. / this->m_px;
  this->m_inv_py = 1. / this->m_py;
}

/*!
 * Initialization with specific parameters using perspective projection without
 * distortion model.
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 *
 * The following sample code shows how to use this function:
 * \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpImage.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<unsigned char> I(480, 640);
 *   double u0 = I.getWidth()  / 2.;
 *   double v0 = I.getHeight() / 2.;
 *   double px = 600;
 *   double py = 600;
 *   vpCameraParameters cam;
 *   cam.initPersProjWithoutDistortion(px, py, u0, v0);
 *   cam.computeFov(I.getWidth(), I.getHeight());
 *   std::cout << cam << std::endl;
 *   std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
 *             << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle())
 *             << " degrees)" << std::endl;
 * }
 * \endcode
 * It produces the following output:
 * \code
 * Camera parameters for perspective projection without distortion:
 *   px = 600   py = 600
 *   u0 = 320   v0 = 240
 *
 * Field of view (horizontal: 56.145 and vertical: 43.6028 degrees)
 * \endcode
 */
void vpCameraParameters::initPersProjWithoutDistortion(double cam_px, double cam_py, double cam_u0, double cam_v0)
{
  this->m_projModel = vpCameraParameters::perspectiveProjWithoutDistortion;

  this->m_px = cam_px;
  this->m_py = cam_py;
  this->m_u0 = cam_u0;
  this->m_v0 = cam_v0;
  this->m_kud = 0;
  this->m_kdu = 0;

  this->m_dist_coefs.clear();

  if (fabs(m_px) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(m_py) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter py = 0"));
  }
  this->m_inv_px = 1. / m_px;
  this->m_inv_py = 1. / m_py;
}

/*!
 * Initialization with specific parameters using perspective projection with
 * distortion model.
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 * \param cam_kud : Undistorted to distorted radial distortion.
 * \param cam_kdu : Distorted to undistorted radial distortion.
 *
 *  The following sample code shows how to use this function:
 *  \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpImage.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<unsigned char> I(480, 640);
 *   double u0 = I.getWidth()  / 2.;
 *   double v0 = I.getHeight() / 2.;
 *   double px = 600;
 *   double py = 600;
 *   double kud = -0.19;
 *   double kdu = 0.20;
 *   vpCameraParameters cam;
 *   cam.initPersProjWithDistortion(px, py, u0, v0, kud, kdu);
 *   cam.computeFov(I.getWidth(), I.getHeight());
 *   std::cout << cam << std::endl;
 *   std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
 *             << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle())
 *             << " degrees)" << std::endl;
 * }
 * \endcode
 *  It produces the following output:
 *  \code
 * Camera parameters for perspective projection with distortion:
 *   px = 600   py = 600
 *   u0 = 320   v0 = 240
 *   kud = -0.19
 *   kdu = 0.2
 *
 * Field of view (horizontal: 56.14497387 and vertical: 43.60281897 degrees)
 * \endcode
 */
void vpCameraParameters::initPersProjWithDistortion(double cam_px, double cam_py, double cam_u0, double cam_v0,
                                                    double cam_kud, double cam_kdu)
{
  this->m_projModel = vpCameraParameters::perspectiveProjWithDistortion;

  this->m_px = cam_px;
  this->m_py = cam_py;
  this->m_u0 = cam_u0;
  this->m_v0 = cam_v0;
  this->m_kud = cam_kud;
  this->m_kdu = cam_kdu;
  this->m_dist_coefs.clear();

  if (fabs(m_px) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(m_py) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->m_inv_px = 1. / m_px;
  this->m_inv_py = 1. / m_py;
}

/*!
 * Initialization with specific parameters using Kannala-Brandt distortion model
 *
 * \param cam_px : Pixel size along x axis (horizontal).
 * \param cam_py : Pixel size along y axis (vertical)
 * \param cam_u0 : Principal point coordinate in pixel along x.
 * \param cam_v0 : Principal point coordinate in pixel along y.
 * \param coefficients  : Distortion coefficients.
 */
void vpCameraParameters::initProjWithKannalaBrandtDistortion(double cam_px, double cam_py, double cam_u0, double cam_v0,
                                                             const std::vector<double> &coefficients)
{
  this->m_projModel = vpCameraParameters::ProjWithKannalaBrandtDistortion;

  this->m_px = cam_px;
  this->m_py = cam_py;
  this->m_u0 = cam_u0;
  this->m_v0 = cam_v0;

  this->m_kud = 0.0;
  this->m_kdu = 0.0;

  if (fabs(m_px) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  if (fabs(m_py) < 1e-6) {
    throw(vpException(vpException::divideByZeroError, "Camera parameter px = 0"));
  }
  this->m_inv_px = 1. / m_px;
  this->m_inv_py = 1. / m_py;

  this->m_dist_coefs = coefficients;
}

/*!
 * Destructor that does nothing.
 */
vpCameraParameters::~vpCameraParameters() { }

/*!
 * Initialization from another vpCameraParameters object.
 */
void vpCameraParameters::init(const vpCameraParameters &c) { *this = c; }

/*!
 * Initialise the camera from a calibration matrix.
 * Using a calibration matrix leads to a camera without distortion.
 *
 * The K matrix in parameters must be like:
 *
 * \f$ K = \left(\begin{array}{ccc}
 * p_x & 0 & u_0 \\
 * 0 & p_y & v_0  \\
 * 0 & 0 & 1
 * \end{array} \right) \f$
 *
 * \param K : the 3-by-3 calibration matrix
 */
void vpCameraParameters::initFromCalibrationMatrix(const vpMatrix &K)
{
  const unsigned int nparam = 3;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  if ((K.getRows() != nparam) || (K.getCols() != nparam)) {
    throw vpException(vpException::dimensionError, "bad size for calibration matrix");
  }
  if (std::fabs(K[index_2][index_2] - 1.0) > std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::badValue, "bad value: K[2][2] must be equal to 1");
  }
  initPersProjWithoutDistortion(K[index_0][index_0], K[index_1][index_1], K[index_0][index_2], K[index_1][index_2]);
}

/*!
 * Initialize the camera model without distortion from the image dimension and
 * the camera field of view.
 * \param w : Image width.
 * \param h : Image height.
 * \param hfov : Camera horizontal field of view angle expressed in radians.
 * \param vfov : Camera vertical field of view angle expressed in radians.
 *
 *  The following sample code shows how to use this function:
 *  \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpImage.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<unsigned char> I(480, 640);
 *   vpCameraParameters cam;
 *   double hfov = vpMath::rad(56);
 *   double vfov = vpMath::rad(43);
 *   cam.initFromFov(I.getWidth(), I.getHeight(), hfov, vfov);
 *
 *   std::cout << cam << std::endl;
 *   std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
 *             << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle()) << " degrees)" << std::endl;
 * }
 * \endcode
 * It produces the following output:
 * \code
 * Camera parameters for perspective projection without distortion:
 *   px = 601.832   py = 609.275
 *   u0 = 320   v0 = 240
 *
 * Field of view (horizontal: 56 and vertical: 43 degrees)
 * \endcode
 */
void vpCameraParameters::initFromFov(const unsigned int &w, const unsigned int &h, const double &hfov,
                                     const double &vfov)
{
  m_projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
  m_u0 = static_cast<double>(w) / 2.;
  m_v0 = static_cast<double>(h) / 2.;
  m_px = m_u0 / tan(hfov / 2.);
  m_py = m_v0 / tan(vfov / 2.);
  m_kud = 0;
  m_kdu = 0;
  m_inv_px = 1. / m_px;
  m_inv_py = 1. / m_py;
  computeFov(w, h);
}

/*!
 * Copy operator.
 */
vpCameraParameters &vpCameraParameters::operator=(const vpCameraParameters &cam)
{
  m_projModel = cam.m_projModel;
  m_px = cam.m_px;
  m_py = cam.m_py;
  m_u0 = cam.m_u0;
  m_v0 = cam.m_v0;
  m_kud = cam.m_kud;
  m_kdu = cam.m_kdu;
  m_dist_coefs = cam.m_dist_coefs;

  m_inv_px = cam.m_inv_px;
  m_inv_py = cam.m_inv_py;

  m_isFov = cam.m_isFov;
  m_hFovAngle = cam.m_hFovAngle;
  m_vFovAngle = cam.m_vFovAngle;
  m_width = cam.m_width;
  m_height = cam.m_height;
  m_fovNormals = cam.m_fovNormals;

  return *this;
}

/*!
 * True if the two objects are absolutely identical.
 */
bool vpCameraParameters::operator==(const vpCameraParameters &c) const
{
  if (m_projModel != c.m_projModel) {
    return false;
  }

  // maximum allowed conditional operators shall be maximum 3
  if ((!vpMath::equal(m_px, c.m_px, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_py, c.m_py, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_u0, c.m_u0, std::numeric_limits<double>::epsilon()))) {
    return false;
  }
  if ((!vpMath::equal(m_v0, c.m_v0, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_kud, c.m_kud, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_kdu, c.m_kdu, std::numeric_limits<double>::epsilon()))) {
    return false;
  }
  if ((!vpMath::equal(m_inv_px, c.m_inv_px, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_inv_py, c.m_inv_py, std::numeric_limits<double>::epsilon()))) {
    return false;
  }

  if (m_dist_coefs.size() != c.m_dist_coefs.size()) {
    return false;
  }

  size_t m_dist_coefs_size = m_dist_coefs.size();
  for (size_t i = 0; i < m_dist_coefs_size; ++i) {
    if (!vpMath::equal(m_dist_coefs[i], c.m_dist_coefs[i], std::numeric_limits<double>::epsilon())) {
      return false;
    }
  }

  if ((m_isFov != c.m_isFov) || (!vpMath::equal(m_hFovAngle, c.m_hFovAngle, std::numeric_limits<double>::epsilon())) ||
      (!vpMath::equal(m_vFovAngle, c.m_vFovAngle, std::numeric_limits<double>::epsilon()))) {
    return false;
  }
  if ((m_width != c.m_width) || (m_height != c.m_height)) {
    return false;
  }

  if (m_fovNormals.size() != c.m_fovNormals.size()) {
    return false;
  }

  std::vector<vpColVector>::const_iterator it1 = m_fovNormals.begin();
  std::vector<vpColVector>::const_iterator it2 = c.m_fovNormals.begin();
  for (; (it1 != m_fovNormals.end()) && (it2 != c.m_fovNormals.end()); ++it1, ++it2) {
    if (*it1 != *it2) {
      return false;
    }
  }

  return true;
}

/*!
 * False if the two objects are absolutely identical.
 */
bool vpCameraParameters::operator!=(const vpCameraParameters &c) const { return !(*this == c); }

/*!
 * Compute angles and normals of the FOV.
 *
 * \param w : Width of the image
 * \param h : Height of the image.
 */
void vpCameraParameters::computeFov(const unsigned int &w, const unsigned int &h)
{
  bool cond1 = (!m_isFov) || (w != m_width) || (h != m_height);
  if (cond1 && (w != 0) && (h != 0)) {
    const unsigned int nparam_3 = 3;
    const unsigned int nparam_4 = 4;
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    m_fovNormals = std::vector<vpColVector>(nparam_4);

    m_isFov = true;

    double hFovAngle = atan((static_cast<double>(w) - m_u0) * (1.0 / m_px));
    double vFovAngle = atan(m_v0 * (1.0 / m_py));
    double minushFovAngle = atan(m_u0 * (1.0 / m_px));
    double minusvFovAngle = atan((static_cast<double>(h) - m_v0) * (1.0 / m_py));

    m_width = w;
    m_height = h;

    vpColVector n(nparam_3);
    n = 0;
    n[0] = 1.0;

    vpRotationMatrix Rleft(0, -minushFovAngle, 0);
    vpRotationMatrix Rright(0, hFovAngle, 0);

    vpColVector nLeft, nRight;

    nLeft = Rleft * (-n);
    m_fovNormals[index_0] = nLeft.normalize();

    nRight = Rright * n;
    m_fovNormals[index_1] = nRight.normalize();

    n = 0;
    n[1] = 1.0;

    vpRotationMatrix Rup(vFovAngle, 0, 0);
    vpRotationMatrix Rdown(-minusvFovAngle, 0, 0);

    vpColVector nUp, nDown;

    nUp = Rup * (-n);
    m_fovNormals[index_2] = nUp.normalize();

    nDown = Rdown * n;
    m_fovNormals[index_3] = nDown.normalize();

    m_hFovAngle = hFovAngle + minushFovAngle;
    m_vFovAngle = vFovAngle + minusvFovAngle;
  }
}

/*!
 * Return the camera matrix \f$K\f$ given by:
 *
 * \f$ K = \left[\begin{array}{ccc}
 * p_x & 0 & u_0 \\
 * 0 & p_y & v_0  \\
 * 0 & 0 & 1
 * \end{array} \right] \f$
 *
 * \sa get_K_inverse()
 */
vpMatrix vpCameraParameters::get_K() const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  vpMatrix K(3, 3, 0.);
  K[index_0][index_0] = m_px;
  K[index_1][index_1] = m_py;
  K[index_0][index_2] = m_u0;
  K[index_1][index_2] = m_v0;
  K[index_2][index_2] = 1.0;

  return K;
}
/*!
 * Return the inverted camera matrix \f$K^{-1}\f$ given by:
 *
 * \f$ K^{-1} = \left[\begin{array}{ccc}
 * 1/p_x & 0 & -u_0/p_x \\
 * 0 & 1/p_y & -v_0/p_y  \\
 * 0 & 0 & 1
 * \end{array} \right] \f$
 *
 * \sa get_K()
 */
vpMatrix vpCameraParameters::get_K_inverse() const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  vpMatrix K_inv(3, 3, 0.);
  K_inv[index_0][index_0] = m_inv_px;
  K_inv[index_1][index_1] = m_inv_py;
  K_inv[index_0][index_2] = -m_u0 * m_inv_px;
  K_inv[index_1][index_2] = -m_v0 * m_inv_py;
  K_inv[index_2][index_2] = 1.0;

  return K_inv;
}

/*!
 * Print the camera parameters on the standard output.
 *
 * \sa operator<<(std::ostream &, const vpCameraParameters &)
 */
void vpCameraParameters::printParameters()
{
  size_t m_dist_coefs_size = m_dist_coefs.size();
  std::ios::fmtflags original_flags(std::cout.flags());
  switch (m_projModel) {
  case vpCameraParameters::perspectiveProjWithoutDistortion: {
    std::cout.precision(10);
    std::cout << "Camera parameters for perspective projection without distortion:" << std::endl;
    std::cout << "  px = " << m_px << "\t py = " << m_py << std::endl;
    std::cout << "  u0 = " << m_u0 << "\t v0 = " << m_v0 << std::endl;
    break;
  }
  case vpCameraParameters::perspectiveProjWithDistortion: {
    std::cout.precision(10);
    std::cout << "Camera parameters for perspective projection with distortion:" << std::endl;
    std::cout << "  px = " << m_px << "\t py = " << m_py << std::endl;
    std::cout << "  u0 = " << m_u0 << "\t v0 = " << m_v0 << std::endl;
    std::cout << "  kud = " << m_kud << std::endl;
    std::cout << "  kdu = " << m_kdu << std::endl;
    break;
  }
  case vpCameraParameters::ProjWithKannalaBrandtDistortion: {
    std::cout << "  Coefficients: ";
    for (size_t i = 0; i < m_dist_coefs_size; ++i) {
      std::cout << " " << m_dist_coefs[i];
    }
    std::cout << std::endl;
    break;
  }
  default: {
    std::cout << "projection model not identified" << std::endl;
  }
  }
  // Restore ostream format
  std::cout.flags(original_flags);
}

/*!
 * Print on the output stream \e os the camera parameters.
 *
 * \param os : Output stream.
 * \param cam : Camera parameters.
 */
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpCameraParameters &cam)
{
  switch (cam.get_projModel()) {
  case vpCameraParameters::perspectiveProjWithoutDistortion: {
    os << "Camera parameters for perspective projection without distortion:" << std::endl;
    os << "  px = " << cam.get_px() << "\t py = " << cam.get_py() << std::endl;
    os << "  u0 = " << cam.get_u0() << "\t v0 = " << cam.get_v0() << std::endl;
    break;
  }
  case vpCameraParameters::perspectiveProjWithDistortion: {
    std::ios_base::fmtflags original_flags = os.flags();
    const unsigned int precision = 10;
    os.precision(precision);
    os << "Camera parameters for perspective projection with distortion:" << std::endl;
    os << "  px = " << cam.get_px() << "\t py = " << cam.get_py() << std::endl;
    os << "  u0 = " << cam.get_u0() << "\t v0 = " << cam.get_v0() << std::endl;
    os << "  kud = " << cam.get_kud() << std::endl;
    os << "  kdu = " << cam.get_kdu() << std::endl;
    os.flags(original_flags); // restore os to standard state
    break;
  }
  case vpCameraParameters::ProjWithKannalaBrandtDistortion: {
    os << "Camera parameters for projection with Kannala-Brandt distortion:" << std::endl;
    os << "  px = " << cam.get_px() << "\t py = " << cam.get_py() << std::endl  << "  u0 = " << cam.get_u0() << "\t v0 = " << cam.get_v0() << std::endl;
    os << "  Coefficients: ";
    std::vector<double> tmp_coefs = cam.getKannalaBrandtDistortionCoefficients();
    size_t tmp_coefs_size = tmp_coefs.size();
    for (size_t i = 0; i < tmp_coefs_size; ++i) {
      os << " " << tmp_coefs[i];
    }
    os << std::endl;
    break;
  }
  default: {
    std::cout << "Unidentified camera parameters model" << std::endl;
  }
  }
  return os;
}
END_VISP_NAMESPACE
