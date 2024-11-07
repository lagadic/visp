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
 * Sphere feature.
 *
*****************************************************************************/

#include <visp3/core/vpFeatureDisplay.h>
#include <visp3/core/vpSphere.h>

BEGIN_VISP_NAMESPACE
/*!
 * Initialize internal sphere parameters.
 */
  void vpSphere::init()
{
  oP.resize(4);
  cP.resize(4);

  p.resize(5);
}

/*!
 * Set sphere 3D parameters from a 4-dim vector that contains 3D
 * coordinates of its center and its radius.
 * The 3D coordinates of the center are defined in the sphere frame.
 *
 * \param oP_ : 4-dim vector that contains [oX oY oZ R]^T where
 * oX, oY, oZ are the 3D coordinates of the sphere center in [m] expressed in the object frame,
 * and R is the sphere radius in [m].
 */
void vpSphere::setWorldCoordinates(const vpColVector &oP_) { this->oP = oP_; }

/*!
 * Set sphere 3D parameters from the 3D coordinates of its center and its radius.
 * The 3D coordinates of the center are defined in the sphere frame.
 *
 * \param oX : 3D coordinate of the sphere center along X axis in [m].
 * \param oY : 3D coordinate of the sphere center along X axis in [m].
 * \param oZ : 3D coordinate of the sphere center along X axis in [m].
 * \param R : Sphere radius in [m].
 */
void vpSphere::setWorldCoordinates(double oX, double oY, double oZ, double R)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  oP[index_0] = oX;
  oP[index_1] = oY;
  oP[index_2] = oZ;
  oP[index_3] = R;
}

/*!
 * Default constructor.
 */
vpSphere::vpSphere() { init(); }

/*!
 * Create a sphere from a 4-dim vector that contains 3D coordinates of its center and its radius.
 * The 3D coordinates of the center are defined in the sphere frame.
 *
 * \param oP_ : 4-dim vector that contains [oX oY oZ R]^T where
 * oX, oY, oZ are the 3D coordinates of the sphere center in [m] expressed in the sphere frame,
 * and R is the sphere radius in [m].
 */
vpSphere::vpSphere(const vpColVector &oP_)
{
  init();
  setWorldCoordinates(oP_);
}

/*!
 * Create a sphere from the 3D coordinates of its center and its radius.
 * The 3D coordinates of the center are defined in the sphere frame.
 *
 * \param oX : 3D coordinate of the sphere center along X axis in [m].
 * \param oY : 3D coordinate of the sphere center along X axis in [m].
 * \param oZ : 3D coordinate of the sphere center along X axis in [m].
 * \param R : Sphere radius in [m].
 */
vpSphere::vpSphere(double oX, double oY, double oZ, double R)
{
  init();
  setWorldCoordinates(oX, oY, oZ, R);
}

/*!
 * Perspective projection of the sphere.
 * This method updates internal parameters (cP and p).
 *
 * See vpSphere::projection(const vpColVector &, vpColVector &) const for a more
 * detailed description of the parameters.
 */
void vpSphere::projection() { projection(cP, p); }

/*!
 * Perspective projection of the sphere.
 * Internal parameters (cP and p) are not modified.
 *
 * \param[in] cP_ : 4-dim vector corresponding to the sphere parameters in the camera frame.
 * \param[out] p_ : 5-dim vector corresponding to the sphere parameters in the image plane. It
 * contains the following parameters: x, y, n20, n11, n02 where:
 * - x, y are the normalized coordinates of the ellipse centroid (ie the
 * perspective projection of a 3D circle becomes a 2D ellipse in the image) in
 * the image plane.
 * - n20, n11, n02 which are the second order centered moments of
 * the ellipse normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
 * \f$\mu_{ij}\f$ are the centered moments and a the area).
 */
void vpSphere::projection(const vpColVector &cP_, vpColVector &p_) const
{
  p_.resize(5, false);
  double x0, y0, z0;
  double E, A, B;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;

  // calcul des parametres M20, M11, M02 de l'ellipse
  double s, r;
  r = cP_[index_3];

  x0 = cP_[index_0];
  y0 = cP_[index_1];
  z0 = cP_[index_2];

  s = (r * r) - (y0 * y0) - (z0 * z0);

  if ((s = ((z0 * z0) - (r * r))) < 0.0) {
    throw(vpException(vpException::fatalError, "Error: Sphere is behind image plane"));
  }

  p_[0] = (x0 * z0) / s; // x
  p_[1] = (y0 * z0) / s; // y

  if (fabs(x0) > 1e-6) {
    double e = y0 / x0;
    double b = r / sqrt(s);
    double a = ((x0 * x0) + (y0 * y0) + (z0 * z0)) - (r * r);
    if (a < 0.0) {
      throw(vpException(vpException::fatalError, "Error: Sphere is behind image plane"));
    }
    a = (r * sqrt(a)) / s;
    if (fabs(e) <= 1.0) {
      E = e;
      A = a;
      B = b;
    }
    else {
      E = -1.0 / e;
      A = b;
      B = a;
    }
  }
  else {
    E = 0.0;
    A = r / sqrt(s);
    B = (r * sqrt(((y0 * y0) + (z0 * z0)) - (r * r))) / s;
  }

  // Chaumette PhD Thesis 1990, eq 2.72 divided by 4 since n_ij = mu_ij_chaumette_thesis / 4
  double det = 4 * (1.0 + vpMath::sqr(E));
  double n20 = (vpMath::sqr(A) + vpMath::sqr(B * E)) / det;
  double n11 = ((vpMath::sqr(A) - vpMath::sqr(B)) * E) / det;
  double n02 = (vpMath::sqr(B) + vpMath::sqr(A * E)) / det;

  p_[index_2] = n20;
  p_[index_3] = n11;
  p_[index_4] = n02;
}

/*!
 * Perspective projection of the sphere.
 * Internal sphere parameters are modified in cP.
 *
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 */
void vpSphere::changeFrame(const vpHomogeneousMatrix &cMo) { changeFrame(cMo, cP); }

/*!
 * Perspective projection of the sphere.
 * This method doesn't modify internal sphere parameters cP.
 *
 * \param cMo : Homogeneous transformation from camera frame to sphere frame.
 * \param cP_ : Parameters of the sphere in the camera frame (cX, cY, cZ, R).
 */
void vpSphere::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_) const
{
  cP_.resize(4, false);

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  double x0, y0, z0; // variables intermediaires

  x0 = (cMo[index_0][0] * oP[0]) + (cMo[index_0][1] * oP[1]) + (cMo[index_0][index_2] * oP[index_2]) + cMo[index_0][index_3];
  y0 = (cMo[index_1][0] * oP[0]) + (cMo[index_1][1] * oP[1]) + (cMo[index_1][index_2] * oP[index_2]) + cMo[index_1][index_3];
  z0 = (cMo[index_2][0] * oP[0]) + (cMo[index_2][1] * oP[1]) + (cMo[index_2][index_2] * oP[index_2]) + cMo[index_2][index_3];

  cP_[index_3] = oP[index_3];

  cP_[index_0] = x0;
  cP_[index_1] = y0;
  cP_[index_2] = z0;
}

//! For memory issue (used by the vpServo class only).
vpSphere *vpSphere::duplicate() const
{
  vpSphere *feature = new vpSphere(*this);
  return feature;
}

/*!
 * Display the projection of a 3D sphere in image \e I.
 * This method is non destructive wrt. cP and p internal sphere parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The sphere is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpSphere::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  projection(v_cP, v_p);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(v_p[index_0], v_p[index_1], v_p[index_2], v_p[index_3], v_p[index_4], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D sphere in image \e I.
 * This method is non destructive wrt. cP and p internal sphere parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The sphere is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpSphere::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  projection(v_cP, v_p);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(v_p[index_0], v_p[index_1], v_p[index_2], v_p[index_3], v_p[index_4], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D sphere in image \e I.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpSphere::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                       unsigned int thickness)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(p[index_0], p[index_1], p[index_2], p[index_3], p[index_4], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D sphere in image \e I.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpSphere::display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color,
                       unsigned int thickness)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(p[index_0], p[index_1], p[index_2], p[index_3], p[index_4], cam, I, color, thickness);
}
END_VISP_NAMESPACE
