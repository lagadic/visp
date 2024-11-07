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
 * Point feature.
 *
*****************************************************************************/

/*!
  \file vpPoint.cpp
  \brief Class that defines what is a 3D point.
*/

#include <visp3/core/vpFeatureDisplay.h>
#include <visp3/core/vpPoint.h>

BEGIN_VISP_NAMESPACE
void vpPoint::init()
{
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  p.resize(3);
  p = 0;
  p[index_2] = 1.;
  oP.resize(4);
  oP = 0.;
  oP[index_3] = 1.;
  cP.resize(4);
  cP = 0.;
  cP[index_3] = 1.;

  // default value Z (1 meters)
  set_Z(1);
}

vpPoint::vpPoint() { init(); }

/*!
  Construction from a 3D point with coordinates in object frame.
  \param oX, oY, oZ: Coordinates of a 3D point in object frame.
*/
vpPoint::vpPoint(double oX, double oY, double oZ)
{
  init();
  setWorldCoordinates(oX, oY, oZ);
}

/*!
  Construction from a 3D point with coordinates in object frame.

  \param oP_: Vector containing the coordinates of the 3D point in the object frame.
  This vector could be of dimension 3 or 4.
  - If dimension is 3, oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, 1) where oP[0]=oX, oP[1]=oY, oP[2]=oZ.
  - If dimension is 4, \e oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, oW) where oP[0]=oX, oP[1]=oY, oP[2]=oZ and oP[3]=oW.
*/
vpPoint::vpPoint(const vpColVector &oP_)
{
  init();
  setWorldCoordinates(oP_);
}

/*!
  Construction from a 3D point with coordinates in object frame.

  \param oP_: Vector containing the coordinates of the 3D point in the object frame.
  This vector could be of dimension 3 or 4.
  - If dimension is 3, oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, 1) where oP[0]=oX, oP[1]=oY, oP[2]=oZ.
  - If dimension is 4, \e oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, oW) where oP[0]=oX, oP[1]=oY, oP[2]=oZ and oP[3]=oW.
*/
vpPoint::vpPoint(const std::vector<double> &oP_)
{
  init();
  setWorldCoordinates(oP_);
}

/*!
  Set the 3D point object frame coordinates oP = (oX, oY, oZ, 1).

  \param oX, oY, oZ: Coordinates of a 3D point in the object frame.
*/
void vpPoint::setWorldCoordinates(double oX, double oY, double oZ)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  oP[index_0] = oX;
  oP[index_1] = oY;
  oP[index_2] = oZ;
  oP[index_3] = 1;
}

/*!
  Set the 3D point object frame coordinates.

  \param oP_: Vector containing the coordinates of the 3D point in the object frame.
  This vector could be of dimension 3 or 4.
  - If dimension is 3, oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, 1) where oP[0]=oX, oP[1]=oY, oP[2]=oZ.
  - If dimension is 4, \e oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, oW) where oP[0]=oX, oP[1]=oY, oP[2]=oZ and oP[3]=oW.
*/
void vpPoint::setWorldCoordinates(const vpColVector &oP_)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int val_3 = 3;
  const unsigned int val_4 = 4;

  if (oP_.size() == val_3) {
    oP[index_0] = oP_[index_0];
    oP[index_1] = oP_[index_1];
    oP[index_2] = oP_[index_2];
    oP[index_3] = 1.;
  }
  else if (oP_.size() == val_4) {
    oP[index_0] = oP_[index_0];
    oP[index_1] = oP_[index_1];
    oP[index_2] = oP_[index_2];
    oP[index_3] = oP_[index_3];
    oP /= oP[index_3];
  }
  else {
    throw(vpException(vpException::dimensionError, "Cannot initialize vpPoint from vector with size %d", oP_.size()));
  }
}

/*!
  Set the 3D point object frame coordinates.

  \param oP_: Vector containing the coordinates of the 3D point in the object frame.
  This vector could be of dimension 3 or 4.
  - If dimension is 3, oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, 1) where oP[0]=oX, oP[1]=oY, oP[2]=oZ.
  - If dimension is 4, \e oP corresponds to the normalized coordinates oP = (oX,
  oY, oZ, oW) where oP[0]=oX, oP[1]=oY, oP[2]=oZ and oP[3]=oW.
*/
void vpPoint::setWorldCoordinates(const std::vector<double> &oP_)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int val_3 = 3;
  const unsigned int val_4 = 4;

  if (oP_.size() == val_3) {
    oP[index_0] = oP_[index_0];
    oP[index_1] = oP_[index_1];
    oP[index_2] = oP_[index_2];
    oP[index_3] = 1.;
  }
  else if (oP_.size() == val_4) {
    oP[index_0] = oP_[index_0];
    oP[index_1] = oP_[index_1];
    oP[index_2] = oP_[index_2];
    oP[index_3] = oP_[index_3];
    oP /= oP[index_3];
  }
  else {
    throw(vpException(vpException::dimensionError, "Cannot initialize vpPoint from vector with size %d", oP_.size()));
  }
}

//! Get the point object frame coordinates.
void vpPoint::getWorldCoordinates(double &oX, double &oY, double &oZ)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  oX = oP[index_0];
  oY = oP[index_1];
  oZ = oP[index_2];
}

/*!
  Get the point object frame normalized coordinates.

  \param oP_: Normalized coordinates of the point in the
  object frame oP = (oX, oY, oZ, oW) as a 4-dim vector.
  */
void vpPoint::getWorldCoordinates(vpColVector &oP_) { oP_ = oP; }

/*!
  Get the point object frame normalized coordinates.

  \param oP_: Normalized coordinates of the point in the
  object frame oP = (oX, oY, oZ, oW) as a 4-dim vector.
  */
void vpPoint::getWorldCoordinates(std::vector<double> &oP_)
{
  oP_.resize(oP.size());
  unsigned int oP_size = oP.size();
  for (unsigned int i = 0; i < oP_size; ++i) {
    oP_[i] = oP[i];
  }
}

/*!
  Return the point object frame normalized coordinates.

  \return Normalized coordinates of the point in
  the object frame oP = (oX, oY, oZ, oW) as a 4-dim vector.
  */
vpColVector vpPoint::getWorldCoordinates(void) { return this->oP; }

/*!
  Compute the perspective projection of a point _cP.

  \param v_cP : 3-dim vector cP = (cX, cY, cZ) or 4-dim vector cP = (cX, cY, cZ, 1) corresponding
  to the normalized coordinates of the 3D point in the camera frame.
  \param v_p : Coordinates of the point in the
  image plane obtained by perspective projection.
*/
void vpPoint::projection(const vpColVector &v_cP, vpColVector &v_p) const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  v_p.resize(3, false);

  v_p[index_0] = v_cP[index_0] / v_cP[index_2];
  v_p[index_1] = v_cP[index_1] / v_cP[index_2];
  v_p[index_2] = 1;
}

/*!
  From the 3D coordinates of the point in the object frame oP that are set using for
  example setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the
  3D coordinates of the point in the camera frame cP = cMo * oP.

  \param cMo : Transformation from camera to object frame.
  \param v_cP : 3D normalized coordinates of the point in the camera frame cP = (cX, cY, cZ, 1).
*/
void vpPoint::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &v_cP) const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  v_cP.resize(4, false);

  v_cP[index_0] = (cMo[index_0][index_0] * oP[index_0]) + (cMo[index_0][index_1] * oP[index_1]) + (cMo[index_0][index_2] * oP[index_2]) + (cMo[index_0][index_3] * oP[index_3]);
  v_cP[index_1] = (cMo[index_1][index_0] * oP[index_0]) + (cMo[index_1][index_1] * oP[index_1]) + (cMo[index_1][index_2] * oP[index_2]) + (cMo[index_1][index_3] * oP[index_3]);
  v_cP[index_2] = (cMo[index_2][index_0] * oP[index_0]) + (cMo[index_2][index_1] * oP[index_1]) + (cMo[index_2][index_2] * oP[index_2]) + (cMo[index_2][index_3] * oP[index_3]);
  v_cP[index_3] = (cMo[index_3][index_0] * oP[index_0]) + (cMo[index_3][index_1] * oP[index_1]) + (cMo[index_3][index_2] * oP[index_2]) + (cMo[index_3][index_3] * oP[index_3]);

  double d = 1 / v_cP[index_3];
  v_cP[index_0] *= d;
  v_cP[index_1] *= d;
  v_cP[index_2] *= d;
  v_cP[index_3] *= d;
}

/*!
  From the 3D coordinates of the point in the object frame oP that are set using for
  example setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the
  3D coordinates of the point in the camera frame cP = cMo * oP.

  \param cMo : Transformation from camera to object frame.

*/
void vpPoint::changeFrame(const vpHomogeneousMatrix &cMo)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  double X = (cMo[index_0][index_0] * oP[index_0]) + (cMo[index_0][index_1] * oP[index_1]) + (cMo[index_0][index_2] * oP[index_2]) + (cMo[index_0][index_3] * oP[index_3]);
  double Y = (cMo[index_1][index_0] * oP[index_0]) + (cMo[index_1][index_1] * oP[index_1]) + (cMo[index_1][index_2] * oP[index_2]) + (cMo[index_1][index_3] * oP[index_3]);
  double Z = (cMo[index_2][index_0] * oP[index_0]) + (cMo[index_2][index_1] * oP[index_1]) + (cMo[index_2][index_2] * oP[index_2]) + (cMo[index_2][index_3] * oP[index_3]);
  double W = (cMo[index_3][index_0] * oP[index_0]) + (cMo[index_3][index_1] * oP[index_1]) + (cMo[index_3][index_2] * oP[index_2]) + (cMo[index_3][index_3] * oP[index_3]);

  double d = 1. / W;
  cP[index_0] = X * d;
  cP[index_1] = Y * d;
  cP[index_2] = Z * d;
  cP[index_3] = 1.;
}

//! For memory issue (used by the vpServo class only).
vpPoint *vpPoint::duplicate() const
{
  vpPoint *feature = new vpPoint(*this);
  return feature;
}

/*!
 * Display the projection of a 3D point in image \e I.
 * This method is non destructive wrt. cP and p internal 3D point parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The point is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpPoint::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  const unsigned int index_2 = 2;

  if (v_cP[index_2] < 0) { // no display if point is behind the camera
    return;
  }

  vpPoint::projection(v_cP, v_p);
  vpFeatureDisplay::displayPoint(v_p[0], v_p[1], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D point in image \e I.
 * This method is non destructive wrt. cP and p internal 3D point parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The point is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */
void vpPoint::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  const unsigned int index_2 = 2;

  if (v_cP[index_2] < 0) { // no display if point is behind the camera
    return;
  }

  vpPoint::projection(v_cP, v_p);
  vpFeatureDisplay::displayPoint(v_p[0], v_p[1], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D point in image \e I.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the point.
 * \param thickness : Thickness used to draw the point.
 */
void vpPoint::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                      unsigned int thickness)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D point in image \e I.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the point.
 * \param thickness : Thickness used to draw the point.
 */
void vpPoint::display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color,
                      unsigned int thickness)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, color, thickness);
}

// Get coordinates
//! Get the point cX coordinate in the camera frame.
double vpPoint::get_X() const { const unsigned int index_0 = 0; return cP[index_0]; }
//! Get the point cY coordinate in the camera frame.
double vpPoint::get_Y() const { const unsigned int index_1 = 1; return cP[index_1]; }
//! Get the point cZ coordinate in the camera frame.
double vpPoint::get_Z() const { const unsigned int index_2 = 2; return cP[index_2]; }
//! Get the point cW coordinate in the camera frame.
double vpPoint::get_W() const { const unsigned int index_3 = 3; return cP[index_3]; }

//! Get the point oX coordinate in the object frame.
double vpPoint::get_oX() const { const unsigned int index_0 = 0; return oP[index_0]; }
//! Get the point oY coordinate in the object frame.
double vpPoint::get_oY() const { const unsigned int index_1 = 1; return oP[index_1]; }
//! Get the point oZ coordinate in the object frame.
double vpPoint::get_oZ() const { const unsigned int index_2 = 2; return oP[index_2]; }
//! Get the point oW coordinate in the object frame.
double vpPoint::get_oW() const { const unsigned int index_3 = 3; return oP[index_3]; }

//! Get the point x coordinate in the image plane.
double vpPoint::get_x() const { const unsigned int index_0 = 0; return p[index_0]; }
//! Get the point y coordinate in the image plane.
double vpPoint::get_y() const { const unsigned int index_1 = 1; return p[index_1]; }
//! Get the point w coordinate in the image plane.
double vpPoint::get_w() const { const unsigned int index_2 = 2; return p[index_2]; }

/*!
  Perspective projection of the 3D point.

  Projection onto the image plane of the point. Update the object
  attribute p (2D homogeneous coordinates) according to object
  attribute cP (current 3D coordinates in the camera frame).

*/
void vpPoint::projection()
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  double d = 1 / cP[index_2];
  p[index_0] = cP[index_0] * d;
  p[index_1] = cP[index_1] * d;
  p[index_2] = 1;
}

//! Set the point cX coordinate in the camera frame.
void vpPoint::set_X(double cX) { const unsigned int index_0 = 0; cP[index_0] = cX; }
//! Set the point cY coordinate in the camera frame.
void vpPoint::set_Y(double cY) { const unsigned int index_1 = 1; cP[index_1] = cY; }
//! Set the point cZ coordinate in the camera frame.
void vpPoint::set_Z(double cZ) { const unsigned int index_2 = 2; cP[index_2] = cZ; }
//! Set the point cW coordinate in the camera frame.
void vpPoint::set_W(double cW) { const unsigned int index_3 = 3; cP[index_3] = cW; }

//! Set the point oX coordinate in the object frame.
void vpPoint::set_oX(double oX) { const unsigned int index_0 = 0; oP[index_0] = oX; }
//! Set the point oY coordinate in the object frame.
void vpPoint::set_oY(double oY) { const unsigned int index_1 = 1; oP[index_1] = oY; }
//! Set the point oZ coordinate in the object frame.
void vpPoint::set_oZ(double oZ) { const unsigned int index_2 = 2; oP[index_2] = oZ; }
//! Set the point oW coordinate in the object frame.
void vpPoint::set_oW(double oW) { const unsigned int index_3 = 3; oP[index_3] = oW; }

//! Set the point x coordinate in the image plane.
void vpPoint::set_x(double x) { const unsigned int index_0 = 0;  p[index_0] = x; }
//! Set the point y coordinate in the image plane.
void vpPoint::set_y(double y) { const unsigned int index_1 = 1; p[index_1] = y; }
//! Set the point w coordinate in the image plane.
void vpPoint::set_w(double w) { const unsigned int index_2 = 2; p[index_2] = w; }

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPoint & /* vpp */) { return (os << "vpPoint"); }
END_VISP_NAMESPACE
