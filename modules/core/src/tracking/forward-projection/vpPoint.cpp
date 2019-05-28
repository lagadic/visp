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
 * Point feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpFeatureDisplay.h>
#include <visp3/core/vpPoint.h>

/*!
  \file vpPoint.cpp
  \brief   class that defines what is a point
*/

void vpPoint::init()
{
  p.resize(3);
  p = 0;
  p[2] = 1;
  oP.resize(4);
  oP = 0;
  oP[3] = 1;
  cP.resize(4);
  cP = 0;
  cP[3] = 1;

  // default value Z (1 meters)
  set_Z(1);
}

vpPoint::vpPoint() { init(); }

/*!
  Construction from 3D coordinates in the object frame.
  \param oX, oY, oZ: Coordinates of a 3D point in the object frame.
*/
vpPoint::vpPoint(double oX, double oY, double oZ)
{
  init();
  setWorldCoordinates(oX, oY, oZ);
}

/*!
  Construction from 3D point world coordinates. We mean here the coordinates
  of the point in the object frame. \param P: Vector containing the
  coordinates of the 3D point in the object frame. This vector could be of
  dimension 3 or 4.
  - If dimension is 3, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, 1) where P[0]=X, P[1]=Y, P[2]=Z.
  - If dimension is 4, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, W) where P[0]=X, P[1]=Y, P[2]=Z and P[3]=W.
*/
vpPoint::vpPoint(const vpColVector &P)
{
  init();
  setWorldCoordinates(P);
}

/*!
  Construction from 3D point world coordinates. We mean here the coordinates
  of the point in the object frame. \param P: Vector containing the
  coordinates of the 3D point in the object frame. This vector could be of
  dimension 3 or 4.
  - If dimension is 3, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, 1) where P[0]=X, P[1]=Y, P[2]=Z.
  - If dimension is 4, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, W) where P[0]=X, P[1]=Y, P[2]=Z and P[3]=W.
*/
vpPoint::vpPoint(const std::vector<double> &P)
{
  init();
  setWorldCoordinates(P);
}

/*!
  Set the 3D point world coordinates. We mean here the coordinates of the
  point in the object frame. \param oX, oY, oZ: Coordinates of a 3D point in
  the object frame.
*/
void vpPoint::setWorldCoordinates(const double oX, const double oY, const double oZ)
{
  oP[0] = oX;
  oP[1] = oY;
  oP[2] = oZ;
  oP[3] = 1;
}

/*!
  Set the 3D point world coordinates. We mean here the coordinates of the
  point in the object frame. \param P: Vector containing the coordinates of
  the 3D point in the object frame. This vector could be of dimension 3 or 4.
  - If dimension is 3, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, 1) where P[0]=X, P[1]=Y, P[2]=Z.
  - If dimension is 4, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, W) where P[0]=X, P[1]=Y, P[2]=Z and P[3]=W.
*/
void vpPoint::setWorldCoordinates(const vpColVector &P)
{
  if (P.size() == 3) {
    oP[0] = P[0];
    oP[1] = P[1];
    oP[2] = P[2];
    oP[3] = 1.;
  } else if (P.size() == 4) {
    oP[0] = P[0];
    oP[1] = P[1];
    oP[2] = P[2];
    oP[3] = P[3];
    oP /= oP[3];
  } else {
    throw(vpException(vpException::dimensionError, "Cannot initialize vpPoint from vector with size %d", P.size()));
  }
}

/*!
  Set the 3D point world coordinates. We mean here the coordinates of the
  point in the object frame. \param P: Vector containing the coordinates of
  the 3D point in the object frame. This vector could be of dimension 3 or 4.
  - If dimension is 3, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, 1) where P[0]=X, P[1]=Y, P[2]=Z.
  - If dimension is 4, \e P corresponds to the normalized coordinates P = (X,
  Y, Z, W) where P[0]=X, P[1]=Y, P[2]=Z and P[3]=W.
*/
void vpPoint::setWorldCoordinates(const std::vector<double> &P)
{
  if (P.size() == 3) {
    oP[0] = P[0];
    oP[1] = P[1];
    oP[2] = P[2];
    oP[3] = 1.;
  } else if (P.size() == 4) {
    oP[0] = P[0];
    oP[1] = P[1];
    oP[2] = P[2];
    oP[3] = P[3];
    oP /= oP[3];
  } else {
    throw(vpException(vpException::dimensionError, "Cannot initialize vpPoint from vector with size %d", P.size()));
  }
}

//! Get the point world coordinates. We mean here the coordinates of the point
//! in the object frame.
void vpPoint::getWorldCoordinates(double &oX, double &oY, double &oZ)
{
  oX = oP[0];
  oY = oP[1];
  oZ = oP[2];
}

/*!
  Get the point world coordinates. We mean here the coordinates of the point
  in the object frame. \param P: Normalized coordinates of the point in the
  object frame P = (X, Y, Z, 1)
  */
void vpPoint::getWorldCoordinates(vpColVector &P) { P = oP; }
/*!
  Get the point world coordinates. We mean here the coordinates of the point
  in the object frame. \param P: Normalized coordinates of the point in the
  object frame P = (X, Y, Z, 1)
  */
void vpPoint::getWorldCoordinates(std::vector<double> &P)
{
  P.resize(oP.size());
  for (unsigned int i = 0; i < oP.size(); i++)
    P[i] = oP[i];
}

/*!
  Return the point world coordinates. We mean here the coordinates of the
  point in the object frame. \return Normalized coordinates of the point in
  the object frame P = (X, Y, Z, 1)
  */
vpColVector vpPoint::getWorldCoordinates(void) { return this->oP; }

/*!
  Compute the perspective projection of a point _cP.

  \param _cP : Three dimension vector that corresponds to the coordinates of
  the point in the camera frame. \param _p : Coordinates of the point in the
  image plane obtained by perspective projection.
*/
void vpPoint::projection(const vpColVector &_cP, vpColVector &_p)
{
  _p.resize(3);

  _p[0] = _cP[0] / _cP[2];
  _p[1] = _cP[1] / _cP[2];
  _p[2] = 1;
}

/*!
  From the 3D coordinates of the point in the object frame set using for
  example setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the
  3D coordinates of the point in the camera frame.

  \param cMo : Transformation from camera to object frame.
  \param _cP : 3D coordinates of the point in the camera frame.
*/
void vpPoint::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{

  _cP.resize(4);

  _cP[0] = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2] + cMo[0][3] * oP[3];
  _cP[1] = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2] + cMo[1][3] * oP[3];
  _cP[2] = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2] + cMo[2][3] * oP[3];
  _cP[3] = cMo[3][0] * oP[0] + cMo[3][1] * oP[1] + cMo[3][2] * oP[2] + cMo[3][3] * oP[3];

  double d = 1 / _cP[3];
  _cP[0] *= d;
  _cP[1] *= d;
  _cP[2] *= d;
  _cP[3] *= d;
  ;
}

/*!
  From the 3D coordinates of the point in the object frame set using for
  example setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the
  3D coordinates of the point in the camera frame.

  \param cMo : Transformation from camera to object frame.

*/
void vpPoint::changeFrame(const vpHomogeneousMatrix &cMo)
{
  double X = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2] + cMo[0][3] * oP[3];
  double Y = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2] + cMo[1][3] * oP[3];
  double Z = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2] + cMo[2][3] * oP[3];
  double W = cMo[3][0] * oP[0] + cMo[3][1] * oP[1] + cMo[3][2] * oP[2] + cMo[3][3] * oP[3];

  double d = 1 / W;
  cP[0] = X * d;
  cP[1] = Y * d;
  cP[2] = Z * d;
  cP[3] = 1;
}

#if 0
/*!
  From the coordinates of the point in camera frame b and the transformation between
  camera frame a and camera frame b computes the coordinates of the point in camera frame a.

  \param aMb : 3D transformation between camera frame a and b.
  \param bP : 3D coordinates of the point in camera frame bP.

  \return A point with 3D coordinates in the camera frame a. The coordinates in the world or object
  frame are set to the same coordinates than the one in the camera frame.
*/
const vpPoint
operator*(const vpHomogeneousMatrix &aMb, const vpPoint& bP)
{
  vpPoint aP ;

  vpColVector v(4),v1(4) ;

  v[0] = bP.get_X() ;
  v[1] = bP.get_Y() ;
  v[2] = bP.get_Z() ;
  v[3] = bP.get_W() ;

  v1[0] = aMb[0][0]*v[0] + aMb[0][1]*v[1]+ aMb[0][2]*v[2]+ aMb[0][3]*v[3] ;
  v1[1] = aMb[1][0]*v[0] + aMb[1][1]*v[1]+ aMb[1][2]*v[2]+ aMb[1][3]*v[3] ;
  v1[2] = aMb[2][0]*v[0] + aMb[2][1]*v[1]+ aMb[2][2]*v[2]+ aMb[2][3]*v[3] ;
  v1[3] = aMb[3][0]*v[0] + aMb[3][1]*v[1]+ aMb[3][2]*v[2]+ aMb[3][3]*v[3] ;

  v1 /= v1[3] ;

  //  v1 = M*v ;
  aP.set_X(v1[0]) ;
  aP.set_Y(v1[1]) ;
  aP.set_Z(v1[2]) ;
  aP.set_W(v1[3]) ;
 
  aP.set_oX(v1[0]) ;
  aP.set_oY(v1[1]) ;
  aP.set_oZ(v1[2]) ;
  aP.set_oW(v1[3]) ;
 
  return aP ;
}

/*!
  From the coordinates of the point in image plane b and the homography between image
  a and b computes the coordinates of the point in image plane a.

  \param aHb : Homography between image a and b.
  \param bP : 2D coordinates of the point in the image plane b.

  \return A point with 2D coordinates in the image plane a.
*/
const vpPoint
operator*(const vpHomography &aHb, const vpPoint& bP)
{
  vpPoint aP ;
  vpColVector v(3),v1(3) ;

  v[0] = bP.get_x() ;
  v[1] = bP.get_y() ;
  v[2] = bP.get_w() ;

  v1[0] = aHb[0][0]*v[0] + aHb[0][1]*v[1]+ aHb[0][2]*v[2] ;
  v1[1] = aHb[1][0]*v[0] + aHb[1][1]*v[1]+ aHb[1][2]*v[2] ;
  v1[2] = aHb[2][0]*v[0] + aHb[2][1]*v[1]+ aHb[2][2]*v[2] ;

  //  v1 = M*v ;
  aP.set_x(v1[0]) ;
  aP.set_y(v1[1]) ;
  aP.set_w(v1[2]) ;

  return aP ;
}
#endif
//! For memory issue (used by the vpServo class only).
vpPoint *vpPoint::duplicate() const
{
  vpPoint *feature = new vpPoint(*this);
  return feature;
}

/*!
  Display the point in the image.
*/
void vpPoint::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const vpColor &color, const unsigned int thickness)
{

  vpColVector _cP, _p;
  changeFrame(cMo, _cP);

  if (_cP[2] < 0) // no display if point is behind the camera
    return;

  vpPoint::projection(_cP, _p);
  vpFeatureDisplay::displayPoint(_p[0], _p[1], cam, I, color, thickness);
}

/*!
  Display the point in the image.
*/
void vpPoint::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                      const vpColor &color, const unsigned int thickness)
{
  vpColVector _cP, _p;
  changeFrame(cMo, _cP);

  if (_cP[2] < 0) // no display if point is behind the camera
    return;

  vpPoint::projection(_cP, _p);
  vpFeatureDisplay::displayPoint(_p[0], _p[1], cam, I, color, thickness);
}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPoint & /* vpp */) { return (os << "vpPoint"); }

vpPoint &vpPoint::operator=(const vpPoint &vpp)
{
  p = vpp.p;
  cP = vpp.cP;
  oP = vpp.oP;
  cPAvailable = vpp.cPAvailable;

  return *this;
}

/*!
  Display the point in the image.
*/
void vpPoint::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                      const unsigned int thickness)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, color, thickness);
}

// Get coordinates
//! Get the point X coordinate in the camera frame.
double vpPoint::get_X() const { return cP[0]; }
//! Get the point Y coordinate in the camera frame.
double vpPoint::get_Y() const { return cP[1]; }
//! Get the point Z coordinate in the camera frame.
double vpPoint::get_Z() const { return cP[2]; }
//! Get the point W coordinate in the camera frame.
double vpPoint::get_W() const { return cP[3]; }

//! Get the point X coordinate in the object frame.
double vpPoint::get_oX() const { return oP[0]; }
//! Get the point Y coordinate in the object frame.
double vpPoint::get_oY() const { return oP[1]; }
//! Get the point Z coordinate in the object frame.
double vpPoint::get_oZ() const { return oP[2]; }
//! Get the point W coordinate in the object frame.
double vpPoint::get_oW() const { return oP[3]; }

//! Get the point x coordinate in the image plane.
double vpPoint::get_x() const { return p[0]; }
//! Get the point y coordinate in the image plane.
double vpPoint::get_y() const { return p[1]; }
//! Get the point w coordinate in the image plane.
double vpPoint::get_w() const { return p[2]; }

/*!
  Perspective projection of the point.

  Projection onto the //image plane of the point. Update the object
  attribute p (2D //homogeneous coordinates) according to object
  attribute cP (current //3D coordinates in the camera frame).

*/
void vpPoint::projection()
{
  double d = 1 / cP[2];
  p[0] = cP[0] * d;
  p[1] = cP[1] * d;
  p[2] = 1;
}

//! Set the point X coordinate in the camera frame.
void vpPoint::set_X(const double X) { cP[0] = X; }
//! Set the point Y coordinate in the camera frame.
void vpPoint::set_Y(const double Y) { cP[1] = Y; }
//! Set the point Z coordinate in the camera frame.
void vpPoint::set_Z(const double Z) { cP[2] = Z; }
//! Set the point W coordinate in the camera frame.
void vpPoint::set_W(const double W) { cP[3] = W; }

//! Set the point X coordinate in the object frame.
void vpPoint::set_oX(const double oX) { oP[0] = oX; }
//! Set the point Y coordinate in the object frame.
void vpPoint::set_oY(const double oY) { oP[1] = oY; }
//! Set the point Z coordinate in the object frame.
void vpPoint::set_oZ(const double oZ) { oP[2] = oZ; }
//! Set the point W coordinate in the object frame.
void vpPoint::set_oW(const double oW) { oP[3] = oW; }

//! Set the point x coordinate in the image plane.
void vpPoint::set_x(const double x) { p[0] = x; }
//! Set the point y coordinate in the image plane.
void vpPoint::set_y(const double y) { p[1] = y; }
//! Set the point w coordinate in the image plane.
void vpPoint::set_w(const double w) { p[2] = w; }
