/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Point feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpPoint.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureDisplay.h>

#include <visp/vpHomography.h>

/*!
  \file vpPoint.cpp
  \brief   class that defines what is a point
*/



void
vpPoint::init()
{
  p.resize(3) ; p = 0 ; p[2] = 1 ;
  oP.resize(4) ; oP = 0 ; oP[3] = 1 ;
  cP.resize(4) ; cP = 0 ; cP[3] = 1 ;

  //default value Z (1 meters)
  set_Z(1) ;
}

vpPoint::vpPoint()
{
  init() ;
}

//! set the point world coordinates
void
vpPoint::setWorldCoordinates(const double ox,
                             const double oy,
                             const double oz)
{
  oP[0] = ox ;
  oP[1] = oy ;
  oP[2] = oz ;
  oP[3] = 1 ;
}


void
vpPoint::setWorldCoordinates(const vpColVector &_oP)
{
  oP[0] = _oP[0] ;
  oP[1] = _oP[1] ;
  oP[2] = _oP[2] ;
  oP[3] = _oP[3] ;

  oP /= oP[3] ;
}

void
vpPoint::getWorldCoordinates(double& ox,
			   double& oy,
			   double& oz)
{
  ox = oP[0] ;
  oy = oP[1] ;
  oz = oP[2] ;
}


void
vpPoint::getWorldCoordinates(vpColVector &_oP)
{
  _oP[0] = oP[0] ;
  _oP[1] = oP[1] ;
  _oP[2] = oP[2] ;
  _oP[3] = oP[3] ;
}


vpColVector
vpPoint::getWorldCoordinates(void)
{
  return this->oP;
}



/*!
  Compute the perspective projection of a point _cP.

  \param _cP : Three dimension vector that corresponds to the coordinates of the point in the camera frame.
  \param _p : Coordinates of the point in the image plane obtained by perspective projection.
*/
void
vpPoint::projection(const vpColVector &_cP, vpColVector &_p)
{
  _p.resize(3) ;

  _p[0] = _cP[0]/_cP[2] ;
  _p[1] = _cP[1]/_cP[2] ;
  _p[2] = 1 ;
}

/*!
  From the 3D coordinates of the point in the object frame set using for example
  setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the 3D coordinates
  of the point in the camera frame.

  \param cMo : Transformation from camera to object frame.
  \param _cP : 3D coordinates of the point in the camera frame.
*/
void
vpPoint::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{

  _cP.resize(4) ;

  _cP[0] = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
  _cP[1] = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
  _cP[2] = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
  _cP[3] = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;

  double d = 1/_cP[3] ;
  _cP[0] *=d ;
  _cP[1] *=d ;
  _cP[2] *=d ; 
  _cP[3] *=d ; ;
}

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

//! For memory issue (used by the vpServo class only).
vpPoint *vpPoint::duplicate() const
{
  vpPoint *feature = new vpPoint(*this) ;
  return feature ;
}

/*!
  Display the point in the image.
*/
void
vpPoint::display(const vpImage<unsigned char> &I,
                 const vpHomogeneousMatrix &cMo,
                 const vpCameraParameters &cam,
                 const vpColor &color,
                 const unsigned int thickness)
{

  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;

  if(_cP[2] < 0)	// no display if point is behind the camera
	  return;

  vpPoint::projection(_cP,_p) ;
  vpFeatureDisplay::displayPoint(_p[0],_p[1], cam, I, color, thickness) ;
}

/*!
  Display the point in the image.
*/
void
vpPoint::display(const vpImage<vpRGBa> &I,
                 const vpHomogeneousMatrix &cMo,
                 const vpCameraParameters &cam,
                 const vpColor &color,
                 const unsigned int thickness)
{
  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;

  if(_cP[2] < 0)	// no display if point is behind the camera
	  return;

  vpPoint::projection(_cP,_p) ;
  vpFeatureDisplay::displayPoint(_p[0],_p[1], cam, I, color, thickness) ;
}

std::ostream& operator<<(std::ostream& os, vpPoint& /* vpp */)
{
  return( os<<"vpPoint" );
}

vpPoint&
vpPoint::operator=(const vpPoint&  vpp )
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
void
vpPoint::display(const vpImage<unsigned char> &I,
                 const vpCameraParameters &cam,
                 const vpColor &color,
                 const unsigned int thickness)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, color, thickness) ;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
