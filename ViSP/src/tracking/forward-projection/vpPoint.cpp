/****************************************************************************
 *
 * $Id: vpPoint.cpp,v 1.10 2007-12-18 15:03:17 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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



//! perspective projection of a point
void
vpPoint::projection(const vpColVector &_cP, vpColVector &_p)
{
  _p.resize(3) ;

  _p[0] = _cP[0]/_cP[2] ;
  _p[1] = _cP[1]/_cP[2] ;
  _p[2] = 1 ;
}

//! perspective projection of the point
void
vpPoint::projection()
{

  p[0] = cP[0]/cP[2] ;
  p[1] = cP[1]/cP[2] ;
  p[2] = 1 ;
}



//! Compute the new 3D coordinates of the point in the new camera frame.
void
vpPoint::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{

  _cP.resize(4) ;

  _cP[0] = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
  _cP[1] = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
  _cP[2] = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
  _cP[3] = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;

  _cP /= _cP[3] ;
}


//! Update the 3D coordinates of the point (camera frame).
void
vpPoint::changeFrame(const vpHomogeneousMatrix &cMo)
{
  cP[0] = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
  cP[1] = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
  cP[2] = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
  cP[3] = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;

  cP /= cP[3] ;
}




/*! \brief change frame
 */
const vpPoint
operator*(const vpHomogeneousMatrix &cMo, const vpPoint& oP)
{
  vpPoint cP ;


  vpColVector v(4),v1(4) ;

  v[0] = oP.get_X() ;
  v[1] = oP.get_Y() ;
  v[2] = oP.get_Z() ;
  v[3] = oP.get_W() ;

  v1[0] = cMo[0][0]*v[0] + cMo[0][1]*v[1]+ cMo[0][2]*v[2]+ cMo[0][3]*v[3] ;
  v1[1] = cMo[1][0]*v[0] + cMo[1][1]*v[1]+ cMo[1][2]*v[2]+ cMo[1][3]*v[3] ;
  v1[2] = cMo[2][0]*v[0] + cMo[2][1]*v[1]+ cMo[2][2]*v[2]+ cMo[2][3]*v[3] ;
  v1[3] = cMo[3][0]*v[0] + cMo[3][1]*v[1]+ cMo[3][2]*v[2]+ cMo[3][3]*v[3] ;

  v1 /= v1[3] ;

  //  v1 = M*v ;
  cP.set_X(v1[0]) ;
  cP.set_Y(v1[1]) ;
  cP.set_Z(v1[2]) ;
  cP.set_W(v1[3]) ;


  return cP ;
}

/*! \brief change frame
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




//! for memory issue (used by the vpServo class only)
vpPoint *vpPoint::duplicate() const
{
  vpPoint *feature = new vpPoint(*this) ;
  return feature ;
}


void
vpPoint::display(vpImage<unsigned char> &I,
		 const vpHomogeneousMatrix &cMo,
		 const vpCameraParameters &cam,
		 const vpColor::vpColorType color)
{

  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  vpPoint::projection(_cP,_p) ;
  vpFeatureDisplay::displayPoint(_p[0],_p[1], cam, I, color) ;

}

void
vpPoint::display(vpImage<unsigned char> &I,
		 const vpHomogeneousMatrix &cMo,
		 const vpCameraParameters &cam,
		 const bool usedistortion,
		 const vpColor::vpColorType color)
{

  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  vpPoint::projection(_cP,_p) ;
  vpFeatureDisplay::displayPoint(_p[0],_p[1], cam, I, usedistortion, color) ;

}

std::ostream& operator<<(std::ostream& os, vpPoint& /* vpp */)
{
  return( os<<"vpPoint" );
}

void
vpPoint::display(vpImage<unsigned char> &I,
		 const vpCameraParameters &cam,
		 const vpColor::vpColorType color)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, color) ;
}

void
vpPoint::display(vpImage<unsigned char> &I,
		 const vpCameraParameters &cam,
		 const bool usedistortion,
		 const vpColor::vpColorType color)
{
  vpFeatureDisplay::displayPoint(p[0], p[1], cam, I, usedistortion, color) ;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
