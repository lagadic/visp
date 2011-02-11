/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Cylinder feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpCylinder.h>
#include <visp/vpFeatureDisplay.h>


void
vpCylinder::init()
{

  oP.resize(7) ;
  cP.resize(7) ;

  p.resize(4) ;
}

void
vpCylinder::setWorldCoordinates(const vpColVector& oP)
{
  this->oP = oP ;
}

void
vpCylinder::setWorldCoordinates(const double A, const double B,
			       const double C,
			       const double X0, const double Y0,
			       const double Z0,
			       const double R)
{
  oP[0] = A ;
  oP[1] = B ;
  oP[2] = C ;
  oP[3] = X0 ;
  oP[4] = Y0 ;
  oP[5] = Z0 ;
  oP[6] = R ;
}



vpCylinder::vpCylinder()
{
  init() ;
}


vpCylinder::vpCylinder(const vpColVector& oP)
{
  init() ;
  setWorldCoordinates(oP) ;
}

vpCylinder::vpCylinder(const double A, const double B,
	   const double C,
	   const double X0, const double Y0,
	   const double Z0,
	   const double R)
{
  init() ;
  setWorldCoordinates(A,  B,   C,
		      X0, Y0, Z0,
		      R) ;
}

vpCylinder::~vpCylinder()
{
}


//! perspective projection of the cylinder
void
vpCylinder::projection()
{
  projection(cP,p) ;
}


//! perspective projection of the cylinder
void
vpCylinder::projection(const vpColVector &cP, vpColVector &p)
{
  //calcul de la scene 2-D

  double co, si, e, x0, y0, z0;
  double A,B,C, X0, Y0, Z0, R ;
  double s, a, b, c;


  A = cP[0] ;
  B = cP[1] ;
  C = cP[2] ;
  X0 = cP[3] ;
  Y0 = cP[4] ;
  Z0 = cP[5] ;
  R= cP[6] ;

  s = X0*X0 + Y0*Y0 + Z0*Z0 - R*R
    - ( A*X0 + B*Y0 + C*Z0) * (A*X0 + B*Y0 + C*Z0);
  s = sqrt(s);
  a = (1-A*A)*X0 - A*B*Y0 - A*C*Z0;
  b = - A*B*X0 + (1-B*B)*Y0 - B*C*Z0;
  c = - A*C*X0  - B*C*Y0  + (1-C*C)*Z0;
  x0 = C*Y0 - B*Z0;
  y0 = A*Z0 - C*X0;
  z0 = B*X0 - A*Y0;


  // rho1 / theta1
  co = R*a/s-x0;
  si = R*b/s-y0;
  e = sqrt(co*co + si*si);
  p[0]  =  -(R*c/s-z0)/e ;  // rho1
  p[1] =  atan2(si,co) ; // theta 1

  while (p[1] > M_PI/2)  { p[1] -= M_PI ; p[0] *= -1 ; }
  while (p[1] < -M_PI/2) { p[1] += M_PI ; p[0] *= -1 ; }

  // rho2 / theta2
  co = R*a/s+x0;
  si = R*b/s+y0;
  e = sqrt(co*co + si*si);
  p[2]  =  -( R*c/s+z0 )/e ; //rho2
  p[3]  =  atan2( si,co ) ;  //theta2


  while (p[3] > M_PI/2)  { p[3] -= M_PI ; p[2] *= -1 ; }
  while (p[3] < -M_PI/2) { p[3] += M_PI ; p[2] *= -1 ; }

//  std::cout << p.t() << std::endl ;
}

//! perspective projection of the cylinder
void
vpCylinder::changeFrame(const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo,cP) ;
}

//! perspective projection of the cylinder
void
vpCylinder::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP)
{
  double X1, Y1, Z1;
  double X2, Y2, Z2;
  double s, a, b, c;

  double oA,oB,oC, oX0, oY0, oZ0 ;
  oA = oP[0] ;
  oB = oP[1] ;
  oC = oP[2] ;
  oX0 = oP[3] ;
  oY0 = oP[4] ;
  oZ0 = oP[5] ;

  //calc_scene-3d /udd/marchand/simu/C++/Lib_Simu/sp_mire.c

  X1 = cMo[0][0]*oA + cMo[0][1]*oB  + cMo[0][2]*oC ;
  Y1 = cMo[1][0]*oA + cMo[1][1]*oB  + cMo[1][2]*oC ;
  Z1 = cMo[2][0]*oA + cMo[2][1]*oB  + cMo[2][2]*oC ;
  s = sqrt ( X1*X1 + Y1*Y1 + Z1*Z1 );
  a = X1 / s;
  b = Y1 / s;
  c = Z1 / s;

  // set axis coordinates  in camera frame
  cP[0] = a ;
  cP[1] = b ;
  cP[2] = c ;


  X2 = cMo[0][3] + cMo[0][0]*oX0 + cMo[0][1]*oY0 + cMo[0][2]*oZ0;
  Y2 = cMo[1][3] + cMo[1][0]*oX0 + cMo[1][1]*oY0 + cMo[1][2]*oZ0;
  Z2 = cMo[2][3] + cMo[2][0]*oX0 + cMo[2][1]*oY0 + cMo[2][2]*oZ0;
 // set point coordinates  in camera frame
  cP[3] = X2 ;
  cP[4] = Y2 ;
  cP[5] = Z2 ;
  /*
  if ( fabs(a) > 0.25 )
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Y2 - b*X2;
    yy1 = a*Z2 - c*X2;
    xx = -( b*xx1 + c*yy1);
    yy = (( a*a + c*c ) * xx1 - b*c*yy1 ) /a;
    zz = ( -b*c*xx1 + ( a*a + b*b )*yy1) /a;

    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  else if ( fabs(b) >0.25 )
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Y2 - b*X2;
    yy1 = c*Y2 - b*Z2;
    xx = - (( b*b + c*c ) * xx1 - a*c*yy1 ) /b;
    yy = a*xx1 + c*yy1;
    zz = - ( -a*c*xx1 + (a*a + b*b) * yy1 ) /b;


    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  else
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Z2 - c*X2;
    yy1 = b*Z2 - c*Y2;
    xx = (-( b*b + c*c ) * xx1 - a*c*yy1 ) /c;
    yy = ( a*b*xx1 - ( a*a + c*c )*yy1) /c;
    zz = a*xx1 + b*yy1;

    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  */
  //radius
  cP[6] = oP[6] ;

}

//! for memory issue (used by the vpServo class only)
vpCylinder *vpCylinder::duplicate() const
{
  vpCylinder *feature = new vpCylinder(*this) ;
  return feature ;
}

void
vpCylinder::display(const vpImage<unsigned char> &I,
		    const vpHomogeneousMatrix &cMo,
		    const vpCameraParameters &cam,
		    const vpColor color,
		    const unsigned int thickness)
{

  vpColVector _cP(7), _p(4) ;
  changeFrame(cMo,_cP) ;
  projection(_cP,_p) ;
  vpFeatureDisplay::displayCylinder(_p[0],_p[1], _p[2], _p[3],
				    cam, I, color, thickness) ;

}


void
vpCylinder::display(const vpImage<unsigned char> &I,
		    const vpCameraParameters &cam,
		    const vpColor color,
		    const unsigned int thickness)
{
  vpFeatureDisplay::displayCylinder(p[0], p[1], p[2], p[3], 
				    cam, I, color, thickness) ;
}
