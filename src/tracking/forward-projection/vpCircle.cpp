/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Visual feature circle.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpCircle.h>

#include <visp/vpFeatureDisplay.h>

void
vpCircle::init()
{

  oP.resize(7) ;
  cP.resize(7) ;

  p.resize(5) ;
}

/*! 
  Set the world coordinates of the circle from the intersection of a plane and a sphere.  
  We mean here the coordinates of the circle in the object frame
 
  \param oP : oP[0], oP[1], oP[2] correspond to A, B, C from the plane equation Ax + By + Cz = 0.
              oP[3], oP[4], oP[5] correspond to X, Y, Z the coordinates of the center of the sphere.
              oP[6] corresponds to the radius of the sphere.
*/
void
vpCircle::setWorldCoordinates(const vpColVector& oP)
{
  this->oP = oP ;
}

/*! 
  Set the world coordinates of the circle from the intersection of a plane and a sphere. 
  We mean here the coordinates of the circle in the object frame
 
  \param A : A from the plane equation Ax + By + Cz = 0.
  \param B : A from the plane equation Ax + By + Cz = 0.
  \param C : A from the plane equation Ax + By + Cz = 0.
  \param X0 : X Coordinate of the center of the sphere.
  \param Y0 : Y Coordinate of the center of the sphere.
  \param Z0 : Z Coordinate of the center of the sphere.
  \param R : Radius of the sphere.
*/
void
vpCircle::setWorldCoordinates(const double A, const double B,
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



vpCircle::vpCircle()
{
  init() ;
}

/*! 
  Construct the circle from the intersection of a plane and a sphere.  
 
  \param oP : oP[0], oP[1], oP[2] correspond to A, B, C from the plane equation Ax + By + Cz = 0.
              oP[3], oP[4], oP[5] correspond to X, Y, Z the coordinates of the center of the sphere.
              oP[6] corresponds to the radius of the sphere.
              
  \sa setWorldCoordinates()
*/
vpCircle::vpCircle(const vpColVector& oP)
{
  init() ;
  setWorldCoordinates(oP) ;
}

/*! 
  Construct the circle from the intersection of a plane and a sphere. 
 
  \param A : A from the plane equation Ax + By + Cz = 0.
  \param B : A from the plane equation Ax + By + Cz = 0.
  \param C : A from the plane equation Ax + By + Cz = 0.
  \param X0 : X Coordinate of the center of the sphere.
  \param Y0 : Y Coordinate of the center of the sphere.
  \param Z0 : Z Coordinate of the center of the sphere.
  \param R : Radius of the sphere.
  
  \sa setWorldCoordinates()
*/
vpCircle::vpCircle(const double A, const double B,
		   const double C,
		   const double X0, const double Y0,
		   const double Z0,
		   const double R)
{
  init() ;
  setWorldCoordinates(A,  B,  C,
		      X0, Y0, Z0,
		      R) ;
}

vpCircle::~vpCircle()
{
}




//! perspective projection of the circle
void
vpCircle::projection()
{
  projection(cP,p) ;
}

//! perspective projection of the circle
void
vpCircle::projection(const vpColVector &cP, vpColVector &p)
{

  vpColVector K(6) ;

  {
    double A = cP[0] ;
    double B = cP[1] ;
    double C = cP[2] ;

    double X0 = cP[3] ;
    double Y0 = cP[4] ;
    double Z0 = cP[5] ;

    double r =  cP[6];

    // projection
    double s = X0*X0 + Y0*Y0 + Z0*Z0 - r*r ;
    double det = A*X0+B*Y0+C*Z0;
    A = A/det ;
    B = B/det ;
    C = C/det ;

    K[0] = 1 - 2*A*X0 + A*A*s;
    K[1] = 1 - 2*B*Y0 + B*B*s;
    K[2] = -A*Y0 - B*X0 + A*B*s;
    K[3] = -C*X0 - A*Z0 + A*C*s;
    K[4] = -C*Y0 - B*Z0 + B*C*s;
    K[5] = 1 - 2*C*Z0 + C*C*s;

  }
  double det  = K[2]*K[2] -K[0]*K[1];
  if (fabs(det) < 1e-8)
  {
    vpERROR_TRACE("division par 0") ;
    throw(vpException(vpException::divideByZeroError,
		      "division par 0")) ;

  }


  double xc = (K[1]*K[3]-K[2]*K[4])/det;
  double yc = (K[0]*K[4]-K[2]*K[3])/det;

  double c = sqrt( (K[0]-K[1])*(K[0]-K[1]) + 4*K[2]*K[2] );
  double s = 2*(K[0]*xc*xc + 2*K[2]*xc*yc + K[1]*yc*yc - K[5]);

  double A,B,E ;

  if (fabs(K[2])<1e-6)
  {
    E = 0.0;
    if (K[0] > K[1])
    {
      A = sqrt(s/(K[0] + K[1] + c));
      B = sqrt(s/(K[0] + K[1] - c));
    }
    else
    {
      A = sqrt(s/(K[0] + K[1] - c));
      B = sqrt(s/(K[0] + K[1] + c));
    }
  }
  else
  {
    E = (K[1] - K[0] + c)/(2*K[2]);
    if ( fabs(E) > 1.0)
    {
      A = sqrt(s/(K[0] + K[1] + c));
      B = sqrt(s/(K[0] + K[1] - c));
    }
    else
    {
      A = sqrt(s/(K[0] + K[1] - c));
      B = sqrt(s/(K[0] + K[1] + c));
      E = -1.0/E;
    }
  }

  det =  (1.0 + vpMath::sqr(E));
  double m20 = (vpMath::sqr(A) +  vpMath::sqr(B*E))  /det ;
  double m11 = (vpMath::sqr(A)  - vpMath::sqr(B)) *E / det ;
  double m02 = (vpMath::sqr(B) + vpMath::sqr(A*E))   / det ;

  p[0] = xc ;
  p[1] = yc ;
  p[2] = m20 ;
  p[3] = m11 ;
  p[4] = m02 ;
}

//! perspective projection of the circle
void
vpCircle::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP)
{

  double A,B,C ;
  A = cMo[0][0]*oP[0] + cMo[0][1]*oP[1]  + cMo[0][2]*oP[2];
  B = cMo[1][0]*oP[0] + cMo[1][1]*oP[1]  + cMo[1][2]*oP[2];
  C = cMo[2][0]*oP[0] + cMo[2][1]*oP[1]  + cMo[2][2]*oP[2];

  double X0,Y0,Z0 ;
  X0 = cMo[0][3] + cMo[0][0]*oP[3] + cMo[0][1]*oP[4] + cMo[0][2]*oP[5];
  Y0 = cMo[1][3] + cMo[1][0]*oP[3] + cMo[1][1]*oP[4] + cMo[1][2]*oP[5];
  Z0 = cMo[2][3] + cMo[2][0]*oP[3] + cMo[2][1]*oP[4] + cMo[2][2]*oP[5];
  double R = oP[6] ;

  cP[0] = A ;
  cP[1] = B ;
  cP[2] = C ;

  cP[3] = X0 ;
  cP[4] = Y0 ;
  cP[5] = Z0 ;

  cP[6] = R ;

  // vpTRACE("_cP :") ; std::cout << _cP.t() ;

}

//! perspective projection of the circle
void
vpCircle::changeFrame(const vpHomogeneousMatrix &cMo)
{

  double A,B,C ;
  A = cMo[0][0]*oP[0] + cMo[0][1]*oP[1]  + cMo[0][2]*oP[2];
  B = cMo[1][0]*oP[0] + cMo[1][1]*oP[1]  + cMo[1][2]*oP[2];
  C = cMo[2][0]*oP[0] + cMo[2][1]*oP[1]  + cMo[2][2]*oP[2];

  double X0,Y0,Z0 ;
  X0 = cMo[0][3] + cMo[0][0]*oP[3] + cMo[0][1]*oP[4] + cMo[0][2]*oP[5];
  Y0 = cMo[1][3] + cMo[1][0]*oP[3] + cMo[1][1]*oP[4] + cMo[1][2]*oP[5];
  Z0 = cMo[2][3] + cMo[2][0]*oP[3] + cMo[2][1]*oP[4] + cMo[2][2]*oP[5];
  double R = oP[6] ;

  cP[0] = A ;
  cP[1] = B ;
  cP[2] = C ;

  cP[3] = X0 ;
  cP[4] = Y0 ;
  cP[5] = Z0 ;

  cP[6] = R ;

  // vpTRACE("_cP :") ; std::cout << _cP.t() ;

}

void vpCircle::display(const vpImage<unsigned char> &I,
                       const vpCameraParameters &cam,
                       const vpColor &color,
                       const unsigned int thickness)
{
  vpFeatureDisplay::displayEllipse(p[0],p[1],p[2],p[3], p[4],
				   cam, I, color, thickness) ;
}

// non destructive wrt. cP and p
void vpCircle::display(const vpImage<unsigned char> &I,
                       const vpHomogeneousMatrix &cMo,
                       const vpCameraParameters &cam,
                       const vpColor &color,
                       const unsigned int thickness)
{
  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  projection(_cP,_p) ;
  vpFeatureDisplay::displayEllipse(_p[0],_p[1],_p[2],_p[3], _p[4],
				   cam, I, color, thickness) ;

}
//! for memory issue (used by the vpServo class only)
vpCircle *vpCircle::duplicate() const
{
  vpCircle *feature = new vpCircle(*this) ;
  return feature ;
}
