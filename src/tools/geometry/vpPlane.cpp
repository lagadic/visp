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
 * Plane geometrical structure.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpPlane.cpp
  \brief definition of the vpPlane class member functions
  \ingroup libtools
*/

#include <visp/vpPlane.h>

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

/*!
  Copy operator.
*/
vpPlane&
vpPlane::operator =(const vpPlane& p)
{
  A = p.A ;
  B = p.B ;
  C = p.C ;
  D = p.D ;

  return *this ;
}

/*!
  Basic constructor that set the plane parameters A, B, C, D to zero.
*/
vpPlane::vpPlane()
{
  setA(0) ;
  setB(0) ;
  setC(0) ;
  setD(0) ;
}

/*!
  Plane constructor from A, B, C, D parameters.

  A plane is given by the equation \f$Ax + By + Cz + D = 0\f$ where
  (x,y,z) are the coordinates of a point and \f$[A,B,C]^T\f$ is the normal
  vector of the plane.

  \param A, B, C, D : Parameters of the plane.

*/
vpPlane::vpPlane(const double A,const double B,const double C, const double D)
{
  setA(A) ;
  setB(B) ;
  setC(C) ;
  setD(D) ;
}

/*!
  Copy constructor.
*/
vpPlane::vpPlane(const vpPlane& P)
{
  setA(P.getA()) ;
  setB(P.getB()) ;
  setC(P.getC()) ;
  setD(P.getD()) ;
}

/*!  

  Plane constructor from a point \e P on the plane and the normal
  \e n to the plane.

  A plane is given by the equation \f$Ax + By + Cz + D = 0\f$ where
  (x,y,z) are the coordinates of a point and \f$[A,B,C]^T\f$ is the normal
  vector of the plane.

  \param P : A point with coordinates (x,y,z) on the plane.
  
  \param n : The normal to the plane.

*/
vpPlane::vpPlane(const vpPoint& P, const vpColVector &n)
{
  //Equation of the plane is given by:
  A = n[0];
  B = n[1];
  C = n[2];

  D=-(A*P.get_X()+B*P.get_Y()+C*P.get_Z());
}

/*!
  Initialize the plane with the parameters of an other plane \e P.

  \param P : Plane used as initializer.
*/
void vpPlane::init(const vpPlane& P)
{
  setA(P.getA()) ;
  setB(P.getB()) ;
  setC(P.getC()) ;
  setD(P.getD()) ;
}

/*!
  Initialize the plane from a point \e P on the plane and the normal
  \e n to the plane.

  \param P : A point with coordinates (x,y,z) on the plane.
  
  \param n : The normal to the plane.

  \sa vpPlane(const vpPoint&, const vpColVector &)
*/
void vpPlane::init(const vpColVector & P, const vpColVector &n)
{
  //Equation of the plane is given by:
  A = n[0];
  B = n[1];
  C = n[2];

  D=-(A*P[0]+B*P[1]+C*P[2]);
}

/*!
  Compute the equation of a plane given three point P, Q, R.

  The normal to the plane is given by
  n = PQ x PR

*/
void
vpPlane::init(const vpPoint &P, const vpPoint &Q, const vpPoint &R)
{
  vpColVector a(3);
  vpColVector b(3);
  vpColVector n(3);
  //Calculate vector corresponding to PQ
  a[0]=P.get_X()-Q.get_X();
  a[1]=P.get_Y()-Q.get_Y();
  a[2]=P.get_Z()-Q.get_Z();

  //Calculate vector corresponding to PR
  b[0]=P.get_X()-R.get_X();
  b[1]=P.get_Y()-R.get_Y();
  b[2]=P.get_Z()-R.get_Z();

  //Calculate normal vector to plane PQ x PR
  n=vpColVector::cross(a,b);

  //Equation of the plane is given by:
  A = n[0];
  B = n[1];
  C = n[2];
  D=-(A*P.get_X()+B*P.get_Y()+C*P.get_Z());

  double norm =  sqrt(A*A+B*B+C*C) ;
  A /= norm ;
  B /= norm ;
  C /= norm ;
  D /= norm ;
}


/*!
  Compute the equation of a plane given three point P, Q, R.

  The normal to the plane is given by
  n = PQ x PR

  \sa init(const vpPoint &, const vpPoint &, const vpPoint &)
*/
vpPlane::vpPlane(const vpPoint &P, const vpPoint &Q, const vpPoint &R)
{
  init(P,Q,R) ;
}

/*!
  Return the normal to the plane.

  A plane is given by the equation \f$Ax + By + Cz + D = 0\f$ where
  (x,y,z) is a point of R^3 and (A,B,C) are the coordinates of the normal.

  \sa getNormal(vpColVector &n)
*/
vpColVector  vpPlane::getNormal() const
{
  vpColVector n(3);
  n[0] = A ;
  n[1] = B ;
  n[2] = C ;

  return n ;
}

/*!
  Return the normal to the plane.

  A plane is given by the equation \f$Ax + By + Cz + D = 0\f$ where
  (x,y,z) are the coordinates of a point and \f$[A,B,C]^T\f$ is normal
  vector of the plane.

  \sa getNormal()

*/
void  vpPlane::getNormal(vpColVector &n) const
{
  n.resize(3) ;
  n[0] = A ;
  n[1] = B ;
  n[2] = C ;
}

/*!
  Compute the coordinates of the projection of a point on the plane.

  \param P : point to be projected on the plane
  \param Pproj : result of the projection (pproj belongs to the plane)
*/
void
vpPlane::projectionPointOnPlan(const  vpPoint& P, vpPoint& Pproj) const
{
  double x0,y0,z0 ;
  double rho ;

  x0 = P.get_X()/P.get_W() ;
  y0 = P.get_Y()/P.get_W() ;
  z0 = P.get_Z()/P.get_W() ;

  rho = - (A*x0+B*y0+C*z0+D)/(A*A+B*B+C*C) ;

  Pproj.set_X(x0+A*rho) ;
  Pproj.set_Y(y0+B*rho) ;
  Pproj.set_Z(z0+C*rho) ;
  Pproj.set_W(1) ;
}


double
vpPlane::rayIntersection(const vpPoint &M0,
			 const vpPoint &M1,
			 vpColVector &H ) const
{

  double k,scal;
  double R[3];

  //  if(M0.get_X()!=0 || M0.get_Y()!=0 || M0.get_Z()!=0)
  if(std::fabs(M0.get_X()) > std::numeric_limits<double>::epsilon()
     || std::fabs(M0.get_Y()) > std::numeric_limits<double>::epsilon()
     || std::fabs(M0.get_Z()) > std::numeric_limits<double>::epsilon())
  {
    R[0]= M1.get_X() - M0.get_X();
    R[1]= M1.get_Y() - M0.get_Y();
    R[2]= M1.get_Z() - M0.get_Z();

    scal = getA()*R[0] + getB()*R[1] + getC()*R[2];
    //if (scal != 0)
    if (std::fabs(scal) > std::numeric_limits<double>::epsilon())
      k =  -( getA()*M0.get_X() + getB()*M0.get_Y() + getC()*M0.get_Z() + getD())/scal;
    else
      k = 0;

    H[0] = M0.get_X()+ k*R[0];
    H[1] = M0.get_Y()+ k*R[1];
    H[2] = M0.get_Z()+ k*R[2];
  }
  else
  {
    scal = getA()*M1.get_X() + getB()*M1.get_Y() + getC()*M1.get_Z();
    //if (scal != 0)
    if (std::fabs(scal) > std::numeric_limits<double>::epsilon())
      k = -getD()/scal;
    else
      k=0;
    H[0] = k*M1.get_X();
    H[1] = k*M1.get_Y();
    H[2] = k*M1.get_Z();
  }

  return k;

}

double vpPlane::getIntersection(const vpColVector &M1,vpColVector &H )const
{

  double k,scal;

  scal = A*M1[0] + B*M1[1] + C*M1[2];
  //if (scal != 0)
  if (std::fabs(scal) > std::numeric_limits<double>::epsilon())
    k = -getD()/scal;
  else
    k=0;
  H[0] = k*M1[0];
  H[1] = k*M1[1];
  H[2] = k*M1[2];

  return k;

}

/*!

  Considering the plane in the Ro frame computes the equation of the
  plane in the Rc frame.

  \param cMo : Homogeneous transformation from Rc to Ro frames.

*/
void vpPlane::changeFrame(const vpHomogeneousMatrix &cMo)
{
  // Save current plane parameters
  double Ao = A, Bo = B, Co = C, Do =D ;
  A = cMo[0][0]*Ao + cMo[0][1]*Bo  + cMo[0][2]*Co;
  B = cMo[1][0]*Ao + cMo[1][1]*Bo  + cMo[1][2]*Co;
  C = cMo[2][0]*Ao + cMo[2][1]*Bo  + cMo[2][2]*Co;
  D = Do - (cMo[0][3]*A + cMo[1][3]*B  + cMo[2][3]*C);
}

