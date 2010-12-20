/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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


#ifndef vpPlane_hh
#define vpPlane_hh




/*!
  \class vpPlane

  \ingroup GeometryPlane

  \brief This class defines the container for a plane geometrical structure.

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  A plane is given by the equation
  ax + by + cz + d = 0
  where
  (x,y,z) is a point of R^3

*/

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpPoint.h>


class VISP_EXPORT vpPlane
{

public:
  double A,B,C,D ;


public:
  vpPlane(const double a, const double b,const  double c,const  double d) ;
  vpPlane() ;
  vpPlane(const vpPlane& P) ;
  vpPlane(const vpPoint& P, const vpColVector &n) ;
  vpPlane(const vpPoint &P, const vpPoint &Q, const vpPoint &R) ;
  void init(const vpPoint& P, const vpPoint& Q, const vpPoint& R) ;
  void init(const vpColVector& P, const vpColVector &n) ;
  void init(const vpPlane& P) ;
  // SET information
public:

  inline void setA(const double _a) {   A = _a ; }
  inline void setB(const double _b) {   B = _b ; }
  inline void setC(const double _c) {   C = _c ; }
  inline void setD(const double _d) {   D = _d ; }
  inline void setABCD(const double _a, const double _b, const double _c, const double _d) {A = _a ;B = _b ;C = _c ;D = _d ; }

  vpPlane& operator =(const vpPlane& f) ;

  // GET information

  double getA() const { return A ; }
  double getB() const { return B ; }
  double getC() const { return C ; }
  double getD() const { return D ; }

  vpColVector abcd();

  vpColVector getNormal() const;
  void getNormal(vpColVector &n) const;

public: // Display, Print Member function
  friend std::ostream& operator<< (std::ostream& os, vpPlane& p)
  {
    return (os  << "("<<p.getA() << ","<<p.getB()<< ","<<p.getC()<< ","<<p.getD() <<") ") ;
  } ;


public: // Operation with  Plane

  void projectionPointOnPlan(const vpPoint& P, vpPoint& p2) ;

  double rayIntersection(const vpPoint &M0,
			 const vpPoint &M1,
			 vpColVector &H )const ;

  double getIntersection(const vpColVector &M1,vpColVector &H )const ;


} ;



#endif
