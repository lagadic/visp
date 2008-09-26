/****************************************************************************
 *
 * $Id: vpPlane.h,v 1.6 2008-09-26 15:20:58 fspindle Exp $
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
  vpPlane(vpPoint& P,vpColVector &n) ;
  vpPlane(vpPoint &P, vpPoint &Q, vpPoint &R) ;
  void init(vpPoint& P,   vpPoint& Q,  vpPoint& R) ;
  void init(vpColVector& P,vpColVector &n) ;
  void init(const vpPlane& P) ;
  // SET information
public:

  void setA(const double _a) {   A = _a ; }
  void setB(const double _b) {   B = _b ; }
  void setC(const double _c) {   C = _c ; }
  void setD(const double _d) {   D = _d ; }


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
