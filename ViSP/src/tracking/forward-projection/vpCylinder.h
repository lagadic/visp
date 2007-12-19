/****************************************************************************
 *
 * $Id: vpCylinder.h,v 1.7 2007-12-19 17:36:29 fspindle Exp $
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
 * Cylinder feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpCylinder.h
  \brief  class that defines what is a cylinder
*/

#ifndef vpCylinder_hh
#define vpCylinder_hh


#include <math.h>
#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpCylinder
  \brief  class that defines what is a cylinder
*/
class VISP_EXPORT vpCylinder : public vpForwardProjection
{
public:
  void init() ;
  vpCylinder() ;
  virtual ~vpCylinder() ;

public:
  typedef enum
    {
      line1,
      line2
    } vpLineCylinderType;

  vpCylinder(const vpColVector& oP) ;
  vpCylinder(const double A, const double B1,
	     const double C,
	     const double X0, const double Y0,
	     const double Z0,
	     const double R) ;


  void setWorldCoordinates(const vpColVector& oP) ;
  void setWorldCoordinates(const double A, const double B1,
			   const double C,
			   const double X0, const double Y0,
			   const double Z0,
			   const double R) ;

  double getRho1()  const  { return p[0] ; }
  double getTheta1() const  { return p[1] ; }

  double getRho2()  const  { return p[2] ; }
  double getTheta2() const { return p[3] ; }

  double getA() const { return cP[0] ; }
  double getB()  const{ return cP[1] ; }
  double getC() const { return cP[2] ; }

  double getX() const { return cP[3] ; }
  double getY() const { return cP[4] ; }
  double getZ()  const{ return cP[5] ; }

  double getR() const { return cP[6] ; }


  void projection(const vpColVector &cP, vpColVector &p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const bool useDistortion=false,
	       const vpColor::vpColorType color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const bool useDistortion=false,
	       const vpColor::vpColorType color=vpColor::green) ;

  vpCylinder *duplicate() const ;
} ;


#endif
