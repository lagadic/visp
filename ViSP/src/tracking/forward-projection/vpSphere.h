/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Sphere feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpSphere.h
  \brief  forward projection of a sphere
*/

#ifndef vpSphere_hh
#define vpSphere_hh


#include <math.h>
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpSphere
  \ingroup TrackingFeature GeometryFeature
  \brief Class that defines what is a sphere.

  Forward projection of a sphere.
*/
class VISP_EXPORT vpSphere : public vpForwardProjection
{
public:
  void init() ;
  vpSphere() ;
  virtual ~vpSphere() ;

public:
  vpSphere(const vpColVector& oP) ;
  vpSphere(const double X0, const double Y0,
	   const double Z0,
	   const double R) ;


  void setWorldCoordinates(const vpColVector& oP) ;
  void setWorldCoordinates(const double X0, const double Y0,
			   const double Z0,
			   const double R) ;

  double get_x() { return p[0] ; }
  double get_y() { return p[1] ; }
  double get_mu20() { return p[2] ; }
  double get_mu11() { return p[3] ; }
  double get_mu02() { return p[4] ; }

  double getX() const { return cP[0] ; }
  double getY() const { return cP[1] ; }
  double getZ()  const{ return cP[2] ; }

  double getR() const { return cP[3] ; }



  void projection() ;
  void projection(const vpColVector &cP, vpColVector &p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) ;
  void changeFrame(const vpHomogeneousMatrix &cMo) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const vpColor color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor color=vpColor::green) ;


  vpSphere *duplicate() const ;
} ;


#endif
