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
 * Point feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpPoint_H
#define vpPoint_H

class vpHomography ;

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

class vpHomography;

/*!
  \class vpPoint
  \ingroup TrackingFeature GeometryFeature
  \brief Class that defines what is a point.
*/
class VISP_EXPORT vpPoint : public vpForwardProjection
{

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpPoint() ;
  //! destructor
  virtual ~vpPoint() { ; }

public:
  /*
    /section Set coordinates
  */

  //@{

  //! set the point coordinates (camera frame)
  inline void set_X(const double X) { cP[0] = X ; }
  inline void set_Y(const double Y) { cP[1] = Y ; }
  inline void set_Z(const double Z) { cP[2] = Z ; }
  inline void set_W(const double W) { cP[3] = W ; }

  //! set the point coordinates (object frame)
  inline void set_oX(const double X) { oP[0] = X ; }
  inline void set_oY(const double Y) { oP[1] = Y ; }
  inline void set_oZ(const double Z) { oP[2] = Z ; }
  inline void set_oW(const double W) { oP[3] = W ; }

  //! get the point coordinates (camera frame)
  double get_X()  const { return cP[0] ; }
  double get_Y()  const { return cP[1] ; }
  double get_Z() const  { return cP[2] ; }
  double get_W()  const { return cP[3] ; }

  //! get the point coordinates (object frame)
  double get_oX() const { return oP[0] ; }
  double get_oY() const { return oP[1] ; }
  double get_oZ() const { return oP[2] ; }
  double get_oW() const { return oP[3] ; }

  //! set the point xyw-coordinates
  inline void set_x(const double x) {  p[0] = x ; }
  inline void set_y(const double y) {  p[1] = y ; }
  inline void set_w(const double w) {  p[2] = w ; }


  //! get the point xyw-coordinates
  double get_x()  const { return p[0] ; }
  double get_y()  const { return p[1] ; }
  double get_w()  const { return p[2] ; }


  //! set the point world coordinates
  void setWorldCoordinates(const double ox,
			   const double oy,
			   const double oz) ;
  //! set the point world coordinates
  void setWorldCoordinates(const vpColVector &_oP) ;
  //! get the point world coordinates
  void getWorldCoordinates(double& ox,
			   double& oy,
			   double& oz) ;
  //! set the point world coordinates
  void getWorldCoordinates(vpColVector &_oP) ;
  vpColVector getWorldCoordinates(void) ;
  //@}

  //! Projection onto the image plane of a point. Input: the 3D coordinates in the camera frame _cP, output : the 2D coordinates _p.
  void projection(const vpColVector &_cP, vpColVector &_p) ;

  /*! 
    Perspective projection of the point. 
    
    Projection onto the //image plane of the point. Update the object
    attribute p (2D //homogeneous coordinates) according to object
    attribute cP (current //3D coordinates in the camera frame).

  */
  inline void projection() {
    double d = 1/cP[2] ;
    p[0] = cP[0]*d ;
    p[1] = cP[1]*d ;
    p[2] = 1 ;
  }

  //!Compute the 3D coordinates _cP  (camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;

  //! Update the 3D coordinates of the point (camera frame).
  //!Update the object attribute cP  (3D coordinates in the camera frame)
  inline void changeFrame(const vpHomogeneousMatrix &cMo) {
    double X = cMo[0][0]*oP[0]+ cMo[0][1]*oP[1]+ cMo[0][2]*oP[2]+ cMo[0][3]*oP[3] ;
    double Y = cMo[1][0]*oP[0]+ cMo[1][1]*oP[1]+ cMo[1][2]*oP[2]+ cMo[1][3]*oP[3] ;
    double Z = cMo[2][0]*oP[0]+ cMo[2][1]*oP[1]+ cMo[2][2]*oP[2]+ cMo[2][3]*oP[3] ;
    double W = cMo[3][0]*oP[0]+ cMo[3][1]*oP[1]+ cMo[3][2]*oP[2]+ cMo[3][3]*oP[3] ;
    
    double d = 1/W ;
    cP[0] =X*d ;
    cP[1] =Y*d ;
    cP[2] =Z*d ;
    cP[3] =1 ;  
  }


  void display(const vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const vpColor color=vpColor::green,
	       const unsigned int thickness=1) ;
  void display(const vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor color=vpColor::green,
	       const unsigned int thickness=1) ;
  void display(const vpImage<vpRGBa> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor color=vpColor::green,
	       const unsigned int thickness=1) ;
  vpPoint *duplicate() const ;

  friend std::ostream& operator<<(std::ostream& os, vpPoint& vpp);
  vpPoint& operator=(const vpPoint& vpp);
} ;


const vpPoint VISP_EXPORT operator*(const vpHomogeneousMatrix &M, const vpPoint& p) ;
const vpPoint VISP_EXPORT operator*(const vpHomography &H, const vpPoint& p) ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
