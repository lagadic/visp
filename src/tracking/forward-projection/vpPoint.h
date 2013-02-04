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


#ifndef vpPoint_H
#define vpPoint_H

class vpHomography ;

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

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
  //! Basic constructor.
  vpPoint() ;
  //! Destructor.
  virtual ~vpPoint() { ; }

public:

  // Compute the 3D coordinates _cP  (camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;

  /*!
    From the 3D coordinates of the point in the object frame set using for example
    setWorldCoordinates() or set_oX(), set_oY(), set_oZ(), compute the 3D coordinates
    of the point in the camera frame.

    \param cMo : Transformation from camera to object frame.

  */
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
               const vpColor &color=vpColor::green,
               const unsigned int thickness=1) ;
  void display(const vpImage<unsigned char> &I,
               const vpHomogeneousMatrix &cMo,
               const vpCameraParameters &cam,
               const vpColor &color=vpColor::green,
               const unsigned int thickness=1) ;
  void display(const vpImage<vpRGBa> &I,
               const vpHomogeneousMatrix &cMo,
               const vpCameraParameters &cam,
               const vpColor &color=vpColor::green,
               const unsigned int thickness=1) ;
  vpPoint *duplicate() const ;

  // Get coordinates
  //! Get the point X coordinate in the camera frame.
  double get_X()  const { return cP[0] ; }
  //! Get the point Y coordinate in the camera frame.
  double get_Y()  const { return cP[1] ; }
  //! Get the point Z coordinate in the camera frame.
  double get_Z() const  { return cP[2] ; }
  //! Get the point W coordinate in the camera frame.
  double get_W()  const { return cP[3] ; }

  //! Get the point X coordinate in the object frame.
  double get_oX() const { return oP[0] ; }
  //! Get the point Y coordinate in the object frame.
  double get_oY() const { return oP[1] ; }
  //! Get the point Z coordinate in the object frame.
  double get_oZ() const { return oP[2] ; }
  //! Get the point W coordinate in the object frame.
  double get_oW() const { return oP[3] ; }

  //! Get the point x coordinate in the image plane.
  double get_x()  const { return p[0] ; }
  //! Get the point y coordinate in the image plane.
  double get_y()  const { return p[1] ; }
  //! Get the point w coordinate in the image plane.
  double get_w()  const { return p[2] ; }

  //! Get the point world coordinates. We mean here the coordinates of the point in the object frame.
  void getWorldCoordinates(double& ox,
                           double& oy,
                           double& oz) ;
  //! Get the point world coordinates. We mean here the coordinates of the point in the object frame.
  void getWorldCoordinates(vpColVector &_oP) ;
  vpColVector getWorldCoordinates(void) ;

  //! Basic construction.
  void init() ;

  friend VISP_EXPORT std::ostream& operator<<(std::ostream& os, vpPoint& vpp);
  vpPoint& operator=(const vpPoint& vpp);

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

  // Set coordinates
  //! Set the point X coordinate in the camera frame.
  inline void set_X(const double X) { cP[0] = X ; }
  //! Set the point Y coordinate in the camera frame.
  inline void set_Y(const double Y) { cP[1] = Y ; }
  //! Set the point Z coordinate in the camera frame.
  inline void set_Z(const double Z) { cP[2] = Z ; }
  //! Set the point W coordinate in the camera frame.
  inline void set_W(const double W) { cP[3] = W ; }

  //! Set the point X coordinate in the object frame.
  inline void set_oX(const double X) { oP[0] = X ; }
  //! Set the point Y coordinate in the object frame.
  inline void set_oY(const double Y) { oP[1] = Y ; }
  //! Set the point Z coordinate in the object frame.
  inline void set_oZ(const double Z) { oP[2] = Z ; }
  //! Set the point W coordinate in the object frame.
  inline void set_oW(const double W) { oP[3] = W ; }

  //! Set the point x coordinate in the image plane.
  inline void set_x(const double x) {  p[0] = x ; }
  //! Set the point y coordinate in the image plane.
  inline void set_y(const double y) {  p[1] = y ; }
  //! Set the point w coordinate in the image plane.
  inline void set_w(const double w) {  p[2] = w ; }

  //! Set the point world coordinates. We mean here the coordinates of the point in the object frame.
  void setWorldCoordinates(const double ox,
                           const double oy,
                           const double oz) ;
  //! Set the point world coordinates. We mean here the coordinates of the point in the object frame.
  void setWorldCoordinates(const vpColVector &_oP) ;
} ;

const vpPoint VISP_EXPORT operator*(const vpHomogeneousMatrix &M, const vpPoint& p) ;
const vpPoint VISP_EXPORT operator*(const vpHomography &H, const vpPoint& p) ;

#endif
