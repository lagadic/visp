/****************************************************************************
 *
 * $Id: vpPoint.h,v 1.12 2007-05-02 13:29:41 fspindle Exp $
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
  \brief  class that defines what is a point
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
  void set_X(const double X) { cP[0] = X ; }
  void set_Y(const double Y) { cP[1] = Y ; }
  void set_Z(const double Z) { cP[2] = Z ; }
  void set_W(const double W) { cP[3] = W ; }

  //! set the point coordinates (object frame)
  void set_oX(const double X) { oP[0] = X ; }
  void set_oY(const double Y) { oP[1] = Y ; }
  void set_oZ(const double Z) { oP[2] = Z ; }
  void set_oW(const double W) { oP[3] = W ; }

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
  void set_x(const double x) {  p[0] = x ; }
  void set_y(const double y) {  p[1] = y ; }
  void set_w(const double w) {  p[2] = w ; }


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

  //! Projection onto the image plane of the point. Update the object attribute p (2D homogeneous coordinates) according to object attribute cP (current 3D coordinates in the camera frame).
  void projection();

  //!Compute the 3D coordinates _cP  (camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;

  //!Update the object attribute cP  (3D coordinates in the camera frame)
  void changeFrame(const vpHomogeneousMatrix &cMo) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const vpColor::vpColorType color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor::vpColorType color=vpColor::green) ;
  vpPoint *duplicate() const ;

  friend std::ostream& operator<<(std::ostream& os, vpPoint& vpp);
} ;


const vpPoint VISP_EXPORT operator*(const vpHomogeneousMatrix &M, const vpPoint& p) ;
const vpPoint VISP_EXPORT operator*(const vpHomography &H, const vpPoint& p) ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
