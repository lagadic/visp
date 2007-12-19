/****************************************************************************
 *
 * $Id: vpLine.h,v 1.5 2007-12-19 17:36:29 fspindle Exp $
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
 * Line feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpLine_H
#define vpLine_H

/*!
  \file vpLine.h
  \brief  class that defines what is a line
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpLine
  \brief  class that defines what is a line

  Line equation in 2D

  x cos(theta) + y sin(theta) - rho = 0

  in 3D the line is represented using the intersection of two planes

  a1 x + b1 y + c1 z + d1 = 0
  a2 x + b2 y + c2 z + d2 = 0

  when converted in the camera frame, four constraints are added in the planes
  equation :
  d1 = 0
  d2 > 0
  a1 a2 + b1 b2 + c1 c2 = 0
  || a2 || = 1

*/
class VISP_EXPORT vpLine : public vpForwardProjection
{

public:
  //! 2D line coordinates
  //double rho,theta ;
  /*
 //! line coordinates expressed in
  //! camera frame : 2 planes
  vpColVector cP1 ;
  vpColVector cP2 ;
  //! line coordinates expressed in
  //! world frame : 2 planes
  vpColVector oP1 ;
  vpColVector oP2 ;
  */
public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpLine() ;
  //! destructor
  virtual ~vpLine() { ; }

public:

  //! set the line coordinates
  void setRho(const double _rho) {  p[0] = _rho ; };
  void setTheta(const double _theta) {  p[1] = _theta ;};

  //! get the line -coordinates
  double getTheta()   const {  return p[1] ; }
  double getRho()  const  {  return p[0] ; }


  //! set the line world coordinates
  void setWorldCoordinates(const double &A1, const double &B1,
			   const double &C1, const double &D1,
			   const double &A2, const double &B2,
			   const double &C2, const double &D2) ;
  //! set the line world coordinates from two planes
  void setWorldCoordinates(const vpColVector &_oP1,
			   const vpColVector &_oP2) ;
  //! set the line world coordinates
  void setWorldCoordinates(const vpColVector &_oP1) ;

  //! projection
  void projection(const vpColVector &_cP, vpColVector &p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;

  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const bool usedistortion=false,
	       const vpColor::vpColorType color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const bool usedistortion=false,
	       const vpColor::vpColorType color=vpColor::green) ;

  vpLine *duplicate() const ;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
