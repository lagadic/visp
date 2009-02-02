/****************************************************************************
 *
 * $Id: vpLine.h,v 1.8 2008-09-26 15:21:00 fspindle Exp $
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
  \ingroup TrackingFeature GeometryFeature
  \brief Class that defines a line in the three different frames : the object frame, the camera frame and the image frame. All the features are given  and must be set in meter.

  In the 3D frames (ie the object frame and the camera frame), the line is defined as the intersection between two plans. Thus, the parameters which define the line are the parameters which define the two equations of the two plans. Each point which belongs to the line is a solution of those two equations :
  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in one of the two 3D frames.

  In this class it is easily possible to compute the "equation" of the line in the camera frame thanks to this in the object frame. But you have to notes that four constraints are added in the planes equations.
  \f[ D1 = 0 \f]
  \f[ D2 > 0 \f]
  \f[ A1 A2 + B1 B2 + C1 C2 = 0 \f]
  \f[ || A2 || = 1 \f]

  In the 2D frame (ie the image frame), the line is defined thanks to its equation.
  \f[ x \times cos(\theta) + y \times sin(\theta) -\rho = 0 \f]
  Here \f$ x \f$ and \f$ y \f$ are the coordinates in the image frame, \f$ \rho \f$ and \f$ \theta \f$ are the parameters used to define the line. The value of \f$ \theta \f$ is between \f$ -\pi/2 \f$ and \f$ \pi/2 \f$ and the value of \f$ \rho \f$ can be positive or negative. The conventions used to choose the sign of \f$ \rho \f$ and the value of \f$ \theta \f$ are illustrated by the following image.

  \image html vpFeatureLine.gif
  \image latex vpFeatureLine.ps  width=10cm

  In this class the line features corresponding to the object frame are located in the oP vector \f[ oP = \left[\begin{array}{c}A1_o \\ B1_o \\ C1_o \\ D1_o \\ A2_o \\ B2_o \\ C2_o \\ D2_o \end{array}\right] \f]

  The line features corresponding to the camera frame are located in the cP vector \f[ cP = \left[\begin{array}{c}A1_c \\ B1_c \\ C1_c \\ D1_c \\ A2_c \\ B2_c \\ C2_c \\ D2_c \end{array}\right] \f]

  The line features corresponding to the camera frame are located in the p vector \f[ p = \left[\begin{array}{c} \rho \\ \theta \end{array}\right] \f]
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

  void init() ;

  vpLine() ;
  //! Destructor
  virtual ~vpLine() { ; }

public:

  /*!
    Sets the \f$ \rho \f$ value.

    \param _rho : The desired value for \f$ \rho \f$.
  */
  void setRho(const double _rho) {  p[0] = _rho ; };

  /*!
    Sets the \f$ \theta \f$ value.

    \param _theta : The desired value for \f$ \theta \f$.
  */
  void setTheta(const double _theta) {  p[1] = _theta ;};

  /*!
    Gets the \f$ \theta \f$ value.

    \return Returns the current value of \f$ \theta \f$.
  */
  double getTheta()   const {  return p[1] ; }

  /*!
    Gets the \f$ \rho \f$ value.

    \return Returns the current value of \f$ \rho \f$.
  */
  double getRho()  const  {  return p[0] ; }



  void setWorldCoordinates(const double &A1, const double &B1,
			   const double &C1, const double &D1,
			   const double &A2, const double &B2,
			   const double &C2, const double &D2) ;


  void setWorldCoordinates(const vpColVector &_oP1,
			   const vpColVector &_oP2) ;


  void setWorldCoordinates(const vpColVector &_oP1) ;


  void projection() ;
  void projection(const vpColVector &_cP, vpColVector &p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;
  void changeFrame(const vpHomogeneousMatrix &cMo) ;

  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const vpColor::vpColorType color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor::vpColorType color=vpColor::green) ;

  vpLine *duplicate() const ;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
