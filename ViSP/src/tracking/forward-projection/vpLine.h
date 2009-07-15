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

  \brief Class that defines a line in the object frame, the
  camera frame and the image plane. All the parameters
  must be set in meter.

  \par Object and camera frame parametrization: 
  In the 3D frames, the object frame parameters (\e oP) and the camera
  frame parameters (\e cP), the line is defined as the intersection
  between two plans. Thus, the parameters which define the line are
  the parameters which define the two equations of the two plans. Each
  point which belongs to the line is a solution of those two
  equations:

  \par
  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in one of the two 3D frames.
  
  \par
  In this class it is easily possible to compute the parameters (\e cP) of the
  line in the camera frame thanks to its parameters (\e oP) in the
  object frame. But you have to notes that four constraints are
  added in the planes equations.
  
  \par
  \f[ D1 = 0 \f]
  \f[ D2 > 0 \f]
  \f[ A1 A2 + B1 B2 + C1 C2 = 0 \f]
  \f[ || A2 || = 1 \f]
  

  \par 
  The line parameters \e oP corresponding to the object frame are
  located in the vpForwardProjection::oP public attribute, where \e oP
  is a vector defined as: \f[ oP = \left[\begin{array}{c}A1_o \\ B1_o
  \\ C1_o \\ D1_o \\ A2_o \\ B2_o \\ C2_o \\ D2_o \end{array}\right]
  \f]

  \par 
  The line parameters corresponding to the camera frame are located
  in the vpTracker::cP public attribute, where \e cP is a vector
  defined as: \f[ cP = \left[\begin{array}{c}A1_c \\ B1_c \\ C1_c \\
  D1_c \\ A2_c \\ B2_c \\ C2_c \\ D2_c \end{array}\right] \f]
  
  \par Image plane parametrization:
  In the image plane, the line is defined thanks to its 2D equation.
  \f[ x \; cos(\theta) + y \; sin(\theta) -\rho = 0 \f] Here \f$ x
  \f$ and \f$ y \f$ are the coordinates of a point belonging to the
  line in the image plane while \f$ \rho \f$ and \f$ \theta \f$ are
  the parameters used to define the line. The value of \f$ \theta
  \f$ is between \f$ -\pi/2 \f$ and \f$ \pi/2 \f$ and the value of
  \f$ \rho \f$ can be positive or negative. The conventions used to
  choose the sign of \f$ \rho \f$ and the value of \f$ \theta \f$
  are illustrated by the following image.
  
  \par
  \image html vpFeatureLine.gif
  \image latex vpFeatureLine.ps  width=10cm

  \par
  The line parameters corresponding to the image frame are located
  in the vpTracker::p public attribute, where \e p is a vector defined
  as: \f[ p = \left[\begin{array}{c} \rho \\ \theta \end{array}\right]
  \f]
*/
class VISP_EXPORT vpLine : public vpForwardProjection
{

public:

  void init() ;

  vpLine() ;
  //! Destructor
  virtual ~vpLine() { ; }

  /*!

    Sets the \f$ \rho \f$ parameter used to define the line in the
    image plane.

    \param rho : The desired value for \f$ \rho \f$.

    \sa setTheta()
  */
  void setRho(const double rho) {  p[0] = rho ; };

  /*!
    Sets the \f$ \theta \f$ angle value used to define the line in the
    image plane.

    \param theta : The desired value for \f$ \theta \f$ angle.

    \sa setRho()
  */
  void setTheta(const double theta) {  p[1] = theta ;};

  /*!

    Gets the \f$ \theta \f$ angle value corresponding to one of the
    two parameters used to define the line parametrization in the
    image plane.

    \return Returns the current value of \f$ \theta \f$.

    \sa getRho()
  */
  double getTheta()   const {  return p[1] ; }

  /*!
    Gets the \f$ \rho \f$ value corresponding to one of the
    two parameters used to define the line parametrization in the
    image plane.

    \return Returns the current value of \f$ \rho \f$.

    \sa getTheta()
  */
  double getRho()  const  {  return p[0] ; }



  void setWorldCoordinates(const double &A1, const double &B1,
			   const double &C1, const double &D1,
			   const double &A2, const double &B2,
			   const double &C2, const double &D2) ;


  void setWorldCoordinates(const vpColVector &oP1,
			   const vpColVector &oP2) ;


  void setWorldCoordinates(const vpColVector &oP) ;


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

  vpLine *duplicate() const ;
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
