/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/core/vpForwardProjection.h>

/*!
  \class vpLine
  \ingroup group_core_geometry

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
  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in one of the two 3D frames.
  In this class it is easily possible to compute the parameters (\e cP) of the
  line in the camera frame thanks to its parameters (\e oP) in the
  object frame. But you have to notes that four constraints are
  added in the planes equations.
  \f[ D1 = 0 \f]
  \f[ D2 > 0 \f]
  \f[ A1 A2 + B1 B2 + C1 C2 = 0 \f]
  \f[ || A2 || = 1 \f]
  The line parameters \e oP corresponding to the object frame are
  located in the vpForwardProjection::oP public attribute, where \e oP
  is a vector defined as: \f[ oP = \left[\begin{array}{c}A1_o \\ B1_o
  \\ C1_o \\ D1_o \\ A2_o \\ B2_o \\ C2_o \\ D2_o \end{array}\right]
  \f]
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
  \image html vpFeatureLine.gif
  \image latex vpFeatureLine.ps  width=10cm
  The line parameters corresponding to the image frame are located
  in the vpTracker::p public attribute, where \e p is a vector defined
  as: \f[ p = \left[\begin{array}{c} \rho \\ \theta \end{array}\right]
  \f]
*/
class VISP_EXPORT vpLine : public vpForwardProjection
{

public:
  void init();

  vpLine();
  //! Destructor
  virtual ~vpLine() { ; }

  /*!

    Sets the \f$ \rho \f$ parameter used to define the line in the
    image plane.

    \param rho : The desired value for \f$ \rho \f$.

    \sa setTheta()
  */
  void setRho(const double rho) { p[0] = rho; };

  /*!
    Sets the \f$ \theta \f$ angle value used to define the line in the
    image plane.

    \param theta : The desired value for \f$ \theta \f$ angle.

    \sa setRho()
  */
  void setTheta(const double theta) { p[1] = theta; };

  /*!

    Gets the \f$ \theta \f$ angle value corresponding to one of the
    two parameters used to define the line parametrization in the
    image plane.

    \return Returns the current value of \f$ \theta \f$.

    \sa getRho()
  */
  double getTheta() const { return p[1]; }

  /*!
    Gets the \f$ \rho \f$ value corresponding to one of the
    two parameters used to define the line parametrization in the
    image plane.

    \return Returns the current value of \f$ \rho \f$.

    \sa getTheta()
  */
  double getRho() const { return p[0]; }

  void setWorldCoordinates(const double &A1, const double &B1, const double &C1, const double &D1, const double &A2,
                           const double &B2, const double &C2, const double &D2);

  void setWorldCoordinates(const vpColVector &oP1, const vpColVector &oP2);

  void setWorldCoordinates(const vpColVector &oP);

  void projection();
  void projection(const vpColVector &cP, vpColVector &p);
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP);
  void changeFrame(const vpHomogeneousMatrix &cMo);

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               const unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);

  vpLine *duplicate() const;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
