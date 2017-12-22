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
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

#include <visp3/core/vpForwardProjection.h>

/*!
  \class vpCylinder
  \ingroup group_core_geometry
  \brief Class that defines what is a cylinder.

  A cylinder may be represented by the equation:
  \f$ (X - X_0)^2 + (Y - Y_0)^2 + (Z - Z_0)^2 - (A \; X + B \; Y + C \; Z)^2 -
  R^2 = 0 \f$ with

  \f$
  \left\{ \begin{array}{l}
  A^2 + B^2 + C^2 = 1  \\
  A \; X_0 + B \; Y_0 + C \; Z_0 = 0
  \end{array} \right.
  \f$

  where \f$R\f$ is the radius of the cylinder, \f$A, B, C\f$ are the
  coordinates of its direction vector and \f$X_0, Y_0, Z_0\f$ are the
  coordinates of the nearest point belonging to the cylinder axis from the
  projection center.

  Setting the cylinder parameters is achieved throw the constructors with
  parameters or the setWorldCoordinates() methods.

  Considering the set of parameters \f$^{o}{\bf P} =
  ({^o}A,{^o}B,{^o}C,{^o}X_0,{^o}Y_0,{^o}Z_0,R)\f$ expressed in the world
  frame, cylinder coordinates expressed in the camera frame are obtained using
  changeFrame().

  The projection of a cylinder on the image plane is (for
  non-degenerated cases) a set of two straight lines with equation:

  \f$
  \left\{ \begin{array}{lll}
  x \;\cos\theta_1 + x \;\sin\theta_1 - \rho_1 = 0 \\
  y \;\cos\theta_2 + y \;\sin\theta_2 - \rho_2 = 0
  \end{array} \right.
  \f$

  The projection is achieved using projection() methods. The methods
  getRho1(), getTheta1() and getRho2(), getTheta2() allow to access to the
  projected line parameters.
*/
class VISP_EXPORT vpCylinder : public vpForwardProjection
{
public:
  typedef enum {
    line1, /*!< First limb of the cylinder. */
    line2  /*!< Second limb of the cylinder. */
  } vpLineCylinderType;

  vpCylinder();
  explicit vpCylinder(const vpColVector &oP);
  vpCylinder(const double A, const double B, const double C, const double X0, const double Y0, const double Z0,
             const double R);
  virtual ~vpCylinder();

  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP);
  void changeFrame(const vpHomogeneousMatrix &cMo);

  double computeZ(const double x, const double y) const;

  void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color = vpColor::green,
               const unsigned int thickness = 1);
  void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
               const vpColor &color = vpColor::green, const unsigned int thickness = 1);

  vpCylinder *duplicate() const;

  /*!
    Return the \f$\rho_1\f$ parameter of the line corresponding to the
    projection of the cylinder in the image plane.
    \sa getTheta1()
    */
  double getRho1() const { return p[0]; }
  /*!
    Return the \f$\theta_1\f$ parameter of the line corresponding to the
    projection of the cylinder in the image plane.
    \sa getRho1()
    */
  double getTheta1() const { return p[1]; }

  /*!
    Return the \f$\rho_2\f$ parameter of the line corresponding to the
    projection of the cylinder in the image plane.
    \sa getTheta2()
    */
  double getRho2() const { return p[2]; }
  /*!
    Return the \f$\theta_2\f$ parameter of the line corresponding to the
    projection of the cylinder in the image plane.
    \sa getRho2()
    */
  double getTheta2() const { return p[3]; }

  /*!
    Return cylinder \f$A\f$ parameter expressed in the camera frame.
  */
  double getA() const { return cP[0]; }
  /*!
    Return cylinder \f$B\f$ parameter expressed in the camera frame.
  */
  double getB() const { return cP[1]; }
  /*!
    Return cylinder \f$C\f$ parameter expressed in the camera frame.
  */
  double getC() const { return cP[2]; }
  /*!
    Return cylinder \f$X_0\f$ parameter expressed in the camera frame.
  */
  double getX() const { return cP[3]; }
  /*!
    Return cylinder \f$Y_0\f$ parameter expressed in the camera frame.
  */
  double getY() const { return cP[4]; }
  /*!
    Return cylinder \f$Z_0\f$ parameter expressed in the camera frame.
  */
  double getZ() const { return cP[5]; }
  /*!
    Return cylinder \f$R\f$ parameter corresponding to the cylinder radius.
  */
  double getR() const { return cP[6]; }

  void init();

  void projection();
  void projection(const vpColVector &cP, vpColVector &p);

  void setWorldCoordinates(const vpColVector &oP);
  void setWorldCoordinates(const double A, const double B, const double C, const double X0, const double Y0,
                           const double Z0, const double R);
};

#endif
