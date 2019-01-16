/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * 2D point visual feature.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFeaturePointPolar.cpp
  \brief Class that defines a 2D point visual feature with polar coordinates..
*/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// math
#include <visp3/core/vpMath.h>

#include <visp3/core/vpFeatureDisplay.h>

/*

  attributes and members directly related to the vpBasicFeature needs
  other functionalities ar useful but not mandatory

*/

/*!

  Initialise the memory space requested for a 2D point visual
  feature with polar coordinates.

  By default this feature is initialized to \f$(\rho, \theta) = (0,
  0)\f$.  The 3D depth of the point requested in the interaction
  matrix (see interaction()) is initialized to \f$Z=1\f$.
*/
void vpFeaturePointPolar::init()
{
  // feature dimension
  dim_s = 2;
  nbParameters = 3;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default value Z (1 meters)
  Z = 1;
}

/*!

  Default constructor that build a 2D point visual feature with
  polar coordinates and initialize it to \f$(\rho, \theta) = (0,
  0)\f$.

  The 3D depth of the point requested in the interaction
  matrix (see interaction()) is initialized to \f$Z=1\f$.

*/
vpFeaturePointPolar::vpFeaturePointPolar() : Z(1.) { init(); }

/*!
  Set the image point \f$\rho\f$ polar coordinate.

  \sa set_theta()
*/
void vpFeaturePointPolar::set_rho(const double rho)
{
  s[0] = rho;
  flags[0] = true;
}
/*!
  Set the image point \f$\theta\f$ polar coordinate.

  \sa set_rho()
*/
void vpFeaturePointPolar::set_theta(const double theta)
{
  s[1] = theta;
  flags[1] = true;
}

/*!
  Set the 3D point depth in the camera frame.

*/
void vpFeaturePointPolar::set_Z(const double Z_)
{
  this->Z = Z_;
  flags[2] = true;
}

/*!
  Initialize the image point visual feature with polar coordinates.

  \param rho, theta : Polar coordinates \f$(\rho,\theta)\f$ of
  the image point.

  \param Z_ : 3D depth of the point in the camera frame.

  \sa set_rho(), set_theta(), set_Z()
*/
void vpFeaturePointPolar::set_rhoThetaZ(const double rho, const double theta, const double Z_)
{
  set_rho(rho);
  set_theta(theta);
  set_Z(Z_);

  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

/*!
  Get the image point \f$\rho\f$ polar coordinate.

  \sa get_theta()
*/
double vpFeaturePointPolar::get_rho() const { return s[0]; }

/*!
  Get the image point \f$\theta\f$ polar coordinate.

  \sa get_rho()
*/
double vpFeaturePointPolar::get_theta() const { return s[1]; }
/*!
  Get the 3D point depth in the camera frame.

*/
double vpFeaturePointPolar::get_Z() const { return this->Z; }

/*!
  Compute and return the interaction matrix \f$ L \f$ associated to a
  subset of the possible 2D image point features with polar
  coordinates \f$(\rho,\theta)\f$.

  \f[
  L = \left[
  \begin{array}{l}
  L_{\rho} \\
  \; \\
  L_{\theta}\\
  \end{array}
  \right]
  =
  \left[
  \begin{array}{cccccc}
  \frac{-\cos \theta}{Z} & \frac{-\sin \theta}{Z}  &  \frac{\rho}{Z} &
  (1+\rho^2)\sin\theta  & -(1+\rho^2)\cos\theta &  0 \\
  \; \\
   \frac{\sin\theta}{\rho Z} & \frac{-\cos\theta}{\rho Z} &  0 &  \cos\theta
  /\rho &  \sin\theta/\rho & -1 \\ \end{array} \right] \f]

  where \f$Z\f$ is the 3D depth of the considered point.

  \param select : Selection of a subset of the possible polar
  point coordinate features.
  - To compute the interaction matrix for all the two
    subset features \f$(\rho,\theta)\f$ use vpBasicFeature::FEATURE_ALL. In
    that case the dimension of the interaction matrix is \f$ [2 \times
    6] \f$
  - To compute the interaction matrix for only one of the subset
    (\f$\rho,\theta\f$) use one of the corresponding function
    selectRho() or selectTheta(). In that case the returned
    interaction matrix is \f$ [1 \times 6] \f$ dimension.

  \return The interaction matrix computed from the 2D point
  polar coordinate features.

  \exception vpFeatureException::badInitializationError : If the point
  is behind the camera \f$(Z < 0)\f$, or if the 3D depth is null \f$(Z
  = 0)\f$, or if the \f$\rho\f$ polar coordinate of the point is null.

  The code below shows how to compute the interaction matrix associated to
  the visual feature \f$s = (\rho,\theta)\f$.
  \code
  vpFeaturePointPolar s;
  double rho   = 0.3;
  double theta = M_PI;
  double Z     = 1;
  // Creation of the current feature s
  s.buildFrom(rho, theta, Z);
  // Build the interaction matrix L_s
  vpMatrix L = s.interaction();
  \endcode

  The interaction matrix could also be build by:
  \code
  vpMatrix L = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode

  In both cases, L is a 2 by 6 matrix. The first line corresponds to
  the \f$\rho\f$ visual feature while the second one to the
  \f$\theta\f$ visual feature.

  It is also possible to build the interaction matrix associated to
  one of the possible features. The code below shows how to consider
  only the \f$\theta\f$ component.

  \code
  vpMatrix L_theta = s.interaction( vpFeaturePointPolar::selectTheta() );
  \endcode

  In that case, L_theta is a 1 by 6 matrix.
*/
vpMatrix vpFeaturePointPolar::interaction(const unsigned int select)
{
  vpMatrix L;

  L.resize(0, 6);

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but rho "
                  "was not set yet");
          break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but theta "
                  "was not set yet");
          break;
        case 2:
          vpTRACE("Warning !!!  The interaction matrix is computed but Z was "
                  "not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  double rho = get_rho();
  double theta = get_theta();
  double Z_ = get_Z();

  double c_ = cos(theta);
  double s_ = sin(theta);

  double rho2 = rho * rho;

  if (fabs(rho) < 1e-6) {
    vpERROR_TRACE("rho polar coordinate of the point is null");
    std::cout << "rho = " << rho << std::endl;

    throw(vpFeatureException(vpFeatureException::badInitializationError, "rho polar coordinate of the point is null"));
  }

  if (Z_ < 0) {
    vpERROR_TRACE("Point is behind the camera ");
    std::cout << "Z = " << Z_ << std::endl;

    throw(vpFeatureException(vpFeatureException::badInitializationError, "Point is behind the camera "));
  }

  if (fabs(Z_) < 1e-6) {
    vpERROR_TRACE("Point Z coordinates is null ");
    std::cout << "Z = " << Z_ << std::endl;

    throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z coordinates is null"));
  }

  if (vpFeaturePointPolar::selectRho() & select) {
    vpMatrix Lrho(1, 6);
    Lrho = 0;

    Lrho[0][0] = -c_ / Z_;
    Lrho[0][1] = -s_ / Z_;
    Lrho[0][2] = rho / Z_;
    Lrho[0][3] = (1 + rho2) * s_;
    Lrho[0][4] = -(1 + rho2) * c_;
    Lrho[0][5] = 0;

    //     printf("Lrho: rho %f theta %f Z %f\n", rho, theta, Z);
    //     std::cout << "Lrho: " << Lrho << std::endl;

    L = vpMatrix::stack(L, Lrho);
  }

  if (vpFeaturePointPolar::selectTheta() & select) {
    vpMatrix Ltheta(1, 6);
    Ltheta = 0;

    Ltheta[0][0] = s_ / (rho * Z_);
    Ltheta[0][1] = -c_ / (rho * Z_);
    Ltheta[0][2] = 0;
    Ltheta[0][3] = c_ / rho;
    Ltheta[0][4] = s_ / rho;
    Ltheta[0][5] = -1;

    //     printf("Ltheta: rho %f theta %f Z %f\n", rho, theta, Z);
    //     std::cout << "Ltheta: " << Ltheta << std::endl;
    L = vpMatrix::stack(L, Ltheta);
  }
  return L;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  For the angular component \f$\theta\f$, we define the error as
  \f$\theta \ominus \theta^*\f$, where \f$\ominus\f$ is modulo \f$2\pi\f$
  substraction.

  \param s_star : Desired 2D image point visual feature with
  polar coordinates.

  \param select : The error can be computed for a selection of a
  subset of the possible 2D point polar coordinate features.
  - To compute the error for all the three coordinates use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 3
    dimension column vector.
  - To compute the error for only one of the polar coordinate
    feature \f$(\rho,\theta)\f$ use one of the
    corresponding function selectRho() or selectTheta(). In
    that case the error vector is a 1 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  The code below shows how to use this method to manipulate the
  \f$\rho\f$ component.

  \code
  vpFeaturePointPolar s;
  // Creation of the current feature s
  s.buildFrom(0.2, ..., 1);         // rho and Z need to be set
  // Build the interaction matrix L associated to rho component
  vpMatrix L_rho = s.interaction( vpFeaturePointPolar::selectRho() );

  vpFeaturePointPolar s_star;
  // Creation of the desired feature s*
  s_star.buildFrom(0.45, ..., 1.2); // rho and Z need to be set

  // Compute the error vector (s-s*) for the rho feature
  vpColVector e = s.error(s_star, vpFeaturePointPolar::selectRho());
  \endcode
*/
vpColVector vpFeaturePointPolar::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  try {
    if (vpFeaturePointPolar::selectRho() & select) {
      vpColVector erho(1);
      erho[0] = s[0] - s_star[0];

      e = vpColVector::stack(e, erho);
    }

    if (vpFeaturePointPolar::selectTheta() & select) {

      //      printf("err: %f - %f = %f\n", s[1], s_star[1], s[1] -
      //      s_star[1]);
      double err = s[1] - s_star[1];

      //      printf("Error: %f ", err );
      while (err < -M_PI)
        err += 2 * M_PI;
      while (err > M_PI)
        err -= 2 * M_PI;
      //     printf(" modif %f \n", err );

      vpColVector etheta(1);
      etheta[0] = err;
      e = vpColVector::stack(e, etheta);
    }
  } catch (...) {
    throw;
  }

  return e;
}

/*!
  Print to stdout the values of the current visual feature.

  \param select : Selection of a subset of the possible 2D image point
  feature coordinates.
  - To print all the two polar coordinates \f$(\rho,\theta)\f$ used as
  features use vpBasicFeature::FEATURE_ALL.
  - To print only one of the polar coordinate
  feature \f$(\rho,\theta)\f$ use one of the
  corresponding function selectRho() or selectTheta().

  \code
  // Creation of the current feature s
  vpFeaturePointPolar s;
  s.buildFrom(0.1, M_PI_2, 1.3);

  s.print(); // print all the 2 components of the image point feature
  s.print(vpBasicFeature::FEATURE_ALL); // same behavior then previous line
  s.print(vpFeaturePointPolar::selectRho()); // print only the rho component
  \endcode
*/
void vpFeaturePointPolar::print(const unsigned int select) const
{

  std::cout << "Point:  Z=" << get_Z();
  if (vpFeaturePointPolar::selectRho() & select)
    std::cout << " rho=" << get_rho();
  if (vpFeaturePointPolar::selectTheta() & select)
    std::cout << " theta=" << get_theta();
  std::cout << std::endl;
}

/*!

  Build a 2D image point visual feature with polar coordinates.

  \param rho, theta : Polar coordinates \f$(\rho,\theta)\f$ of
  the image point.

  \param Z_ : 3D depth of the point in the camera frame.

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is negative. That means that the 3D point is
  behind the camera which is not possible.

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is null. That means that the 3D point is
  on the camera which is not possible.
*/
void vpFeaturePointPolar::buildFrom(const double rho, const double theta, const double Z_)
{

  s[0] = rho;
  s[1] = theta;

  this->Z = Z_;

  if (Z < 0) {
    vpERROR_TRACE("Point is behind the camera ");
    std::cout << "Z = " << Z << std::endl;

    throw(vpFeatureException(vpFeatureException::badInitializationError, "Point is behind the camera "));
  }

  if (fabs(Z) < 1e-6) {
    vpERROR_TRACE("Point Z coordinates is null ");
    std::cout << "Z = " << Z << std::endl;

    throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z coordinates is null"));
  }

  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

/*!

  Display image point feature.

  \param cam : Camera parameters.
  \param I : Image.
  \param color : Color to use for the display
  \param thickness : Thickness of the feature representation.

*/
void vpFeaturePointPolar::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color,
                                  unsigned int thickness) const
{
  try {
    double rho, theta;
    rho = get_rho();
    theta = get_theta();

    double x, y;
    x = rho * cos(theta);
    y = rho * sin(theta);

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Display image point feature.

  \param cam : Camera parameters.
  \param I : color Image.
  \param color : Color to use for the display
  \param thickness : Thickness of the feature representation.

 */
void vpFeaturePointPolar::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                                  unsigned int thickness) const
{
  try {
    double rho, theta;
    rho = get_rho();
    theta = get_theta();

    double x, y;
    x = rho * cos(theta);
    y = rho * sin(theta);

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeaturePointPolar s;
  s_star = s.duplicate(); // s_star is now a vpFeaturePointPolar
  \endcode

*/
vpFeaturePointPolar *vpFeaturePointPolar::duplicate() const
{
  vpFeaturePointPolar *feature = new vpFeaturePointPolar;
  return feature;
}

/*!

  Function used to select the \f$\rho\f$ subset polar
  coordinate of the image point visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$\rho\f$ feature.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  \code
  vpFeaturePointPolar p;
  vpServo task;
  ...
  // Add only the rho subset coordinate feature from an image point to the
  task task.addFeature(p, vpFeaturePointPolar::selectRho());
  \endcode

  \sa selectTheta()
*/
unsigned int vpFeaturePointPolar::selectRho() { return FEATURE_LINE[0]; }

/*!

  Function used to select the \f$\theta\f$ subset polar
  coordinate of the image point visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$\theta\f$ feature.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  \code
  vpFeaturePointPolar p;
  vpServo task;
  ...
  // Add only the theta subset coordinate feature from an image point to the
  task task.addFeature(p, vpFeaturePointPolar::selectTheta()); \endcode

  \sa selectRho()
*/
unsigned int vpFeaturePointPolar::selectTheta() { return FEATURE_LINE[1]; }
