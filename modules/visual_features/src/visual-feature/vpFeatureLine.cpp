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
 * 2D line visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpFeatureLine.cpp
  \brief Class that defines 2D line visual feature
*/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureLine.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// simple math function (round)
#include <visp3/core/vpMath.h>

// Display Issue

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>

// Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpFeatureDisplay.h>

/*



attributes and members directly related to the vpBasicFeature needs
other functionalities ar useful but not mandatory





*/

/*!
  Initialize the memory space requested for 2D line visual feature.
*/
void vpFeatureLine::init()
{
  // feature dimension
  dim_s = 2;
  nbParameters = 6;

  // memory allocation
  //  x cos(theta) + y sin(theta) - rho = 0
  // s[0] = rho
  // s[1] = theta
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  A = B = C = D = 0.0;
}

/*!
  Default constructor that build a visual feature.
*/
vpFeatureLine::vpFeatureLine() : A(0), B(0), C(0), D(0) { init(); }

/*!
  Sets the values of \f$ \rho \f$ and \f$ \theta \f$ which represent the
  parameters of the 2D line feature.

  \param rho : \f$ \rho \f$ value to set.
  \param theta : \f$ \theta \f$ value to set.
*/
void vpFeatureLine::setRhoTheta(const double rho, const double theta)
{
  s[0] = rho;
  s[1] = theta;
  for (int i = 0; i < 2; i++)
    flags[i] = true;
}

/*!
  Sets the values of A, B, C and D which represent the parameters used to
  describe the equation of a plan in the camera frame. \f[ AX + BY + CZ + D =
  0 \f] Those parameters are needed to compute the interaction matrix
  associated to a visual feature. Normally, two plans are needed to describe a
  line (the intersection of those two plans). But to compute the interaction
  matrix only one plan equation is required. The only one restrictions is that
  the value of D must not be equal to zero !

  \param A_ : A value to set.
  \param B_ : B value to set.
  \param C_ : C value to set.
  \param D_ : D value to set.
*/
void vpFeatureLine::setABCD(const double A_, const double B_, const double C_, const double D_)
{
  this->A = A_;
  this->B = B_;
  this->C = C_;
  this->D = D_;
  for (unsigned int i = 2; i < nbParameters; i++)
    flags[i] = true;
}

/*!

  Compute and return the interaction matrix \f$ L \f$. The computation
  is made thanks to the values of the line feature \f$ \rho \f$ and
  \f$ \theta \f$ and the equation of a plan to which the line belongs.

  \f[ L = \left[\begin{array}{c}L_{\rho} \\ L_{\theta}\end{array}\right] =
  \left[\begin{array}{cccccc}
  \lambda_{\rho}cos(\theta) & \lambda_{\rho}sin(\theta) & -\lambda_{\rho}\rho
  & (1+\rho^2)sin(\theta) & -(1+\rho^2)cos(\theta) & 0 \\
  \lambda_{\theta}cos(\theta) & \lambda_{\theta}sin(\theta) &
  -\lambda_{\theta}\rho & -\rho cos(\theta) & -\rho sin(\theta) & -1
  \end{array}\right]\f]

  Where :
  \f[ \lambda_{\rho} = (A \; \rho \; cos(\theta) + B \; \rho \; sin(\theta) +
  C) / D \f] \f[ \lambda_{\theta} = (A \; sin(\theta) - B \; cos(\theta)) / D
  \f]

  \param select : Selection of a subset of the possible line features.
  - To compute the interaction matrix for all the two line features
    use vpBasicFeature::FEATURE_ALL. In that case the dimension of the
    interaction matrix is \f$ [2 \times 6] \f$
  - To compute the interaction matrix for only one of the line
    component feature (\f$ \rho, \theta \f$) use one of the
    corresponding function selectRho() or selectTheta(). In that case
    the returned interaction matrix is \f$ [1 \times 6] \f$ dimension.

  \return The interaction matrix computed from the line features.

  The code below shows how to compute the interaction matrix
  associated to the visual feature \f$ s = \theta \f$.
  \code
  // Creation of the current feature s
  vpFeatureLine s;
  s.buildFrom(0, 0, 0, 0, 1, -1);

  vpMatrix L_theta = s.interaction( vpFeatureLine::selectTheta() );
  \endcode

  The code below shows how to compute the interaction matrix
  associated to the visual feature \f$ s = (\rho, \theta) \f$.

  \code
  // Creation of the current feature s
  vpFeatureLine s;
  s.buildFrom(0, 0, 0, 0, 1, -1);

  vpMatrix L_theta = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode
*/
vpMatrix vpFeatureLine::interaction(const unsigned int select)
{
  vpMatrix L;

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
          vpTRACE("Warning !!!  The interaction matrix is computed but A was "
                  "not set yet");
          break;
        case 3:
          vpTRACE("Warning !!!  The interaction matrix is computed but B was "
                  "not set yet");
          break;
        case 4:
          vpTRACE("Warning !!!  The interaction matrix is computed but C was "
                  "not set yet");
          break;
        case 5:
          vpTRACE("Warning !!!  The interaction matrix is computed but D was "
                  "not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }
  double rho = s[0];
  double theta = s[1];

  double co = cos(theta);
  double si = sin(theta);

  if (fabs(D) < 1e-6) {
    vpERROR_TRACE("Incorrect plane  coordinates D is null, D = %f", D);

    throw(vpFeatureException(vpFeatureException::badInitializationError, "Incorrect plane  coordinates D"));
  }

  double lambda_theta = (A * si - B * co) / D;
  double lambda_rho = (C + rho * A * co + rho * B * si) / D;

  if (vpFeatureLine::selectRho() & select) {
    vpMatrix Lrho(1, 6);

    Lrho[0][0] = co * lambda_rho;
    Lrho[0][1] = si * lambda_rho;
    Lrho[0][2] = -rho * lambda_rho;
    Lrho[0][3] = si * (1.0 + rho * rho);
    Lrho[0][4] = -co * (1.0 + rho * rho);
    Lrho[0][5] = 0.0;

    L.stack(Lrho);
  }

  if (vpFeatureLine::selectTheta() & select) {
    vpMatrix Ltheta(1, 6);

    Ltheta[0][0] = co * lambda_theta;
    Ltheta[0][1] = si * lambda_theta;
    Ltheta[0][2] = -rho * lambda_theta;
    Ltheta[0][3] = -rho * co;
    Ltheta[0][4] = -rho * si;
    Ltheta[0][5] = -1.0;

    L.stack(Ltheta);
  }
  return L;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  \param s_star : Desired visual feature.

  \param select : The error can be computed for a selection of a
  subset of the possible line features.
  - To compute the error for all the two line features use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 2
    dimension column vector.
  - To compute the error for only one of the line component feature
    (\f$ \rho, \theta \f$) use one of the corresponding function
    selectRho() or selectTheta(). In that case the error vector is a 1
    dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  The code below shows how to use this method to manipulate the \f$
  \theta \f$ subset:

  \code
  // Creation of the current feature s
  vpFeatureLine s;
  s.buildFrom(0, 0, 0, 0, 1, -1);

  // Creation of the desired feature s*
  vpFeatureLine s_star;
  s_star.buildFrom(0, 0, 0, 0, 1, -5);

  // Compute the interaction matrix for the theta feature
  vpMatrix L_theta = s.interaction( vpFeatureLine::selectTheta() );

  // Compute the error vector (s-s*) for the Theta feature
  s.error(s_star, vpFeatureLine::selectTheta());
  \endcode
*/
vpColVector vpFeatureLine::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  try {
    if (vpFeatureLine::selectRho() & select) {
      vpColVector erho(1);
      erho[0] = s[0] - s_star[0];

      e = vpColVector::stack(e, erho);
    }

    if (vpFeatureLine::selectTheta() & select) {

      double err = s[1] - s_star[1];
      while (err < -M_PI)
        err += 2 * M_PI;
      while (err > M_PI)
        err -= 2 * M_PI;

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
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible line features.
  - To print all the two line features use vpBasicFeature::FEATURE_ALL.
  - To print only one of the line component feature (\f$ \rho, \theta \f$)
    use one of the corresponding function selectRho() or selectTheta().

  \code
  vpFeatureLine s; // Current visual feature s

  // Creation of the current feature s
  s.buildFrom(0, 0);

  s.print(); // print all the 2 components of the feature
  s.print(vpBasicFeature::FEATURE_ALL);  // same behavior then previous line
  s.print(vpFeatureLine::selectRho()); // print only the rho component
  \endcode
*/

void vpFeatureLine::print(const unsigned int select) const
{

  std::cout << "Line:\t  " << A << "X+" << B << "Y+" << C << "Z +" << D << "=0" << std::endl;
  ;
  if (vpFeatureLine::selectRho() & select)
    std::cout << "     \trho=" << s[0];
  if (vpFeatureLine::selectTheta() & select)
    std::cout << "     \ttheta=" << s[1];
  std::cout << std::endl;
}

/*!

  Build a 2D line visual feature from the line equation parameters \f$
  \rho \f$ and \f$ \theta \f$ given in the image plan.

  \f[ x \; cos(\theta) + y \; sin(\theta) -\rho = 0 \f]

  See the vpFeatureLine class description for more details about \f$
  \rho \f$ and \f$ \theta \f$.

  \param rho : The \f$ \rho \f$ parameter.
  \param theta : The \f$ \theta \f$ parameter.

*/
void vpFeatureLine::buildFrom(const double rho, const double theta)
{
  s[0] = rho;
  s[1] = theta;
  for (int i = 0; i < 2; i++)
    flags[i] = true;
}

/*!

  Build a 2D line visual feature from the line equation parameters \f$
  \rho \f$ and \f$ \theta \f$ given in the image plan. The
  parameters A, B, C and D which describe the equation of a plan to
  which the line belongs, are set in the same time.

  \f[ x \; cos(\theta) + y \; sin(\theta) -\rho = 0 \f]
  \f[ AX + BY + CZ + D = 0 \f]

  See the vpFeatureLine class description for more details about \f$
  \rho \f$ and \f$ \theta \f$.

  The A, B, C, D parameters are needed to compute the interaction
  matrix associated to a visual feature. Normally, two plans are
  needed to describe a line (the intersection of those two plans). But
  to compute the interaction matrix only one plan equation is
  required. The only one restrictions is that the value of D must not
  be equal to zero !

  \param rho : The \f$ \rho \f$ parameter.
  \param theta : The \f$ \theta \f$ parameter.
  \param A_ : A parameter of the plan equation.
  \param B_ : B parameter of the plan equation.
  \param C_ : C parameter of the plan equation.
  \param D_ : D parameter of the plan equation.

*/
void vpFeatureLine::buildFrom(const double rho, const double theta, const double A_, const double B_, const double C_,
                              const double D_)
{
  s[0] = rho;
  s[1] = theta;
  this->A = A_;
  this->B = B_;
  this->C = C_;
  this->D = D_;
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLine s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLine
  \endcode

*/
vpFeatureLine *vpFeatureLine::duplicate() const
{
  vpFeatureLine *feature = new vpFeatureLine;
  return feature;
}

/*!

  Display line feature.

  \param cam : Camera parameters.
  \param I : Image on which features have to be displayed.
  \param color : Color used to display the feature.
  \param thickness : Thickness of the feature representation.

*/
void vpFeatureLine::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color,
                            unsigned int thickness) const
{
  try {
    double rho, theta;
    rho = getRho();
    theta = getTheta();

    vpFeatureDisplay::displayLine(rho, theta, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Display line feature.

  \param cam : Camera parameters.
  \param I : Color image on which features have to be displayed.
  \param color : Color used to display the feature.
  \param thickness : Thickness of the feature representation.

 */
void vpFeatureLine::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                            unsigned int thickness) const
{
  try {
    double rho, theta;
    rho = getRho();
    theta = getTheta();

    vpFeatureDisplay::displayLine(rho, theta, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Function used to select the \f$ \rho \f$ subset of the line visual feature.

  This function is to use in conjunction with interaction() in order to
  compute the interaction matrix associated to \f$ \rho \f$.

  This function is also useful in the vpServo class to indicate that a subset
  of the visual feature is to use in the control law:

  \code
  vpFeatureLine s;
  vpServo task;
  ...
  // Add the (rho) subset features from the 2D line
  task.addFeature(s, vpFeatureLine::selectRho());
  \endcode
*/
unsigned int vpFeatureLine::selectRho() { return FEATURE_LINE[0]; }

/*!

  Function used to select the \f$ \theta \f$ subset of the line visual
  feature.

  This function is to use in conjunction with interaction() in order to
  compute the interaction matrix associated to \f$ \theta \f$.

  This function is also useful in the vpServo class to indicate that a subset
  of the visual feature is to use in the control law:

  \code
  vpFeatureLine s;
  vpServo task;
  ...
  // Add the (rho) subset features from the 2D line
  task.addFeature(s, vpFeatureLine::selectTheta());
  \endcode
*/
unsigned int vpFeatureLine::selectTheta() { return FEATURE_LINE[1]; }
