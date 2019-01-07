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
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpFeatureDepth.cpp
  \brief Class that defines 2D point visual feature
*/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureDepth.h>

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
  Initialize the memory space requested for 3D depth visual feature.
*/
void vpFeatureDepth::init()
{
  // feature dimension
  dim_s = 1;
  nbParameters = 3;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;
  x = y = 0.;
  Z = 1.;
}

/*!
  Default constructor that build a visual feature.
*/
vpFeatureDepth::vpFeatureDepth() : x(0), y(0), Z(1.) { init(); }

/*!
  Set the value of \f$ log(\frac{Z}{Z^*}) \f$ which represents the logarithm
  of the current depth relative to the desired depth.

  \param LogZoverZstar : \f$ log(\frac{Z}{Z^*}) \f$ value to set.
*/
void vpFeatureDepth::set_LogZoverZstar(const double LogZoverZstar) { s[0] = LogZoverZstar; }

/*!
  Get the value of \f$ log(\frac{Z}{Z^*}) \f$ which represents the logarithm
  of the current depth relative to the desired depth.

  \return The value of \f$ log(\frac{Z}{Z^*}) \f$.
*/
double vpFeatureDepth::get_LogZoverZstar() const { return s[0]; }

/*!
  Set the value of \f$ x \f$ which represents the x coordinate of the point in
  the camera frame.

  \param x_ : \f$ x \f$ value to set.
*/
void vpFeatureDepth::set_x(const double x_)
{
  this->x = x_;
  flags[0] = true;
}

/*!
  Get the value of \f$ x \f$ which represents the x coordinate of the point in
  the camera frame.

  \return The value of \f$ x \f$.
*/
double vpFeatureDepth::get_x() const { return x; }

/*!
  Set the value of \f$ y \f$ which represents the y coordinate of the point in
  the camera frame.

  \param y_ : \f$ y \f$ value to set.
*/
void vpFeatureDepth::set_y(const double y_)
{
  this->y = y_;
  flags[1] = true;
}

/*!
  Get the value of \f$ y \f$ which represents the y coordinate of the point in
  the camera frame.

  \return The value of \f$ y \f$.
*/
double vpFeatureDepth::get_y() const { return y; }

/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \param Z_ : \f$ Z \f$ value to set.
*/
void vpFeatureDepth::set_Z(const double Z_)
{
  this->Z = Z_;
  flags[2] = true;
}

/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \return The value of \f$ Z \f$.
*/
double vpFeatureDepth::get_Z() const { return Z; }

/*!
  Set the value of \f$ x \f$, \f$ y \f$, \f$ Z \f$ and \f$ log(\frac{Z}{Z^*})
  \f$. \f$ x \f$ and \f$ y \f$ represent the coordinates of the point in the
  camera frame. \f$ Z \f$ is the 3D coordinate representing the depth. \f$
  log(\frac{Z}{Z^*}) \f$ represents the logarithm of the current depth
  relative to the desired depth.

  \param x_ : \f$ x \f$ value to set.
  \param y_ : \f$ y \f$ value to set.
  \param Z_ : \f$ Z \f$ value to set.
  \param LogZoverZstar : \f$ log(\frac{Z}{Z^*}) \f$ value to set.
*/
void vpFeatureDepth::set_xyZLogZoverZstar(const double x_, const double y_, const double Z_, const double LogZoverZstar)
{
  set_x(x_);
  set_y(y_);
  set_Z(Z_);
  set_LogZoverZstar(LogZoverZstar);
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

/*!
  Compute and return the interaction matrix \f$ L \f$. The computation is made
  thanks to the values of the point coordinates \f$ x \f$ and \f$ y \f$ and
  the depth \f$ Z \f$.

  \f[ L =
  \left[\begin{array}{cccccc}
  0 & 0 & -1/Z & -y & x & 0
  \end{array}\right]\f]

  \param select : unuseful in the case of vpFeatureDepth. Always set to
  FEATURE_ALL.

  \return The interaction matrix computed from the point feature.

  The code below shows how to compute the interaction matrix associated to the
  visual feature \f$ s = log(\frac{Z}{Z^*}) \f$.
  \code
  // Creation of the current feature s
  vpFeatureDepth s;
  s.buildFrom(0, 0, 5, log(5/1)); //The current depth is 5 metters and the
  desired is 1 metter.

  vpMatrix L_x = s.interaction();
  \endcode
*/
vpMatrix vpFeatureDepth::interaction(const unsigned int select)
{
  vpMatrix L;

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but x was "
                  "not set yet");
          break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but y was "
                  "not set yet");
          break;
        case 2:
          vpTRACE("Warning !!!  The interaction matrix is computed but z was "
                  "not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  L.resize(1, 6);

  double x_ = get_x();
  double y_ = get_y();
  double Z_ = get_Z();

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

  if (FEATURE_ALL & select) {
    L = 0;
    L[0][0] = 0;
    L[0][1] = 0;
    L[0][2] = -1 / Z_;
    L[0][3] = -y_;
    L[0][4] = x_;
    L[0][5] = 0;
  }

  return L;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  Since this visual feature \f$ s \f$ represent the current depth relative to
  the desired depth, the desired visual feature \f$ s^* \f$ should be zero.
  Thus, the error is here equal to the current visual feature \f$ s \f$.

  \param s_star : Desired visual visual feature that should be equal to zero.
  \param select : unuseful in the case of vpFeatureDepth. Always set to
  FEATURE_ALL.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  \exception vpFeatureException::badInitializationError : If the
  desired visual feature \f$ s^* \f$ is not equal to zero.

  The code below shows how to use this method:

  \code
  // Creation of the current feature s
  vpFeatureDepth s;

  // Creation of the desired feature s^*. By default this feature is
  // initialized to zero
  vpFeatureDepth s_star;
  s_star.setLogZoverZstar(0)

  // Compute the interaction matrix for the ThetaU_z feature
  vpMatrix L_z = s.interaction();

  // Compute the error vector (s-s*) for the ThetaU_z feature
  s.error(s_star);
  \endcode
*/
vpColVector vpFeatureDepth::error(const vpBasicFeature &s_star, const unsigned int select)
{

  if (fabs(s_star.get_s().sumSquare()) > 1e-6) {
    vpERROR_TRACE("s* should be zero ! ");
    throw(vpFeatureException(vpFeatureException::badInitializationError, "s* should be zero !"));
  }

  vpColVector e(1);
  if (FEATURE_ALL & select) {
    e[0] = s[0];
  }

  return e;
}

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : unuseful in the case of vpFeatureDepth. Always set to
  FEATURE_ALL.

  \code
  vpFeatureDepth s; // Current visual feature s

  // Creation of the current feature s
  s.buildFrom(0, 0, 5, log(5/1));

  s.print(); // print all the 2 components of the feature
  \endcode
*/
void vpFeatureDepth::print(const unsigned int select) const
{
  if (FEATURE_ALL & select) {
    std::cout << "Point:  x=" << get_x();
    std::cout << " Point:  y=" << get_y();
    std::cout << " Point:  Z=" << get_Z();

    std::cout << " log(Z/Z*)=" << get_LogZoverZstar();

    std::cout << std::endl;
  }
}

/*!
  Build a 3D depth visual feature from the point coordinates \f$ x \f$ and \f$
  y \f$ given in the camera frame, \f$ Z \f$ which describes the depth and \f$
  log(\frac{Z}{Z^*}) \f$ which represents the logarithm of the current depth
  relative to the desired depth.

  \param x_ : The \f$ x \f$ parameter.
  \param y_ : The \f$ y \f$ parameter.
  \param Z_ : The \f$ Z \f$ parameter.
  \param LogZoverZstar : The \f$ log(\frac{Z}{Z^*}) \f$ parameter.
*/
void vpFeatureDepth::buildFrom(const double x_, const double y_, const double Z_, const double LogZoverZstar)
{

  s[0] = LogZoverZstar;

  this->x = x_;
  this->y = y_;
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
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureDepth s;
  s_star = s.duplicate(); // s_star is now a vpFeatureDepth
  \endcode

*/
vpFeatureDepth *vpFeatureDepth::duplicate() const
{
  vpFeatureDepth *feature = new vpFeatureDepth;
  return feature;
}

/*!

  Not implemented.

*/
void vpFeatureDepth::display(const vpCameraParameters & /* cam */, const vpImage<unsigned char> & /* I */,
                             const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}
/*!

  Not implemented.

 */
void vpFeatureDepth::display(const vpCameraParameters & /* cam */, const vpImage<vpRGBa> & /* I */,
                             const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
