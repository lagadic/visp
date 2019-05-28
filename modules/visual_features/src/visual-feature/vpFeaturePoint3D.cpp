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
 * 3D point visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

/*

  attributes and members directly related to the vpBasicFeature needs
  other functionalities are useful but not mandatory

*/

/*!

  Initialise the memory space requested for a 3D point visual
  feature.

  By default this feature is initialized to \f${\bf X} = (0, 0, 1)\f$.
*/
void vpFeaturePoint3D::init()
{
  // feature dimension
  dim_s = 3;
  nbParameters = 3;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default value XYZ
  s[0] = 0;
  s[1] = 0;
  s[2] = 1;
}

/*!

  Default constructor that build a 3D point visual feature and
  initialize it to \f${\bf X} = (0, 0, 1)\f$.

*/
vpFeaturePoint3D::vpFeaturePoint3D() { init(); }

/*!

  Initialise the \f$X\f$ coordinate in the camera frame of the 3D Point
  visual feature \f${\bf X} = (X,Y,Z)\f$.

  \param X : \f$X\f$ coordinate of the visual feature.
  \sa get_X()

*/
void vpFeaturePoint3D::set_X(const double X)
{
  s[0] = X;
  flags[0] = true;
}

/*!

  Initialise the \f$Y\f$ coordinate in the camera frame of the 3D Point
  visual feature \f${\bf X} = (X,Y,Z)\f$.

  \param Y : \f$Y\f$ coordinate of the visual feature.
  \sa get_Y()

*/
void vpFeaturePoint3D::set_Y(const double Y)
{
  s[1] = Y;
  flags[1] = true;
}

/*!

  Initialise the \f$Z\f$ coordinate in the camera frame of the 3D Point
  visual feature \f${\bf X} = (X,Y,Z)\f$.

  \param Z : \f$Z\f$ coordinate or depth of the visual feature.
  \sa get_Z()

*/
void vpFeaturePoint3D::set_Z(const double Z)
{
  s[2] = Z;
  flags[2] = true;
}

/*!
  Initialize the 3D point coordinates.

  \param X,Y,Z : \f$(X,Y,Z)\f$ coordinates in the camera frame of the
  3D point visual feature.

  \sa set_X(), set_Y(), set_Z()
*/
void vpFeaturePoint3D::set_XYZ(const double X, const double Y, const double Z)
{
  set_X(X);
  set_Y(Y);
  set_Z(Z);

  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

//! Return the \f$X\f$ coordinate in the camera frame of the 3D point.
double vpFeaturePoint3D::get_X() const { return s[0]; }

//! Return the \f$Y\f$ coordinate in the camera frame of the 3D point.
double vpFeaturePoint3D::get_Y() const { return s[1]; }

//! Return the \f$Z\f$ coordinate in the camera frame of the 3D point.
double vpFeaturePoint3D::get_Z() const { return s[2]; }

/*!
  Compute and return the interaction matrix \f$ L \f$ associated to a subset
  of the possible 3D point features \f$(X,Y,Z)\f$ that
  represent the 3D point coordinates expressed in the camera frame.

  \f[
  L = \left[
  \begin{array}{rrrrrr}
  -1 &  0 &  0 &  0 & -Z &  Y \\
   0 & -1 &  0 &  Z &  0 & -X \\
   0 &  0 & -1 & -Y &  X &  0 \\
  \end{array}
  \right]
  \f]


  \param select : Selection of a subset of the possible 3D point coordinate
  features.
  - To compute the interaction matrix for all the three
    subset features \f$(X,Y,Z)\f$ use vpBasicFeature::FEATURE_ALL. In
    that case the dimension of the interaction matrix is \f$ [3 \times
    6] \f$
  - To compute the interaction matrix for only one of the
    subset (\f$X, Y,Z\f$) use
    one of the corresponding function selectX(), selectY() or
    selectZ(). In that case the returned interaction matrix is \f$ [1
    \times 6] \f$ dimension.

  \return The interaction matrix computed from the 3D point coordinate
  features.

  The code below shows how to compute the interaction matrix
  associated to the visual feature \f$s = X \f$.

  \code
  vpPoint point;
  ...
  // Creation of the current feature s
  vpFeaturePoint3D s;
  s.buildFrom(point);

  vpMatrix L_X = s.interaction( vpFeaturePoint3D::selectX() );
  \endcode

  The code below shows how to compute the interaction matrix
  associated to the \f$s = (X,Y) \f$
  subset visual feature:

  \code
  vpMatrix L_XY = s.interaction( vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectY() );
  \endcode

  L_XY is here now a 2 by 6 matrix. The first line corresponds to
  the \f$ X \f$ visual feature while the second one to the \f$
  Y \f$ visual feature.

  It is also possible to build the interaction matrix from all the
  3D point coordinates by:

  \code
  vpMatrix L_XYZ = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode

  In that case, L_XYZ is a 3 by 6 interaction matrix where the last
  line corresponds to the \f$ Z \f$ visual feature.

*/
vpMatrix vpFeaturePoint3D::interaction(const unsigned int select)
{
  vpMatrix L;

  L.resize(0, 6);

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but X was "
                  "not set yet");
          break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but Y was "
                  "not set yet");
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

  double X = get_X();
  double Y = get_Y();
  double Z = get_Z();

  if (vpFeaturePoint3D::selectX() & select) {
    vpMatrix Lx(1, 6);
    Lx = 0;

    Lx[0][0] = -1;
    Lx[0][1] = 0;
    Lx[0][2] = 0;
    Lx[0][3] = 0;
    Lx[0][4] = -Z;
    Lx[0][5] = Y;

    L = vpMatrix::stack(L, Lx);
  }

  if (vpFeaturePoint3D::selectY() & select) {
    vpMatrix Ly(1, 6);
    Ly = 0;

    Ly[0][0] = 0;
    Ly[0][1] = -1;
    Ly[0][2] = 0;
    Ly[0][3] = Z;
    Ly[0][4] = 0;
    Ly[0][5] = -X;

    L = vpMatrix::stack(L, Ly);
  }
  if (vpFeaturePoint3D::selectZ() & select) {
    vpMatrix Lz(1, 6);
    Lz = 0;

    Lz[0][0] = 0;
    Lz[0][1] = 0;
    Lz[0][2] = -1;
    Lz[0][3] = -Y;
    Lz[0][4] = X;
    Lz[0][5] = 0;

    L = vpMatrix::stack(L, Lz);
  }
  return L;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  \param s_star : Desired 3D point visual feature.

  \param select : The error can be computed for a selection of a
  subset of the possible 3D point coordinate features.
  - To compute the error for all the three coordinates use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 3
    dimension column vector.
  - To compute the error for only one of the coordinate
    feature \f$(X,Y, or Z)\f$ use one of the
    corresponding function selectX(), selectY() or selectZ(). In
    that case the error vector is a 1 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  The code below shows how to use this method to manipulate the \f$
  Z \f$ subset:

  \code
  // Creation of the current feature s
  vpFeaturePoint3D s;
  s.set_Z(0.8); // Initialization of the current Z feature

  // Creation of the desired feature s*.
  vpFeatureTranslation s_star;
  s_star.set_Z(1); // Initialization of the current Z* feature to Z*=1 meter

  // Compute the interaction matrix for the Z coordinate feature
  vpMatrix L_Z = s.interaction( vpFeaturePoint3D::selectZ() );

  // Compute the error vector (s-s*) for the Z feature
  s.error(s_star, vpFeaturePoint3D::selectZ());
  \endcode

  To manipulate the subset features \f$s=(Y, Z)\f$,
  the code becomes:
  \code
  // Compute the interaction matrix for the Y, Z feature coordinates
  vpMatrix L_YZ = s.interaction( vpFeaturePoint3D::selectY() |
  vpFeaturePoint3D::selectZ() );

  // Compute the error vector e = (s-s*) for the Y, Z feature coordinates
  vpColVector e = s.error(s_star, vpFeaturePoint3D::selectY() |
  vpFeaturePoint3D::selectZ());
  \endcode

*/
vpColVector vpFeaturePoint3D::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  try {
    if (vpFeaturePoint3D::selectX() & select) {
      vpColVector ex(1);
      ex[0] = s[0] - s_star[0];

      e = vpColVector::stack(e, ex);
    }

    if (vpFeaturePoint3D::selectY() & select) {
      vpColVector ey(1);
      ey[0] = s[1] - s_star[1];
      e = vpColVector::stack(e, ey);
    }

    if (vpFeaturePoint3D::selectZ() & select) {
      vpColVector ez(1);
      ez[0] = s[2] - s_star[2];
      e = vpColVector::stack(e, ez);
    }
  } catch (...) {
    throw;
  }

  return e;
}

/*!

  Build a 3D point visual feature from the camera frame coordinates
  \f$(X,Y,Z)\f$ of a point.

  \param p : A point with camera frame coordinates \f${^c}P=(X,Y,Z)\f$
  up to date (see vpPoint class).

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is negative. That means that the 3D point is
  behind the camera which is not possible.

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is null. That means that the 3D point is
  on the camera which is not possible.
*/
void vpFeaturePoint3D::buildFrom(const vpPoint &p)
{

  // cP is expressed in homogeneous coordinates
  // we devide by the fourth coordinate
  s[0] = p.cP[0] / p.cP[3];
  s[1] = p.cP[1] / p.cP[3];
  s[2] = p.cP[2] / p.cP[3];

  double Z = s[2];
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

  Build a 3D point visual feature from the camera frame coordinates
  \f$(X,Y,Z)\f$ of a point.

  \param X,Y,Z : Camera frame coordinates \f$(X,Y,Z)\f$ of a 3D point.

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is negative. That means that the 3D point is
  on the camera which is not possible.

  \exception vpFeatureException::badInitializationError: If the depth
  (\f$Z\f$ coordinate) is null. That means that the 3D point is
  on the camera which is not possible.

*/
void vpFeaturePoint3D::buildFrom(const double X, const double Y, const double Z)
{

  s[0] = X;
  s[1] = Y;
  s[2] = Z;

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
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible 3D point
  feature coordinates.
  - To print all the three coordinates used as features use
  vpBasicFeature::FEATURE_ALL.
  - To print only one of the coordinate
  feature \f$(X,Y,Z)\f$ use one of the
  corresponding function selectX(), selectX() or selectZ().

  \code
  vpPoint point;

  // Creation of the current feature s
  vpFeaturePoint3D s;
  s.buildFrom(point);

  s.print(); // print all the 3 components of the translation feature
  s.print(vpBasicFeature::FEATURE_ALL); // same behavior then previous line
  s.print(vpFeaturePoint3D::selectZ()); // print only the Z component
  \endcode
*/
void vpFeaturePoint3D::print(const unsigned int select) const
{

  std::cout << "Point3D:  ";
  if (vpFeaturePoint3D::selectX() & select)
    std::cout << " X=" << get_X();
  if (vpFeaturePoint3D::selectY() & select)
    std::cout << " Y=" << get_Y();
  if (vpFeaturePoint3D::selectZ() & select)
    std::cout << " Z=" << get_Z();
  std::cout << std::endl;
}

/*!

  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeaturePoint3D s;
  s_star = s.duplicate(); // s_star is now a vpFeaturePoint3D
  \endcode

*/
vpFeaturePoint3D *vpFeaturePoint3D::duplicate() const
{
  vpFeaturePoint3D *feature = new vpFeaturePoint3D;
  return feature;
}

/*!

  Not implemented.
*/
void vpFeaturePoint3D::display(const vpCameraParameters & /*cam*/, const vpImage<unsigned char> & /* I */,
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
void vpFeaturePoint3D::display(const vpCameraParameters & /*cam*/, const vpImage<vpRGBa> & /* I */,
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

  Function used to select the \f$ X\f$ subset coordinate of the 3D point
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ X\f$ feature.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  \code
  vpFeaturePoint3D p;
  vpServo task;
  ...
  // Add the (X,Y) subset coordinates features from a 3D point to the task
  task.addFeature(p, vpFeaturePoint3D::selectX() |
  vpFeaturePoint3D::selectY());
  \endcode

  \sa selectY(), selectZ()

*/
unsigned int vpFeaturePoint3D::selectX() { return FEATURE_LINE[0]; }

/*!

  Function used to select the \f$ Y\f$ subset coordinate of the 3D point
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ Y\f$ feature.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  \code
  vpFeaturePoint3D p;
  vpServo task;
  ...
  // Add the (X,Y) subset coordinates features from a 3D point to the task
  task.addFeature(p, vpFeaturePoint3D::selectX() |
  vpFeaturePoint3D::selectY());
  \endcode

  \sa selectX(), selectZ()

*/
unsigned int vpFeaturePoint3D::selectY() { return FEATURE_LINE[1]; }

/*!

  Function used to select the \f$ Z\f$ subset coordinate of the 3D point
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ Z\f$ feature.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  \code
  vpFeaturePoint3D p;
  vpServo task;
  ...
  // Add the (Z) subset coordinate feature from a 3D point to the task
  task.addFeature(p, vpFeaturePoint3D::selectZ());
  \endcode

  \sa selectX(), selectY()

*/
unsigned int vpFeaturePoint3D::selectZ() { return FEATURE_LINE[2]; }
