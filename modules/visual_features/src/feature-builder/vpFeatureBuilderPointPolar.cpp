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
 * Conversion between tracker and visual feature point with
 * polar coordinates.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderPointPolar.cpp

  \brief Conversion between tracker and visual feature point with
  polar coordinates.
*/
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureException.h>

#ifdef VISP_HAVE_MODULE_BLOB
/*!

  Initialize a point feature with polar coordinates
  \f$(\rho,\theta)\f$ using the coordinates of the point in pixels
  obtained by image processing. This point is the center
  of gravity of a dot tracked using vpDot. Using the camera
  parameters, the pixels coordinates of the dot are first
  converted in cartesian \f$(x,y)\f$ coordinates in meter in the
  camera frame and than in polar coordinates by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  \warning This function does not initialize \f$Z\f$ which is
  requested to compute the interaction matrix by
  vpfeaturePointPolar::interaction().

  \param s : Visual feature \f$(\rho,\theta)\f$ to initialize. Be
  aware, the 3D depth \f$Z\f$ requested to compute the interaction
  matrix is not initialized by this function.

  \param cam : Camera parameters.

  \param dot : Tracked dot. The center of gravity corresponds to the
  coordinates of the point in the image plane.

  The code below shows how to initialize a vpFeaturePointPolar visual
  feature. First, we initialize the \f$\rho,\theta\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpDot2 dot;               // Dot tracker

  vpFeaturePointPolar s;    // Point feature with polar coordinates
  ...
  // Tracking on the dot
  dot.track(I);

  // Initialize rho,theta visual feature
  vpFeatureBuilder::create(s, cam, dot);

  // A pose estimation is requested to initialize Z, the depth of the
  // point in the camera frame.
  double Z = 1; // Depth of the point in meters
  ....
  s.set_Z(Z);
  \endcode

*/
void vpFeatureBuilder::create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpDot &dot)
{
  try {
    double x = 0, y = 0;

    vpImagePoint cog;
    cog = dot.getCog();

    vpPixelMeterConversion::convertPoint(cam, cog, x, y);

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);

    s.set_rho(rho);
    s.set_theta(theta);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Initialize a point feature with polar coordinates
  \f$(\rho,\theta)\f$ using the coordinates of the point in pixels
  obtained by image processing. This point is the center
  of gravity of a dot tracked using vpDot2. Using the camera
  parameters, the pixels coordinates of the dot are first
  converted in cartesian \f$(x,y)\f$ coordinates in meter in the
  camera frame and than in polar coordinates by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  \warning This function does not initialize \f$Z\f$ which is
  requested to compute the interaction matrix by
  vpfeaturePointPolar::interaction().

  \param s : Visual feature \f$(\rho,\theta)\f$ to initialize. Be
  aware, the 3D depth \f$Z\f$ requested to compute the interaction
  matrix is not initialized by this function.

  \param cam : Camera parameters.

  \param dot : Tracked dot. The center of gravity corresponds to the
  coordinates of the point in the image plane.

  The code below shows how to initialize a vpFeaturePointPolar visual
  feature. First, we initialize the \f$\rho,\theta\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpDot2 dot;               // Dot tracker

  vpFeaturePointPolar s;    // Point feature with polar coordinates
  ...
  // Tracking on the dot
  dot.track(I);

  // Initialize rho,theta visual feature
  vpFeatureBuilder::create(s, cam, dot);

  // A pose estimation is requested to initialize Z, the depth of the
  // point in the camera frame.
  double Z = 1; // Depth of the point in meters
  ....
  s.set_Z(Z);
  \endcode

*/
void vpFeatureBuilder::create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpDot2 &dot)
{
  try {
    double x = 0, y = 0;

    vpImagePoint cog;
    cog = dot.getCog();

    vpPixelMeterConversion::convertPoint(cam, cog, x, y);

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);

    s.set_rho(rho);
    s.set_theta(theta);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
#endif //#ifdef VISP_HAVE_MODULE_BLOB

/*!

  Initialize a point feature with polar coordinates
  \f$(\rho,\theta)\f$ using the coordinates of the point in pixels
  obtained by image processing. The points coordinates are stored in a
  vpImagePoint. Using the camera parameters, the pixels coordinates of the
  point are first converted in cartesian \f$(x,y)\f$ coordinates in meter in
  the camera frame and than in polar coordinates by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  \warning This function does not initialize \f$Z\f$ which is
  requested to compute the interaction matrix by
  vpfeaturePointPolar::interaction().

  \param s : Visual feature \f$(\rho,\theta)\f$ to initialize. Be
  aware, the 3D depth \f$Z\f$ requested to compute the interaction
  matrix is not initialized by this function.

  \param cam : Camera parameters.

  \param iP : The vpImagePoint used to create the vpFeaturePoint.

  The code below shows how to initialize a vpFeaturePointPolar visual
  feature. First, we initialize the \f$\rho,\theta\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpImagePoint iP;               // the point in the image

  vpFeaturePointPolar s;    // Point feature with polar coordinates
  ...
  // Set the point coordinates in the image (here the coordinates are given in
  the (i,j) frame iP.set_i(0); iP.set_j(0);

  // Initialize rho,theta visual feature
  vpFeatureBuilder::create(s, cam, iP);

  // A pose estimation is requested to initialize Z, the depth of the
  // point in the camera frame.
  double Z = 1; // Depth of the point in meters
  ....
  s.set_Z(Z);
  \endcode

*/
void vpFeatureBuilder::create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpImagePoint &iP)
{
  try {
    double x = 0, y = 0;

    vpPixelMeterConversion::convertPoint(cam, iP, x, y);

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);

    s.set_rho(rho);
    s.set_theta(theta);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Initialize a point feature with polar coordinates
  \f$(\rho,\theta)\f$ using the coordinates of the point
  \f$(x,y,Z)\f$, where \f$(x,y)\f$ correspond to the perspective
  projection of the point in the image plane and \f$Z\f$ the 3D depth
  of the point in the camera frame. The values of \f$(x,y,Z)\f$ are
  expressed in meters. From the coordinates in the image plane, the
  polar coordinates are computed by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  \param s : Visual feature \f$(\rho,\theta)\f$ and \f$Z\f$ to initialize.

  \param p : A point with \f$(x,y)\f$ cartesian coordinates in the
  image plane corresponding to the camera perspective projection, and
  with 3D depth \f$Z\f$.
*/
void vpFeatureBuilder::create(vpFeaturePointPolar &s, const vpPoint &p)
{
  try {

    double x = p.get_x();
    double y = p.get_y();

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);

    s.set_rho(rho);
    s.set_theta(theta);

    s.set_Z(p.get_Z());

    if (s.get_Z() < 0) {
      vpERROR_TRACE("Point is behind the camera ");
      std::cout << "Z = " << s.get_Z() << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point is behind the camera "));
    }

    if (fabs(s.get_Z()) < 1e-6) {
      vpERROR_TRACE("Point Z coordinates is null ");
      std::cout << "Z = " << s.get_Z() << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z coordinates is null"));
    }

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Initialize a point feature with polar coordinates
  \f$(\rho,\theta)\f$ using the coordinates of the point
  \f$(x,y,Z)\f$, where \f$(x,y)\f$ correspond to the perspective
  projection of the point in the image plane and \f$Z\f$ the 3D depth
  of the point in the camera frame. The values of \f$(x,y,Z)\f$ are
  expressed in meters.

  This function intends to introduce noise in the conversion from
  cartesian to polar coordinates. Cartesian \f$(x,y)\f$ coordinates
  are first converted in pixel coordinates in the image using \e
  goodCam camera parameters. Then, the pixels coordinates of the point
  are converted back to cartesian coordinates \f$(x^{'},y^{'})\f$ using
  the noisy camera parameters \e wrongCam. From these new coordinates
  in the image plane, the polar coordinates are computed by:

  \f[\rho = \sqrt{x^2+y^2}  \hbox{,}\; \; \theta = \arctan \frac{y}{x}\f]

  \param s : Visual feature \f$(\rho,\theta)\f$ and \f$Z\f$ to initialize.

  \param goodCam : Camera parameters used to introduce noise. These
  parameters are used to convert cartesian coordinates of the point \e
  p in the image plane in pixel coordinates.

  \param wrongCam : Camera parameters used to introduce noise. These
  parameters are used to convert pixel coordinates of the point in
  cartesian coordinates of the point in the image plane.

  \param p : A point with \f$(x,y)\f$ cartesian coordinates in the
  image plane corresponding to the camera perspective projection, and
  with 3D depth \f$Z\f$.
*/
void vpFeatureBuilder::create(vpFeaturePointPolar &s, const vpCameraParameters &goodCam,
                              const vpCameraParameters &wrongCam, const vpPoint &p)
{
  try {
    double x = p.get_x();
    double y = p.get_y();

    s.set_Z(p.get_Z());

    double u = 0, v = 0;
    vpMeterPixelConversion::convertPoint(goodCam, x, y, u, v);
    vpPixelMeterConversion::convertPoint(wrongCam, u, v, x, y);

    double rho = sqrt(x * x + y * y);
    double theta = atan2(y, x);

    s.set_rho(rho);
    s.set_theta(theta);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
