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
 * Conversion between tracker and visual feature point.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderPoint.cpp
  \brief  conversion between tracker
  and visual feature Point
*/
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureException.h>

#ifdef VISP_HAVE_MODULE_BLOB
/*!
  Create a vpFeaturePoint thanks to a vpDot and the parameters of the camera.
  The vpDot contains only the pixel coordinates of the point in an image.
  Thus this method uses the camera parameters to compute the meter coordinates
  \f$ x \f$ and \f$ y \f$ in the image plan. Those coordinates are stored in
  the vpFeaturePoint.

  \warning It is not possible to compute the depth of the point \f$ Z \f$ in
  the camera frame thanks to a vpDot. This coordinate is needed in
  vpFeaturePoint to compute the interaction matrix. So this value must be
  computed outside this function.

  \param s : Visual feature \f$(x, y)\f$ to initialize. Be
  aware, the 3D depth \f$Z\f$ requested to compute the interaction
  matrix is not initialized by this function.
  \param cam : The parameters of the camera used to acquire the image
  containing the vpDot. \param d : The vpDot used to create the
  vpFeaturePoint.

  The code below shows how to initialize a vpFeaturePoint visual
  feature. First, we initialize the \f$x,y\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpDot dot;               // Dot tracker

  vpFeaturePoint s;    // Point feature
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
void vpFeatureBuilder::create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot &d)
{
  try {
    double x = 0, y = 0;

    vpImagePoint cog;
    cog = d.getCog();

    vpPixelMeterConversion::convertPoint(cam, cog, x, y);

    s.set_x(x);
    s.set_y(y);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!
  Create a vpFeaturePoint thanks to a vpDot2 and the parameters of the camera.
  The vpDot2 contains only the pixel coordinates of the point in an image.
  Thus this method uses the camera parameters to compute the meter coordinates
  \f$ x \f$ and \f$ y \f$ in the image plan. Those coordinates are stored in
  the vpFeaturePoint.

  \warning It is not possible to compute the depth of the point \f$ Z \f$ in
  the camera frame thanks to a vpDot2. This coordinate is needed in
  vpFeaturePoint to compute the interaction matrix. So this value must be
  computed outside this function.

  \param s : The feature point.
  \param cam : The parameters of the camera used to acquire the image
  containing the vpDot2. \param d : The vpDot2 used to create the
  vpFeaturePoint.

  The code below shows how to initialize a vpFeaturePoint visual
  feature. First, we initialize the \f$x,y\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpDot2 dot;               // Dot tracker

  vpFeaturePoint s;    // Point feature
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
void vpFeatureBuilder::create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot2 &d)
{
  try {
    double x = 0, y = 0;

    vpImagePoint cog;
    cog = d.getCog();

    vpPixelMeterConversion::convertPoint(cam, cog, x, y);

    s.set_x(x);
    s.set_y(y);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
#endif //#ifdef VISP_HAVE_MODULE_BLOB

/*!
  Create a vpFeaturePoint thanks to a vpImagePoint and the parameters of the
  camera. The vpImagePoint contains only the pixel coordinates of the point in
  an image. Thus this method uses the camera parameters to compute the meter
  coordinates \f$ x \f$ and \f$ y \f$ in the image plan. Those coordinates are
  stored in the vpFeaturePoint.

  \warning It is not possible to compute the depth of the point \f$ Z \f$ in
  the camera frame thanks to a vpImagePoint. This coordinate is needed in
  vpFeaturePoint to compute the interaction matrix. So this value must be
  computed outside this function.

  \param s : The feature point.
  \param cam : The parameters of the camera used to acquire the image
  containing the point. \param ip : The vpImagePoint used to create the
  vpFeaturePoint.

  The code below shows how to initialize a vpFeaturePoint visual
  feature. First, we initialize the \f$x,y\f$, and lastly we
  set the 3D depth \f$Z\f$ of the point which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpImagePoint iP;               // the point in the image

  vpFeaturePoint s;    // Point feature
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
void vpFeatureBuilder::create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpImagePoint &ip)
{
  try {
    double x = 0, y = 0;

    vpPixelMeterConversion::convertPoint(cam, ip, x, y);

    s.set_x(x);
    s.set_y(y);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!
  Create a vpFeaturePoint thanks to a vpPoint.
  This method uses the point coordinates \f$ x \f$ and \f$ y \f$ in the image
  plan to set the visual feature parameters. The value of the depth \f$ Z \f$
  in the camera frame is also computed thanks to the coordinates in the camera
  frame which are stored in vpPoint.

  \warning To be sure that the vpFeaturePoint is well initialized, you have to
  be sure that at least the point coordinates in the image plan and in the
  camera frame are computed and stored in the vpPoint.

  \param s : The feature point.
  \param p : The vpPoint used to create the vpFeaturePoint.
*/
void vpFeatureBuilder::create(vpFeaturePoint &s, const vpPoint &p)
{
  try {
    s.set_x(p.get_x());
    s.set_y(p.get_y());

    s.set_Z(p.cP[2] / p.cP[3]);

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
  Create a vpFeaturePoint thanks to a vpPoint. In this method noise is
  introduced during the initialization of the vpFeaturePoint. This method uses
  the point coordinates \f$ x \f$ and \f$ y \f$ in the image plan to set the
  visual feature parameters. The value of the depth \f$ Z \f$ in the camera
  frame is also computed thanks to the coordinates in the camera frame which
  are stored in vpPoint.

  This function intends to introduce noise after the initialization of the
  parameters. Cartesian \f$(x,y)\f$ coordinates are first converted in pixel
  coordinates in the image using \e goodCam camera parameters. Then, the
  pixels coordinates of the point are converted back to cartesian coordinates
  \f$(x^{'},y^{'})\f$ using the noisy camera parameters \e wrongCam. These
  last parameters are stored in the vpFeaturePoint.

  \warning To be sure that the vpFeaturePoint is well initialized, you have to
  be sure that at least the point coordinates in the image plan and in the
  camera frame are computed and stored in the vpPoint.

  \param s : The feature point.

  \param goodCam : Camera parameters used to introduce noise. These
  parameters are used to convert cartesian coordinates of the point \e
  p in the image plane in pixel coordinates.

  \param wrongCam : Camera parameters used to introduce noise. These
  parameters are used to convert pixel coordinates of the point in
  cartesian coordinates of the point in the image plane.

  \param p : The vpPoint used to create the vpFeaturePoint.
*/
void vpFeatureBuilder::create(vpFeaturePoint &s, const vpCameraParameters &goodCam, const vpCameraParameters &wrongCam,
                              const vpPoint &p)
{
  try {
    double x = p.p[0];
    double y = p.p[1];

    s.set_Z(p.cP[2] / p.cP[3]);

    double u = 0, v = 0;
    vpMeterPixelConversion::convertPoint(goodCam, x, y, u, v);
    vpPixelMeterConversion::convertPoint(wrongCam, u, v, x, y);

    s.set_x(x);
    s.set_y(y);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
