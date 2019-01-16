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
 * Conversion between tracker and visual feature ellipse.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderEllipse.cpp
  \brief  conversion between tracker
  and visual feature Ellipse
*/

#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

/*!
  Initialize an ellipse feature thanks to a vpCircle.
  The vpFeatureEllipse is initialized thanks to the parameters of the circle
  in the camera frame and in the image plan. All the parameters are given in
  meter.

  \warning To be sure that the vpFeatureEllipse is well initialized,
  you have to be sure that at least the circle coordinates in the image
  plan and in the camera frame are computed and stored in the vpCircle.

  \param s : Visual feature to initialize.

  \param t : The vpCircle used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCircle &t)
{
  try {

    // 3D data
    double alpha = t.cP[0];
    double beta = t.cP[1];
    double gamma = t.cP[2];

    double X0 = t.cP[3];
    double Y0 = t.cP[4];
    double Z0 = t.cP[5];

    // equation p 318 prior eq (39)
    double d = alpha * X0 + beta * Y0 + gamma * Z0;

    double A = alpha / d;
    double B = beta / d;
    double C = gamma / d;

    s.setABC(A, B, C);

    // 2D data
    s.buildFrom(t.p[0], t.p[1], t.p[2], t.p[3], t.p[4]);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!
  Initialize an ellipse feature thanks to a vpSphere.
  The vpFeatureEllipse is initialized thanks to the parameters
  of the sphere in the camera frame and in the image plan.
  All the parameters are given in meter.

  \warning To be sure that the vpFeatureEllipse is well initialized,
  you have to be sure that at least the sphere coordinates in the image
  plan and in the camera frame are computed and stored in the vpSphere.

  \param s : Visual feature to initialize.

  \param t : The vpSphere used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpSphere &t)
{
  try {

    // 3D data
    double X0 = t.cP[0];
    double Y0 = t.cP[1];
    double Z0 = t.cP[2];
    double R = t.cP[3];

    double d = vpMath::sqr(X0) + vpMath::sqr(Y0) + vpMath::sqr(Z0) - vpMath::sqr(R);

    double A = X0 / d;
    double B = Y0 / d;
    double C = Z0 / d;

    s.setABC(A, B, C);

    // 2D data
    s.buildFrom(t.p[0], t.p[1], t.p[2], t.p[3], t.p[4]);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

#ifdef VISP_HAVE_MODULE_BLOB
/*!
  Initialize an ellipse feature thanks to a vpDot and camera parameters.
  The vpFeatureEllipse is initialized thanks to the parameters
  of the dot given in pixel. The camera parameters are used to convert the
  pixel parameters to parameters given in meter.

  \warning With a vpDot there is no information about 3D parameters.
  Thus the parameters \f$(A,B,C)\f$ can not be set. You have to compute them
  and initialized them outside the method.

  \param s : Visual feature to initialize.

  \param cam : The parameters of the camera used to acquire the image
  containing the vpDot.

  \param t : The vpDot used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot &t)
{
  try {

    unsigned int order = 3;
    vpMatrix mp(order, order);
    mp = 0;
    vpMatrix m(order, order);
    m = 0;

    mp[0][0] = t.m00;
    mp[1][0] = t.m10;
    mp[0][1] = t.m01;
    mp[2][0] = t.m20;
    mp[1][1] = t.m11;
    mp[0][2] = t.m02;

    vpPixelMeterConversion::convertMoment(cam, order, mp, m);

    double m00 = m[0][0];
    double m01 = m[0][1];
    double m10 = m[1][0];
    double m02 = m[0][2];
    double m11 = m[1][1];
    double m20 = m[2][0];

    double xc = m10 / m00; // sum j /S
    double yc = m01 / m00; // sum i /S

    double mu20 = 4 * (m20 - m00 * vpMath::sqr(xc)) / (m00);
    double mu02 = 4 * (m02 - m00 * vpMath::sqr(yc)) / (m00);
    double mu11 = 4 * (m11 - m00 * xc * yc) / (m00);

    s.buildFrom(xc, yc, mu20, mu11, mu02);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!
  Initialize an ellipse feature thanks to a vpDot2 and camera parameters.
  The vpFeatureEllipse is initialized thanks to the parameters
  of the dot given in pixel. The camera parameters are used to convert the
  pixel parameters to parameters given in meter.

  \warning With a vpDot2 there is no information about 3D parameters.
  Thus the parameters \f$(A,B,C)\f$ can not be set. You have to compute them
  and initialized them outside the method.

  \param s : Visual feature to initialize.

  \param cam : The parameters of the camera used to acquire the image
  containing the vpDot2.

  \param t : The vpDot2 used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot2 &t)
{
  try {

    unsigned int order = 3;
    vpMatrix mp(order, order);
    mp = 0;
    vpMatrix m(order, order);
    m = 0;

    mp[0][0] = t.m00;
    mp[1][0] = t.m10;
    mp[0][1] = t.m01;
    mp[2][0] = t.m20;
    mp[1][1] = t.m11;
    mp[0][2] = t.m02;

    vpPixelMeterConversion::convertMoment(cam, order, mp, m);

    double m00 = m[0][0];
    double m01 = m[0][1];
    double m10 = m[1][0];
    double m02 = m[0][2];
    double m11 = m[1][1];
    double m20 = m[2][0];

    double xc = m10 / m00; // sum j /S
    double yc = m01 / m00; // sum i /S

    double mu20 = 4 * (m20 - m00 * vpMath::sqr(xc)) / (m00);
    double mu02 = 4 * (m02 - m00 * vpMath::sqr(yc)) / (m00);
    double mu11 = 4 * (m11 - m00 * xc * yc) / (m00);

    s.buildFrom(xc, yc, mu20, mu11, mu02);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
#endif //#ifdef VISP_HAVE_MODULE_BLOB

#ifdef VISP_HAVE_MODULE_ME
/*!
  Initialize an ellipse feature thanks to a vpMeEllipse and camera parameters.
  The vpFeatureEllipse is initialized thanks to the parameters
  of the ellipse given in pixel. The camera parameters are used to convert the
  pixel parameters to parameters given in meter.

  \warning With a vpMeEllipse there is no information about 3D parameters.
  Thus the parameters \f$(A,B,C)\f$ can not be set. You have to compute them
  and initialized them outside the method.

  \param s : Visual feature to initialize.

  \param cam : The parameters of the camera used to acquire the image
  containing the vpDot2.

  \param t : The vpMeEllipse used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpMeEllipse &t)
{
  try {

    unsigned int order = 3;
    vpMatrix mp(order, order);
    mp = 0;
    vpMatrix m(order, order);
    m = 0;

    // The opposite of vpDot and vpDot2 because moments in vpMeEllipse
    // are computed in the ij coordinate system whereas the moments in vpDot
    // and vpDot2  are computed in the uv coordinate system
    mp[0][0] = t.get_m00();
    mp[1][0] = t.get_m01();
    mp[0][1] = t.get_m10();
    mp[2][0] = t.get_m02();
    mp[1][1] = t.get_m11();
    mp[0][2] = t.get_m20();

    vpPixelMeterConversion::convertMoment(cam, order, mp, m);

    double m00 = m[0][0];
    double m01 = m[0][1];
    double m10 = m[1][0];
    double m02 = m[0][2];
    double m11 = m[1][1];
    double m20 = m[2][0];

    double xc = m10 / m00; // sum j /S
    double yc = m01 / m00; // sum i /S

    double mu20 = 4 * (m20 - m00 * vpMath::sqr(xc)) / (m00);
    double mu02 = 4 * (m02 - m00 * vpMath::sqr(yc)) / (m00);
    double mu11 = 4 * (m11 - m00 * xc * yc) / (m00);

    s.buildFrom(xc, yc, mu20, mu11, mu02);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
#endif //#ifdef VISP_HAVE_MODULE_ME
