/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
*****************************************************************************/

/*!
  \file vpFeatureBuilderEllipse.cpp
  \brief  conversion between tracker
  and visual feature Ellipse
*/

#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

BEGIN_VISP_NAMESPACE
/*!
  Initialize an ellipse feature thanks to a vpCircle.
  The vpFeatureEllipse is initialized thanks to the parameters of the circle
  in the camera frame and in the image plane. All the parameters are given in
  meter.

  \warning To be sure that the vpFeatureEllipse is well initialized,
  you have to be sure that at least the circle coordinates in the image
  plane and in the camera frame are computed and stored in the vpCircle.

  \param s : Visual feature to initialize.

  \param t : The vpCircle used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCircle &t)
{
  // 3D data
  double alpha = t.cP[0]; // A
  double beta = t.cP[1];  // B
  double gamma = t.cP[2]; // C

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
  s.build(t.p[0], t.p[1], t.p[2], t.p[3], t.p[4]);
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
  s.build(t.p[0], t.p[1], t.p[2], t.p[3], t.p[4]);
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

  \param blob : The blob used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot &blob)
{
  double xc = 0, yc = 0;
  vpPixelMeterConversion::convertPoint(cam, blob.getCog(), xc, yc);
  vpColVector nij = blob.get_nij();

  s.build(xc, yc, nij[0], nij[1], nij[2]);
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

  \param blob : The blob used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot2 &blob)
{
  double xc = 0, yc = 0;
  vpPixelMeterConversion::convertPoint(cam, blob.getCog(), xc, yc);
  vpColVector nij = blob.get_nij();

  s.build(xc, yc, nij[0], nij[1], nij[2]);
}
#endif //#ifdef VISP_HAVE_MODULE_BLOB

#ifdef VISP_HAVE_MODULE_ME
/*!
  Initialize an ellipse feature thanks to a vpMeEllipse and camera parameters.
  The vpFeatureEllipse is initialized thanks to the parameters
  of the ellipse given in pixel. The camera parameters are used to convert the
  pixel parameters to parameters given in meters in the image plane.

  \warning With a vpMeEllipse there is no information about 3D parameters.
  Thus the parameters \f$(A,B,C)\f$ can not be set. You have to compute them
  and initialize them outside the method.

  \param[out] s : Visual feature to initialize.

  \param[in] cam : The parameters of the camera used to acquire the image
  containing the vpMeEllipse

  \param[in] ellipse : The tracked vpMeEllipse used to create the vpFeatureEllipse.
*/
void vpFeatureBuilder::create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpMeEllipse &ellipse)
{
  double xg, yg, n20, n11, n02;
  vpPixelMeterConversion::convertEllipse(cam, ellipse.getCenter(), ellipse.get_nij()[0], ellipse.get_nij()[1],
                                         ellipse.get_nij()[2], xg, yg, n20, n11, n02);

  s.build(xg, yg, n20, n11, n02);
}

#endif //#ifdef VISP_HAVE_MODULE_ME
END_VISP_NAMESPACE
