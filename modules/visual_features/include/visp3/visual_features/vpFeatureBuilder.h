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
 * Conversion between tracker and visual feature.
 *
*****************************************************************************/

/*!
  \file vpFeatureBuilder.h
  \brief  class  that defines conversion between tracker and visual feature
*/

#ifndef vpFeatureBuilder_H
#define vpFeatureBuilder_H


#include <visp3/core/vpConfig.h>

// tracker
#ifdef VISP_HAVE_MODULE_BLOB
#include <visp3/blob/vpDot.h>
#include <visp3/blob/vpDot2.h>
#endif

#ifdef VISP_HAVE_MODULE_ME
#include <visp3/me/vpMeEllipse.h>
#include <visp3/me/vpMeLine.h>
#endif

// forward projection tracker
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpSphere.h>

// visual feature
#include <visp3/visual_features/vpFeatureEllipse.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>
#include <visp3/visual_features/vpFeatureSegment.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>

// others
#include <visp3/core/vpImagePoint.h>

// pixel / meter conversion
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpFeatureBuilder

  \ingroup group_visual_features_builder
  \brief Class that defines conversion between trackers and visual features.
*/
class VISP_EXPORT vpFeatureBuilder
{
public:
// create vpFeaturePoint feature
#ifdef VISP_HAVE_MODULE_BLOB
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot &d);
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpDot2 &d);
#endif
  static void create(vpFeaturePoint &s, const vpCameraParameters &cam, const vpImagePoint &t);
  static void create(vpFeaturePoint &s, const vpPoint &p);
  static void create(vpFeaturePoint &s, const vpCameraParameters &goodCam, const vpCameraParameters &wrongCam,
                     const vpPoint &p);

#ifdef VISP_HAVE_MODULE_BLOB
  static void create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot &d1, const vpDot &d2);
  static void create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot2 &d1, const vpDot2 &d2);
#endif
  static void create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpImagePoint &ip1,
                     const vpImagePoint &ip2);
  static void create(vpFeatureSegment &s, vpPoint &P1, vpPoint &P2);

// create vpFeaturePointPolar feature
#ifdef VISP_HAVE_MODULE_BLOB
  static void create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpDot &dot);
  static void create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpDot2 &dot);
#endif
  static void create(vpFeaturePointPolar &s, const vpCameraParameters &cam, const vpImagePoint &iP);
  static void create(vpFeaturePointPolar &s, const vpPoint &p);
  static void create(vpFeaturePointPolar &s, const vpCameraParameters &goodCam, const vpCameraParameters &wrongCam,
                     const vpPoint &p);

  // create vpFeaturePoint3D feature
  static void create(vpFeaturePoint3D &s, const vpPoint &p);

  // create vpFeatureLine feature
  static void create(vpFeatureLine &s, const vpLine &l);
  static void create(vpFeatureLine &s, const vpCylinder &c, int line);

#ifdef VISP_HAVE_MODULE_ME
  static void create(vpFeatureLine &s, const vpCameraParameters &cam, const vpMeLine &mel);
#endif

  //! create vpFeatureEllipse feature
  static void create(vpFeatureEllipse &s, const vpCircle &c);
  static void create(vpFeatureEllipse &s, const vpSphere &sphere);
#ifdef VISP_HAVE_MODULE_BLOB
  static void create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot &blob);
  static void create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpDot2 &blob);
#endif

#ifdef VISP_HAVE_MODULE_ME
  static void create(vpFeatureEllipse &s, const vpCameraParameters &cam, const vpMeEllipse &ellipse);
#endif

  // To keep compat with previous releases, we set cartesian coordinates as default initialising select var to selectX()
  // or selectY()
  static void create(vpFeatureVanishingPoint &s, const vpPoint &p,
                     unsigned int select = (vpFeatureVanishingPoint::selectX() | vpFeatureVanishingPoint::selectY()));
  static void create(vpFeatureVanishingPoint &s, const vpFeatureLine &l1, const vpFeatureLine &l2,
                     unsigned int select = (vpFeatureVanishingPoint::selectX() | vpFeatureVanishingPoint::selectY()));
  static void create(vpFeatureVanishingPoint &s, const vpLine &l1, const vpLine &l2,
                     unsigned int select = (vpFeatureVanishingPoint::selectX() | vpFeatureVanishingPoint::selectY()));
  // This function is new that's why select is not initialized
  static void create(vpFeatureVanishingPoint &s, const vpCameraParameters &cam, const vpImagePoint &line1_ip1,
                     const vpImagePoint &line1_ip2, const vpImagePoint &line2_ip1, const vpImagePoint &line2_ip2,
                     unsigned int select);
};
END_VISP_NAMESPACE
#endif
