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
 * Segment creation out of dots.
 *
 * Authors:
 * Filip Novotny
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderSegment.cpp
  \brief  Segment creation out of dots.
*/

#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

#ifdef VISP_HAVE_MODULE_BLOB

/*!
  Initialize a segment feature out of vpDots and camera parameters.

  \param s : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image
  containing the point. \param d1 : The dot corresponding to the first point
  of the segment. \param d2 : The dot corresponding to the second point of the
  segment.

*/
void vpFeatureBuilder::create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot &d1, const vpDot &d2)
{
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1);
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2);

  double xc = (x1 + x2) / 2.;
  double yc = (y1 + y2) / 2.;
  double l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  if (s.isNormalized()) {
    s.setXc(xc / l);
    s.setYc(yc / l);
    s.setL(1 / l);
  } else {
    s.setXc(xc);
    s.setYc(yc);
    s.setL(l);
  }

  s.setAlpha(atan2(y1 - y2, x1 - x2));
}

/*!
  Initialize a segment feature out of vpDots and camera parameters.

  \param s : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image
  containing the point. \param d1 : The dot corresponding to the first point
  of the segment. \param d2 : The dot corresponding to the second point of the
  segment.

*/
void vpFeatureBuilder::create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpDot2 &d1, const vpDot2 &d2)
{
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1);
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2);

  double xc = (x1 + x2) / 2.;
  double yc = (y1 + y2) / 2.;
  double l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  if (s.isNormalized()) {
    s.setXc(xc / l);
    s.setYc(yc / l);
    s.setL(1 / l);
  } else {
    s.setXc(xc);
    s.setYc(yc);
    s.setL(l);
  }

  s.setAlpha(atan2(y1 - y2, x1 - x2));
}
#endif //#ifdef VISP_HAVE_MODULE_BLOB

/*!
  Initialize a segment feature out of image points and camera parameters.

  \param s : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image
  containing the point. \param ip1 : The image point corresponding to the
  first point of the segment. \param ip2 : The image point corresponding to
  the second point of the segment.

*/
void vpFeatureBuilder::create(vpFeatureSegment &s, const vpCameraParameters &cam, const vpImagePoint &ip1,
                              const vpImagePoint &ip2)
{
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

  vpPixelMeterConversion::convertPoint(cam, ip1, x1, y1);
  vpPixelMeterConversion::convertPoint(cam, ip2, x2, y2);

  double xc = (x1 + x2) / 2.;
  double yc = (y1 + y2) / 2.;
  double l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  if (s.isNormalized()) {
    s.setXc(xc / l);
    s.setYc(yc / l);
    s.setL(1 / l);
  } else {
    s.setXc(xc);
    s.setYc(yc);
    s.setL(l);
  }

  s.setAlpha(atan2(y1 - y2, x1 - x2));
}

/*!

  Build a segment visual feature from two points.

  \param s : Visual feature to initialize.
  \param P1, P2 : Two points defining the segment. These points must contain
  the 3D coordinates in the camera frame (cP) and the projected coordinates in
  the image plane (p).

*/
void vpFeatureBuilder::create(vpFeatureSegment &s, vpPoint &P1, vpPoint &P2)
{
  double x1 = P1.get_x();
  double y1 = P1.get_y();
  double x2 = P2.get_x();
  double y2 = P2.get_y();

  double Z1 = P1.cP[2] / P1.cP[3];
  double Z2 = P2.cP[2] / P2.cP[3];

  s.buildFrom(x1, y1, Z1, x2, y2, Z2);
}
