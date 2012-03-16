/****************************************************************************
 *
 * $Id: vpFeatureBuilderLine.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Segment creation out of dots.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilderSegment.cpp
  \brief  Segment creation out of dots
*/

#include <visp/vpFeatureBuilder.h>


#include <visp/vpMath.h>



/*!
  Initialize a segment feature out of vpDots, depth coordinates and camera parameters.

  \param seg : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image containing the point.
  \param d1 : The vpDot describing the first point of the segment.
  \param Z1 : The depth coordinate of the first point.

  \param d2 : The vpDot describing the first point of the segment.
  \param Z2 : The depth coordinate of the second point.

*/
void vpFeatureBuilder::create(vpFeatureSegment &seg, const vpCameraParameters &cam, const vpDot &d1, const double Z1, const vpDot &d2, const double Z2 ) {
  double x1=0, y1=0, x2=0, y2=0;


  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1) ;
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2) ;

  seg.buildFrom(x1, y1, Z1, x2, y2, Z2);
}

/*!
  Initialize a segment feature out of vpDots, depth coordinates and camera parameters.

  \param seg : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image containing the point.
  \param d1 : The vpDot2 describing the first point of the segment.
  \param Z1 : The depth coordinate of the first point.

  \param d2 : The vpDot describing the first point of the segment.
  \param Z2 : The depth2 coordinate of the second point.

*/
void vpFeatureBuilder::create(vpFeatureSegment &seg, const vpCameraParameters &cam, const vpDot2 &d1, const double Z1, const vpDot2 &d2, const double Z2) {
  double x1=0, y1=0, x2=0, y2=0;

  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1) ;
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2) ;

  seg.buildFrom(x1, y1, Z1, x2, y2, Z2);
}

/*!
  Initialize a segment feature out of vpDots and camera parameters.

  \param seg : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image containing the point.
  \param d1 : The vpDot describing the first point of the segment.

  \param d2 : The vpDot describing the first point of the segment.

*/
void vpFeatureBuilder::create(vpFeatureSegment &seg, const vpCameraParameters &cam, const vpDot &d1, const vpDot &d2 ) {
  double x1=0, y1=0, x2=0, y2=0;


  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1) ;
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2) ;

  seg.buildFrom(x1, y1, x2, y2);
}

/*!
  Initialize a segment feature out of vpDots and camera parameters.

  \param seg : Visual feature to initialize.
  \param cam : The parameters of the camera used to acquire the image containing the point.
  \param d1 : The vpDot2 describing the first point of the segment.

  \param d2 : The vpDot describing the first point of the segment.

*/
void vpFeatureBuilder::create(vpFeatureSegment &seg, const vpCameraParameters &cam, const vpDot2 &d1, const vpDot2 &d2) {
  double x1=0, y1=0, x2=0, y2=0;

  vpPixelMeterConversion::convertPoint(cam, d1.getCog(), x1, y1) ;
  vpPixelMeterConversion::convertPoint(cam, d2.getCog(), x2, y2) ;

  seg.buildFrom(x1, y1, x2, y2);
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
