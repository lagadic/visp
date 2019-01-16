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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpProjectionDisplay.cpp
  \brief interface with the image for feature display
*/

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_DISPLAY)

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPoint.h>

// Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpDisplay.h>

#include <visp3/gui/vpProjectionDisplay.h>

//#include <visp3/visual_features/vpBasicFeature.h>

void vpProjectionDisplay::insert(vpForwardProjection &fp)
{
  // vpForwardProjection *f ;
  //  f = fp.duplicate() ;
  //  f->setDeallocate(vpForwardProjection::vpDisplayForwardProjection) ;

  listFp.push_back(&fp);
}

void vpProjectionDisplay::init()
{
  o.setWorldCoordinates(0, 0, 0);
  x.setWorldCoordinates(0.1, 0, 0);
  y.setWorldCoordinates(0, 0.1, 0);
  z.setWorldCoordinates(0, 0, 0.1);
  traj.resize(0, 2);
}
void vpProjectionDisplay::init(const int select)
{
  if (select & vpProjectionDisplay::internalView()) {
    Icam.resize(256, 256);
    dIcam.init(Icam, 100, 100);
  }
  if (select & vpProjectionDisplay::externalView()) {
    Iext.resize(256, 256);
    dIext.init(Iext, 400, 100);
  }

  init();
}

void vpProjectionDisplay::close() {}

void vpProjectionDisplay::display(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cextMo,
                                  const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor &color,
                                  const bool &displayTraj, const unsigned int thickness)
{

  for (std::list<vpForwardProjection *>::const_iterator it = listFp.begin(); it != listFp.end(); ++it) {
    vpForwardProjection *fp = *it;
    fp->display(I, cextMo, cam, color, thickness);
  }

  if (displayTraj) // display past camera positions
    for (unsigned int i = 0; i < traj.getRows(); ++i)
      vpDisplay::displayCircle(I, (int)traj[i][0], (int)traj[i][1], 2, vpColor::green, true);

  displayCamera(I, cextMo, cMo, cam, thickness);

  if (displayTraj) // store current camera position
  {
    const unsigned int n = traj.getRows();
    traj.resize(n + 1, 2, false);
    vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], traj[n][1], traj[n][0]);
  }
}

void vpProjectionDisplay::displayCamera(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cextMo,
                                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                                        const unsigned int thickness)
{
  vpHomogeneousMatrix c1Mc;
  c1Mc = cextMo * cMo.inverse();

  o.track(c1Mc);

  if (o.get_Z() < 0) // do not print camera if behind the external camera
    return;

  x.track(c1Mc);
  y.track(c1Mc);
  z.track(c1Mc);

  vpImagePoint ipo;
  vpImagePoint ipx;

  vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo);

  vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ipx);
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::green, 4 + thickness, 2 + thickness, thickness);

  vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ipx);
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::blue, 4 + thickness, 2 + thickness, thickness);

  vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ipx);
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::red, 4 + thickness, 2 + thickness, thickness);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpProjectionDisplay.cpp.o)
// has no symbols
void dummy_vpProjectionDisplay(){};
#endif
