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
 * Define a curve for the vpPlot class.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlotCurve.h>

#if defined(VISP_HAVE_DISPLAY)
vpPlotCurve::vpPlotCurve()
  : color(vpColor::red), curveStyle(point), thickness(1), nbPoint(0), lastPoint(), pointListx(), pointListy(),
    pointListz(), legend(), xmin(0), xmax(0), ymin(0), ymax(0)
{
}

vpPlotCurve::~vpPlotCurve()
{
  pointListx.clear();
  pointListy.clear();
  pointListz.clear();
}

void vpPlotCurve::plotPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const double x, const double y)
{
  nbPoint++;

  if (nbPoint > 1) {
    vpDisplay::displayLine(I, lastPoint, iP, color, thickness);
  }
#if defined(VISP_HAVE_DISPLAY)
  double top;
  double left;
  double width;
  double height;

  if (iP.get_i() <= lastPoint.get_i()) {
    top = iP.get_i() - 5;
    height = lastPoint.get_i() - top + 10;
  } else {
    top = lastPoint.get_i() - 5;
    height = iP.get_i() - top + 10;
  }
  if (iP.get_j() <= lastPoint.get_j()) {
    left = iP.get_j() - 5;
    width = lastPoint.get_j() - left + 10;
  } else {
    left = lastPoint.get_j() - 5;
    width = iP.get_j() - left + 10;
  }
  vpDisplay::flushROI(I, vpRect(left, top, width, height));
#endif
  lastPoint = iP;
  pointListx.push_back(x);
  pointListy.push_back(y);
  pointListz.push_back(0.0);
}

void vpPlotCurve::plotList(const vpImage<unsigned char> &I, const double xorg, const double yorg, const double zoomx,
                           const double zoomy)
{
  std::list<double>::const_iterator it_ptListx = pointListx.begin();
  std::list<double>::const_iterator it_ptListy = pointListy.begin();

  unsigned int k = 0;
  vpImagePoint iP;
  while (k < nbPoint) {
    iP.set_ij(yorg - (zoomy * (*it_ptListy)), xorg + (zoomx * (*it_ptListx)));

    if (k > 0)
      vpDisplay::displayLine(I, lastPoint, iP, color, thickness);

    lastPoint = iP;

    ++it_ptListx;
    ++it_ptListy;
    k++;
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpPlotCurve.cpp.o) has no
// symbols
void dummy_vpPlotCurve(){};
#endif
#endif
