/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpPlotCurve_H
#define vpPlotCurve_H

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRect.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPoint.h>

#include <list>

#if defined(VISP_HAVE_DISPLAY)

class vpPlotCurve
{
public:
  //! Different styles to plot the curve.
  typedef enum { point, line, dashed_line, marker } vpCurveStyle;
  vpColor color;
  vpCurveStyle curveStyle;
  unsigned int thickness;
  // vpMarkerStyle markerStyle;
  // char lineStyle[20];
  // vpList<vpImagePoint> pointList;
  unsigned int nbPoint;
  vpImagePoint lastPoint;
  std::list<double> pointListx;
  std::list<double> pointListy;
  std::list<double> pointListz;
  std::string legend;
  double xmin;
  double xmax;
  double ymin;
  double ymax;

public:
  vpPlotCurve();
  ~vpPlotCurve();
  void plotPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, const double x, const double y);
  void plotList(const vpImage<unsigned char> &I, const double xorg, const double yorg, const double zoomx,
                const double zoomy);
};

#endif
#endif
#endif
