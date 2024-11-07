/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Define a curve for the vpPlot class.
 */

#ifndef VP_PLOT_CURVE_H
#define VP_PLOT_CURVE_H

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRect.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPoint.h>

#include <list>

#if defined(VISP_HAVE_DISPLAY)

BEGIN_VISP_NAMESPACE

class vpPlotCurve
{
public:
  //! Different styles to plot the curve.
  typedef enum { point, line, dashed_line, marker } vpCurveStyle;
  vpColor color;
  vpCurveStyle curveStyle;
  unsigned int thickness;
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
  virtual ~vpPlotCurve();
  void plotPoint(const vpImage<unsigned char> &I, const vpImagePoint &iP, double x, double y);
  void plotList(const vpImage<unsigned char> &I, double xorg, double yorg, double zoomx, double zoomy);
};


END_VISP_NAMESPACE
#endif
#endif
#endif
