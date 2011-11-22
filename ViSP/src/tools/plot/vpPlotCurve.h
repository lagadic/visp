/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Define a curve for the vpPlot class.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpPlotCurve_H
#define vpPlotCurve_H

#include <visp/vpColor.h>
#include <visp/vpImage.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRect.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>

#include <list>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) 

    //! Different styles to plot the curve.
typedef enum 
{
  point,
  line,
  dashed_line,
  marker
} vpCurveStyle;

class vpPlotCurve
{
  public:
    vpColor color;
    vpCurveStyle curveStyle; 
    unsigned int thickness ;
    //vpMarkerStyle markerStyle;
    //char lineStyle[20];
    //vpList<vpImagePoint> pointList;
    unsigned int nbPoint;
    vpImagePoint lastPoint;
    std::list<double> pointListx;
    std::list<double> pointListy;
    std::list<double> pointListz;
    char legend[256];
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    
  public:
    vpPlotCurve();
    ~vpPlotCurve();
    void plotPoint(vpImage<unsigned char> &I, vpImagePoint iP, const double x, const double y);
    void plotPoint(vpImage<unsigned char> &I, vpImagePoint iP, const double x, const double y, const double z);
    void plotList(vpImage<unsigned char> &I, const double xorg, const double yorg, const double zoomx, const double zoomy);
};

#endif
#endif
#endif
