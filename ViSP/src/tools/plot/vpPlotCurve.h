/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Define a curve for the vpPlot class.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpPlotCurve_H
#define vpPlotCurve_H

#include <visp/vpConfig.h>
#include <visp/vpColor.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRect.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>


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
    //vpMarkerStyle markerStyle;
    //char lineStyle[20];
    //vpList<vpImagePoint> pointList;
    int nbPoint;
    vpImagePoint lastPoint;
    vpList<double> pointListx;
    vpList<double> pointListy;
    vpList<double> pointListz;
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