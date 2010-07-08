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
 * Plot curves.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/


#ifndef vpPlot_H
#define vpPlot_H

#include <visp/vpConfig.h>
#include <visp/vpPlotGraph.h>
#include <visp/vpColor.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRect.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>

#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>


class vpPlot
{
  public:
    vpImage<unsigned char> I;
    
    #if defined VISP_HAVE_X11
    vpDisplayX display;
    #elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
    #elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
    #endif
    
    int graphNbr;
    vpPlotGraph* graphList;
    
    int margei;
    int margej;
    
  public:
    vpPlot(const int nbGraph);
    ~vpPlot();
    
    void init (int nbGraph);
    void initGraph (int graphNum, int curveNbr);
    void displayGrid();
    void initRange (const int graphNum, double xmin, double xmax, double xdelt, double ymin, double ymax, double ydelt, const bool gx=0, const bool gy=0);
    void setColor (const int graphNum, const int curveNum, vpColor color);
    void setTitle (const int graphNum, const char *title);
    void setUnitX (const int graphNum, const char *unitx);
    void setUnitY (const int graphNum, const char *unity);
    void setUnitZ (const int graphNum, const char *unitz);
    void setLegend (const int graphNum, const int curveNum, const char *legend);
    
    void plot (const int graphNum, const int curveNum, const double x, const double y);
    void plot (const int graphNum, const int curveNum, const double x, const double y, const double z);
    
    void navigate ();
    
    void getPixelValue(const bool block);
    
    void resetPointList (const int graphNum);
    void resetPointList (const int graphNum, const int curveNum);
};

#endif
