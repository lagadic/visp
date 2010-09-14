/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Define a graph for the vpPlot class.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpPlotGraph_H
#define vpPlotGraph_H

#include <visp/vpConfig.h>

#include <visp/vpPlotCurve.h>

#include <visp/vpColor.h>
#include <visp/vpList.h>
#include <visp/vpImage.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRect.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) 

class vpPlotGraph
{
  public:
    double xorg;
    double yorg;
    double zoomx;
    double zoomy;
    double xmax;
    double ymax;
    double xmin;
    double ymin;
    double xdelt;
    double ydelt;
    bool gridx;
    bool gridy;
    vpColor gridColor;
    char title[256];
    char unitx[256];
    char unity[256];
    int curveNbr;
    vpPlotCurve* curveList;
    bool textdispayed;
    bool scaleInitialized;
    bool firstPoint;
    
    int nbDivisionx;
    int nbDivisiony;
    
    //Graph complet
    vpImagePoint topLeft;
    int width;
    int height;
    vpRect graphZone;
    
    //Zone d'affichage
    vpImagePoint dTopLeft;
    int dWidth;
    int dHeight;
    vpRect dGraphZone;
    
    //Zone d'affichage
    vpImagePoint dTopLeft3D;
//     int dWidth;
//     int dHeight;
    vpRect dGraphZone3D;
    
    //3D part
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix cMf;
    double w_xval;
    double w_xsize;
    double w_yval;
    double w_ysize;
    double w_zval;
    double w_zsize;
    double ptXorg;
    double ptYorg;
    double ptZorg;
    double zoomx_3D;
    double zoomy_3D;
    double zoomz_3D;
    
    int nbDivisionz;
    
    double zorg;
    double zoomz;
    double zmax;
    double zmin;
    double zdelt;
    char unitz[256];
    vpImagePoint old_iPr;
    vpImagePoint old_iPz;
    bool blockedr;
    bool blockedz;
    bool blocked;
    
    int epsi;
    int epsj;
    
    bool dispUnit;
    bool dispTitle;
    bool dispLegend;
    
    
  public:
    vpPlotGraph();
    ~vpPlotGraph();
    
    void clearGraphZone(vpImage<unsigned char> &I);
    void initGraph (int nbCurve);
    void initSize (vpImagePoint topLeft, int width, int height, int margei, int margej);
    void computeGraphParameters();
    void computeGraphParameters3D();
    void displayGrid (vpImage<unsigned char> &I);
    void displayUnit (vpImage<unsigned char> &I);
    void displayTitle (vpImage<unsigned char> &I);
    void displayLegend (vpImage<unsigned char> &I);
    void displayGrid3D (vpImage<unsigned char> &I);
    
    void setCurveColor(const int curveNum, const vpColor color);
    void setTitle (const char *title);
    void setUnitX (const char *unitx);
    void setUnitY (const char *unity);
    void setUnitZ (const char *unitz);
    void setLegend (const int curveNum, const char *legend);
    
    void rescalex(int side, double extremity);
    void rescaley(int side, double extremity);
    void rescalez(int side, double extremity);
    //void rescale(double &min, double &max, double &delta, const int nbDiv, int side);
    
    void initScale(vpImage<unsigned char> &I, const double xmin, const double xmax, const int nbDivx, const double ymin, const double ymax, const int nbDivy, const bool gx, const bool gy);
    
    void initScale(vpImage<unsigned char> &I, const double xmin, const double xmax, const int nbDivx, const double ymin, const double ymax, const int nbDivy, const double zmin, const double zmax, const int nbDivz, const bool gx, const bool gy);
    
    void plot (vpImage<unsigned char> &I, const int curveNb, const double x, const double y);
    void plot (vpImage<unsigned char> &I, const int curveNb, const double x, const double y, const double z);
    void replot (vpImage<unsigned char> &I);
    void replot3D (vpImage<unsigned char> &I);
    
    bool getPixelValue(vpImage<unsigned char> &I, vpImagePoint &iP);
    
    vpHomogeneousMatrix navigation(vpImage<unsigned char> &I, bool &changed);
    
    void findPose();
    bool move(vpImage<unsigned char> &I);
    bool check3Dline(vpImagePoint &iP1, vpImagePoint &iP2);
    bool check3Dpoint(vpImagePoint &iP);
    
    void resetPointList(const int curveNum);
    
};

#endif
#endif
#endif

