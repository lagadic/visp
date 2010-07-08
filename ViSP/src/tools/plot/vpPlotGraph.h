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

