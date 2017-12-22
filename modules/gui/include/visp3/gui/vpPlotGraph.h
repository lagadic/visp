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
 * Define a graph for the vpPlot class.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpPlotGraph_H
#define vpPlotGraph_H

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpMouseButton.h>
#include <visp3/gui/vpPlotCurve.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRect.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPoint.h>

#if defined(VISP_HAVE_DISPLAY)

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
  std::string title;
  std::string unitx;
  std::string unity;
  unsigned int curveNbr;
  vpPlotCurve *curveList;
  bool scaleInitialized;
  bool firstPoint;

  int nbDivisionx;
  int nbDivisiony;

  // Graph complet
  vpImagePoint topLeft;
  unsigned int width;
  unsigned int height;
  vpRect graphZone;

  // Zone d'affichage
  vpImagePoint dTopLeft;
  unsigned int dWidth;
  unsigned int dHeight;
  vpRect dGraphZone;

  // Zone d'affichage
  vpImagePoint dTopLeft3D;
  //     int dWidth;
  //     int dHeight;
  vpRect dGraphZone3D;

  // 3D part
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
  std::string unitz;
  vpImagePoint old_iPr;
  vpImagePoint old_iPz;
  bool blockedr;
  bool blockedz;
  bool blocked;

  unsigned int epsi;
  unsigned int epsj;

  bool dispUnit;
  bool dispTitle;
  bool dispLegend;

  unsigned int gridThickness;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpPlotGraph(const vpPlotGraph &)
  //      : xorg(0.), yorg(0.), zoomx(1.), zoomy(1.), xmax(10), ymax(10),
  //      xmin(0), ymin(-10),
  //        xdelt(1), ydelt(1), gridx(true), gridy(true), gridColor(),
  //        curveNbr(1), curveList(NULL), scaleInitialized(false),
  //        firstPoint(true), nbDivisionx(10), nbDivisiony(10), topLeft(),
  //        width(0), height(0), graphZone(), dTopLeft(), dWidth(0),
  //        dHeight(0), dGraphZone(), dTopLeft3D(), dGraphZone3D(), cam(),
  //        cMo(), cMf(), w_xval(0), w_xsize(0), w_yval(0), w_ysize(0),
  //        w_zval(0), w_zsize(0), ptXorg(0), ptYorg(0), ptZorg(0),
  //        zoomx_3D(1.), zoomy_3D(1.), zoomz_3D(1.), nbDivisionz(10),
  //        zorg(1.), zoomz(1.), zmax(10), zmin(-10), zdelt(1), old_iPr(),
  //        old_iPz(), blockedr(false), blockedz(false), blocked(false),
  //        epsi(5), epsj(6), dispUnit(false), dispTitle(false),
  //        dispLegend(false), gridThickness(1)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpPlotGraph &operator=(const vpPlotGraph &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpPlotGraph();
  ~vpPlotGraph();

  bool check3Dline(vpImagePoint &iP1, vpImagePoint &iP2);
  bool check3Dpoint(vpImagePoint &iP);
  void clearGraphZone(vpImage<unsigned char> &I);
  void computeGraphParameters();
  void computeGraphParameters3D();

  void initGraph(unsigned int nbCurve);
  void initSize(vpImagePoint topLeft, unsigned int width, unsigned int height, unsigned int margei,
                unsigned int margej);
  void initScale(vpImage<unsigned char> &I, const double xmin, const double xmax, const int nbDivx, const double ymin,
                 const double ymax, const int nbDivy, const bool gx, const bool gy);
  void initScale(vpImage<unsigned char> &I, const double xmin, const double xmax, const int nbDivx, const double ymin,
                 const double ymax, const int nbDivy, const double zmin, const double zmax, const int nbDivz,
                 const bool gx, const bool gy);

  void displayGrid(vpImage<unsigned char> &I);
  void displayUnit(vpImage<unsigned char> &I);
  void displayTitle(vpImage<unsigned char> &I);
  void displayLegend(vpImage<unsigned char> &I);
  void displayGrid3D(vpImage<unsigned char> &I);

  void findPose();

  bool getPixelValue(vpImage<unsigned char> &I, vpImagePoint &iP);

  bool move(const vpImage<unsigned char> &I, vpMouseButton::vpMouseButtonType &button);
  vpHomogeneousMatrix navigation(const vpImage<unsigned char> &I, bool &changed, vpMouseButton::vpMouseButtonType &b);

  void plot(vpImage<unsigned char> &I, const unsigned int curveNb, const double x, const double y);
  vpMouseButton::vpMouseButtonType plot(vpImage<unsigned char> &I, const unsigned int curveNb, const double x,
                                        const double y, const double z);
  void replot(vpImage<unsigned char> &I);
  void replot3D(vpImage<unsigned char> &I);

  void rescalex(unsigned int side, double extremity);
  void rescaley(unsigned int side, double extremity);
  void rescalez(unsigned int side, double extremity);
  // void rescale(double &min, double &max, double &delta, const int nbDiv,
  // int side);
  void resetPointList(const unsigned int curveNum);

  void setCurveColor(const unsigned int curveNum, const vpColor &color);
  void setCurveThickness(const unsigned int curveNum, const unsigned int thickness);
  void setGridThickness(const unsigned int thickness) { this->gridThickness = thickness; };
  void setLegend(const unsigned int curveNum, const std::string &legend);
  void setTitle(const std::string &title);
  void setUnitX(const std::string &unitx);
  void setUnitY(const std::string &unity);
  void setUnitZ(const std::string &unitz);
};

#endif
#endif
#endif
