/****************************************************************************
 *
 * $Id: vpPlot.h,v 1.13 2008-12-15 15:11:27 nmelchio Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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

/*!
  \file vpPlot.h
  \brief Plot curves.
*/

#ifndef vpPlot_H
#define vpPlot_H

#include <visp/vpConfig.h>
#include <visp/vpColor.h>
#include <visp/vpList.h>

#if defined(VISP_HAVE_LIBPLOTTER)

#include <plotter.h>

#include <fstream>

/*!
  \class vpPlot
  \ingroup plot

  \brief This class enables real time drawing of graphics. An instance of the class open an X window which contains between 1 and 4 graphics. Each one contains a desired number of curves. It is possible to save the graphics in the post script format.

  \warning This class is available only in unix system. It uses the libplotter library which is the C++ version of the libplot (in C). You are able to find this library with the plotutils package that can be download at this adress ftp://ftp.gnu.org/. For more details about the installation of the library, refers to the ViSP website (Third-party libraries section).

  The example below shows how to use the vpPlot class.

  \code
  #include <visp/vpPlot.h>

  int main ()
  {
  //Create a window with two graphics
  vpPlot A(2);

  //The first graphic contains 1 curve and the second graphic contains 2 curves
  A.initGraph(0,1);
  A.initGraph(1,2);

  //The color of the curve in the first graphic is red
  A.setColor(0,0,55000,0,0);
 //The first curve in the second graphic is green
  A.setColor(1,0,0,55000,0);
 //The second curve in the second graphic is blue
  A.setColor(1,1,0,0,55000);

  //For the first graphic : along the x axis the expected values are between 0 and 200 and the step is 1
  //For the first graphic : along the y axis the expected values are between -2 and 0.5 and the step is 1
  A.initRange(0,0,200,1,-2,0.5,1);

  //For the second graphic : along the x axis the expected values are between 0 and 200 and the step is 1
  //For the second graphic : along the y axis the expected values are between 0 and 3 and the step is 0.5
  A.initRange(1,0,200,1,0,3,0.5);

  //Add the point (0,0) in the first graphic 
  A.plot(0,0,0,0);

  //Add the point (0,1) to the first curve of the second graphic
  A.plot(1,0,0,1);

  //Add the point (0,2) to the second curve of the second graphic
  A.plot(1,1,0,2);

  for (int i = 0; i < 50; i++)
  {
    //Add the point (i,0) in the first graphic 
    A.plot(0,0,i,0);

    //Add the point (i,1) to the first curve of the second graphic
    A.plot(1,0,i,1);

    //Add the point (i,2) to the second curve of the second graphic
    A.plot(1,1,i,2);
  }

  return 0;
  }
  \endcode
*/


class vpPlot
{
  public:

    //! Different types of marker usable in vpPlot
    typedef enum 
    {
      dot = 1,
      plus,
      asterisk,
      circle,
      cross,
      square,
      triangle,
      diamond,
      star,
      inverted_triangle,
      starburst,
      fancy_plus,
      fancy_cross,
      fancy_square,
      fancy_diamond,
      filled_circle,
      filled_square,
      filled_triangle,
      filled_diamond,
      filled_inverted_triangle,
      filled_fancy_square,
      filled_fancy_diamond,
      half_filled_circle,
      half_filled_square,
      half_filled_triangle,
      half_filled_diamond,
      half_filled_inverted_triangle,
      half_filled_fancy_square,
      half_filled_fancy_diamond,
      octagon,
      filled_octagon
    } vpMarkerStyle;

//     typedef enum 
//     {
//       none,
//       solid,             //--------------------------------
//       dotted,            //-   -   -   -   -   -   -   -   
//       dotdashed,         //----   -   ----   -   ----   -
//       shortdashed,       //----    ----    ----    ----    
//       longdashed,        //-------    -------    -------    
//       dotdotdashed,      //----   -   -   ----   -   -
//       dotdotdotdashed   //----   -   -   -   ----   -   -   -
//     } vpLineStyle;

    //! Different styles to plot the curve.
    typedef enum 
    {
      point,
      line,
      dashed_line,
      marker
    } vpCurveStyle;


  protected:
    //! The X window plot class
    XPlotter* XPlot;
    //! The post script plot class
    PSPlotter* PsPlot;
//     GIFPlotter* GifPlot;
//     FigPlotter* FigPlot;
//     TekPlotter* TekPlot;
//     XDrawablePlotter
//     PNMPlotter
//     AIPlotter
//     PCLPlotter
//     HPGLPlotter
//     MetaPlotter

    //! Type of plotter
    typedef enum 
    {
      X,
      PS
    } vpPlotType;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    //! Structure which defines all the parameters describing a curve.
    typedef struct 
    {
      int color[3];
      vpCurveStyle curveStyle; 
      vpMarkerStyle markerStyle;
      char lineStyle[20];
      vpList<double> pointListx;
      vpList<double> pointListy;
      char legend[256];
      double ymin;
      double ymax;
    } vpCurve;

    //! Structure which defines all the parameters describing a graph.
    typedef struct 
    {
      double xorg;
      double yorg;
      double zoomx;
      double zoomy;
      double xmax_rel;
      double ymax_rel;
      double xmin_rel;
      double ymin_rel;
      double xdelt_rel;
      double ydelt_rel;
      double lgx;
      double lgy;
      double ltx;
      double lty;
      bool gridx;
      bool gridy;
      char title[256];
      char unitx[256];
      char unity[256];
      int curveNbr;
      vpCurve* curveList;
      bool textdispayed;
    } vpGraph;
#endif

    //! Number of graphic in the window. Maximum 4.
    int graphNbr;
    //! List of the curves
    vpGraph *graph;

    //! marge for the x axis
    int margex;
    //! marge for the y axis
    int margey;
    //! Small space used while displaying text
    int eps;
    //!Coefficient used during the rescale step
    int coef;

    bool refresh_mode;

  public:
    vpPlot(int graphNbr);
    virtual ~vpPlot();

    void init(int graphNbr);
    void initSize(int graphNum, const double lx, const double ly, const double x, const double y);
    void initGraph(int graphNum, int curveNbr);
    void initRange(const int graphNum, const double xmin, const double xmax, double xdelt, const double ymin, const double ymax, double ydelt, const bool gx = 0, const bool gy = 0);

    void setBgColor(const int r, const int g, const int b);
    void setBgColor(const vpColor color);
    //void setUserSpaceCoordinate(const double x0, const double y0, const double x1, double y1);
    void flush();

    void drawPoint(const int graphNum,  const int curveNum, const double x, const double y, vpPlotType plotType = X);
    void drawPath(const int graphNum,  const int curveNum, const double x, const double y, vpPlotType plotType = X);
    void drawLine(const int graphNum,  const int curveNum, const double x0, const double y0, const double x1, double y1, vpPlotType plotType = X);
    void drawDashedLine(const int graphNum,  const int curveNum, const double x0, const double y0, const double x1, double y1, vpPlotType plotType = X);
    void drawMarker(const int graphNum,  const int curveNum, const double x, const double y, vpMarkerStyle type, const double size, vpPlotType plotType = X);

    void setColor(const int graphNum,  const int curveNum, const int r, const int g, const int b);
    void setLineStyle(const int graphNum,  const int curveNum, const char* style);

    void plot(const int graphNum,  const int curveNum, const double x, const double y);
    void plotExtremities(const int graphNum,  const int curveNum, vpPlotType plotType = X);

    int out_range(const int graphNum, double x, double y);
    void replot(const int graphNum);

    void setLegend(const int graphNum,  const int curveNum, char* legend);
    void setTitle(const int graphNum, char* title);
    void setUnitX(const int graphNum, char* unitx);
    void setUnitY(const int graphNum, char* unity);

    void savePS(char* psFile);


  protected:
    vpPlot();
    void draw(const int graphNum,  const int curveNum, const double x0, const double y0, const double x1, double y1, vpPlotType plotType = X);
    void drawLegend(const int graphNum);
    void setUserSpaceCoordinate(const double x0, const double y0, const double x1, double y1);

};


#endif 

#endif