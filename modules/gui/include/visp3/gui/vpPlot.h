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
 * Plot curves.
 */

/*!
  \file vpPlot.h
  \brief Plot curves.
*/

#ifndef VP_PLOT_H
#define VP_PLOT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpPlotGraph.h>

#if defined(VISP_HAVE_DISPLAY)

BEGIN_VISP_NAMESPACE

/*!
 * \class vpPlot
 * \ingroup group_gui_plotter
 *
 * \brief This class enables real time drawing of 2D or 3D graphics. An
 * instance of the class open a window which contains between 1 and 4
 * graphics. Each one contains a desired number of curves.
 *
 * \warning This class is only available if one of the display functionalities
 * (X11, GDI, GTK, OpenCV or Direct3D) is available. In visp3/core/vpConfig.h
 * header file, you should have VISP_HAVE_DISPLAY define.
 *
 * The example below shows how to use the vpPlot class. An other example
 * provided in tutorial-ibvs-plotter.cpp and described in \ref tutorial-plotter
 * shows how to use this class to plot in real-time some curves during an
 * image-based visual servo.
 *
 * \code
 * #include <visp3/gui/vpPlot.h>
 *
 * int main ()
 * {
 * #if defined(VISP_HAVE_DISPLAY)
 *   // Create a window (700 by 700) at position (100, 200) with two graphics
 *   vpPlot A(2, 700, 700, 100, 200, "Curves...");
 *
 *   // The first graphic contains 1 curve and the second graphic contains 2 curves
 *   A.initGraph(0,1); A.initGraph(1,2);
 *
 *   // The color of the curve in the first graphic is red
 *   A.setColor(0,0,vpColor::red);
 *   // The first curve in the second graphic is green
 *   A.setColor(1,0,vpColor::green);
 *   // The second curve in the second graphic is blue
 *   A.setColor(1,1,vpColor::blue);
 *
 *   // Add the point (0,0) in the first graphic
 *   A.plot(0,0,0,0);
 *
 *   // Add the point (0,1) to the first curve of the second graphic
 *   A.plot(1,0,0,1);
 *
 *   // Add the point (0,2) to the second curve of the second graphic
 *   A.plot(1,1,0,2);
 *
 *   for (int i = 0; i < 50; ++i) {
 *     // Add the point (i,sin(i*pi/10) in the first graphic
 *     A.plot(0,0,i,sin(i*M_PI/10));
 *
 *     // Add the point (i,1) to the first curve of the second graphic
 *     A.plot(1,0,i,1);
 *
 *     // Add the point (i,2) to the second curve of the second graphic
 *     A.plot(1,1,i,2);
 *   }
 *
 *   return 0;
 * #endif
 * }
 * \endcode
*/
class VISP_EXPORT vpPlot
{
public:
  vpImage<unsigned char> I;

private:
  vpDisplay *display;

  unsigned int graphNbr;
  vpPlotGraph *graphList;

  unsigned int margei;
  unsigned int margej;

  float factori;
  float factorj;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpPlot(const vpPlot &)
  //      : I(), display(nullptr), graphNbr(0), graphList(nullptr), margei(0),
  //      margej(0),
  //        factori(0), factorj(0)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpPlot &operator=(const vpPlot &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpPlot();
  vpPlot(unsigned int nbGraph, unsigned int height = 700, unsigned int width = 700, int x = -1, int y = -1,
         const std::string &title = "");
  virtual ~vpPlot();
  void closeDisplay();
  void getPixelValue(bool block);
  void init(unsigned int nbGraph, unsigned int height = 700, unsigned int width = 700, int x = -1, int y = -1,
            const std::string &title = "");
  void initGraph(unsigned int graphNum, unsigned int curveNbr);

  void initRange(unsigned int graphNum, double xmin, double xmax, double ymin, double ymax);
  void initRange(unsigned int graphNum, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
  void navigate(void);

  void plot(unsigned int graphNum, unsigned int curveNum, double x, double y);
  void plot(unsigned int graphNum, double x, const vpColVector &v_y);
  void plot(unsigned int graphNum, double x, const vpRowVector &v_y);
  void plot(unsigned int graphNum, double x, const vpPoseVector &v_y);
  void plot(unsigned int graphNum, double x, const vpTranslationVector &v_y);
  void plot(unsigned int graphNum, double x, const vpRotationVector &v_y);
  vpMouseButton::vpMouseButtonType plot(unsigned int graphNum, unsigned int curveNum, double x, double y, double z);
  vpMouseButton::vpMouseButtonType plot(unsigned int graphNum, double x, const vpColVector &v_y,
                                        const vpColVector &v_z);

  void resetPointList(unsigned int graphNum);
  void resetPointList(unsigned int graphNum, unsigned int curveNum);

  void saveData(unsigned int graphNum, const std::string &dataFile, const std::string &title_prefix = "");
  void setColor(unsigned int graphNum, unsigned int curveNum, vpColor color);
  void setGraphThickness(unsigned int graphNum, unsigned int thickness);
  void setGridThickness(unsigned int graphNum, unsigned int thickness);

  /*!
   * Set the font of the characters. The display should be initialized before.
   *
   * To know which font are available, on Unix you can use xfontsel or xlsfonts
   * utilities.
   */
  void setFont(const std::string &font)
  {
    if (display->isInitialised())
      vpDisplay::setFont(I, font.c_str());
  }
  void setLegend(unsigned int graphNum, unsigned int curveNum, const std::string &legend);
  void setTitle(unsigned int graphNum, const std::string &title);
  void setUnitX(unsigned int graphNum, const std::string &unitx);
  void setUnitY(unsigned int graphNum, const std::string &unity);
  void setUnitZ(unsigned int graphNum, const std::string &unitz);
  void setThickness(unsigned int graphNum, unsigned int curveNum, unsigned int thickness);

private:
  void initNbGraph(unsigned int nbGraph);
  void displayGrid();
};

END_VISP_NAMESPACE
#endif

#endif
