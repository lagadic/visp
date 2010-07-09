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

/*!
  \file vpPlot.h
  \brief Plot curves.
*/

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


/*!
  \class vpPlot
  \ingroup plot

  \brief This class enables real time drawing of 2D or 3D graphics. An instance of the class open a window which contains between 1 and 4 graphics. Each one contains a desired number of curves.

  The example below shows how to use the vpPlot class.

  \code
#include <visp/vpConfig.h>
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
    vpImage<unsigned char> I;
  
  private:
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
    
    void initGraph (int graphNum, int curveNbr);
    void initRange (const int graphNum, double xmin, double xmax, double xdelt, double ymin, double ymax, double ydelt, const bool gx=0, const bool gy=0);
    void setColor (const int graphNum, const int curveNum, vpColor color);
    void setTitle (const int graphNum, const char *title);
    void setUnitX (const int graphNum, const char *unitx);
    void setUnitY (const int graphNum, const char *unity);
    void setUnitZ (const int graphNum, const char *unitz);
    void setLegend (const int graphNum, const int curveNum, const char *legend);
    
    void plot (const int graphNum, const int curveNum, const double x, const double y);
    void plot(const int graphNum, const double x, const vpColVector v);
    void plot (const int graphNum, const int curveNum, const double x, const double y, const double z);
    
    void navigate ();
    
    void getPixelValue(const bool block);
    
    void resetPointList (const int graphNum);
    void resetPointList (const int graphNum, const int curveNum);
    
    void saveData(const int graphNum, const char* dataFile);
    
  private:
    void init (int nbGraph);
    void displayGrid();
};

#endif
