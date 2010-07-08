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

#include <visp/vpConfig.h>
#include <visp/vpPlot.h>
#include <visp/vpMath.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>
#include <fstream>

vpPlot::vpPlot(const int nbGraph)
{
  I.init(700,700,255);
  
  graphList = NULL;
  
  display.init(I);
  
  vpDisplay::display(I);
  
  margei = 30;
  margej = 40;
  
  init(nbGraph);
}

vpPlot::~vpPlot()
{
  if (graphList != NULL)
  {
    delete[] graphList;
    graphList = NULL;
  }
}

void
vpPlot::init (int nbGraph)
{
  graphList = new vpPlotGraph[nbGraph];
  graphNbr = nbGraph;
  
  switch (nbGraph)
  {
    case 1 : 
      graphList[0].initSize(vpImagePoint(0,0),700,700,margei,margej);
      break;
    case 2 :
      graphList[0].initSize(vpImagePoint(0,0),700,350,margei,margej);
      graphList[1].initSize(vpImagePoint(350,0),700,350,margei,margej);
      break;
    case 3 :
      graphList[0].initSize(vpImagePoint(0,0),350,350,margei,margej);
      graphList[1].initSize(vpImagePoint(0,350),350,350,margei,margej);
      graphList[2].initSize(vpImagePoint(350,0),700,350,margei,margej);
      break;
    case 4 :
      graphList[0].initSize(vpImagePoint(0,0),350,350,margei,margej);
      graphList[1].initSize(vpImagePoint(0,350),350,350,margei,margej);
      graphList[2].initSize(vpImagePoint(350,0),350,350,margei,margej);
      graphList[3].initSize(vpImagePoint(350,350),350,350,margei,margej);
      break;
  }
  
  for (int i = 0; i < graphNbr; i++)
  {
    strcpy(graphList[i].title, "");
    strcpy(graphList[i].unitx, "");
    strcpy(graphList[i].unity, "");
    strcpy(graphList[i].unitz, "");
    graphList[i].textdispayed=false;
  }
}

void
vpPlot::initGraph (int graphNum, int curveNbr)
{
  (graphList+graphNum)->initGraph(curveNbr);
}

void
vpPlot::initRange (const int graphNum, double xmin, double xmax, double /*xdelt*/, double ymin, double ymax, double /*ydelt*/, const bool gx, const bool gy)
{
  (graphList+graphNum)->initScale(I,xmin,xmax,10,ymin,ymax,10,gx,gy);
}

void
vpPlot::setColor (const int graphNum, const int curveNum, vpColor color)
{
  (graphList+graphNum)->setCurveColor(curveNum, color);
}

void
vpPlot::displayGrid()
{
  for (int i = 0; i < graphNbr; i++)
    graphList[i].displayGrid(I);
}

void
vpPlot::plot (const int graphNum, const int curveNum, const double x, const double y)
{
  (graphList+graphNum)->plot(I,curveNum,x,y);
}

void
vpPlot::plot (const int graphNum, const int curveNum, const double x, const double y, const double z)
{
  (graphList+graphNum)->plot(I,curveNum,x,y,z);
}

void
vpPlot::navigate()
{
  vpImagePoint trash;
  vpMouseButton::vpMouseButtonType b = vpMouseButton::button1;
  
  bool blocked = false;
  int iblocked = 0;
  vpImagePoint iP;
  
  while (b != vpMouseButton::button3)
  {
    if (!blocked)
    {
      vpDisplay::getPointerPosition(I,iP);
      for (int i = 0; i < graphNbr ; i++)
      {
	if (iP.inRectangle((graphList+i)->graphZone))
	{
	  iblocked = i;
	  break;
	}
      }
      if ((graphList+iblocked)->move(I))
      {
	(graphList+iblocked)->replot3D(I);
      }
      
      blocked = (graphList+iblocked)->blocked;
    }
    else
    {
      if ((graphList+iblocked)->move(I))
      {
	(graphList+iblocked)->replot3D(I);
      }
      blocked = (graphList+iblocked)->blocked;
    }
  }
}

void
vpPlot::getPixelValue(const bool block)
{
  vpImagePoint iP;
  
  if (block)
    vpDisplay::getClick(I,iP);
  else
    vpDisplay::getPointerPosition(I,iP);
  
  for (int i = 0; i < graphNbr; i++)
  {
    if ((graphList+i)->getPixelValue(I,iP)) break;
  }
}

void
vpPlot::setTitle (const int graphNum, const char *title)
{
  (graphList+graphNum)->setTitle(title);
}

void
vpPlot::setUnitX (const int graphNum, const char *unitx)
{
  (graphList+graphNum)->setUnitX(unitx);
}

void
vpPlot::setUnitY (const int graphNum, const char *unity)
{
  (graphList+graphNum)->setUnitY(unity);
}

void
vpPlot::setUnitZ (const int graphNum, const char *unitz)
{
  (graphList+graphNum)->setUnitZ(unitz);
}

void
vpPlot::setLegend (const int graphNum, const int curveNum, const char *legend)
{
  (graphList+graphNum)->setLegend(curveNum, legend);
}

void 
vpPlot::resetPointList (const int graphNum)
{
  for (int i = 0; i < (graphList+graphNum)->curveNbr; i++)
    (graphList+graphNum)->resetPointList(i);
}

void 
vpPlot::resetPointList (const int graphNum, const int curveNum)
{
  (graphList+graphNum)->resetPointList(curveNum);
}

/*!
  This function enables to save in a text file all the plotted points of a graphic.

  The first line of the text file is the graphic title. Then the points coordinates are given. If the graphic has to curves: 
  - the first column corresponds to the x axis of the first curve
  - the second column corresponds to the y axis of the first curve
  - the third column corresponds to the x axis of the second curve
  - the fourth column corresponds to the y axis of the second curve

  The column are delimited thanks to tabultaions.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param dataFile : Name of the text file.
*/
void vpPlot::saveData(const int graphNum, const char* dataFile)
{
  std::ofstream fichier;
  fichier.open(dataFile);

  int ind;
  double *p = new double[3];
  bool end=false;

  fichier << (graphList+graphNum)->title << std::endl;

  for(ind=0;ind<(graphList+graphNum)->curveNbr;ind++)
  {
    (graphList+graphNum)->curveList[ind].pointListx.front();
    (graphList+graphNum)->curveList[ind].pointListy.front();
    (graphList+graphNum)->curveList[ind].pointListz.front();
  }

  while (end == false)
  {
    end = true;
    for(ind=0;ind<(graphList+graphNum)->curveNbr;ind++)
    {
      if (!(graphList+graphNum)->curveList[ind].pointListx.outside() 
	  && !(graphList+graphNum)->curveList[ind].pointListy.outside()
	  && !(graphList+graphNum)->curveList[ind].pointListz.outside())
      {
        p[0] = (graphList+graphNum)->curveList[ind].pointListx.value();
        p[1] = (graphList+graphNum)->curveList[ind].pointListy.value();
	p[2] = (graphList+graphNum)->curveList[ind].pointListz.value();
        fichier << p[0] << "\t" << p[1] << "\t" << p[2] << "\t";
        (graphList+graphNum)->curveList[ind].pointListx.next();
        (graphList+graphNum)->curveList[ind].pointListy.next();
	(graphList+graphNum)->curveList[ind].pointListz.next();
        if(!(graphList+graphNum)->curveList[ind].pointListx.nextOutside() 
	   && !(graphList+graphNum)->curveList[ind].pointListy.nextOutside()
	   && !(graphList+graphNum)->curveList[ind].pointListz.nextOutside()) 
	  end = false;
      }
      else
      {
        p[0] = (graphList+graphNum)->curveList[ind].pointListx.value();
        p[1] = (graphList+graphNum)->curveList[ind].pointListy.value();
	p[2] = (graphList+graphNum)->curveList[ind].pointListz.value();
        fichier << p[0] << "\t" << p[1] << "\t" << p[2] << "\t";
      }
    }
    fichier << std::endl;
  }

  delete[] p;
  fichier.close();
}