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

#include <visp/vpConfig.h>
#include <visp/vpPlot.h>
//#include <unistd.h>
#include <visp/vpMath.h>

#if defined(VISP_HAVE_LIBPLOTTER)



/*!
  Constructor. This constructor create a new window where the curves will be drawn. The number of graphic in the window must be set.

  \param graphNbr : The number of graph in the window.

  \note The basic constructor is not available.
*/
vpPlot::vpPlot(int graphNbr)
{
  this->graphNbr = graphNbr;

  this->init(graphNbr);
}


/*!
  Basic destructor
*/
vpPlot::~vpPlot()
{
  if (XPlot != NULL)
  {
    XPlot->closepl();
    delete XPlot;
  }

  for (int i = 0; i < graphNbr; i++)
  {
    for (int j = 0; j < graph[i].curveNbr; j++)
    {
      graph[i].curveList[j].pointListx.kill();
      graph[i].curveList[j].pointListy.kill();
    }
    delete[] graph[i].curveList;
  }

  if (graph != NULL)
    delete[] graph;
}


/*!
  Function called by the constructor to initialize the window and the basic parameters.

  \param graphNbr : The number of graph in the window.
*/
void vpPlot::init(int graphNbr)
{

  // set a Plotter parameter
  Plotter::parampl ("BITMAPSIZE", (void*)("660x660"));

  XPlot = new XPlotter;
  if (XPlot->openpl () < 0)          // open Plotter
  {
    std::cout << "Couldn't open XPlot\n" << std::endl;
  }
  XPlot->erase();

  setUserSpaceCoordinate(0,0,620,620);

  graph = new vpGraph[graphNbr];

  switch(graphNbr)
  {
  case 1 : initSize(0,560,560,0,0); break;
  case 2 : initSize(1,560,240,0,0);
           initSize(0,560,240,0,300); break;
  case 3 : initSize(2,560,240,0,0);
           initSize(0,240,240,0,300);
           initSize(1,240,240,320,300); break;
  case 4 : initSize(2,240,240,0,0);
           initSize(3,240,240,320,0);
           initSize(0,240,240,0,300);
           initSize(1,240,240,320,300); break;
  }

  //XPlot->fontsize(10);

  refresh_mode = true;
  margex=40;
  margey=40;
  eps=5;
  coef=1;

  for (int i = 0; i < graphNbr; i++)
  {
    strcpy(graph[i].title, "title");
    strcpy(graph[i].unitx, "unitx");
    strcpy(graph[i].unity, "unity");
    graph[i].textdispayed=false;
  }
}


/*!
  Function to initialize the parameters linked to the position and the size of a graphic in the window.
  The coordinates are given relative to the window parameters. So they are relative coordinates. Here we decided to define the size of all window equal to 620x620 (see the function init).

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param lx : The graphic width.
  \param ly : The graphic height.
  \param x : The coordinate of the bottom left corner along the x axis.
  \param y : The coordinate of the bottom left corner along the y axis.
*/
void vpPlot::initSize(int graphNum, const double lx, const double ly, 
		      const double x, const double y)
{
  graph[graphNum].lgx=lx;
  graph[graphNum].lgy=ly;
  graph[graphNum].ltx=x;
  graph[graphNum].lty=y;
}


/*!
  Function which enables to initialize the number of curves which belongs to a graphic.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNbr : The number of curves belonging to the graphic
*/
void vpPlot::initGraph(int graphNum, int curveNbr)
{
  graph[graphNum].curveNbr = curveNbr;
  graph[graphNum].curveList = new vpCurve[curveNbr];

  vpColor colors[6] = {vpColor::blue,vpColor::green,vpColor::red,vpColor::cyan,vpColor::orange,vpColor::yellow};

	for (int i = 0; i < curveNbr; i++)
	{
	setColor(graphNum, i, colors[i%6]);
	/*graph[graphNum].curveList[i].color[0] = 0; //R
	graph[graphNum].curveList[i].color[1] = 0; //G
	graph[graphNum].curveList[i].color[2] = 0; //B*/
    graph[graphNum].curveList[i].curveStyle = line;
    graph[graphNum].curveList[i].pointListx.kill();
    graph[graphNum].curveList[i].pointListy.kill();
    strcpy(graph[graphNum].curveList[i].lineStyle,"none");
    strcpy(graph[graphNum].curveList[i].legend,"legend");

  }
}


/*!
  This function has two goals. The first one is to computes any parameters which enable to link the user coordinates given in the user unit system (meter, speed, weight,...) with the relative coordinates of the window. Thus the minimum and maximum values are asked to initialize the graphic.

  The second goal of this function is to draw the axis and if necessary a grid to help the graphic reading.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param xmin : The initial minimum value along the x axis given in the user coordinates.
  \param xmax : The initial maximum value along the x axis given in the user coordinates.
  \param xdelt :  The initial step use to write the values along the x axis.
  \param ymin : The initial minimum value along the y axis given in the user coordinates.
  \param ymax : The initial maximum value along the y axis given in the user coordinates.
  \param ydelt :  The initial step use to write the values along the y axis.
  \param gx : If true, a grid is drawn allog the x axis to help the user to read the graphic.
  \param gy : If true, a grid is drawn allog the y axis to help the user to read the graphic.
*/
void vpPlot::initRange(const int graphNum, const double xmin, 
		       const double xmax,  double xdelt,
		       const double ymin, const double ymax, 
		       double ydelt, const bool gx, const bool gy)
{
  if(gx||gy)
  {
    graph[graphNum].gridx = gx;
    graph[graphNum].gridy = gy;
  }

  char valeur[20];
  double x1 = graph[graphNum].ltx+margex-eps;
  double y1,x2,y2;
  double xp,yp,t;

  // modif EM
  if ((ymax-ymin)/ydelt>10) ydelt = (ymax-ymin)/10 ;
  if ((xmax-xmin)/xdelt>10) xdelt = (xmax-xmin)/10 ;
  //initialisation attributs fenetre
  graph[graphNum].zoomx = graph[graphNum].lgx/(xmax-xmin);
  graph[graphNum].zoomy = graph[graphNum].lgy/(ymax-ymin);
  graph[graphNum].xmin_rel = xmin;
  graph[graphNum].xmax_rel = xmax;
  graph[graphNum].xdelt_rel = xdelt;
  graph[graphNum].ymin_rel = ymin;
  graph[graphNum].ymax_rel = ymax;
  graph[graphNum].ydelt_rel = ydelt;
  graph[graphNum].xorg = graph[graphNum].ltx + margex;
  graph[graphNum].yorg = graph[graphNum].lty + margey - (ymin*graph[graphNum].zoomy);

  //test pour range differe
  if(ymax>ymin)
  {
    //trace de la grille ou unitee
    if(gy) { x2 = graph[graphNum].ltx+margex+graph[graphNum].lgx; XPlot->linemod("dotted"); }
    else { x2 = graph[graphNum].ltx+margex+eps; XPlot->linemod("solid"); }

    for(t=ymin;t<=ymax;t=t+ydelt)
    {
      XPlot->color(40000,40000,40000);
      yp = graph[graphNum].yorg+(graph[graphNum].zoomy*t); //+ changé en -
      XPlot->fline(x1,yp,x2,yp);
      if((t<1e-5)&&(t>-1e-5)) t=0;
      sprintf(valeur, "%g", t);
      XPlot->fmove(x1,yp);
      XPlot->color(20000,20000,50000);
      XPlot->alabel('r','x',valeur);
    }

    if(gx) { y1 = graph[graphNum].lty+margey; y2= graph[graphNum].lty+margey+graph[graphNum].lgy; XPlot->linemod("dotted"); }
    else
    {
      if (graph[graphNum].yorg >= graph[graphNum].lty+margey) {y1=graph[graphNum].yorg-eps; y2=graph[graphNum].yorg+eps;}
      else {y1=graph[graphNum].lty+margey-eps; y2=graph[graphNum].lty+margey+eps;}
      XPlot->linemod("solid");
    }

    for(t=xmin;t<=xmax;t=t+xdelt)
    {
      XPlot->color(40000,40000,40000);
      xp = graph[graphNum].xorg+(graph[graphNum].zoomx*t); //+ changé en -
      XPlot->fline(xp,y1,xp,y2);
      if((t<1e-5)&&(t>-1e-5)) t=0;
      sprintf(valeur, "%g", t);
      if (graph[graphNum].yorg >= graph[graphNum].lty+margey) XPlot->fmove(xp,graph[graphNum].yorg-3*eps);
      else XPlot->fmove(xp,graph[graphNum].lty+margey-3*eps);
      XPlot->color(20000,20000,50000);
      XPlot->alabel('c','x',valeur);
    }

     //trace des axes
     XPlot->color(0,0,0);
     XPlot->linemod("solid");
     XPlot->fline(graph[graphNum].ltx,graph[graphNum].yorg,graph[graphNum].ltx+margex+graph[graphNum].lgx,graph[graphNum].yorg);
     XPlot->fline(graph[graphNum].xorg,graph[graphNum].lty+margey,graph[graphNum].xorg,graph[graphNum].lty+margey+graph[graphNum].lgy);
   }
}


/*!
  This function enables to set the color of the window's background.

  \param r : The red value.
  \param g : The green value.
  \param b : The blue value.

  \note This function is only available for the X window.
*/
void vpPlot::setBgColor(const int r, const int g, const int b)
{
  if (XPlot != NULL)
  {
    XPlot->bgcolor(r,g,b);
    XPlot->erase();
  }
}


/*!
  This function enables to define the relative coordinates of the window.

  \param x0 : The relative coordinate of the window's bottom left corner along the x axis.
  \param y0 : The relative coordinate of the window's bottom left corner along the y axis.
  \param x1 : The relative coordinate of the window's top right corner along the x axis.
  \param y1 : The relative coordinate of the window's top right corner along the y axis.
*/
void vpPlot::setUserSpaceCoordinate(const double x0, const double y0, 
				    const double x1, const double y1)
{
  if (XPlot != NULL)
  {
    XPlot->fspace(x0, y0, x1, y1);
  }
}


/*!
  This function enables to flush the display
*/
void vpPlot::flush()
{
  if (XPlot != NULL)
  {
    XPlot->flushpl();
  }
}


/*!
  This function enables to draw a point for one of the curves belonging to one of the graphics. The point coordinates are given in the relative coordinate system.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x : The point coordinate along the x axis given in the relative coordinate system.
  \param y : The point coordinate along the y axis given in the relative coordinate system.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::drawPoint(const int graphNum,  const int curveNum, 
		       const double x, const double y, vpPlotType plotType)
{
  if (XPlot != NULL && plotType == X)
  {
    XPlot->fmove (0, 0);
    XPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		    graph[graphNum].curveList[curveNum].color[1], 
		    graph[graphNum].curveList[curveNum].color[2]);
    XPlot->fpointrel(x, y);
  }

  if (PsPlot != NULL && plotType == PS)
  {
    PsPlot->fmove (0, 0);
    PsPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		     graph[graphNum].curveList[curveNum].color[1], 
		     graph[graphNum].curveList[curveNum].color[2]);
    PsPlot->fpointrel(x, y);
  }
}


/*!
  This function enables to draw a path between a previous point and a new point for one of the curves belonging to one of the graphics.
  The new point coordinates are given in the user unit system.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x : The point coordinate along the x axis given in the relative coordinate system.
  \param y : The point coordinate along the y axis given in the relative coordinate system.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::drawPath(const int graphNum,  const int curveNum, 
		      const double x, const double y, vpPlotType plotType)
{
  if (XPlot != NULL && plotType == X)
  {
    XPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		    graph[graphNum].curveList[curveNum].color[1], 
		    graph[graphNum].curveList[curveNum].color[2]);
    XPlot->fcontrel(x, y);
  }

  if (PsPlot != NULL && plotType == PS)
  {
    PsPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		     graph[graphNum].curveList[curveNum].color[1],
		     graph[graphNum].curveList[curveNum].color[2]);
    PsPlot->fcontrel(x, y);
  }
}


/*!
  This function enables to draw a line beetween two points for one of the curves belonging to one of the graphics. The points coordinates are given in the user unit system.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x0 : The first point coordinate along the x axis given in the relative coordinate system.
  \param y0 : The first point coordinate along the y axis given in the relative coordinate system.
  \param x1 : The second point coordinate along the x axis given in the relative coordinate system.
  \param y1 : The second point coordinate along the y axis given in the relative coordinate system.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::drawLine(const int graphNum,  const int curveNum,
		      const double x0, const double y0, 
		      const double x1, const double y1, vpPlotType plotType)
{
  if (XPlot != NULL && plotType == X)
  {
    XPlot->fmove (0, 0);
    XPlot->pencolor(graph[graphNum].curveList[curveNum].color[0],
		    graph[graphNum].curveList[curveNum].color[1], 
		    graph[graphNum].curveList[curveNum].color[2]);
    XPlot->flinerel (x0, y0, x1, y1);
  }

  if (PsPlot != NULL && plotType == PS)
  {
    PsPlot->fmove (0, 0);
    PsPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		     graph[graphNum].curveList[curveNum].color[1], 
		     graph[graphNum].curveList[curveNum].color[2]);
    PsPlot->flinerel (x0, y0, x1, y1);
  }
}


/*!
  This function enables to draw a dashed line beetween two points for one of the curves belonging to one of the graphics. The points coordinates are given in the user unit system.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x0 : The first point coordinate along the x axis given in the relative coordinate system.
  \param y0 : The first point coordinate along the y axis given in the relative coordinate system.
  \param x1 : The second point coordinate along the x axis given in the relative coordinate system.
  \param y1 : The second point coordinate along the y axis given in the relative coordinate system.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::drawDashedLine(const int graphNum,  const int curveNum, 
			    const double x0, const double y0, 
			    const double x1, const double y1, 
			    vpPlotType plotType)
{
  if (XPlot != NULL && plotType == X)
  {
    XPlot->linemod("dotted");
    XPlot->fmove (0, 0);
    XPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		    graph[graphNum].curveList[curveNum].color[1], 
		    graph[graphNum].curveList[curveNum].color[2]);
    XPlot->flinerel (x0, y0, x1, y1);
    XPlot->linemod("solid");
  }

  if (PsPlot != NULL && plotType == PS)
  {
    PsPlot->linemod("dotted");
    PsPlot->fmove (0, 0);
    PsPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		     graph[graphNum].curveList[curveNum].color[1], 
		     graph[graphNum].curveList[curveNum].color[2]);
    PsPlot->flinerel (x0, y0, x1, y1);
    PsPlot->linemod("solid");
  }
}


/*!
  This function enables to draw a special marker for one of the curves belonging to one of the graphics. The marker coordinates are given in the user unit system.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x : The marker coordinate along the x axis given in the relative coordinate system.
  \param y : The marker coordinate along the y axis given in the relative coordinate system.
  \param type : It exists numerous type of marker. This parameter enable you to choose the one you want.
  \param size : The size of the marker.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::drawMarker(const int graphNum,  const int curveNum, 
			const double x, const double y,
			vpMarkerStyle type, const double size,
			vpPlotType plotType)
{
  if (XPlot != NULL && plotType == X)
  {
    XPlot->fmove (0, 0);
    XPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		    graph[graphNum].curveList[curveNum].color[1], 
		    graph[graphNum].curveList[curveNum].color[2]);
    XPlot->fmarkerrel(x, y, type, size);
  }

  if (PsPlot != NULL && plotType == PS)
  {
    PsPlot->fmove (0, 0);
    PsPlot->pencolor(graph[graphNum].curveList[curveNum].color[0], 
		     graph[graphNum].curveList[curveNum].color[1], 
		     graph[graphNum].curveList[curveNum].color[2]);
    PsPlot->fmarkerrel(x, y, type, size);
  }
}


/*!
  This function enables you to choose the color used to draw a given curve.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param r : The red value.
  \param g : The green value.
  \param b : The blue value.
*/
void vpPlot::setColor(const int graphNum,  const int curveNum, 
		      const int r, const int g, const int b)
{
  graph[graphNum].curveList[curveNum].color[0] = r; //R
  graph[graphNum].curveList[curveNum].color[1] = g; //G
  graph[graphNum].curveList[curveNum].color[2] = b; //B
}

/*!
  This function enables you to choose the color used to draw a given curve.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param color : The color you want to use
*/
void vpPlot::setColor(const int graphNum,  const int curveNum, vpColor color)
{
  switch(color.id)
  {
    case vpColor::id_black: this->setColor(graphNum, curveNum, 0,0,0);break;
    case vpColor::id_white: this->setColor(graphNum, curveNum, (256*256)-1,(256*256)-1,(256*256)-1);break;
    case vpColor::id_red: this->setColor(graphNum, curveNum, (256*256)-1,0,0);break;
    case vpColor::id_green: this->setColor(graphNum, curveNum, 0,(256*256)-1,0);break;
    case vpColor::id_blue: this->setColor(graphNum, curveNum, 0,0,(256*256)-1);break;
    case vpColor::id_yellow: this->setColor(graphNum, curveNum, (256*256)-1,(256*256)-1,0);break;
    case vpColor::id_cyan: this->setColor(graphNum, curveNum, 0,(256*256)-1,(256*256)-1);break;
    case vpColor::id_orange: this->setColor(graphNum, curveNum, (256*256)-1,(256*128)-1,0);break;
    case vpColor::id_unknown: this->setColor(graphNum, curveNum, color.R*256, color.G*256, color.B*256);break;
  }
}


/*!
  This function enables to set the line style used to draw a line for a given curve.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param style : The line style.

  \note The enable line styles are :
  - solid             --------------------------------
  - dotted            -   -   -   -   -   -   -   -   
  - dotdashed         ----   -   ----   -   ----   -
  - shortdashed       ----    ----    ----    ----    
  - longdashed        -------    -------    -------    
  - dotdotdashed      ----   -   -   ----   -   -
  - dotdotdotdashed   ----   -   -   -   ----   -   -   -
*/
void vpPlot::setLineStyle(const int graphNum,  const int curveNum, 
			  const char* style)
{
  strcpy(graph[graphNum].curveList[curveNum].lineStyle,style);
}


/*!
  This function enables you to add a new point in the curve. This point is drawn with the parameters of the curve.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x : The coordinate of the new point along the x axis and given in the user unit system.
  \param y : The coordinate of the new point along the y axis and given in the user unit system.
*/
void vpPlot::plot(const int graphNum,  const int curveNum, 
		  const double x, const double y)
{
  double x1=0.,x2=0.,y1=0.,y2=0.;
  double *p_1 = new double[2];
  double *p = new double[2];
  int err_range;

  //test pour range differe
  if(graph[graphNum].ymax_rel<=graph[graphNum].ymin_rel)
  {
    if(y>0) {
      initRange(graphNum,graph[graphNum].xmin_rel,
		graph[graphNum].xmax_rel,
		graph[graphNum].xdelt_rel,-y/5,y*1.1,y/5);
    }
    else {
      if(y==0) {
	initRange(graphNum, graph[graphNum].xmin_rel,
		  graph[graphNum].xmax_rel,
		  graph[graphNum].xdelt_rel,-1,1,0.5);
      }
      else {
	initRange(graphNum,graph[graphNum].xmin_rel,
		  graph[graphNum].xmax_rel,
		  graph[graphNum].xdelt_rel,y*1.1,-y/5,y/5);
      }
    }
  }

  if(!graph[graphNum].curveList[curveNum].pointListx.empty())
  {
    p_1[0] = graph[graphNum].curveList[curveNum].pointListx.lastValue();
    p_1[1] = graph[graphNum].curveList[curveNum].pointListy.lastValue();
    x1=graph[graphNum].xorg+(graph[graphNum].zoomx*(*(p_1))); //Changer - en +
    y1=graph[graphNum].yorg+(graph[graphNum].zoomy*(*(p_1+1))); //Changer - en +
  }
  else 
  {
    graph[graphNum].curveList[curveNum].ymin = y;
    graph[graphNum].curveList[curveNum].ymax = y;
  }

  if(y > graph[graphNum].curveList[curveNum].ymax)
  {
    graph[graphNum].curveList[curveNum].ymax = y;
  }

  if(y < graph[graphNum].curveList[curveNum].ymin)
  {
    graph[graphNum].curveList[curveNum].ymin = y;
  }

  x2=graph[graphNum].xorg+(graph[graphNum].zoomx*x); //changement - en +
  y2=graph[graphNum].yorg+(graph[graphNum].zoomy*y); //changement - en +

  err_range = out_range(graphNum,x2,y2);
  if(err_range)
  {
    if( (refresh_mode==false) && ((err_range==2) || (err_range==4)))
    {
      if(!graph[graphNum].curveList[curveNum].pointListx.empty()) 
	draw(graphNum, curveNum, x1,y1,x2,y2);
      *p = x; *(p+1) = y;
      graph[graphNum].curveList[curveNum].pointListx+=*p;
      graph[graphNum].curveList[curveNum].pointListy+=*(p+1);
    }
    else
    {
      *p = x; *(p+1) = y;
      graph[graphNum].curveList[curveNum].pointListx+=*p;
      graph[graphNum].curveList[curveNum].pointListy+=*(p+1);
      XPlot->color(0xffff,0xffff,0xffff);
      XPlot->fillcolor (0xffff,0xffff,0xffff);
      XPlot->filltype(1);
      XPlot->box (graph[graphNum].ltx,graph[graphNum].lty,
		  graph[graphNum].ltx+graph[graphNum].lgx+margex,
		  graph[graphNum].lty+graph[graphNum].lgy+margey);
      switch(err_range)
      {
        case 1:
          initRange(graphNum,
		    graph[graphNum].xmin_rel,
		    graph[graphNum].xmax_rel*1.5,
		    graph[graphNum].xdelt_rel*coef,
		    graph[graphNum].ymin_rel,
		    graph[graphNum].ymax_rel,
		    graph[graphNum].ydelt_rel,
		    graph[graphNum].gridx,
		    graph[graphNum].gridy);
	 break;
        case 4:
	  if(y>graph[graphNum].ymax_rel+vpMath::abs(graph[graphNum].ymax_rel*coef))

            initRange(graphNum,graph[graphNum].xmin_rel,
		      graph[graphNum].xmax_rel,
		      graph[graphNum].xdelt_rel,
		      graph[graphNum].ymin_rel,
		      y*1.5,vpMath::abs(y/4),
		      graph[graphNum].gridx,graph[graphNum].gridy);

	  else
            initRange(graphNum,
		      graph[graphNum].xmin_rel,
		      graph[graphNum].xmax_rel,
		      graph[graphNum].xdelt_rel,
		      graph[graphNum].ymin_rel,
		      graph[graphNum].ymax_rel+vpMath::abs(graph[graphNum].ymax_rel*coef),
		      graph[graphNum].ydelt_rel*coef,
		      graph[graphNum].gridx,graph[graphNum].gridy);

	  break;
        case 3:
          initRange(graphNum,
		    graph[graphNum].xmin_rel*coef,
		    graph[graphNum].xmax_rel,
		    graph[graphNum].xdelt_rel*coef,
		    graph[graphNum].ymin_rel,
		    graph[graphNum].ymax_rel,
		    graph[graphNum].ydelt_rel,
		    graph[graphNum].gridx,graph[graphNum].gridy);

	  break;
        case 2:
	  if(y<graph[graphNum].ymin_rel-vpMath::abs(graph[graphNum].ymin_rel*coef))
            initRange(graphNum,
		      graph[graphNum].xmin_rel,
		      graph[graphNum].xmax_rel,
		      graph[graphNum].xdelt_rel,
		      y*1.5,
		      graph[graphNum].ymax_rel,
		      vpMath::abs(y/4),
		      graph[graphNum].gridx,
		      graph[graphNum].gridy);

	  else
            initRange(graphNum,
		      graph[graphNum].xmin_rel,
		      graph[graphNum].xmax_rel,
		      graph[graphNum].xdelt_rel,
		      graph[graphNum].ymin_rel-vpMath::abs(graph[graphNum].ymin_rel*coef),
		      graph[graphNum].ymax_rel,
		      graph[graphNum].ydelt_rel*coef,
		      graph[graphNum].gridx,
		      graph[graphNum].gridy);

	  break;
      }
      drawLegend(graphNum);
      replot(graphNum);
    }
  }
  else
  {
    if(!graph[graphNum].curveList[curveNum].pointListx.empty())
      draw(graphNum, curveNum, x1,y1,x2,y2);
    *p = x; *(p+1) = y;
    graph[graphNum].curveList[curveNum].pointListx+=*p;
    graph[graphNum].curveList[curveNum].pointListy+=*(p+1);
  }

  if (!graph[graphNum].textdispayed)
  {
    drawLegend(graphNum);
    graph[graphNum].textdispayed = true;
  }

  delete[] p;
  delete[] p_1;
}


/*!
  Function to test if the new point is out of the graphic place.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param x : The coordinate of the new point along the x axis and given in the relative coordinate system.
  \param y : The coordinate of the new point along the y axis and given in the relative coordinate system.

  \return If 0, the point is in the graphic. If 1, The x coordinate is to high. If 2, The y coordinate is too small. If 3, the x coordinate is too small. If 4, the y coordinate is to high.
*/
int vpPlot::out_range(const int graphNum, double x, double y){
   if(x>graph[graphNum].ltx+margex+graph[graphNum].lgx) return 1;
   if(y<graph[graphNum].lty+margey) return 2;
   if(x<graph[graphNum].ltx+margex) return 3;
   if(y>graph[graphNum].lty+margey+graph[graphNum].lgy) return 4;
   return 0;
}


/*!
  This function draws the legend of one graphic in its top right corner.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
*/
void vpPlot::drawLegend(const int graphNum){
  int ind;

  XPlot->pencolor(0,0,0);
  XPlot->fmove(graph[graphNum].ltx+margex+graph[graphNum].lgx/2,
	       graph[graphNum].lty+margey+graph[graphNum].lgy+3*eps);
  XPlot->alabel('c','x',graph[graphNum].title);
  XPlot->fmove(graph[graphNum].ltx+margex+graph[graphNum].lgx-eps,
	       graph[graphNum].yorg+2*eps);
  XPlot->alabel('r','x',graph[graphNum].unitx);
  XPlot->fmove(graph[graphNum].xorg+2*eps,
	       graph[graphNum].lty+margey+graph[graphNum].lgy-eps);
  XPlot->alabel('l','x',graph[graphNum].unity);

  for(ind=0;ind<graph[graphNum].curveNbr;ind++)
  {
    XPlot->pencolor(graph[graphNum].curveList[ind].color[0], 
		    graph[graphNum].curveList[ind].color[1],
		    graph[graphNum].curveList[ind].color[2]);
    XPlot->fmove(graph[graphNum].ltx+margex+graph[graphNum].lgx-eps,
		 graph[graphNum].lty+margex+graph[graphNum].lgy-(10*ind));
    XPlot->alabel('r','x',graph[graphNum].curveList[ind].legend);
  }
}


/*!
  This function is used if the scale of the grphic changed (typically if a point was out of the graphic limits. The goal is to replot all the old points with the new scale.
*/
void vpPlot::replot(const int graphNum)
{
  int nb,t,ind;
  double x1=0.,y1=0.,x2=0.,y2=0.;
  double *p_1 = new double[2];
  double *p = new double[2];

  for(ind=0;ind<graph[graphNum].curveNbr;ind++)
  {
    nb = graph[graphNum].curveList[ind].pointListx.nbElements();
    graph[graphNum].curveList[ind].pointListx.front();
    graph[graphNum].curveList[ind].pointListy.front();
    for(t=0;t<nb;t++)
    {
      if(t!=0)
      {
        p_1[0] = graph[graphNum].curveList[ind].pointListx.value();
        p_1[1] = graph[graphNum].curveList[ind].pointListy.value();
        graph[graphNum].curveList[ind].pointListx.next();
        graph[graphNum].curveList[ind].pointListy.next();
        x1 = graph[graphNum].xorg + graph[graphNum].zoomx*(*p_1);
        y1 = graph[graphNum].yorg+(graph[graphNum].zoomy*(*(p_1+1)));
      }

      p[0] = graph[graphNum].curveList[ind].pointListx.value();
      p[1] = graph[graphNum].curveList[ind].pointListy.value();
      x2 = graph[graphNum].xorg + graph[graphNum].zoomx*(*p);
      y2=graph[graphNum].yorg+(graph[graphNum].zoomy*(*(p+1))); //Changement - en +

      if(t!=0) draw(graphNum, ind, x1,y1,x2,y2);
    }
  }
  delete[] p;
  delete[] p_1;
}


/*!
  Sets the legend of a curve.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param legend : The legend of the curve.
*/
void vpPlot::setLegend(const int graphNum,  const int curveNum,
		       const char* legend)
{
  strcpy(graph[graphNum].curveList[curveNum].legend, legend);
}


/*!
  Sets the title of a graphic.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param title : The graphic title.
*/
void vpPlot::setTitle(const int graphNum, const char* title)
{
  strcpy(graph[graphNum].title, title);
}


/*!
  Sets the unit system of the x axis.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param unitx : The name of the unit of the x axis.
*/
void vpPlot::setUnitX(const int graphNum, const char* unitx)
{
  strcpy(graph[graphNum].unitx, unitx);
}


/*!
  Sets the unit system of the y axis.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param unity : The name of the unit of the y axis.
*/
void vpPlot::setUnitY(const int graphNum, const char* unity)
{
  strcpy(graph[graphNum].unity, unity);
}


/*!
  This function enables to call the drawing function corresponding to the curve when a new point is plotted.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param x0 : The first point coordinate along the x axis given in the relative coordinate system. (Used to draw a line)
  \param y0 : The first point coordinate along the y axis given in the relative coordinate system. (Used to draw a line)
  \param x1 : The second point coordinate along the x axis given in the relative coordinate system.
  \param y1 : The second point coordinate along the y axis given in the relative coordinate system.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::draw(const int graphNum,  const int curveNum, 
		  const double x0, const double y0, 
		  const double x1, const double y1, vpPlotType plotType)
{
  switch (graph[graphNum].curveList[curveNum].curveStyle)
  {
    case point: drawPoint(graphNum, curveNum, x1, y1,plotType); break;
    case line: drawLine(graphNum, curveNum, x0, y0, x1, y1,plotType); break;
    case dashed_line: drawDashedLine(graphNum, curveNum, x0, y0, x1, y1,
				     plotType); break;
    case marker: drawMarker(graphNum, curveNum, x1, y1, 
			    graph[graphNum].curveList[curveNum].markerStyle,
			    2,plotType); break;
  }
}


/*!
  This function enables to save the graph as a post script file.

  \param psFile : Name of the file
*/
void vpPlot::savePS(const char* psFile)
{
  ofstream fichier;
  fichier.open(psFile);

  PsPlot = new PSPlotter(cin, fichier, cerr);
  if (PsPlot->openpl () < 0)          // open Plotter
  {
    std::cout << "Couldn't open PsPlot\n" << std::endl;
  }
  PsPlot->erase(); 
  PsPlot->fspace(0,0,620,620);
  PsPlot->color(0xffff,0xffff,0xffff);
  PsPlot->fbox (0,0,620,620);

  char valeur[20];
  double x1,y1,x2,y2;
  double xp,yp,t;
  double *p_1 = new double[2];
  double *p = new double[2];

  for(int i = 0; i < graphNbr; i++)
  {
    //trace de la grille ou unitee
    x1 = graph[i].ltx+margex-eps;
    if(graph[i].gridy) {
      x2 = graph[i].ltx+margex+graph[i].lgx;
      PsPlot->linemod("dotted"); 
    }
    else { x2 = graph[i].ltx+margex+eps; PsPlot->linemod("solid"); }

    for(t=graph[i].ymin_rel;t<=graph[i].ymax_rel;t=t+graph[i].ydelt_rel){
      PsPlot->color(40000,40000,40000);
      yp = graph[i].yorg+(graph[i].zoomy*t); //+ changé en -
      PsPlot->fline(x1,yp,x2,yp);
      if((t<1e-5)&&(t>-1e-5)) t=0;
      sprintf(valeur, "%g", t);
      PsPlot->fmove(x1,yp);
      PsPlot->color(20000,20000,50000);
      PsPlot->alabel('r','x',valeur);
    }

    if(graph[i].gridx) { y1 = graph[i].lty+margey; y2= graph[i].lty+margey+graph[i].lgy; PsPlot->linemod("dotted"); }
    else
    {
      if (graph[i].yorg >= graph[i].lty+margey) {
	y1=graph[i].yorg-eps; y2=graph[i].yorg+eps;
      }
      else {y1=graph[i].lty+margey-eps; y2=graph[i].lty+margey+eps;}
      PsPlot->linemod("solid");
    }

    for(t=graph[i].xmin_rel;t<=graph[i].xmax_rel;t=t+graph[i].xdelt_rel)
    {
      PsPlot->color(40000,40000,40000);
      xp = graph[i].xorg+(graph[i].zoomx*t); //+ changé en -
      PsPlot->fline(xp,y1,xp,y2);
      if((t<1e-5)&&(t>-1e-5)) t=0;
      sprintf(valeur, "%g", t);
      if (graph[i].yorg >= graph[i].lty+margey) 
	PsPlot->fmove(xp,graph[i].yorg-3*eps);
      else 
	PsPlot->fmove(xp,graph[i].lty+margey-3*eps);
      PsPlot->color(20000,20000,50000);
      PsPlot->alabel('c','x',valeur);
    }

    //trace des axes
    PsPlot->color(0,0,0);
    PsPlot->linemod("solid");
    PsPlot->fline(graph[i].ltx,
		  graph[i].yorg,
		  graph[i].ltx+margex+graph[i].lgx,
		  graph[i].yorg);
    PsPlot->fline(graph[i].xorg,
		  graph[i].lty+margey,graph[i].xorg,
		  graph[i].lty+margey+graph[i].lgy);

    PsPlot->pencolor(0,0,0);
    PsPlot->fmove(graph[i].ltx+margex+graph[i].lgx/2,
		  graph[i].lty+margey+graph[i].lgy+3*eps);
    PsPlot->alabel('c','x',graph[i].title);
    PsPlot->fmove(graph[i].ltx+margex+graph[i].lgx-eps,graph[i].yorg+2*eps);
    PsPlot->alabel('r','x',graph[i].unitx);
    PsPlot->fmove(graph[i].xorg+2*eps,graph[i].lty+margey+graph[i].lgy-eps);
    PsPlot->alabel('l','x',graph[i].unity);

    for(int ind=0;ind<graph[i].curveNbr;ind++)
    {
      PsPlot->pencolor(graph[i].curveList[ind].color[0], 
		       graph[i].curveList[ind].color[1], 
		       graph[i].curveList[ind].color[2]);
      PsPlot->fmove(graph[i].ltx+margex+graph[i].lgx-eps,
		    graph[i].lty+margex+graph[i].lgy-(10*ind));
      PsPlot->alabel('r','x',graph[i].curveList[ind].legend);

      int nb = graph[i].curveList[ind].pointListx.nbElements();
      graph[i].curveList[ind].pointListx.front();
      graph[i].curveList[ind].pointListy.front();
      for(t=0;t<nb;t++)
      {
        if(t!=0)
        {
          p_1[0] = graph[i].curveList[ind].pointListx.value();
          p_1[1] = graph[i].curveList[ind].pointListy.value();
          graph[i].curveList[ind].pointListx.next();
          graph[i].curveList[ind].pointListy.next();
          x1 = graph[i].xorg + graph[i].zoomx*(*p_1);
          y1 = graph[i].yorg+(graph[i].zoomy*(*(p_1+1)));
        }
        p[0] = graph[i].curveList[ind].pointListx.value();
        p[1] = graph[i].curveList[ind].pointListy.value();
        x2 = graph[i].xorg + graph[i].zoomx*(*p);
        y2=graph[i].yorg+(graph[i].zoomy*(*(p+1))); //Changement - en +
        if(t!=0) draw(i, ind, x1,y1,x2,y2,PS);
      }
    }
  }


  PsPlot->closepl ();

  delete PsPlot;
  delete[] p;
  delete[] p_1;
}


/*!
  This functions enable to draw two lines parallels to the x axis. They corresponds to the minimum and the maximum values along the y axis.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param curveNum : The index of the curve in the list of the curves belonging to the graphic.
  \param plotType : Not usefull. Use the default value.
*/
void vpPlot::plotExtremities(const int graphNum,  const int curveNum, 
			     vpPlotType plotType)
{
  if(XPlot != NULL && plotType == X)
  {
    double yp;
    XPlot->color(40000,0,0);
    yp = graph[graphNum].yorg+(graph[graphNum].zoomy*graph[graphNum].curveList[curveNum].ymax); //+ changé en -
    XPlot->fline(graph[graphNum].ltx+margex,yp,graph[graphNum].ltx+margex+graph[graphNum].lgx,yp);
    yp = graph[graphNum].yorg+(graph[graphNum].zoomy*graph[graphNum].curveList[curveNum].ymin); //+ changé en -
    XPlot->fline(graph[graphNum].ltx+margex,yp,graph[graphNum].ltx+margex+graph[graphNum].lgx,yp);
  }
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
  ofstream fichier;
  fichier.open(dataFile);

  int ind;
  double *p = new double[2];
  bool end=false;

  fichier << graph[graphNum].title << std::endl;

  for(ind=0;ind<graph[graphNum].curveNbr;ind++)
  {
    graph[graphNum].curveList[ind].pointListx.front();
    graph[graphNum].curveList[ind].pointListy.front();
  }

  while (end == false)
  {
    end = true;
    for(ind=0;ind<graph[graphNum].curveNbr;ind++)
    {
      if (!graph[graphNum].curveList[ind].pointListy.outside() 
	  && !graph[graphNum].curveList[ind].pointListy.outside())
      {
        p[0] = graph[graphNum].curveList[ind].pointListx.value();
        p[1] = graph[graphNum].curveList[ind].pointListy.value();
        fichier << p[0] << "\t" << p[1] << "\t";
        graph[graphNum].curveList[ind].pointListx.next();
        graph[graphNum].curveList[ind].pointListy.next();
        if(!graph[graphNum].curveList[ind].pointListy.nextOutside() 
	   && !graph[graphNum].curveList[ind].pointListy.nextOutside()) 
	  end = false;
      }
      else
      {
        p[0] = graph[graphNum].curveList[ind].pointListx.previousValue();
        p[1] = graph[graphNum].curveList[ind].pointListy.previousValue();
        fichier << p[0] << "\t" << p[1] << "\t";
      }
    }
    fichier << std::endl;
  }

  delete[] p;
  fichier.close();
}

/*!
  This function enables you to add new points in all curves of a plot. These points are drawn with the parameters of the curves.

  \param graphNum : The index of the graph in the window. As the number of graphic in a window is less or equal to 4, this parameter is between 0 and 3.
  \param x : The coordinate of the new points along the x axis and given in the user unit system.
  \param v : The coordinates of the new points along the y axis and given in the user unit system.
*/
void vpPlot::plot(const int graphNum, const double x, const vpColVector v)
{
	if(graph[graphNum].curveNbr == v.getRows())
	{
		for(int i = 0;i < v.getRows();++i)
			this->plot(graphNum, i, x, v[i]);
	}
	else
		vpTRACE("error in plot vector : not the right dimension");
}


/*!
  This method clears the display and erases the list of points already displayed.
  
  The parameters of the legend and the axis are kept.
*/
void vpPlot::clear()
{
  if (XPlot != NULL)
  {
    XPlot->bgcolor(0xffff,0xffff,0xffff);
    XPlot->erase();
  }
  
  for(int i = 0; i < graphNbr; i++)
  {
    for(int j = 0; j < graph[i].curveNbr; j++)
    {
      graph[i].curveList[j].pointListx.kill();
      graph[i].curveList[j].pointListy.kill();
    }
  }
}

#endif

