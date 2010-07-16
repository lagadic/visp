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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>
#include <visp/vpPlotCurve.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) 
vpPlotCurve::vpPlotCurve()
{
  color = vpColor::red;
  pointListx.front();
  pointListy.front();
  pointListz.front();
  nbPoint = 0;
}

vpPlotCurve::~vpPlotCurve()
{
  pointListx.kill();
  pointListy.kill();
}

void
vpPlotCurve::plotPoint(vpImage<unsigned char> &I, vpImagePoint iP, const double x, const double y)
{
  pointListx.end();
  pointListy.end();
  pointListz.end();
  
  if(nbPoint)
  {
    vpDisplay::displayLine(I,lastPoint, iP, color);
  }
#if( defined VISP_HAVE_X11 || defined VISP_HAVE_GDI )
  double top;
  double left;
  double width;
  double height;
  
  if (iP.get_i() <= lastPoint.get_i()) {top = iP.get_i()-5; height = lastPoint.get_i() - top+10;}
  else {top = lastPoint.get_i()-5; height = iP.get_i() - top+10;}
  if (iP.get_j() <= lastPoint.get_j()) {left = iP.get_j()-5; width = lastPoint.get_j() - left+10;}
  else {left = lastPoint.get_j()-5; width = iP.get_j() - left+10;}
  vpDisplay::flushROI(I,vpRect(left,top,width,height));
#endif
  lastPoint = iP;
  pointListx.addRight(x);
  pointListy.addRight(y);
  pointListz.addRight(0.0);
  nbPoint++;
  
  
}

void 
vpPlotCurve::plotList(vpImage<unsigned char> &I, const double xorg, const double yorg, const double zoomx, const double zoomy)
{
  pointListx.front();
  pointListy.front();
  
  int k = 0;
  vpImagePoint iP;
  while (k < nbPoint)
  {
    iP.set_ij(yorg-(zoomy*pointListy.value()),xorg+(zoomx*pointListx.value()));
    
    if (k > 0)
      vpDisplay::displayLine(I,lastPoint, iP, color);
    
    lastPoint = iP;
    
    pointListx.next();
    pointListy.next();
    k++;
  }
}

#endif
#endif
