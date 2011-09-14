/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
  pointListx.clear();
  pointListy.clear();
  pointListz.clear();
  nbPoint = 0;
  thickness = 1 ;
}

vpPlotCurve::~vpPlotCurve()
{
  pointListx.clear();
  pointListy.clear();
  pointListz.clear();
}

void
vpPlotCurve::plotPoint(vpImage<unsigned char> &I, vpImagePoint iP, const double x, const double y)
{  
  nbPoint++;
  
  if (nbPoint > 1)
  {
    vpDisplay::displayLine(I,lastPoint, iP, color, thickness);
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
  pointListx.push_back(x);
  pointListy.push_back(y);
  pointListz.push_back(0.0);
}

void 
vpPlotCurve::plotList(vpImage<unsigned char> &I, const double xorg, const double yorg, const double zoomx, const double zoomy)
{
  std::list<double>::const_iterator it_ptListx = pointListx.begin();
  std::list<double>::const_iterator it_ptListy = pointListy.begin();
  
  unsigned int k = 0;
  vpImagePoint iP;
  while (k < nbPoint)
  {
    iP.set_ij(yorg-(zoomy*(*it_ptListy)),xorg+(zoomx*(*it_ptListx)));
    
    if (k > 0)
      vpDisplay::displayLine(I,lastPoint, iP, color, thickness);
    
    lastPoint = iP;
    
    ++it_ptListx;
    ++it_ptListy;
    k++;
  }
}

#endif
#endif
