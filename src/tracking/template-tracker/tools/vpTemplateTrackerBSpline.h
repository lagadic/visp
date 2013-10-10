/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Description:
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerBSpline.h
 \brief
*/

#ifndef vpTemplateTrackerBSpline_hh
#define vpTemplateTrackerBSpline_hh

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMath.h>

class VISP_EXPORT vpTemplateTrackerBSpline
{
public:
  static double Bspline2(double diff);
  static double Bspline3(double diff);
  static double Bspline4(double diff);

  static inline double Bspline4i(double diff,int &interv)
  {
    switch(interv)
    {
    case -1:
      return ((2.+diff)*(2.+diff)*(2.+diff)/6.);
    case 0:
      return (-diff*diff*diff/2.-diff*diff+4./6.);
    case 1:
      return (diff*diff*diff/2.-diff*diff+4./6.);
    case 2:
      return ((2.-diff)*(2.-diff)*(2.-diff)/6.);
    default:
      return 0;
    }
  }

  static double dBspline4(double diff);

  static double getSubPixBspline4(const vpImage<double> &I, double r, double t);
  static double getSubPixBspline4dx(const vpImage<unsigned char> &I, double r, double t);
  static double getSubPixBspline4dy(const vpImage<unsigned char> &I, double r, double t);

  static void PutPVBsplineD(double *Prt, int cr, double er, int ct, double et,int Nc, double val, int &degre);
  static void PutPVBsplineD2(double *Prt, int cr, double er, int ct, double et,int Nc, double val);
  static void PutPVBsplineD3(double *Prt, int cr, double er, int ct, double et,int Nc, double val);
  static void PutPVBsplineD4(double *Prt, int cr, double er, int ct, double et,int Nc, double val);
};
#endif

