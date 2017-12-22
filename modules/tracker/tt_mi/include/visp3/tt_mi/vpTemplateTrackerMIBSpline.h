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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerMIBSpline.h
 \brief
*/

#ifndef vpTemplateTrackerMIBSpline_hh
#define vpTemplateTrackerMIBSpline_hh

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/tt/vpTemplateTrackerBSpline.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>

#include <visp3/tt_mi/vpTemplateTrackerMI.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class VISP_EXPORT vpTemplateTrackerMIBSpline
{
public:
  static void PutPVBsplineD(double *Prt, int cr, double er, int ct, double et, int Nc, double val, const int &degre);
  static void PutPVBsplineD3(double *Prt, int cr, double er, int ct, double et, int Nc, double val);
  static void PutPVBsplineD4(double *Prt, int cr, double er, int ct, double et, int Nc, double val);

  static void PutTotPVBspline(double *Prt, int cr, double &er, int ct, double &et, int Nc, double *val,
                              unsigned int &NbParam, int &degree);
  static void PutTotPVBspline(double *Prt, double *dPrt, double *d2Prt, int cr, double &er, int ct, double &et, int Ncb,
                              double *val, unsigned int &NbParam, int &degree);
  static void PutTotPVBspline3(double *Prt, int cr, double &er, int ct, double &et, int Nc, double *val,
                               unsigned int &NbParam);
  static void PutTotPVBspline3(double *Prt, double *dPrt, double *d2Prt, int cr, double &er, int ct, double &et,
                               int Ncb, double *val, unsigned int &NbParam);
  static void PutTotPVBspline4(double *Prt, int cr, double er, int ct, double et, int Nc, double *val,
                               unsigned int &NbParam);
  static void PutTotPVBspline4(double *Prt, double *dPrt, double *d2Prt, int cr, double er, int ct, double et, int Ncb,
                               double *val, unsigned int &NbParam);

  // AY Optimisation
  static void PutTotPVBspline3(double *Prt, double &er, double *et, unsigned int NbParam);
  static void PutTotPVBspline4(double *Prt, double &er, double *et, unsigned int NbParam);
  //

  static void PutTotPVBsplineNoSecond(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc, double *val,
                                      unsigned int &NbParam, int &degree);
  static void PutTotPVBsplineNoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct, double &et, int &Ncb,
                                      double *val, unsigned int &NbParam, int &degree);
  static void PutTotPVBspline3NoSecond(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc, double *val,
                                       unsigned int &NbParam);
  static void PutTotPVBspline3NoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct, double &et, int &Ncb,
                                       double *val, unsigned int &NbParam);
  static void PutTotPVBspline4NoSecond(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc, double *val,
                                       unsigned int &NbParam);
  static void PutTotPVBspline4NoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct, double &et, int &Ncb,
                                       double *val, unsigned int &NbParam);

  static void PutTotPVBsplinePrtTout(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc,
                                     unsigned int &NbParam, int &degree);
  static void PutTotPVBspline3PrtTout(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc,
                                      unsigned int &NbParam);
  static void PutTotPVBspline4PrtTout(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc,
                                      unsigned int &NbParam);

  static void PutTotPVBsplinePrt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb, unsigned int &NbParam,
                                 int &degree);
  static void PutTotPVBspline3Prt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb);
  static void PutTotPVBspline4Prt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb);

  static double Bspline3(double diff);
  static double Bspline4i(double diff, int &interv);

  static double dBspline3(double diff);
  static double dBspline4(double diff);

  static double d2Bspline3(double diff);
  static double d2Bspline4(double diff);
};

#endif
#endif
