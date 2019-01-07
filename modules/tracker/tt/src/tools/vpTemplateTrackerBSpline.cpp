/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
#include <visp3/tt/vpTemplateTrackerBSpline.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

double vpTemplateTrackerBSpline::getSubPixBspline4(const vpImage<double> &I, double r, double t)
{
  double res = 0;
  int cr = (int)(r);
  int ct = (int)(t);
  double er = (double)r - cr;
  double et = (double)t - ct;
  int height = (int)I.getHeight(); // r
  int width = (int)I.getWidth();   // t
  for (int ir = -1; ir <= 2; ir++) {
    int tr = ir + cr;
    for (int it = -1; it <= 2; it++) {
      int tt = it + ct;
      if (tr >= 0 && tr < height && tt >= 0 && tt < width)
        res += Bspline4((double)ir - er) * Bspline4((double)it - et) * I[tr][tt];
    }
  }
  return res;
}

double vpTemplateTrackerBSpline::Bspline4(double diff)
{
  // double result;
  double aDiff = vpMath::abs(diff);
  if (aDiff < 1.)
    return (aDiff * aDiff * aDiff / 2. - aDiff * aDiff + 4. / 6.);
  // return
  // (0.5*(1.-aDiff)*(1.-aDiff)*(1.-aDiff)+0.5*(1.-aDiff)*(1.-aDiff)-0.5*(1.-aDiff)+1./6.);
  else if (aDiff < 2.)
    return ((2. - aDiff) * (2. - aDiff) * (2. - aDiff) / 6.);
  else
    return 0;
}

#endif
