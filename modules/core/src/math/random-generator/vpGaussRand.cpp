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
 * Generation of random number with uniform and normal probability density.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <math.h>
#include <visp3/core/vpGaussRand.h>

/*!
  Generate a normal random variable using the Box-Muller generator.

  Generate a normal random variable with mean 0 and standard deviation of 1.
  To adjust to some other distribution, multiply by the standard deviation and
  add the mean.  Box-Muller method
*/
double vpGaussRand::gaussianDraw()
{
  static bool AlreadyDone = false;
  static double x2;

  if (AlreadyDone) {
    AlreadyDone = false;
    return x2;
  }

  else {
    double v1 = 0, v2 = 0, rsq = 0;
    do {
      v1 = 2 * draw1() - 1;
      v2 = 2 * draw1() - 1;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1);

    double fac = sqrt(-2 * log(rsq) / rsq);
    x2 = v2 * fac;
    AlreadyDone = true;
    return v1 * fac;
  }
}
