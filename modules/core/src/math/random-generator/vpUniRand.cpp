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
#include <visp3/core/vpUniRand.h>

/*!
  Minimal random number generator of Park and Miller \cite Park:1988. Returns
  a uniform random deviate between 0.0 and 1.0.

*/
inline void vpUniRand::draw0()
{
  long k = x / q; // temp value for computing without overflow
  x = a * (x - k * q) - k * r;
  if (x < 0)
    x += m; // compute x without overflow
}

/*!
  Bays-Durham Shuffling of Park-Miller generator.

  Minimal random number generator of Park and Miller with Bays-Durham
  shuffle. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of
  the endpoint values).
*/
double vpUniRand::draw1()
{
  const long ntab = 33; // we work on a 32 elements array.
                        // the 33rd one is actually the first value of y.
  const long modulo = ntab - 2;

  static long y = 0;
  static long T[ntab];

  long j; // index of T

  // step 0
  if (!y) { // first time
    for (j = 0; j < ntab; j++) {
      draw0();
      T[j] = x;
    } // compute table T
    y = T[ntab - 1];
  }

  // step 1
  j = y & modulo; // compute modulo ntab+1 (the first element is the 0th)

  // step 3
  y = T[j];
  double ans = (double)y / normalizer;

  // step 4
  // generate x(k+i) and set y=x(k+i)
  draw0();

  // refresh T[j];
  T[j] = x;

  return ans;
}
