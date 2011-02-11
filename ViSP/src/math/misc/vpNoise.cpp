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
 * Generation of random number with uniform and normal probability density.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp/vpNoise.h>
#include <math.h>

/*!
  \file vpNoise.cpp
  \brief Classe for generating random number
  with uniform and normal probability density

  The algorithms and notations used are described in

  James E. Gentle, Random Number Generation and Monte Carlo Methods,
  Springer 1998
*/


/*!  \brief Minimal random number generator of Park and Miller. Returns a
  uniform random deviate between 0.0 and 1.0.

  S.K. Park and K.W. Miller, " Random Number Generators: Good Ones Are Hard To
  Find", Communications of the ACM, October 1988, pp. 1192-1201.


*/
inline void
vpUniRand::draw0()
//minimal standard
//Park and Miller random number generator
{
  /*unsigned*/ long k= x/q;//temp value for computing without overflow
  x = a*(x-k*q)-k*r;
  if (x < 0) x += m; //compute x without overflow
}

/*!
  \brief Bays-Durham Shuffling of Park-Miller generator

  Minimal random number generator of Park and Miller with Bays-Durham
  shuffle. Returns a uniform random deviate between 0.0 and 1.0 (exclusive of
  the endpoint values).
*/
double
vpUniRand::draw1()
{
  const long ntab = 33;  //we work on a 32 elements array.
                                  //the 33rd one is actually the first value of y.
  const long modulo = ntab-2;

  static long y = 0;
  static long T[ntab];

  long j; //index of T

  //step 0
  if (!y) { //first time
    for(j = 0; j < ntab; j++) {
      draw0();
      T[j]=x;
    } //compute table T
    y=T[ntab-1];
  }

  //step 1
  j = y & modulo; //compute modulo ntab+1 (the first element is the 0th)

  //step 3
  y=T[j];
  double ans = (double)y/normalizer;

  //step 4
  //generate x(k+i) and set y=x(k+i)
  draw0();

  //refresh T[j];
  T[j]=x;

  return ans;
}

/*!
  \brief Generate a normal random variable using the Box-Muller generator

  Generate a normal random variable with mean 0 and standard deviation of 1.
   To adjust to some other distribution, multiply by the standard deviation and
   add the mean.  Box-Muller method
*/
double
vpGaussRand::gaussianDraw()
{
  double v1, v2, rsq;
  static bool AlreadyDone = false;
  static double x2;

  if (AlreadyDone) {
    AlreadyDone=false;
    return x2;
  }

  else {

    do {
      v1=2*draw1()-1;
      v2=2*draw1()-1;
      rsq=v1*v1+v2*v2;
    } while (rsq >= 1);

  double fac=sqrt(-2*log(rsq)/rsq);
  x2=v2*fac;
  AlreadyDone=true;
  return v1*fac;
  }
}

