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
 * Generation of random number with uniform and normal probability density.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpUniRand_hh
#define vpUniRand_hh

#include <visp3/core/vpConfig.h>

/*!
  \class vpUniRand

  \ingroup group_core_random
  \brief Class for generating random numbers with uniform probability density.

  The algorithms and notations used are described in \cite Gentle:2004.

  The following example shows how to use this class to generate 10 numbers between 0 and 5.
\code
#include <iostream>
#include <visp3/core/vpUniRand.h>

int main()
{
  vpUniRand r;
  for(unsigned int i=0;i<10;++i)
    std::cout << 5*r() << std::endl;
}
\endcode
 */
class VISP_EXPORT vpUniRand
{
  long a;
  long m;            // 2^31-1
  long q;            // integer part of m/a
  long r;            // r=m mod a
  double normalizer; // we use a normalizer > m to ensure ans will never be 1
                     // (it is the case if x = 739806647)

private:
  void draw0();

protected:
  long x;
  double draw1();

public:
  //! Default constructor.
  explicit vpUniRand(const long seed = 0)
    : a(16807), m(2147483647), q(127773), r(2836), normalizer(2147484721.0), x((seed) ? seed : 739806647)
  {
  }

  //! Default destructor.
  virtual ~vpUniRand(){};

  //! Operator that allows to get a random value.
  double operator()() { return draw1(); }
};

#endif
