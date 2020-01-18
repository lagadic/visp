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
 * Pseudo random number generator.
 *
 *****************************************************************************/
 /*
  * PCG Random Number Generation for C.
  *
  * Copyright 2014 Melissa O'Neill <oneill@pcg-random.org>
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * For additional information about the PCG random number generation scheme,
  * including its license and other licensing options, visit
  *
  *     http://www.pcg-random.org
  */

  /*
   * This code is derived from the full C implementation, which is in turn
   * derived from the canonical C++ PCG implementation. The C++ version
   * has many additional features and is preferable if you can use C++ in
   * your project.
   */

#ifndef _vpUniRand_h_
#define _vpUniRand_h_

#include <visp3/core/vpConfig.h>
#include <inttypes.h>

/*!
  \class vpUniRand

  \ingroup group_core_random
  \brief Class for generating random numbers with uniform probability density.

  The algorithms and notations used are described in \cite oneill:pcg2014.

  The following example shows how to use this class to generate 10 numbers between 0 and 5.
\code
#include <iostream>
#include <visp3/core/vpUniRand.h>

int main()
{
  vpUniRand rng;
  for (int i = 0; i < 10; i++) {
    std::cout << rng.uniform(0, 6) << std::endl; // produces int values
    std::cout << rng.uniform(0.0, 6.0) << std::endl; // produces double values
  }
}
\endcode
*/
class VISP_EXPORT vpUniRand
{
private:
  struct pcg_state_setseq_64 {  // Internals are *Private*.
    uint64_t state;             // RNG state.  All values are possible.
    uint64_t inc;               // Controls which RNG sequence (stream) is
                                // selected. Must *always* be odd.

    pcg_state_setseq_64(uint64_t state_ = 0x853c49e6748fea9bULL, uint64_t inc_ = 0xda3e39cb94b95bdbULL) :
      state(state_), inc(inc_)
    {
    }
  };
  typedef struct pcg_state_setseq_64 pcg32_random_t;

public:
  vpUniRand();
  vpUniRand(uint64_t seed, uint64_t seq=0x123465789ULL);

  double operator()();

  uint32_t next();
  int uniform(int a, int b);
  float uniform(float a, float b);
  double uniform(double a, double b);
  void setSeed(uint64_t initstate, uint64_t initseq);

private:
  uint32_t boundedRand(uint32_t bound);

  double m_maxInvDbl;
  float m_maxInvFlt;
  pcg32_random_t m_rng;
};

#endif
