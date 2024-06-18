/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

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

#ifndef VP_UNIRAND_H
#define VP_UNIRAND_H

#include <visp3/core/vpConfig.h>
// Visual Studio 2010 or previous is missing inttypes.h
#if defined(_MSC_VER) && (_MSC_VER < 1700)
typedef unsigned __int64 uint64_t;
typedef unsigned __int32 uint32_t;
#else
#include <inttypes.h>
#endif

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)
#include <algorithm> // std::shuffle
#include <random>    // std::mt19937
#include <numeric>   // std::iota
#else
#include <algorithm> // std::random_shuffle
#endif

#include <vector>

BEGIN_VISP_NAMESPACE
/*!
  \class vpUniRand

  \ingroup group_core_random
  \brief Class for generating random numbers with uniform probability density.

  The algorithms and notations used are described in \cite oneill:pcg2014.

  The following example also available in random.cpp shows how to use this class to generate 10 numbers between 0 and 5.
  \include random.cpp

  Once build, this previous code should produces an output similar to the following:
  \code
  1
  0.0582619
  1
  5.84875
  1
  3.86449
  1
  0.216396
  5
  5.41692
  1
  1.65448
  0
  3.31304
  4
  2.70563
  0
  4.86741
  2
  5.65826
  Original vector = [ 0 1 2 3 4 5 6 7 8 9 ]
  Shuffled vector = [ 2 4 7 8 5 1 3 6 9 0 ]
  \endcode
*/
class VISP_EXPORT vpUniRand
{
public:
  vpUniRand();
  vpUniRand(uint64_t seed, uint64_t seq = 0x123465789ULL);

  double operator()();

  uint32_t next();
  int uniform(int a, int b);
  float uniform(float a, float b);
  double uniform(double a, double b);
  void setSeed(uint64_t initstate, uint64_t initseq);

  /**
 * @brief Create a new vector that is a shuffled version of the \b inputVector.
 *
 * @tparam T : A class that possesses a copy constructor.
 * @param inputVector : The input vector that must be shuffled. It will not be modified.
 * @return std::vector<T> A vector containing the same objects than \b inputVector, but that are shuffled.
 */
  template<typename T>
  inline static std::vector<T> shuffleVector(const std::vector<T> &inputVector)
  {
    std::vector<T> shuffled = inputVector;
#if (VISP_CXX_STANDARD <= VISP_CXX_STANDARD_11)
    std::random_shuffle(shuffled.begin(), shuffled.end());
#else
    std::shuffle(shuffled.begin(), shuffled.end(), std::mt19937 { std::random_device{}() });
#endif
    return shuffled;
  }

private:
  struct vpPcgStateSetSeq64t
  { // Internals are *Private*.
    uint64_t state;            // RNG state.  All values are possible.
    uint64_t inc;              // Controls which RNG sequence (stream) is
    // selected. Must *always* be odd.

    vpPcgStateSetSeq64t(uint64_t state_ = 0x853c49e6748fea9bULL, uint64_t inc_ = 0xda3e39cb94b95bdbULL)
      : state(state_), inc(inc_)
    { }
  };
  typedef struct vpPcgStateSetSeq64t pcg32_random_t;

private:
  uint32_t boundedRand(uint32_t bound);

  double m_maxInvDbl;
  float m_maxInvFlt;
  pcg32_random_t m_rng;
};
END_VISP_NAMESPACE
#endif
