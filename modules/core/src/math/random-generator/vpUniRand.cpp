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

// To ensure UINT32_MAX, INT32_MX are defined on centos, ubuntu 12.04 we define __STDC_LIMIT_MACROS

#define __STDC_LIMIT_MACROS

#include <stdint.h>
#include <visp3/core/vpUniRand.h>

BEGIN_VISP_NAMESPACE
vpUniRand::vpUniRand()
  : m_maxInvDbl(1.0 / static_cast<double>(UINT32_MAX)), m_maxInvFlt(1.0f / static_cast<float>(UINT32_MAX)), m_rng()
{ }

/*!
  Create a pseudorandom number generator with uniform distribution.
  \param seed : Starting state for the RNG, you can pass any 64-bit value.
  \param seq : Select the output sequence for the RNG, you can pass any 64-bit value,
  although only the low 63 bits are significant.

  \sa setSeed
*/
vpUniRand::vpUniRand(uint64_t seed, uint64_t seq)
  : m_maxInvDbl(1.0 / static_cast<double>(UINT32_MAX)), m_maxInvFlt(1.0f / static_cast<float>(UINT32_MAX)), m_rng()
{
  setSeed(seed, seq);
}

/*!
  Generates a pseudorandom uniformly distributed double number between [0, 1) range.
  This is equivalent to call uniform(0.0, 1.0);
*/
double vpUniRand::operator()() { return uniform(0.0, 1.0); }

/*!
  Generates a uniformly distributed 32-bit unsigned integer less than bound
  (i.e., x where 0 <= x < bound).

  \note
  <quote>
  Some programmers may think that they can just run rng.next() % bound,
  but doing so introduces nonuniformity when bound is not a power of two.
  The code for boundedRand() avoids the nonuniformity by dropping a portion
  of the RNG's output.
  </quote>
*/
uint32_t vpUniRand::boundedRand(uint32_t bound)
{
  // To avoid bias, we need to make the range of the RNG a multiple of
  // bound, which we do by dropping output less than a threshold.
  // A naive scheme to calculate the threshold would be to do
  //
  //     uint32_t threshold = 0x100000000ull % bound;
  //
  // but 64-bit div/mod is slower than 32-bit div/mod (especially on
  // 32-bit platforms).  In essence, we do
  //
  //     uint32_t threshold = (0x100000000ull-bound) % bound;
  //
  // because this version will calculate the same modulus, but the LHS
  // value is less than 2^32.

  uint32_t threshold = -bound % bound;

  // Uniformity guarantees that this loop will terminate.  In practice, it
  // should usually terminate quickly; on average (assuming all bounds are
  // equally likely), 82.25% of the time, we can expect it to require just
  // one iteration.  In the worst case, someone passes a bound of 2^31 + 1
  // (i.e., 2147483649), which invalidates almost 50% of the range.  In
  // practice, bounds are typically small and only a tiny amount of the range
  // is eliminated.
  for (;;) {
    uint32_t r = next();
    if (r >= threshold) {
      return r % bound;
    }
  }
}

/*!
  Generates a pseudorandom uniformly distributed 32-bit unsigned integer
  (i.e., x where, 0 <= x < 2^32)
*/
uint32_t vpUniRand::next()
{
  uint64_t oldstate = m_rng.state;
  m_rng.state = (oldstate * 6364136223846793005ULL) + m_rng.inc;
  uint32_t xorshifted = static_cast<uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
  uint32_t rot = oldstate >> 59u;
  return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

/*!
  Generates a pseudorandom uniformly distributed int number between [a, b) range.
  \param a : lower inclusive boundary of the returned random number.
  \param b : upper non-inclusive boundary of the returned random number.
*/
int vpUniRand::uniform(int a, int b)
{
  if (a == b) {
    return a;
  }
  return boundedRand(b - a) + a;
}

/*!
  Generates a pseudorandom uniformly distributed float number between [a, b) range.
  \param a : lower inclusive boundary of the returned random number.
  \param b : upper non-inclusive boundary of the returned random number.
*/
float vpUniRand::uniform(float a, float b) { return (next() * m_maxInvFlt * (b - a)) + a; }

/*!
  Generates a pseudorandom uniformly distributed double number between [a, b) range.
  \param a : lower inclusive boundary of the returned random number.
  \param b : upper non-inclusive boundary of the returned random number.
*/
double vpUniRand::uniform(double a, double b) { return (next() * m_maxInvDbl * (b - a)) + a; }

/*!
  Initialize the random number generator.
  \param initstate : Starting state for the RNG, you can pass any 64-bit value.
  \param initseq : Select the output sequence for the RNG, you can pass any 64-bit value,
  although only the low 63 bits are significant.

  \note
  > For this generator, there are 2^63 possible sequences of pseudorandom numbers.
  > Each sequence is entirely distinct and has a period of 2^64.
  > The initseq argument selects which stream you will use.
  > The initstate argument specifies where you are in that 264 period.
  >
  > Calling setSeed with the same arguments produces the same output,
  > allowing programs to use random number sequences repeatably.
  >
  > If you want truly nondeterministic output for each run of your program,
  > you should pass values that will be different from run to run.
  > On a Unix system, /dev/random provides random bytes that can be used for initialization,
  > but if you want a quick and dirty way to do the initialization,
  > one option is to pass the current time and the address of the RNG itself.
*/
void vpUniRand::setSeed(uint64_t initstate, uint64_t initseq)
{
  m_rng.state = 0U;
  m_rng.inc = (initseq << 1u) | 1u;
  next();
  m_rng.state += initstate;
  next();
}
END_VISP_NAMESPACE
