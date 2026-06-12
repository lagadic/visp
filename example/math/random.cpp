/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Random number generator example
 */
/*!
  \file random.cpp

  \brief Random number generator example
*/

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpUniRand.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpUniRand rng;
  for (int i = 0; i < 10; i++) {
    std::cout << rng.uniform(0, 6) << std::endl; // produces int values
    std::cout << rng.uniform(0.0, 6.0) << std::endl; // produces double values
  }

  std::vector<int> v;
  for (size_t i = 0; i < 10; ++i) {
    v.push_back(i);
  }

  std::vector<int> shuffled_v = vpUniRand::shuffleVector<int>(v);
  std::cout << "Original vector = [\t";
  for (size_t i = 0; i < 10; ++i) {
    std::cout << v[i] << "\t";
  }
  std::cout << "]" <<  std::endl;

  std::cout << "Shuffled vector = [\t";
  for (size_t i = 0; i < 10; ++i) {
    std::cout << shuffled_v[i] << "\t";
  }
  std::cout << "]" <<  std::endl;
}
