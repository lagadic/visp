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
 * Test vpRect.
 *
 *****************************************************************************/

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRect.h>

int main()
{
  vpRect c(10.1, 15.05, 19.63, 7.84);
  vpRect a(c.getLeft() - 12.456, c.getTop() - 7.75, c.getWidth() + 12.456, c.getHeight() + 7.75);
  vpRect b(c.getLeft(), c.getTop(), c.getWidth() + 8.81, c.getHeight() + 14.57);

  vpRect intersect = a & b;

  std::cout << "Test intersection." << std::endl;
  std::cout << "a=" << a << std::endl;
  std::cout << "b=" << b << std::endl;
  std::cout << "c=" << c << std::endl;
  std::cout << "intersect=" << intersect << std::endl;

  if (intersect != c) {
    std::cerr << "Problem with the intersection function!" << std::endl;
    return EXIT_FAILURE;
  }

  a &= b;
  std::cout << "a=" << a << std::endl;

  if (a != c) {
    std::cerr << "Problem with the intersection function!" << std::endl;
    return EXIT_FAILURE;
  }

  a = vpRect(11.45, 15.67, 3.5, 7.81);
  b = vpRect(24.78, 31.46, 7.48, 11.28);
  intersect = a & b;

  std::cout << "\nTest no intersection." << std::endl;
  std::cout << "a=" << a << std::endl;
  std::cout << "b=" << b << std::endl;
  std::cout << "intersect=" << intersect << std::endl;

  if (intersect != vpRect()) {
    std::cerr << "Problem with the intersection function!" << std::endl;
    return EXIT_FAILURE;
  }

  a &= b;
  std::cout << "a=" << a << std::endl;

  if (a != vpRect()) {
    std::cerr << "Problem with the intersection function!" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "vpRect is ok." << std::endl;
  return EXIT_SUCCESS;
}
