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
 * Display information about a plugged RealSense sensor using librealsense2.
 *
 *****************************************************************************/

/*!
  \example getRealSense2Info.cpp
  This example displays information about a plugged RealSense sensor
  using librealsense2.
*/

#include <iostream>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_CPP11_COMPATIBILITY)

int main()
{
  try {
    vpRealSense2 rs;
    rs.open();
    std::cout << "RealSense characteristics:\n" << rs << std::endl;
    std::cout << "Depth scale: " << std::setprecision(std::numeric_limits<float>::max_digits10) << rs.getDepthScale() << std::endl;

  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "You do not have realsense2 SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install librealsense2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#endif
#if !defined(VISP_HAVE_CPP11_COMPATIBILITY)
  std::cout << "You do not build ViSP with C++11 compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CPP11=ON, and build again this example" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
