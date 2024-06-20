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
 * Image acquisition with Structure Core sensor and libStructure.
 */

/*!
  \example testOccipitalStructure_Core_images.cpp
  This example shows how to retrieve images from a Occipital Structure Core sensor
  with libStructure.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && ( VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11 ) \
  && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpOccipitalStructure.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    double t;
    unsigned int display_scale = 1;
    vpOccipitalStructure sc;

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;
    settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.

    sc.open(settings);

    vpImage<float> I_depth_raw;
    vpImage<vpRGBa> I_depth;
    vpImage<vpRGBa> I_visible;

#if defined(VISP_HAVE_X11)
    vpDisplayX display_visible; // Visible image
    vpDisplayX display_depth;   // Depth image
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_visible; // Visible image
    vpDisplayGDI display_depth;   // Depth image
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
    I_visible =
      vpImage<vpRGBa>(sc.getHeight(vpOccipitalStructure::visible), sc.getWidth(vpOccipitalStructure::visible), 0);
    display_visible.setDownScalingFactor(display_scale);
    display_visible.init(I_visible, 10, 10, "Visible image");

    I_depth_raw =
      vpImage<float>(sc.getHeight(vpOccipitalStructure::depth), sc.getWidth(vpOccipitalStructure::depth), 0);
    I_depth = vpImage<vpRGBa>(sc.getHeight(vpOccipitalStructure::depth), sc.getWidth(vpOccipitalStructure::depth));
    display_depth.setDownScalingFactor(display_scale);
    display_depth.init(I_depth, static_cast<int>(I_visible.getWidth() / display_scale) + 20, 10, "Depth image");
#endif

    while (true) {
      t = vpTime::measureTimeMs();

      sc.acquire((unsigned char *)I_visible.bitmap, (unsigned char *)I_depth_raw.bitmap);

      vpDisplay::display(I_visible);
      vpDisplay::displayText(I_visible, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::flush(I_visible);

      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
      vpDisplay::display(I_depth);
      vpDisplay::displayText(I_depth, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::flush(I_depth);

      if (vpDisplay::getClick(I_visible, false) || vpDisplay::getClick(I_depth, false))
        break;

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }
  }
  catch (const vpException &e) {
    std::cerr << "Structure SDK error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_OCCIPITAL_STRUCTURE)
  std::cout << "You do not have Occipital Structure SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install libStructure, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif ( VISP_CXX_STANDARD < VISP_CXX_STANDARD_11 )
  std::cout << "You do not build ViSP with c++11 or higher compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CXX_STANDARD=11, and build again this example" << std::endl;
#elif !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
  std::cout << "You don't have X11 or GDI display capabilities" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
