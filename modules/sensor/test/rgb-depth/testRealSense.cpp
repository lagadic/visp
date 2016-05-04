/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test RealSense RGB-D sensor.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRealSense.cpp
  This example shows how to retrieve data from a RealSense RGB-D sensor.

*/

#include <iostream>

#include <visp3/sensor/vpRealSense.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>


int main()
{
//  u_int16_t y16;
//  std::cout << "size y16: " << sizeof(y16) << " " << (2<<15) << std::endl;
//  int val = 65520;

//  vpImage<u_int16_t> Y16(480, 640, val);
//  vpImage<unsigned char> Y8(480, 640);
//  vpImageConvert::MONO16ToGrey((unsigned char*)Y16.bitmap, (unsigned char*)Y8.bitmap, Y8.getSize());
//  vpImageIo::write(Y8, "Y8.png");
//  for(unsigned int i=0; i< Y16.getHeight(); i++)
//    for(unsigned int j=0; j< Y16.getWidth(); j++)
//      Y8[i][j] = Y16[i][j] >> 8;
//  vpImageIo::write(Y8, "Y8-new.png");

//  return 0;


#ifdef VISP_HAVE_REALSENSE
  try {
    vpRealSense rs;
    //rs.setDeviceBySerialNumber("541142003219");
    rs.open();

    vpImage<vpRGBa> color;
    vpImage<u_int16_t> infrared;
    vpImage<unsigned char> infrared_display;
    vpImage<u_int16_t> depth;
    vpImage<vpRGBa> depth_display;
    std::vector<vpPoint3dTextured> point_cloud;

    rs.acquire(color, infrared, depth, point_cloud);
    vpImageConvert::convert(infrared, infrared_display);
    vpImageConvert::createDepthHistogram(depth, depth_display);

    std::cout << "DBG: Infrared: " << infrared.getWidth() << " " << infrared.getHeight() << std::endl;
    vpImageIo::write(infrared_display, "infrared.png");

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(color, 10, 10, "Color image");
    vpDisplayX di(infrared_display, color.getWidth()+80, 10, "Infrared image");
    vpDisplayX dd(depth_display, 10, color.getHeight()+80, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(color, 10, 10, "Color image");
    vpDisplayGDI di(infrared_display, color.getWidth()+80, 10, "Infrared image");
    vpDisplayGDI dd(depth_display, 10, color.getHeight()+80, "Depth image");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      rs.acquire(color, infrared, depth, point_cloud);

      vpImageConvert::convert(infrared, infrared_display);
      vpImageConvert::createDepthHistogram(depth, depth_display);

      vpDisplay::display(color);
      vpDisplay::display(infrared_display);
      vpDisplay::display(depth_display);
      vpDisplay::displayText(color, 15, 15, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(color, false) || vpDisplay::getClick(infrared_display, false) || vpDisplay::getClick(depth_display, false))
        break;
      vpDisplay::flush(color);
      vpDisplay::flush(infrared_display);
      vpDisplay::flush(depth_display);
    }

    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

    rs.close();
  }
  catch(const vpException &e) {
    std::cerr << "RealSense error " << e.getStringMessage() << std::endl;
  }
  catch(const rs::error & e)  {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
  }
  catch(const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

#else
  std::cout << "Install RealSense SDK to make this test working" << std::endl;
#endif
  return 0;
}

