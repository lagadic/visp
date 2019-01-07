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
 * Images grabbing example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrabDirectShow.cpp

  \brief Images grabbing example with the vpDirectShowGrabber class.

 */
/*!
  \example manGrabDirectShow.cpp

  \brief Images grabbing example with the vpDirectShowGrabber class.

 */

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpDirectShowGrabber.h>

int main()
{
  try {
    vpImage<unsigned char> I; // Grey level image

#ifdef VISP_HAVE_DIRECTSHOW
    vpDirectShowGrabber g;        // Create the grabber
    if (g.getDeviceNumber() == 0) // test if a camera is connected
    {
      g.close();
      return -1;
    }

    g.open(); // Initialize the grabber

    g.setImageSize(640, 480); // If the camera supports 640x480 image size
    g.setFramerate(30);       // If the camera supports 30fps framerate

    for (;;)
      g.acquire(I); // Acquire an image
#endif

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
