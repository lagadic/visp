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
  \file manGrabV4l2.cpp

  \brief Images grabbing example with the vpV4l2Grabber class.

 */
/*!
  \example manGrabV4l2.cpp

  \brief Images grabbing example with the vpV4l2Grabber class.

 */

#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpV4l2Grabber.h>

int main()
{
  try {
    vpImage<unsigned char> I; // Grey level image

#ifdef VISP_HAVE_V4L2
    vpV4l2Grabber g;
    g.setInput(2);    // Input 2 on the board
    g.setWidth(768);  // Acquired images are 768 width
    g.setHeight(576); // Acquired images are 576 height
    g.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
    g.open(I);        // Open the grabber
    for (;;)
      g.acquire(I); // Acquire a 768x576 grey image
#endif

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
