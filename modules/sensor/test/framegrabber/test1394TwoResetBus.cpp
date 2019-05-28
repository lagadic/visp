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
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file test1394TwoResetBus.cpp

  \brief Resets the IEEE1394 bus using libdc1394-2.x library.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iostream>

#if defined(VISP_HAVE_DC1394)

#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

/*!
  \example test1394TwoResetBus.cpp

  Resets the IEEE1394 bus which first camera is attached to. Resetting
  the bus is "rude" to other devices because it causes them to
  re-enumerate on the bus and may cause a temporary disruption in
  their current activities.  Thus, use it sparingly.  Its primary use
  is if a program shuts down uncleanly and needs to free leftover ISO
  channels or bandwidth.  A bus reset will free those things as a side
  effect.

*/
int main()
{
  try {
    std::cout << "IEEE1394 bus reset in progress..." << std::endl;
    vp1394TwoGrabber g;
    g.resetBus(); // Reset the bus attached to the first camera found
    std::cout << "IEEE1394 bus was reset." << std::endl;

    vpImage<unsigned char> I;
    g.acquire(I);
    //     std::cout << "write /tmp/test.pgm" << std::endl;
    //     vpImageIo::write(I, "/tmp/test.pgm");
  } catch (...) {
    vpCERROR << "Failure: exit" << std::endl;
  }
}
#else
int main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available...\n"
          "You should install libdc1394-2 to use this binary.");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
