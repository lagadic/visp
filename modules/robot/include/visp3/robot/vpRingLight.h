/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Ring light management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRingLight_h
#define vpRingLight_h

/*!
  \file vpRingLight.h
  \brief Ring light management under unix.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_IO) && defined(VISP_HAVE_PARPORT)

#include <iostream>

#include <visp3/io/vpParallelPort.h>
#include <visp3/robot/vpRingLight.h>
/*!

  \class vpRingLight
  \ingroup group_robot_real_gantry
  \brief Ring light management under unix.

  \warning This class works only at Irisa with the Edixia's ring light system.

  Here is an example showing how to synchronise the framegrabbing with
  the lighting system.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRingLight.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_PARPORT) && defined(VISP_HAVE_DC1394)
  vp1394TwoGrabber g; // Firewire framegrabber based on libdc1394-2.x third party lib
  vpImage<unsigned char> I;

  vpRingLight light; // Open the device to access to the ring light.

  for (int i=0; i < 10; i++) {
    light.pulse(); // Send a pulse to the lighting system
    g.acquire(I); // Acquire an image
  }
#endif
}
  \endcode

  Here is an example showing how to turn on the lighting during 10 seconds.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRingLight.h>

int main()
{
#ifdef VISP_HAVE_PARPORT
  vpRingLight light;         // Open the device to access to the ring light.

  int nsec = 10;             // Time to wait in seconds
  light.on();                // Turn the ring light on
  vpTime::wait(nsec * 1000); // Wait 10 s
  light.off();               // and then turn the ring light off
#endif
}
  \endcode



*/
class VISP_EXPORT vpRingLight
{

public:
  vpRingLight();
  ~vpRingLight();

  void pulse();
  void pulse(double time);
  void on();
  void off();

private:
  vpParallelPort parport;
};

#endif

#endif
