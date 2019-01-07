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
 * Ring light management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_IO) && defined(VISP_HAVE_PARPORT)

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRingLight.h>

/*!
  \file vpRingLight.cpp
  \brief Ring light management under unix.
*/

/*!

  Constructor to acces to the ring light device connected to the parallel
  port.

  Open and initialise the default parallel port device "/dev/parport0" to
  communicate with the ring light.

  \exception vpParallelPortException::opening If the device used to access to
  the parallel port can't be opened. A possible reason is that you don't have
  write access.

  Turn the ring light off.
*/
vpRingLight::vpRingLight() : parport() { off(); }

/*!
  Destructor to close the device.

  Turn the ring light off.

  \exception vpParallelPortException::closing If the device used to access to
  the parallel port can't be closed.
*/
vpRingLight::~vpRingLight() { off(); }

/*!
  Activates the ring light by sending a pulse throw the parallel port.

  The pulse width is 500 us. This pulse activates a NE555 which turns on on
  the light during 10ms.

*/
void vpRingLight::pulse()
{
  // Data set by the parallel port:
  // - D1: need to send a 500us pulse width
  // - D2: 0 }
  // - D3: 0 } To control the light throw the NE555
  // D2 and D3 are used to select the multiplexer output.
  // Light must be connected to output 1+,1-

  // To activates the light we send a pulse
  int mask_mode_pulse_d2 = 0x00; // D2 is low
  int mask_pulse_d1 = 0x02;      // we send a pulse on D1 : L, H, L
  unsigned char data = 0x00;
  //  data = parport.getData(); // actual value of the data bus
  // vpTRACE("Actual data 0x%x = %d\n", data, data);

  data = data | mask_pulse_d1 | mask_mode_pulse_d2;
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 0-1 pulse

  // Wait 500 micro seconds
  struct timeval ti, tc; // Initial and current time
  struct timeval tempo;
  tempo.tv_usec = 500;
  gettimeofday(&ti, 0L);
  do {
    gettimeofday(&tc, 0L);
  } while (tc.tv_usec < ti.tv_usec + tempo.tv_usec);

  data = data & (~mask_pulse_d1);
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 1-0 pulse
}

/*!

  Activates the ring light by sending a pulse throw the parallel port during a
  specified duration.

  \param time : Duration in milli-second (ms) during while the light is turned
  on.

*/
void vpRingLight::pulse(double time)
{
  // Data set by the parallel port:
  // - D1: a pulse with duration fixed by time
  // - D2: 0 }
  // - D3: 1 } To control the light directly throw the pulse comming from D1
  // D2 and D3 are used to select the multiplexer output.
  // Light must be connected to output 1+,1-

  // To activates the light we send a pulse
  int mask_mode_pulse_d3 = 0x08; // D3 is hight, D2 is low
  int mask_pulse_d1 = 0x02;      // we send a pulse on D1 : L, H, L
  unsigned char data = 0x00;
  //  data = parport.getData(); // actual value of the data bus
  // vpTRACE("Actual data 0x%x = %d\n", data, data);

  data = data | mask_pulse_d1 | mask_mode_pulse_d3;
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 0-1 pulse

  // Wait 500 micro seconds
  struct timeval ti, tc; // Initial and current time
  gettimeofday(&ti, 0);
  do {
    gettimeofday(&tc, 0);
  } while (tc.tv_usec < ti.tv_usec + time * 1000);

  data = data & (~mask_pulse_d1);
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 1-0 pulse
}

/*!
  Turn the ring light on.

  To turn the ring light off, see off().

*/
void vpRingLight::on()
{
  // Data set by the parallel port:
  // - D1: 0 to turn OFF, 1 to turn ON
  // - D2: 1 }
  // - D3: 0 } To control the light throw D1
  // D2 and D3 are used to select the multiplexer output.
  // Light must be connected to output 1+,1-

  // To activates the light we send a pulse
  int mask_mode_onoff_d2 = 0x04; // D2 is Hight
  int mask_on_d1 = 0x02;         // D1 is Hight to turn the light on
  unsigned char data = 0x00;
  // data = parport.getData(); // actual value of the data bus

  data = data | mask_on_d1 | mask_mode_onoff_d2;
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data);
}

/*!
  Turn the ring light off.

  To turn the ring light on, see on().

*/
void vpRingLight::off()
{
  // Data set by the parallel port:
  // - D1: 0 to turn OFF, 1 to turn ON
  // - D2: 1 }
  // - D3: 0 } To control the light throw D1
  // D2 and D3 are used to select the multiplexer output.
  // Light must be connected to output 1+,1-

  // To activates the light we send a pulse
  int mask_mode_onoff_d2 = 0x04; // D2 is Hight
  int mask_off_d1 = 0x00;        // D1 is Low to turn the light off
  unsigned char data = 0x00;
  // data = parport.getData(); // actual value of the data bus

  data = data | mask_off_d1 | mask_mode_onoff_d2;
  // vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRingLight.cpp.o) has no
// symbols
void dummy_vpRingLight(){};
#endif
