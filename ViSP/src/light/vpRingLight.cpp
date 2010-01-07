/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Ring light management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#  include <visp/vpConfig.h>

#ifdef VISP_HAVE_PARPORT

#  include <sys/types.h>
#  include <sys/stat.h>
#  include <fcntl.h>
#  include <sys/ioctl.h>
#  include <sys/time.h>
#  include <unistd.h>

#  include <visp/vpRingLight.h>
#  include <visp/vpDebug.h>
#  include <visp/vpTime.h>

/*!
  \file vpRingLight.cpp
  \brief Ring light management under unix.
*/


/*!

  Contructor to acces to the ring light device connected to the parallel port.

  Open and initialise the default parallel port device "/dev/parport0" to
  communicate with the ring light.

  \exception vpParallelPortException::opening If the device used to access to
  the parallel port can't be opened. A possible reason is that you don't have
  write access.

  Turn the ring light off.
*/
vpRingLight::vpRingLight()
{
  off();
}

/*!
  Destructor to close the device.

  Turn the ring light off.

  \exception vpParallelPortException::closing If the device used to access to
  the parallel port can't be closed.
*/
vpRingLight::~vpRingLight()
{
  off();
}

/*!
  Activates the ring light by sending a pulse throw the parallel port.

  The pulse width is 500 us. This pulse activates a NE555 which turns on on the
  light during 10ms.

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
  //vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 0-1 pulse

  // Wait 500 micro seconds
  int usTempo = 500;
  struct timeval ti, tc; // Initial and current time
  gettimeofday(&ti,0);
  do {
    gettimeofday(&tc,0);
  } while (tc.tv_usec < ti.tv_usec + usTempo);

  data = data & (~mask_pulse_d1);
  //vpTRACE("Send 0x%x = %d\n", data, data);
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
  //vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 0-1 pulse

  // Wait 500 micro seconds
  struct timeval ti, tc; // Initial and current time
  gettimeofday(&ti,0);
  do {
    gettimeofday(&tc,0);
  } while (tc.tv_usec < ti.tv_usec + time*1000);

  data = data & (~mask_pulse_d1);
  //vpTRACE("Send 0x%x = %d\n", data, data);
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
  int mask_on_d1 = 0x02;      // D1 is Hight to turn the light on
  unsigned char data = 0x00;
  //data = parport.getData(); // actual value of the data bus

  data = data | mask_on_d1 | mask_mode_onoff_d2;
  //vpTRACE("Send 0x%x = %d\n", data, data);
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
  int mask_off_d1 = 0x00;      // D1 is Low to turn the light off
  unsigned char data = 0x00;
  //data = parport.getData(); // actual value of the data bus

  data = data | mask_off_d1 | mask_mode_onoff_d2;
  //vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data);
}

#endif
