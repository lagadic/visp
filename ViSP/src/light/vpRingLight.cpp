/****************************************************************************
 *
 * $Id: vpRingLight.cpp,v 1.1 2007-09-04 09:32:19 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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



#if defined UNIX

#  include <sys/types.h>
#  include <sys/stat.h>
#  include <fcntl.h>
#  include <sys/ioctl.h>

#  include <visp/vpRingLight.h>
#  include <visp/vpDebug.h>
#  include <visp/vpTime.h>

/*!
  \file vpRingLight.cpp
  \brief Ring light management under unix.
*/

/*!

  \class vpRingLight
  \brief Ring light management under unix.

  \warning This class works only at Irisa with the Edixia's ring light system.

  \code
  vpRingLight light; // Open the device to access to the ring light.

  light.activate(); // Send a pulse to the lighting system

  \endcode

*/

/*!

  Contructor to acces to the ring light device connected to the parallel port.

  Open and initialise the default parallel port device "/dev/parport0" to
  communicate with the ring light.

  \exception vpParallelPortException::opening If the device used to access to
  the parallel port can't be opened. A possible reason is that you don't have
  write access.

*/
vpRingLight::vpRingLight()
{

}

/*!
  Destructor to close the device.

  \exception vpParallelPortException::closing If the device used to access to
  the parallel port can't be closed.
*/
vpRingLight::~vpRingLight()
{

}

/*!
  Activates the ring light by sending a pulse throw the parallel port.


*/
void vpRingLight::activate()
{
  // Output 1+,1- is connected to the D1 pin on the parallel port connector
  // Output 2+,2- is connected to the D2 pin on the parallel port connector

  // We want to control the ring light 1+,1- output

  // To activates the light we send a pulse
  int mask_d1 = 0x02;
  unsigned char data;
  data = parport.getData(); // actual value of the data bus

  data = data | mask_d1;
  //vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 0-1 pulse

  vpTime::wait(10);
  data = data & (~mask_d1);
  //vpTRACE("Send 0x%x = %d\n", data, data);
  parport.sendData(data); // send a 1-0 pulse
}



#endif // defined UNIX
