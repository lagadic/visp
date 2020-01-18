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
 * Parallel port management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_PARPORT

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <visp3/io/vpParallelPort.h>

/*!
  \file vpParallelPort.cpp
  \brief Parallel port management under unix.
*/

static unsigned char vpParallelPortData;

/*!

  Constructor to acces to a parallel port.

  Open and initialise the parallel port by sending 0 to the data bus.

  Set the default device to "/dev/parport0".

  \exception vpParallelPortException::opening If the device used to access to
  the parallel port can't be opened. A possible reason is that you don't have
  write access.

*/
vpParallelPort::vpParallelPort() : fd(0), device()
{
  sprintf(device, "/dev/parport0");

  this->open();

  unsigned char data = 0;
  this->sendData(data);
}

/*!
  Destructor to close the parallel port.

  \exception vpParallelPortException::closing If the device used to access to
  the parallel port can't be closed.
*/
vpParallelPort::~vpParallelPort() { this->close(); }

/*!
  Open the parallel port with write access and initialise it.

  \exception vpParallelPortException::opening If the device used to access to
  the parallel port can't be opened. A possible reason is that you don't have
  write access.

  \sa close()
*/
void vpParallelPort::open()
{
  fd = ::open("/dev/parport0", O_WRONLY);
  if (fd < 0) {
    printf("Can't open /dev/parport0\n");
    printf("Check if you have write access to /dev/parport0\n");
    perror("Open parallel port");
    throw(vpParallelPortException(vpParallelPortException::opening, "Can't open /dev/parport0"));
  }

  int i;

  ioctl(fd, PPCLAIM);
  i = PARPORT_MODE_COMPAT;
  ioctl(fd, PPSETMODE, &i);
  i = IEEE1284_MODE_COMPAT;
  ioctl(fd, PPNEGOT, &i);
}

/*!
  Send a data to the parallel port.

  \param data : Value [D7, D6, ... D0] to send to the data bus.

  The code bellow shows how to set D0 and D2 to logical level 1:

  \code
  vpParallelPort parport;

  unsigned char data = 5; // 0x00000101 = 5 in decimal
  parport.sendData(data); // D0 and D2 are set to logical level 1

  \endcode

  \return true if the device was close, false if an error occurs.
*/
void vpParallelPort::sendData(unsigned char &data)
{
  ioctl(fd, PPWDATA, &data);

  // Memorise the last sended data to the static variable
  vpParallelPortData = data;
}

/*!

  Get the last data sent to the parallel port.

*/
unsigned char vpParallelPort::getData() { return vpParallelPortData; }

/*!
  Close the parallel port.

  \exception vpParallelPortException::closing If the device used to access to
  the parallel port can't be closed.

*/
void vpParallelPort::close()
{
  ioctl(fd, PPRELEASE);

  int err;
  err = ::close(fd);

  if (err != 0) {
    printf("Can't close the parallel port\n");
    throw(vpParallelPortException(vpParallelPortException::closing, "Can't close the parallel port"));
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpParallelPort.cpp.o) has no
// symbols
void dummy_vpParallelPort(){};
#endif
