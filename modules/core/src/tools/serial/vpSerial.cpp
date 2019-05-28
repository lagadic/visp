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
 * Serial communication.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32)

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <visp3/core/vpSerial.h>
#include <visp3/core/vpException.h>

#ifndef TIOCINQ
#  ifdef FIONREAD
#    define TIOCINQ FIONREAD
#  else
#    define TIOCINQ 0x541B
#  endif
#endif

/*!
  Creates a serial port object that opens the port if the parameter is not empty.
  \code
#include <visp3/core/vpSerial.h>

int main()
{
#ifndef WIN32
  vpSerial serial("/dev/ttyUSB0");
#endif
}
  \endcode

  Otherwise the port needs to be opened using open().
  \code
#include <visp3/core/vpSerial.h>

int main()
{
#ifndef WIN32
  vpSerial serial;
  serial.setPort("/dev/ttyUSB0");
  serial.open();
#endif
}
  \endcode

  \param[in] port : Serial port name. A string similar to `/dev/ttyUSB0`, `/dev/ttySO`, `/dev/ttyAMA0`...
  \param[in] baudrate : The baudrate parameter. Common values are 9600, 115200...
  \param[in] bytesize : Size of each byte in the serial transmission of data. Default is 8 bits.
  \param[in] parity : Parity parameter. Default is without parity check.
  \param[in] stopbits : Number of stop bits used. Default is 1 stop bit.
  \param[in] flowcontrol : Type of flowcontrol used. Default is no flow control.

 */
vpSerial::vpSerial(const std::string &port, unsigned long baudrate,
                   bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
                   flowcontrol_t flowcontrol)
  : m_port(port), m_fd(-1), m_is_open(false), m_xonxoff(false), m_rtscts(false),
    m_baudrate(baudrate), m_parity(parity),
    m_bytesize(bytesize), m_stopbits(stopbits), m_flowcontrol(flowcontrol)
{
  if (m_port.empty () == false)
    open();
}

/*!
 * Destructor that closes the serial port.
 */
vpSerial::~vpSerial()
{
  close();
}

/*!
  Set serial baud rate. Typical values are 9600, 19200, 38400, 57600, 115200...
  \sa getBaudrate()
 */
void vpSerial::setBaudrate(const unsigned long baudrate)
{
  m_baudrate = baudrate;
}

/*!
  Set byte size.
  \sa getBytesize()
 */
void vpSerial::setBytesize(const bytesize_t &bytesize)
{
  m_bytesize = bytesize;
}

/*!
  Set flow control type.
  \sa getFlowcontrol()
 */
void vpSerial::setFlowcontrol(const flowcontrol_t &flowcontrol)
{
  m_flowcontrol = flowcontrol;
}

/*!
  Set parity.
  \sa getParity()
 */
void vpSerial::setParity(const parity_t &parity)
{
  m_parity = parity;
}

/*!
  Set number of stop bits.
  \sa getStopbits()
 */
void vpSerial::setStopbits(const stopbits_t &stopbits)
{
  m_stopbits = stopbits;
}

/*!
  Set the serial port name. The name is a string similar to `/dev/ttyUSB0`, `/dev/ttySO`, `/dev/ttyAMA0`...
  \code
#include <visp3/core/vpSerial.h>

int main()
{
#ifndef WIN32
  vpSerial serial;
  serial.setPort("/dev/ttyUSB0");
  serial.open();
#endif
}
  \endcode

  \sa getPort()
 */
void vpSerial::setPort(const std::string &port)
{
  m_port = port;
}

/*!
  Return the number of characters in the buffer.
 */
int vpSerial::available()
{
  if (!m_is_open) {
    return 0;
  }
  int count = 0;
  if (-1 == ioctl (m_fd, TIOCINQ, &count)) {
    throw(vpException(vpException::fatalError, "Cannot check is serial port data available"));
  } else {
    return count;
  }
}

/*!
   Closes the serial port.
   \sa open()
 */
void vpSerial::close()
{
  if (m_is_open == true) {
    if (m_fd != -1) {
      int ret;
      ret = ::close (m_fd);
      if (ret == 0) {
        m_fd = -1;
      } else {
        throw(vpException(vpException::fatalError, "Cannot close serial port"));
      }
    }
    m_is_open = false;
  }
}

/*!
   Open the serial port.
   If the serial port name is empty, or if the serial port is already openned an exception
   vpException::fatalError is thrown.

   The following example shows how to open the serial port `/dev/ttyUSB0` without using the constructor:
   \code
#include <visp3/core/vpSerial.h>

int main()
{
#ifndef WIN32
  vpSerial serial;

  serial.setPort("/dev/ttyUSB0");
  serial.setBaudrate(9600);
  serial.setBytesize(vpSerial::eightbits);
  serial.setParity(vpSerial::parity_none);
  serial.setStopbits(vpSerial::stopbits_one);
  serial.setFlowcontrol(vpSerial::flowcontrol_none);

  serial.open();
#endif
}
   \endcode

 */
void vpSerial::open()
{
  if (m_port.empty ()) {
    throw(vpException(vpException::fatalError, "Serial port empty"));
  }
  if (m_is_open == true) {
    throw(vpException(vpException::fatalError, "Serial port already open"));
  }

  m_fd = ::open (m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (m_fd == -1) {
    switch (errno) {
    case EINTR:
      // Try again because this is a recoverable error.
      open();
      return;
    case ENFILE:
    case EMFILE:
      throw(vpException(vpException::fatalError, "Serial port has to many handles open"));
    default:
      throw(vpException(vpException::fatalError, "Serial port opening error"));
    }
  }

  configure();
  m_is_open = true;
}

/*!
 * Read one character at a time.
 * \param[out] c : Character that is read.
 * \param[in] timeout_s : Timeout in seconds.
 * \return true if success, false otherwise.
 */
bool vpSerial::read(char *c, long timeout_s)
{
  if (m_is_open == false) {
    throw(vpException(vpException::fatalError, "Serial port not opened"));
  }

  fd_set readfds;                          /* list of fds for select to listen to */
  struct timeval timeout = {timeout_s, 0}; // seconde, micro-sec

  FD_ZERO(&readfds);
  FD_SET(static_cast<unsigned int>(m_fd), &readfds);

  int ret = select(FD_SETSIZE, &readfds, (fd_set *)NULL, (fd_set *)NULL, &timeout);

  if (ret < 0) {
    throw(vpException(vpException::fatalError, "Serial i/o exception"));
  }
  else if (ret == 0) {
    // Timeout occured
    return false;
  }
  else {
    ssize_t n = ::read(m_fd, c, 1); // read one character at a time
    if (n != 1)
      return false;
  }
  return true;
}

/*!
  Reads a set of characters until a given delimiter has been received.
  \param[in] eol : End of line delimiter. A typical example is "\n".
  \return A string containing the data that has been read.
 */
std::string vpSerial::readline(const std::string &eol)
{
  char c;
  size_t read_so_far = 0;
  size_t eol_len = eol.length ();
  std::string line;

  while(true) {
    size_t bytes_read = this->read(&c, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occured on reading 1 byte
    }
    line.append(&c, 1);
    if (std::string(line, line.size() - eol_len, eol_len) == eol) {
      break; // EOL found
    }
  }
  return line;
}

/*!
  Writes a string to the serial port.
  \param[in] s : The data to write.
 */
void vpSerial::write(const std::string &s)
{
  if (m_is_open == false) {
    throw(vpException(vpException::fatalError, "Serial port not opened"));
  }

  ssize_t r = ::write(m_fd, s.c_str(), s.size());
  if (r != (ssize_t)(s.size())) {
    throw(vpException(vpException::fatalError, "Serial port write error"));
  }
}

void vpSerial::configure()
{
  if (m_fd == -1) {
    throw(vpException(vpException::fatalError, "Serial port not opened"));
  }

  struct termios options;

  if (tcgetattr(m_fd, &options) == -1) {
    ::close(m_fd);
    throw vpException(vpException::fatalError, "Cannot get serial configuration");
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                  ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

#ifdef IUCLC
  options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

  speed_t baudrate;
  switch(m_baudrate) {
#ifdef B0
  case 0: baudrate = B0; break;
#endif
#ifdef B50
  case 50: baudrate = B50; break;
#endif
#ifdef B75
  case 75: baudrate = B75; break;
#endif
#ifdef B110
  case 110: baudrate = B110; break;
#endif
#ifdef B134
  case 134: baudrate = B134; break;
#endif
#ifdef B150
  case 150: baudrate = B150; break;
#endif
#ifdef B200
  case 200: baudrate = B200; break;
#endif
#ifdef B300
  case 300: baudrate = B300; break;
#endif
#ifdef B600
  case 600: baudrate = B600; break;
#endif
#ifdef B1200
  case 1200: baudrate = B1200; break;
#endif
#ifdef B1800
  case 1800: baudrate = B1800; break;
#endif
#ifdef B2400
  case 2400: baudrate = B2400; break;
#endif
#ifdef B4800
  case 4800: baudrate = B4800; break;
#endif
#ifdef B9600
  case 9600: baudrate = B9600; break;
#endif
#ifdef B14400
  case 14400: baudrate = B14400; break;
#endif
#ifdef B19200
  case 19200: baudrate = B19200; break;
#endif
#ifdef B38400
  case 38400: baudrate = B38400; break;
#endif
#ifdef B57600
  case 57600: baudrate = B57600; break;
#endif
#ifdef B115200
  case 115200: baudrate = B115200; break;
#endif
#ifdef B230400
  case 230400: baudrate = B230400; break;
#endif
#ifdef B460800
  case 460800: baudrate = B460800; break;
#endif
#ifdef B500000
  case 500000: baudrate = B500000; break;
#endif
#ifdef B576000
  case 576000: baudrate = B576000; break;
#endif
#ifdef B921600
  case 921600: baudrate = B921600; break;
#endif
#ifdef B1000000
  case 1000000: baudrate = B1000000; break;
#endif
#ifdef B1152000
  case 1152000: baudrate = B1152000; break;
#endif
#ifdef B1500000
  case 1500000: baudrate = B1500000; break;
#endif
#ifdef B2000000
  case 2000000: baudrate = B2000000; break;
#endif
#ifdef B2500000
  case 2500000: baudrate = B2500000; break;
#endif
#ifdef B3000000
  case 3000000: baudrate = B3000000; break;
#endif
#ifdef B3500000
  case 3500000: baudrate = B3500000; break;
#endif
#ifdef B4000000
  case 4000000: baudrate = B4000000; break;
#endif
  default:
    throw vpException(vpException::fatalError, "Cannot set serial baudrate to %ld", m_baudrate);
  }

#ifdef _BSD_SOURCE
    ::cfsetspeed(&options, baudrate);
#else
    ::cfsetispeed(&options, baudrate);
    ::cfsetospeed(&options, baudrate);
#endif

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  switch(m_bytesize) {
  case eightbits: options.c_cflag |= CS8; break;
  case sevenbits: options.c_cflag |= CS7; break;
  case sixbits:   options.c_cflag |= CS6; break;
  case fivebits:  options.c_cflag |= CS5; break;
  }

  switch(m_stopbits) {
  case stopbits_one: options.c_cflag &= (tcflag_t) ~(CSTOPB); break;
  case stopbits_two: options.c_cflag |= (CSTOPB); break;
  }

  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  switch(m_parity) {
  case parity_none:
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD); break;
  case parity_even:
    options.c_cflag &= (tcflag_t) ~(PARODD);
    options.c_cflag |=  (PARENB); break;
  case parity_odd:
    options.c_cflag |=  (PARENB | PARODD); break;
  }

  // setup flow control
  switch(m_flowcontrol) {
  case flowcontrol_none:
    m_xonxoff = false;
    m_rtscts = false;
    break;
  case flowcontrol_software:
    m_xonxoff = true;
    m_rtscts = false;
    break;
  case flowcontrol_hardware:
    m_xonxoff = false;
    m_rtscts = true;
    break;
  }

  // xonxoff
  if (m_xonxoff)
    options.c_iflag |=  (IXON | IXOFF);
  else
#ifdef IXANY
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif

  // rtscts
#ifdef CRTSCTS
  if (m_rtscts)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);
#elif defined CNEW_RTSCTS
  if (m_rtscts)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS doesn't support serial rtscts"
#endif

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (m_fd, TCSANOW, &options);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_ar.a(vpAROgre.cpp.o) has no symbols
void dummy_vpSerial(){};
#endif
