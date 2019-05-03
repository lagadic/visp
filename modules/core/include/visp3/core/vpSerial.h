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

#ifndef _vpSerial_h_
#define _vpSerial_h_

#if !defined(_WIN32)

#include <string>
#include <stdint.h>

#include <visp3/core/vpConfig.h>

/*!
  \class vpSerial
  \ingroup group_core_com_serial

  This class allows a serial port communication between 2 devices.

  The following example shows how to write a string on port `/dev/ttyUSB0` using the default constructor:
  \code
#include <visp3/core/vpSerial.h>

int main()
{
#ifndef WIN32
  vpSerial serial("/dev/ttyUSB0");
  serial.write("Hello world");
#endif
}
  \endcode

  \note This class is not implemented on windows-like OS.
*/
class VISP_EXPORT vpSerial
{
public:
  /*!
   * Defines the possible byte sizes for the serial port.
   */
  typedef enum {
    fivebits  = 5, //!< Data is encoded with 5 bits
    sixbits   = 6, //!< Data is encoded with 6 bits
    sevenbits = 7, //!< Data is encoded with 7 bits
    eightbits = 8  //!< Data is encoded with 8 bits
  } bytesize_t;

  /*!
   * Defines the possible parity types for the serial port.
   */
  typedef enum {
    parity_none = 0, //!< No parity check
    parity_odd  = 1, //!< Check for odd parity
    parity_even = 2  //!< Check for even parity
  } parity_t;

  /*!
   * Defines the possible stopbit types for the serial port.
   */
  typedef enum {
    stopbits_one = 1, //!< 1 stop bit is used
    stopbits_two = 2, //!< 2 stop bits are used
  } stopbits_t;

  /*!
   * Defines the possible flowcontrol types for the serial port.
   */
  typedef enum {
    flowcontrol_none = 0, //!< No flow control
    flowcontrol_software, //!< Software flow control
    flowcontrol_hardware  //!< Hardware flow control
  } flowcontrol_t;

  vpSerial(const std::string &port="", unsigned long baudrate = 9600,
           bytesize_t bytesize = eightbits, parity_t parity = parity_none, stopbits_t stopbits = stopbits_one,
           flowcontrol_t flowcontrol = flowcontrol_none);
  virtual ~vpSerial();

  int available();
  void close();

  /*!
    Return the baud rate; 9600, 115200...
    \sa setBaudrate()
   */
  unsigned long getBaudrate() {
    return m_baudrate;
  }

  /*!
    Return the byte size.
    \sa setBytesize()
   */
  bytesize_t getBytesize() {
    return m_bytesize;
  }

  /*!
    Return the flow control type.
    \sa setFlowcontrol()
   */
  flowcontrol_t getFlowcontrol() {
    return m_flowcontrol;
  }

  /*!
    Return parity.
    \sa setParity()
   */
  parity_t getParity() {
    return m_parity;
  }

  /*!
    Return the serial port name like `/dev/ttyUSB0`, `/dev/ttySO`, `/dev/ttyAMA0`...
    \sa setPort()
   */
  std::string getPort() {
    return m_port;
  }

  /*!
    Return number of stop bits used.
    \sa setStopbits()
   */
  stopbits_t getStopbits() {
    return m_stopbits;
  }

  void open();
  bool read(char *c, long timeout_s);
  std::string readline(const std::string &eol);
  void setBaudrate(const unsigned long baudrate);
  void setBytesize(const bytesize_t &bytesize);
  void setFlowcontrol(const flowcontrol_t &flowcontrol);
  void setParity(const parity_t &parity);
  void setPort(const std::string &port);
  void setStopbits(const stopbits_t &stopbits);
  void write(const std::string &s);

private:
  void configure();

  std::string m_port;
  int m_fd;

  bool m_is_open;
  bool m_xonxoff;
  bool m_rtscts;

  unsigned long m_baudrate;

  parity_t m_parity;
  bytesize_t m_bytesize;
  stopbits_t m_stopbits;
  flowcontrol_t m_flowcontrol;

};

#endif
#endif
