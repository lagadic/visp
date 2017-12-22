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
 * ATI Force torque interface.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef __vpComedi_
#define __vpComedi_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COMEDI

#include <string>

#include <comedilib.h>

#include <visp3/core/vpColVector.h>

/*!
  \class vpComedi

  \ingroup group_sensor_ft

  Interface for data acquisition devices supported by Comedi. Comedi is a
linux control and measurement device interface. For more information see
http://www.comedi.org.

  This class was tested with ATI Gamma 65-SI FT sensor connected to a
  National Instrument NI DAQmx PCI-6220 board.

  \warning If you experience an exception like
  \code
Could not open device /dev/comedi0
  \endcode
  you may set up udev permissions for Comedi device files. Once Comedi is
installed you also need to make sure that the user has appropriate permissions
for accessing the Comedi device files. That is you need to be able to have
read and write access to the /dev/comedi* files. One elegant way to achieve
this to create a new group and tell udev to add the Comedi device files to
this group. To this end:
  1. Login as root
  2. Create a new group "iocard":
  \code
$ addgroup --system iocard
  \endcode
  3. Add udev rules to the /etc/udev/rules.d directory:
  \code
$ echo 'KERNEL=="comedi*", MODE="0660", GROUP="iocard"' >
/etc/udev/rules.d/95-comedi.rules \endcode
  4. Add users to the "iocard" group:
  \code
$ adduser <username> iocard
  \endcode
  5. Reboot

  The following example shows how to run an synchronous data acquisition at
500 Hz, calling getPhyData() each 2 ms:

\code
#include <visp3/sensor/vpComedi.h>

int main()
{
  vpComedi comedi;
  comedi.setDevice("/dev/comedi0");
  comedi.setChannelNumbers(6); // to read a F/T tensor
  comedi.open();

  for(unsigned int i=0; i < 500; i++) {
    std::cout << "Physical data (in " << comedi.getPhyDataUnits() << "): " << comedi.getPhyData().t() << std::endl;
    vpTime::wait(2);
  }
  comedi.close();
}
  \endcode
*/
class VISP_EXPORT vpComedi
{
public:
  vpComedi();
  virtual ~vpComedi();

  //! @name Inherited functionalities from vpComedi
  //@{
  void close();
  //! Get native Comedi handler.
  comedi_t *getHandler() const { return m_handler; }
  //! Get max data per channel. The returned vector is of dimension the number
  //! of channels.
  std::vector<lsampl_t> getMaxData() const { return m_maxdata; }
  //! Get number of channels.
  unsigned int getNChannel() const { return m_nchannel; }

  vpColVector getPhyData() const;
  std::string getPhyDataUnits() const;

  //! Get data range information per channel. The returned vector is of
  //! dimension the number of channels.
  std::vector<comedi_range *> getRangeInfo() const { return m_range_info; }
  //! Get comedi subdevice.
  unsigned int getSubDevice() const { return m_subdevice; }

  void open();

  /*!
    Set analog reference type.
    \param aref : The aref parameter specifies an analog reference to use:
    AREF_GROUND, AREF_COMMON, AREF_DIFF, or AREF_OTHER.
   */
  void setAnalogReference(const unsigned int &aref) { m_aref = aref; }

  /*!
    Number of channels to read from sensor. For a 6-dim force/torque sensor
    use 6.
    */
  void setChannelNumbers(const unsigned int &nchannel) { m_nchannel = nchannel; }

  //! Set comedi device name. Default value is /dev/comedi0.
  void setDevice(const std::string &device) { m_device = device; }

  /*!
     Set the range parameter that is the zero-based index of one of the gain
     ranges supported by the channel. This is a number from 0 to N-1 where N
     is the number of ranges supported by the channel.
   */
  void setRange(const unsigned int &range) { m_range = range; }
  //! Set comedi analog input subdevice.
  void setSubDevice(const unsigned int &subdevice) { m_subdevice = subdevice; }
  //@}

protected:
  std::vector<lsampl_t> getRawData() const;

protected:
  //! @name Protected Member Functions Inherited from vpComedi
  //@{
  std::string m_device;                     /*!< Comedi device name (default: /dev/comedi0) */
  comedi_t *m_handler;                      /*!< Comedi handler */
  unsigned int m_subdevice;                 /*!< Input subdevice */
  unsigned int m_range;                     /*!< Range of a channel */
  unsigned int m_aref;                      /*!< Analog reference */
  unsigned int m_nchannel;                  /*!< Number of channels */
  std::vector<comedi_range *> m_range_info; /*!< Range information */
  std::vector<lsampl_t> m_maxdata;          /*!< Max data value */
  std::vector<unsigned int> m_chanlist;     /*!< Channel list */
  //@}
};

#endif
#endif
