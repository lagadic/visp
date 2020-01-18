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
 * ATI Force torque interface.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COMEDI

#include <unistd.h>

#include <visp3/core/vpException.h>
#include <visp3/sensor/vpComedi.h>

/*!
  Default constructor.
 */
vpComedi::vpComedi()
  : m_device("/dev/comedi0"), m_handler(NULL), m_subdevice(0), m_range(0), m_aref(AREF_DIFF), m_nchannel(6),
    m_range_info(6), m_maxdata(6), m_chanlist(6)
{
}

/*!
  Destructor that closes the connection to the device if it is not already
  done calling close(). \sa close()
 */
vpComedi::~vpComedi() { close(); }

/*!
   Open the connection to the device.
 */
void vpComedi::open()
{
  if (!m_handler) {
    m_handler = comedi_open(m_device.c_str());

    if (!m_handler) {
      throw vpException(vpException::fatalError, "Could not open device %s", m_device.c_str());
    }

    // Print NaN for clipped inputs
    comedi_set_global_oor_behavior(COMEDI_OOR_NAN);

    // Setup data range and max value
    m_range_info.resize(m_nchannel);
    m_maxdata.resize(m_nchannel);
    m_chanlist.resize(m_nchannel);
    for (unsigned int channel = 0; channel < m_nchannel; channel++) {
      m_chanlist[channel] = CR_PACK(channel, m_range, m_aref);
      m_range_info[channel] = comedi_get_range(m_handler, m_subdevice, channel, m_range);
      m_maxdata[channel] = comedi_get_maxdata(m_handler, m_subdevice, channel);
    }
  }
}

/*!
   Close the connection to the device.
 */
void vpComedi::close()
{
  if (m_handler) {
    comedi_close(m_handler);
    m_handler = NULL;
  }
}

/*!
   Get raw data from device.
   If you selected an analog input subdevice, the output is an unsigned
   number, for example between 0 and 65535 for a 16 bit analog input, with 0
   representing the lowest voltage of the ADC, and a hardware-dependent
   maximum value representing the highest voltage.
 */
std::vector<lsampl_t> vpComedi::getRawData() const
{
  if (m_handler == NULL) {
    throw vpException(vpException::fatalError, "Comedi device not open");
  }
  // Get raw data
  std::vector<lsampl_t> raw_data(m_nchannel);

  for (unsigned int channel = 0; channel < m_nchannel; channel++) {
    // When switching the multiplexor from one channel to the next, the A/D
    // input needs time to settle to the new input voltage. The greater the
    // voltage difference, the more time it takes. Here we wait for 1us
    int ret = comedi_data_read_delayed(m_handler, m_subdevice, channel, m_range, m_aref, &raw_data[channel], 1000);
    if (ret < 0) {
      throw vpException(vpException::fatalError,
                        "Cannot get %d data from device=%s subdevice=%d "
                        "channel=%d range=%d analog reference=%d",
                        m_nchannel, m_device.c_str(), m_subdevice, channel, m_aref);
    }
  }

  return raw_data;
}

/*!
   Get physical data from device with units in Volts or mA. To know which unit
   is used, call getPhyDataUnits().
 */
vpColVector vpComedi::getPhyData() const
{
  if (m_handler == NULL) {
    throw vpException(vpException::fatalError, "Comedi device not open");
  }
  // Get raw data
  std::vector<lsampl_t> raw_data = this->getRawData();
  vpColVector phy_data(m_nchannel);

  // Convert data to physical data
  for (unsigned int channel = 0; channel < m_nchannel; channel++) {
    phy_data[channel] = comedi_to_phys(raw_data[channel], m_range_info[channel], m_maxdata[channel]);
    if (vpMath::isNaN(phy_data[channel])) {
      throw vpException(vpException::fatalError, "Comedi DAQ get NaN value. Check the connection with your device");
    }
  }

  return phy_data;
}

//! Get units (V or mA) of the physical data acquired by getPhyData() or
//! getPhyDataAsync().
std::string vpComedi::getPhyDataUnits() const
{
  if (m_handler == NULL) {
    throw vpException(vpException::fatalError, "Comedi device not open");
  }
  std::string units;
  unsigned int channel = 0;
  switch (m_range_info[channel]->unit) {
  case UNIT_volt:
    units = "V";
    break;
  case UNIT_mA:
    units = "mA";
    break;
  case UNIT_none:
    break;
  }
  return units;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpComedi.cpp.o) has no
// symbols
void dummy_vpComedi(){};
#endif
