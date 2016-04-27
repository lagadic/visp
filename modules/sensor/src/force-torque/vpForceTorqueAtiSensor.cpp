/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#if defined(VISP_HAVE_ATIDAQ) && defined(VISP_HAVE_COMEDI)

#include <visp3/core/vpException.h>
#include <visp3/sensor/vpForceTorqueAtiSensor.h>

/*!
 * Default constructor.
 */
vpForceTorqueAtiSensor::vpForceTorqueAtiSensor()
  : m_calibfile(""), m_index(1), m_num_axes(6), m_num_channels(6), m_sample_bias(), m_calibinfo(NULL)
{
}

/*!
 * Open the connection to the device.
 * \sa close()
 */
void vpForceTorqueAtiSensor::open()
{
  // Open access to device
  vpComedi::setChannelNumbers(m_num_channels);
  vpComedi::open();
}

/*!
  Bias the sensor storing an unloaded measurement; this removes the effect of tooling weight.
  \sa unbias()
 */
void vpForceTorqueAtiSensor::bias()
{
  open();

  // Get FT from device
  m_sample_bias = vpComedi::getPhyData();

  if (m_sample_bias.size() != m_num_channels)
    throw vpException(vpException::fatalError, "Physical data size (%d) and number of channels (%d) doesn't match", m_sample_bias.size(), m_num_channels);

  float *sample_bias = new float [m_num_channels];
  for(unsigned int i=0; i<m_num_channels; i++)
    sample_bias[i] = m_sample_bias[i];

  Bias(m_calibinfo, sample_bias);

  delete [] sample_bias;
}

/*!
  Unbias the sensor.
  \sa bias()
 */
void vpForceTorqueAtiSensor::unbias()
{
  open();

  // Get FT from device
  m_sample_bias = vpComedi::getPhyData();

  // Reset sample bias
  m_sample_bias = 0;

  if (m_sample_bias.size() != m_num_channels)
    throw vpException(vpException::fatalError, "Physical data size (%d) and number of channels (%d) doesn't match", m_sample_bias.size(), m_num_channels);

  float *sample_bias = new float [m_num_channels];
  for(unsigned int i=0; i<m_num_channels; i++)
    sample_bias[i] = m_sample_bias[i];

  Bias(m_calibinfo, sample_bias);

  delete [] sample_bias;
}

/*!
 * Close the calibration structure opened using setCalibrationFile() and close the connection to the device.
 */
void vpForceTorqueAtiSensor::close()
{
  if(m_calibinfo != NULL) {
    // free memory allocated to calibration structure
    destroyCalibration(m_calibinfo);
    m_calibinfo = NULL;
  }
  vpComedi::close();
}

/*!
  Get a 6-dimension force/torque vector from device. This function performs synchronously one single data acquisition.
  "Synchronous" means that the calling process blocks until the data acquisition has finished.

  \return A sampled measure from device with forces and torques. Forces units are given by
  getForceUnits(), while torque units by getTorqueUnits().
 */
vpColVector vpForceTorqueAtiSensor::getForceTorque() const
{
  vpColVector phydata = vpComedi::getPhyData();

  if (phydata.size() != m_num_channels)
    throw vpException(vpException::fatalError, "Physical data size (%d) and number of channels (%d) doesn't match",
                      phydata.size(), m_num_channels);

  float *voltage = new float [m_num_channels];
  float *ft = new float [m_num_axes];

  for(unsigned int i=0; i<m_num_channels; i++) {
    voltage[i] = phydata[i];
  }

  // convert a loaded measurement into forces and torques
  ConvertToFT(m_calibinfo, voltage, ft);

  vpColVector sample(m_num_axes);
  for(unsigned int i=0; i<m_num_axes; i++)
    sample[i] = ft[i];

  delete [] voltage;
  delete [] ft;

  return sample;
}

/*!
  Get force units.
 */
std::string vpForceTorqueAtiSensor::getForceUnits() const
{
  std::string units(m_calibinfo->ForceUnits);
  return units;
}
/*!
  Get torque units.
 */
std::string vpForceTorqueAtiSensor::getTorqueUnits() const
{
  std::string units(m_calibinfo->TorqueUnits);
  return units;
}

/*!
 * Destructor that closes the connection to the device.
 */
vpForceTorqueAtiSensor::~vpForceTorqueAtiSensor()
{
  close();
}

/*!
   Open ATI calibration file that should correspond to your F/T sensor.
   \param calibfile : ATI calibration file. This file has the following pattern: FT*.cal.
   \param index : Index of calibration in file (default = 1).
   \sa getCalibrationFile(), close()
 */
void vpForceTorqueAtiSensor::setCalibrationFile(const std::string &calibfile, unsigned short index)
{
  m_calibfile = calibfile;
  m_index = index;

  if (m_calibinfo)
    destroyCalibration(m_calibinfo);

  char file[FILENAME_MAX];
  sprintf(file, "%s", m_calibfile.c_str());

  // Create calibration struct
  m_calibinfo = createCalibration(file, m_index);
  if (m_calibinfo==NULL) {
    throw vpException(vpException::fatalError, "Calibration file %s couldn't be loaded", m_calibfile.c_str());
  }

  m_num_channels = m_calibinfo->rt.NumChannels;
  m_num_axes = m_calibinfo->rt.NumAxes;
}

/*!
  \relates vpForceTorqueAtiSensor

  Return information from calibration file.
  \param os : Input stream.
  \param ati : ATI F/T sensor interface.

  The following example shows how to use this method.
  \code
#include <visp3/sensor/vpForceTorqueAtiSensor.h>

int main()
{
  vpForceTorqueAtiSensor ati;
  ati.setCalibrationFile("FT12345.cal");
  std::cout << "ATI F/T sensor characteristics: \n" << ati << std::endl;
}
  \endcode
 */
std::ostream & operator<<(std::ostream &os, const vpForceTorqueAtiSensor &ati)
{
  Calibration *calibinfo = ati.getCalibrationInfo();

  if (calibinfo==NULL) {
    os << "Calibration Information is not available" << std::endl;
    return os;
  }

  // display info from calibration file
  os << "Calibration Information for " << ati.m_calibfile << ", index #" << ati.m_index << ":" << std::endl;
  os << "                  Serial: " << calibinfo->Serial << std::endl;
  os << "              Body Style: " << calibinfo->BodyStyle << std::endl;
  os << "             Calibration: " << calibinfo->PartNumber << std::endl;
  os << "        Calibration Date: " << calibinfo->CalDate << std::endl;
  os << "                  Family: " << calibinfo->Family << std::endl;
  os << "              # Channels: " << calibinfo->rt.NumChannels << std::endl;
  os << "                  # Axes: " << calibinfo->rt.NumAxes << std::endl;
  os << "             Force Units: " << calibinfo->ForceUnits << std::endl;
  os << "            Torque Units: " << calibinfo->TorqueUnits << std::endl;
  os << "Temperature Compensation: " << (calibinfo->TempCompAvailable ? "Yes" : "No") << std::endl;

  // print maximum loads of axes
  os << "\nRated Loads:" << std::endl;
  for (unsigned short i=0;i<calibinfo->rt.NumAxes;i++) {
    char *units;
    if ((calibinfo->AxisNames[i])[0]=='F') {
      units=calibinfo->ForceUnits;
    } else units=calibinfo->TorqueUnits;
    os << calibinfo->AxisNames[i] << ": " << calibinfo->MaxLoads[i] << " " << units << std::endl;
  }

  // print temperature compensation information, if available
  if (calibinfo->TempCompAvailable) {
    os << "\nTemperature Compensation Information:" << std::endl;
    os << "BS: ";
    for (unsigned short i=0;i<calibinfo->rt.NumChannels-1;i++) {
      os << calibinfo->rt.bias_slopes[i] << " ";
    }
    os << "\nGS: ";
    for (unsigned short i=0;i<calibinfo->rt.NumChannels-1;i++) {
      os << calibinfo->rt.gain_slopes[i] << " ";
    }
    os << "\nTherm: " << calibinfo->rt.thermistor << std::endl;
  }

  return os;
}


#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpForceTorqueAtiSensor.cpp.o) has no symbols
void dummy_vpForceTorqueAtiSensor() {};
#endif
