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
#ifndef __vpForceTorqueAtiSensor_
#define __vpForceTorqueAtiSensor_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_ATIDAQ) && defined(VISP_HAVE_COMEDI)

#include <ostream>

#include <visp3/core/vpColVector.h>
#include <visp3/sensor/vpComedi.h>

/*!
  \class vpForceTorqueAtiSensor

  \ingroup group_sensor_ft

  Interface for ATI force/torque sensor. This class works only under
linux-like OS. It requires Comedi 3rd party. Installation instructions are
provided here https://visp.inria.fr/3rd_comedi.

  Comedi is the linux control and measurement device interface. For more
information see http://www.comedi.org.

  This class was tested with ATI Gamma 65-SI FT sensor connected to a
  National Instrument NI DAQmx PCI-6220 board.

  Synchronous F/T data acquisition is performed using getForceTorque(). The
call to the function blocks until the whole acquisition has finished.

  The following example shows how to get single measures from an ATI F/T
device each 10 ms (100 Hz).
\code
#include <visp3/core/vpTime.h>
#include <visp3/sensor/vpForceTorqueAtiSensor.h>

int main(int argc, char** argv)
{
  vpForceTorqueAtiSensor ati;
  ati.setCalibrationFile("FT12345.cal");
  ati.open();
  ati.bias();
  for(unsigned int i=0; i < 20; i++) {
    std::cout << "F/T: " << ati.getForceTorque().t() << std::endl;
    vpTime::wait(10);
  }
  ati.close();
#endif
}
  \endcode

*/
class VISP_EXPORT vpForceTorqueAtiSensor : public vpComedi
{
public:
  vpForceTorqueAtiSensor();
  virtual ~vpForceTorqueAtiSensor();

  void bias();
  void close();

  /*!
     Return the calibration file location specified using
     setCalibrationFile(). \sa setCalibrationFile()
   */
  std::string getCalibrationFile() const { return m_calibfile; }
  vpColVector getForceTorque() const;
  vpColVector getForceTorqueAsync() const;
  std::string getForceUnits() const;
  std::string getTorqueUnits() const;

  void open();

  void setCalibrationFile(const std::string &calibfile, unsigned short index = 1);
  void unbias();

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpForceTorqueAtiSensor &ati);

protected:
  std::string m_calibfile;       //!< ATI calibration file FT*.cal
  unsigned short m_index;        //!< Index of calibration in file (default: 1)
  unsigned short m_num_axes;     //!< Number of axis or gages available from the sensor
  unsigned short m_num_channels; //!< Number of channels available from the sensor
  vpColVector m_sample_bias;     //!< Sample value used for bias
};

#endif
#endif
