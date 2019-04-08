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
 * Interface for the qb robotics devices.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef _vpQbDevice_h_
#define _vpQbDevice_h_

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_QBDEVICE

#include <vector>
#include <map>
#include <mutex>
#include <memory>

#include <qb_device_driver.h>

/*!

  \class vpQbDevice

  \ingroup group_robot_haptic

  Interface for qbrobotics devices.

  See https://qbrobotics.com/ for more details.

  This class was tested with the [qbSoftHand](https://qbrobotics.com/products/qb-softhand/).

*/
class VISP_EXPORT vpQbDevice
{
public:
  vpQbDevice();
  vpQbDevice(std::shared_ptr<qb_device_driver::qbDeviceAPI> device_api);
  virtual ~vpQbDevice();

  /** @name Inherited public functionalities from vpQbDevice */
  /**
   * Return the maximum current supported by the device.
   */
  inline double getCurrentMax() const {
    return m_current_max;
  }
  //@}

protected:
  /** @name Inherited protected functionalities from vpQbDevice */
  //@{
  virtual int activate(const int &id, const bool &command, const int &max_repeats);
  virtual int activate(const int &id, const int &max_repeats);
  virtual bool close(const std::string &serial_port);
  virtual int deactivate(const int &id, const int &max_repeats);

  virtual int getCurrents(const int &id, const int &max_repeats, std::vector<short int> &currents);

  virtual int getInfo(const int &id, const int &max_repeats, std::string &info);
  virtual int getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents, std::vector<short int> &positions);

  virtual int getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions);
  virtual int getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions);
  virtual int getSerialPortsAndDevices(const int &max_repeats);

  virtual bool init(const int &id);

  virtual int isActive(const int &id, const int &max_repeats, bool &status);
  int isConnected(const int &id, const int &max_repeats);
  virtual bool isInConnectedSet(const int &id);
  virtual bool isInOpenMap(const std::string &serial_port);
  /**
   * Check whether the reading failures are in the given range.
   * \param failures The current number of communication failures per serial resource reading.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return \p true if the failures are less than the given threshold.
   */
  inline bool isReliable(int const &failures, int const &max_repeats) { return failures >= 0 && failures <= max_repeats; }

  virtual int open(const std::string &serial_port);

  virtual int setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands);
  virtual int setCommandsAsync(const int &id, std::vector<short int> &commands);
  /**
   * Set the maximum number of consecutive repetitions to mark retrieved data as corrupted. This value is set by default to 1.
   */
  inline void setMaxRepeats(const int &max_repeats) {
    m_max_repeats = max_repeats;
  }
  //@}

protected:
#if (defined(_WIN32) || defined (_WIN64))
  std::unique_ptr<std::mutex> m_mutex_dummy; // FS: cannot build without this line with msvc
#endif
  std::shared_ptr<qb_device_driver::qbDeviceAPI> m_device_api;
  std::map<std::string, std::unique_ptr<std::mutex>> m_serial_protectors;  // only callbacks must lock the serial resources
  std::map<std::string, comm_settings> m_file_descriptors;
  std::map<int, std::string> m_connected_devices;
  std::vector<short int> m_position_limits; // min and max position values in ticks
  int m_max_repeats;
  double m_current_max;
  bool m_init_done;
};


#endif
#endif
