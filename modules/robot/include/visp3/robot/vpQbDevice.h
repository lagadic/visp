/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
*****************************************************************************/

#ifndef _vpQbDevice_h_
#define _vpQbDevice_h_

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_QBDEVICE) && defined(VISP_HAVE_THREADS)

#include <map>
#include <memory>
#include <mutex>
#include <vector>

BEGIN_VISP_NAMESPACE
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
  virtual ~vpQbDevice();

  /** @name Inherited public functionalities from vpQbDevice */
  double getCurrentMax() const;
  std::vector<short int> getPositionLimits() const;
  void setMaxRepeats(const int &max_repeats);
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
  virtual int getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents,
                              std::vector<short int> &positions);

  virtual int getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions);
  virtual int getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions);
  virtual int getSerialPortsAndDevices(const int &max_repeats);

  virtual bool init(const int &id);

  virtual int isActive(const int &id, const int &max_repeats, bool &status);
  int isConnected(const int &id, const int &max_repeats);
  virtual bool isInConnectedSet(const int &id);
  virtual bool isInOpenMap(const std::string &serial_port);
  bool isReliable(int const &failures, int const &max_repeats);
  virtual int open(const std::string &serial_port);

  virtual int setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands);
  virtual int setCommandsAsync(const int &id, std::vector<short int> &commands);
  //@}

private:
  vpQbDevice(const vpQbDevice &);            // noncopyable
  vpQbDevice &operator=(const vpQbDevice &); //

  // Implementation
  class Impl;
  Impl *m_impl;

protected:
  int m_max_repeats; //!< Max number of trials to send a command.
  bool m_init_done;  //!< Flag used to indicate if the device is initialized.
};
END_VISP_NAMESPACE
#endif
#endif
