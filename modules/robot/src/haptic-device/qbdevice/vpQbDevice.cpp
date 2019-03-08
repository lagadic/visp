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

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_QBDEVICE

#include <regex>

#include <visp3/robot/vpQbDevice.h>
#include <visp3/core/vpIoTools.h>

/*!
 * Default constructor that does nothing.
 * To connect to a device call init().
 */
vpQbDevice::vpQbDevice()
  : vpQbDevice(std::make_shared<qb_device_driver::qbDeviceAPI>())
{
}

/**
 * Constructor called from the default constructor with the real API smart pointer,
 * but it can be called with a pointer to the mock class inherited from the \p qbDeviceAPI itself.
 * \param device_api The shared pointer to the current API derived from \p qb_device_driver::qbDeviceAPI.
 *
 * Initialises the current max to 750. The value is updated after a call to init().
 */
vpQbDevice::vpQbDevice(std::shared_ptr<qb_device_driver::qbDeviceAPI> device_api)
  : m_device_api(device_api), m_serial_protectors(), m_file_descriptors(),
    m_connected_devices(), m_position_limits(2), m_max_repeats(1), m_current_max(750.), m_init_done(false)
{
  // Default values updated after a call to init()
  m_position_limits[0] = 0;
  m_position_limits[1] = 19000;
}

/**
 * Close all the still open serial ports.
 * \sa close()
 */
vpQbDevice::~vpQbDevice()
{
  for (auto it = m_file_descriptors.begin(); it != m_file_descriptors.end(); ) {
    if (close(it->first)) {
      it = m_file_descriptors.erase(it);
    }
    else {
      ++ it;
    }
  }
}

/**
 * Activate (or deactivate, according to the given command) the motors of the given device. Do nothing if the device
 * is not connected in the Communication Handler.
 * \param id The ID of the device to be activated (or deactivated), in range [\p 1, \p 128].
 * \param command \p true to turn motors on, \p false to turn them off.
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \sa activate(const int &, const int &), deactivate(), isActive()
 */
int vpQbDevice::activate(const int &id, const bool &command, const int &max_repeats)
{
  std::string command_prefix = command ? "" : "de";
  bool status = false;
  int failures = 0;

  failures = isActive(id, max_repeats, status);
  if (status != command) {
    m_device_api->activate(&m_file_descriptors.at(m_connected_devices.at(id)), id, command);
    failures = std::max(failures, isActive(id, max_repeats, status));
    if (status != command) {
      std::cout << "Device [" << id << "] fails on " << command_prefix << "activation." << std::endl;;
      return -1;
    }
    std::cout << "Device [" << id << "] motors have been " << command_prefix << "activated!" << std::endl;
    return failures;
  }
  std::cout << "Device [" << id << "] motors were already " << command_prefix << "activated!" << std::endl;
  return failures;
}

/**
 * Activate the motors of the given device. Do nothing if the device is not connected in the Communication Handler.
 * \param id The ID of the device to be activated, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \sa isActive()
 */
int vpQbDevice::activate(const int &id, const int &max_repeats)
{
  return activate(id, true, max_repeats);
}

/**
 * Close the communication with all the devices connected to the given serial port.
 * \param serial_port The serial port which has to be closed, e.g. \p /dev/ttyUSB* (Unix),
 * \p /dev/tty.usbserial-* (MacOS) or \p COM* (Windows).
 * \sa isInOpenMap(), open()
 */
bool vpQbDevice::close(const std::string &serial_port)
{
  if (!isInOpenMap(serial_port)) {
    std::cout << "has not handled [" << serial_port << "]." << std::endl;
    return false;  // no error: the communication is close anyway
  }

  for (auto const &device : m_connected_devices) {
    if (device.second == serial_port) {
      deactivate(device.first, m_max_repeats);
      m_connected_devices.erase(device.first);
      break;
    }
  }

  m_device_api->close(&m_file_descriptors.at(serial_port));

  // Note that m_file_descriptors.erase(serial_port) is done in the destructor.
  // Cannot be done here since the iterator that is used in the destructor would be lost

  std::cout << "does not handle [" << serial_port << "] anymore." << std::endl;
  return true;
}

/**
 * Deactivate the motors of the given device. Do nothing if the device is not connected in the Communication Handler.
 * \param id The ID of the device to be deactivated, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \sa activate(const int &, const bool &, const int &), isActive()
 */
int vpQbDevice::deactivate(const int &id, const int &max_repeats)
{
  return activate(id, false, max_repeats);
}

/**
 * Retrieve the motor currents of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
 * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
 * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
 * \return \p 0 on success.
 * \sa getMeasurements(), getPositions()
 */
int vpQbDevice::getCurrents(const int &id, const int &max_repeats, std::vector<short int> &currents)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);  // required by 'getCurrents()'
  while (failures <= max_repeats) {
    if (m_device_api->getCurrents(&m_file_descriptors.at(m_connected_devices.at(id)), id, currents) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Retrieve the printable configuration setup of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \param info The configuration setup formatted as a plain text string (empty string on communication error).
 * \return The number of failure reads between \p 0 and \p max_repeats.
 * \sa getParameters()
 */
int vpQbDevice::getInfo(const int &id, const int &max_repeats, std::string &info)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    info = m_device_api->getInfo(&m_file_descriptors.at(m_connected_devices.at(id)), id);
    if (info == "") {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Retrieve the motor currents of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
 * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
 * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
 * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
 * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
 * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
 * not directly actuated).
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \return The number of failure reads between \p 0 and \p max_repeats.
 * \sa getCurrents(), getPositions()
 */
int vpQbDevice::getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents, std::vector<short int> &positions) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);
  positions.resize(3);
  std::vector<short int> measurements(5, 0);  // required by 'getMeasurements()'
  while (failures <= max_repeats) {
    if (m_device_api->getMeasurements(&m_file_descriptors.at(m_connected_devices.at(id)), id, measurements) < 0) {
      failures++;
      continue;
    }
    std::copy(measurements.begin(), measurements.begin()+2, currents.begin());
    std::copy(measurements.begin()+2, measurements.end(), positions.begin());
    break;
  }
  return failures;
}

/**
 * Retrieve some of the parameters from the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param[out] limits The vector of motor position limits expressed in \em ticks: two values for each motor,
 * respectively [\p lower_limit, \p upper_limit].
 * \param[out] resolutions The vector of encoder resolutions, each in range [\p 0, \p 8]: one value for each encoder
 * (\b note: the \em qbmove has also the shaft encoder even if it is not actuated). The word "resolution" could be
 * misunderstood: taken the resolution \p r, \f$2^r\f$ is the number of turns of the wire inside the device mechanics.
 * It is used essentially to convert the measured position of the motors in \em ticks to \em radians or \em degrees.
 * \return \p 0 on success.
 * \sa getInfo(), init()
 */
int vpQbDevice::getParameters(const int &id, std::vector<int> &limits, std::vector<int> &resolutions)
{
  std::vector<int> input_mode = {-1};
  std::vector<int> control_mode = {-1};
  m_device_api->getParameters(&m_file_descriptors.at(m_connected_devices.at(id)), id, input_mode, control_mode, resolutions, limits);
  if (!input_mode.front() && !control_mode.front()) {  // both input and control modes equals 0 are required, i.e. respectively USB connected and position controlled
    return 0;
  }
  return -1;
}

/**
 * Retrieve the motor positions of the given device.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
 * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
 * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
 * not directly actuated).
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \return The number of failure reads between \p 0 and \p max_repeats.
 * \sa getMeasurements()
 */
int vpQbDevice::getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  positions.resize(3);  // required by 'getPositions()'
  while (failures <= max_repeats) {
    if (m_device_api->getPositions(&m_file_descriptors.at(m_connected_devices.at(id)), id, positions) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Scan for all the serial ports of type \p /dev/ttyUSB* detected in the system, initialize their mutex protector
 * (each serial port connected to the system has to be accessed in a mutually exclusive fashion), and retrieve all
 * the qbrobotics devices connected to them. For each device, store its ID in the private map \p m_connected_devices,
 * i.e. insert a pair [\p device_id, \p serial_port]. The map \p m_connected_devices is constructed from scratch at
 * each call.
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \return the number of connected devices.
 * \sa isInConnectedSet(), open()
 */
int vpQbDevice::getSerialPortsAndDevices(const int &max_repeats)
{
  std::map<int, std::string> connected_devices;
  std::array<char[255], 10> serial_ports;
  int serial_ports_number = m_device_api->getSerialPorts(serial_ports);

  for (int i=0; i<serial_ports_number; i++) {
    int failures = 0;
    while (failures <= max_repeats) {
      if (open(serial_ports.at(i)) != 0) {
        failures++;
        continue;
      }
      break;
    }
    if (failures >= max_repeats) {
      continue;
    }

    // 'serial_protectors_' is not cleared because of the previously acquired lock, do not do it!
#if defined(VISP_HAVE_CXX14)
    m_serial_protectors.insert(std::make_pair(serial_ports.at(i), std::make_unique<std::mutex>()));  // never override
#else
    m_serial_protectors.insert(std::make_pair(serial_ports.at(i), std::unique_ptr<std::mutex>(new std::mutex())));  // never override
#endif

    std::array<char, 255> devices;
    int devices_number = m_device_api->getDeviceIds(&m_file_descriptors.at(serial_ports.at(i)), devices);
    for (int j=0; j<devices_number; j++) {

      if (devices.at(j) == 120) {
        continue;  // ID 120 is reserved for dummy board which should not be considered as a connected device
      }
      // actually a std::map does not let same-id devices on distinct serial ports
      connected_devices.insert(std::make_pair(static_cast<int>(devices.at(j)), static_cast<std::string>(serial_ports.at(i))));
    }
  }

  std::cout << "has found [" << connected_devices.size() << "] devices connected:" << std::endl;
  for (auto const &device : connected_devices) {
    std::cout << " - device [" << device.first << "] connected through [" << device.second << "]" << std::endl;
  }

  m_connected_devices = connected_devices;
  return static_cast<int>(m_connected_devices.size());
}

/**
 * Initialize the device if the relative physical device is connected through any serial port to the system.
 * If the device is found, retrieve some of its parameter and activate its motors, if requested.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \return \p true if the call succeed.
 * \sa getSerialPortsAndDevices()
 */
bool vpQbDevice::init(const int &id)
{
  std::vector<int> encoder_resolutions;
  std::vector<std::unique_lock<std::mutex>> serial_locks;  // need to lock on all the serial resources to scan for new ports/devices
  for (auto const &mutex : m_serial_protectors) {
    serial_locks.push_back(std::unique_lock<std::mutex>(*mutex.second));
  }

  // update connected devices
  getSerialPortsAndDevices(m_max_repeats);

  if (!isInConnectedSet(id) || !isReliable(isConnected(id, m_max_repeats), m_max_repeats)) {
    std::cout << "fails while initializing device [" << id << "] because it is not connected." << std::endl;
    return false;
  }

  std::vector<int> position_limits;

  if (getParameters(id, position_limits, encoder_resolutions)) {
    std::cout << "fails while initializing device [" << id << "] because it requires 'USB' input mode and 'Position' control mode." << std::endl;
    return false;
  }

  m_position_limits.resize( position_limits.size() );
  for (size_t i = 0; i < position_limits.size(); i++) {
    m_position_limits[i] = static_cast<short int>(position_limits[i]);
  }

  std::string info;
  int failures = getInfo(id, m_max_repeats, info);
  if (!isReliable(failures, m_max_repeats)) {
    std::cout << "has not initialized device [" << id << "] because it cannot get info." << std::endl;
    return false;
  }

  std::string sep = "\n";
  std::string current_limit = "Current limit:";
  std::vector<std::string> subChain = vpIoTools::splitChain(info, sep);
  bool current_max_found = false;
  for (size_t i=0; i < subChain.size(); i++) {
    if (subChain[i].compare(0, current_limit.size(), current_limit) == 0) {
      sep = ":";
      std::vector<std::string> subChainLimit = vpIoTools::splitChain(subChain[i], sep);
      m_current_max = std::atof(subChainLimit[1].c_str());
      current_max_found = true;
      break;
    }
  }
  if (! current_max_found) {
    std::cout << "has not initialized device [" << id << "] because it cannot get the max current." << std::endl;
    return false;
  }


  failures = activate(id, m_max_repeats);
  if (!isReliable(failures, m_max_repeats)) {
    std::cout << "has not initialized device [" << id << "] because it cannot activate its motors (please, check the motor positions)." << std::endl;
    return false;
  }

  std::string serial_port = m_connected_devices.at(id);
  std::cout << "Device [" + std::to_string(id) + "] connected on port [" << serial_port << "] initialization succeeds." << std::endl;

  m_init_done = true;
  return true;
}

/**
 * Check whether the motors of the device specified by the given ID are active.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \param status \p true if the device motors are on.
 * \return The number of failure reads between \p 0 and \p max_repeats.
 */
int vpQbDevice::isActive(const int &id, const int &max_repeats, bool &status)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  status = false;
  while (failures <= max_repeats) {
    if (!m_device_api->getStatus(&m_file_descriptors.at(m_connected_devices.at(id)), id, status)) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Check whether the the device specified by the given ID is connected through the serial port.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \return The number of failure reads between \p 0 and \p max_repeats.
 */
int vpQbDevice::isConnected(const int &id, const int &max_repeats)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    if (!m_device_api->getStatus(&m_file_descriptors.at(m_connected_devices.at(id)), id)) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Check whether the physical device specified by the given ID is connected to the Communication Handler.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \return \p true if the given device belongs to the connected device vector, i.e. \p m_connected_devices.
 * \sa getSerialPortsAndDevices()
 */
bool vpQbDevice::isInConnectedSet(const int &id)
{
  return (m_connected_devices.count(id) ? true : false);
}

/**
 * Check whether the given serial port is managed by the communication handler, i.e. is open.
 * \param serial_port The name of the serial port of interest, e.g. \p /dev/ttyUSB0.
 * \return \p true if the given serial port belongs to the open file descriptor map, i.e. \p m_file_descriptors.
 * \sa open(), close()
 */
bool vpQbDevice::isInOpenMap(const std::string &serial_port)
{
  return (m_file_descriptors.count(serial_port) ? true : false);
}

/**
 * Open the serial communication on the given serial port. On success, store the opened file descriptor in the
 * private map \p m_file_descriptors, i.e. insert a pair [\p serial_port, \p file_descriptor].
 * \param serial_port The serial port which has to be opened, e.g. \p /dev/ttyUSB0.
 * \return \p 0 on success.
 * \sa close(), isInOpenMap()
 */
int vpQbDevice::open(const std::string &serial_port)
{
  // TODO: test on MAC and windows
#if (defined(__APPLE__) && defined(__MACH__))
  if (!std::regex_match(serial_port, std::regex("/dev/tty.usbserial-[[:digit:]]+"))) {
    std::cout << "vpQbDevice fails while opening [" << serial_port << "] because it does not match the expected pattern [/dev/tty.usbserial-*]." << std::endl;
    return -1;
  }
#elif defined(__unix__) || defined(__unix)
  if (!std::regex_match(serial_port, std::regex("/dev/ttyUSB[[:digit:]]+"))) {
    std::cout << "vpQbDevice fails while opening [" << serial_port << "] because it does not match the expected pattern [/dev/ttyUSB*]." << std::endl;
    return -1;
  }
#elif defined(_WIN32)
  if (!std::regex_match(serial_port, std::regex("COM[[:digit:]]+"))) {
    std::cout << "vpQbDevice fails while opening [" << serial_port << "] because it does not match the expected pattern [COM*]." << std::endl;
    return -1;
  }
#endif

  if (isInOpenMap(serial_port)) {
    std::cout << "vpQbDevice already handles [" << serial_port << "]." << std::endl;
    return 0;  // no error: the communication is open anyway
  }

  m_device_api->open(&m_file_descriptors[serial_port], serial_port);  // also create a pair in the map
  if(m_file_descriptors.at(serial_port).file_handle == INVALID_HANDLE_VALUE) {
    std::cout << "vpQbDevice fails while opening [" << serial_port << "] and sets errno [" << strerror(errno) << "]." << std::endl;
    // remove file descriptor entry
    m_file_descriptors.erase(serial_port);
    return -1;
  }

  std::cout << "Connect qb device to [" << serial_port << "]." << std::endl;
  return 0;
}

/**
 * Send the reference command to the motors of the given device and wait for acknowledge.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
 * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
 * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
 * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
 * \return \p 0 on success.
 */
int vpQbDevice::setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands)
{
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  commands.resize(2);  // required by 'setCommandsAndWait()'
  while (failures <= max_repeats) {
    if (m_device_api->setCommandsAndWait(&m_file_descriptors.at(m_connected_devices.at(id)), id, commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

/**
 * Send the reference command to the motors of the given device in a non-blocking fashion.
 * \param id The ID of the device of interest, in range [\p 1, \p 128].
 * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
 * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
 * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
 * \return Always \p 0 (note that this is a non reliable method).
 */
int vpQbDevice::setCommandsAsync(const int &id, std::vector<short int> &commands) {
  // qbhand sets only inputs.at(0), but setCommandsAsync expects two-element vector (ok for both qbhand and qbmove)
  commands.resize(2);  // required by 'setCommandsAsync()'
  m_device_api->setCommandsAsync(&m_file_descriptors.at(m_connected_devices.at(id)), id, commands);
  return 0;  // note that this is a non reliable method
}

#endif

