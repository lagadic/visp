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
 * Wrapper over IIT force-torque sensor.
 *
 * Authors:
 * Alexander Oliva
 *
*****************************************************************************/

/*!
  \file vpForceTorqueIitSensor.h
  \brief Wrapper over IIT force-torque sensor.
*/

#ifndef vpForceTorqueIitSensor_h
#define vpForceTorqueIitSensor_h

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FT_IIT_SDK) &&  defined(VISP_HAVE_THREADS)

#include <chrono>
#include <mutex>

#include <visp3/core/vpColVector.h>

#include <ftSensorLib/ftSensorLib.h>

BEGIN_VISP_NAMESPACE

/*!
  \class vpForceTorqueIitSensor
  \ingroup group_sensor_ft

  This class is a wrapper over six axis load measurement (forces and torques) from IIT (ISTITUTO ITALIANO DI TECNOLOGIA)
  sell by [Alberobotics](https://alberobotics.it/).

  The following example shows how to use this class to stream force-torque data at 1kHz and print a measurement each
  second.
  \code
  #include <iostream>

  #include <visp3/core/vpTime.h>
  #include <visp3/sensor/vpForceTorqueIitSensor.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_FT_IIT_SDK
    vpForceTorqueIitSensor iit_ft;

    if ( ! iit_ft.connected() ) {
      std::cout << "Unable to connect to IIT force-torque sensor" << std::endl;
      return EXIT_SUCCESS;
    }

    iit_ft.bias();
    iit_ft.startStreaming();
    vpColVector ft;
    for(int i=0; i < 10; i++) {
      ft = iit_ft.getForceTorque();
      std::cout << ft.t() << std::endl;
      vpTime::sleepMs(1000);
    }
    iit_ft.stopStreaming();

  #else
    std::cout << "ViSP is not build with IIT force-torque SDK support" << std::endl;
  #endif
    return EXIT_SUCCESS;
  }
  \endcode

  Once build, in the same folder as the binary you should find a configuration file named `configurationSettings.ini`.
  - Such a file is provided in `$VISP_WS/modules/sensor/test/force-torque/configurationSettings.ini` with default
    settings.
    \include configurationSettings.ini
  - To modify default settings you need first to identify your Ethernet link:
  \code
  $ ifconfig
  enp0s31f6: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
          inet 192.168.100.100  netmask 255.255.255.0  broadcast 192.168.100.255
  \endcode
  - and then modify accordingly the following lines:
  \verbatim
  LOCAL_IFACE_IP = 192.168.100.100       ; Host Computer's local interface IP
  LOCAL_IFACE = enp0s31f6                ; Host Computer's local interface name (it is required only in Linux)
  \endverbatim
  - The sensor default IP is set to `192.168.1.1`. Browsing to this address with Firefox allows to modify it's default
    IP address. If the new sensor IP address is for example `192.168.100.10`, you need to modify the following line:
  \verbatim
  USE_DEFAULT_SETTINGS = false           ; If "true", the library will use the
                                        ; DEFAULT_SETTINGS. Note that the sensor
                                        ; needs to be in the "DEF" IP position.
                                        ; If "false", the library will use the
                                        ; USER_SETTINGS to initialize the communication
                                        ; with the sensor.
  [USER_SETTINGS]                        ; USER SETTINGS FOR USE IN SINGLE_SENSOR_MODE when USE_DEFAULT_SETTINGS=false
  USER_IP = 192.168.100.10               ; User Sensor IP
  \endverbatim

  When running a binary that uses vpForceTorqueIitSensor class,
  - if you experience the following error:
  \code
  $ cd $VISP_WS/modules/sensor
  $ ./testForceTorqueIitSensor-simple
  ./testForceTorqueIitSensor-simple: error while loading shared libraries: libftSensorLib.so.0.0.1: cannot open shared
  object file: No such file or directory
  \endcode
  - it means that you need to add the location of the library in `LD_LIBRARY_PATH` environment variable. This could be
    achieved running:
  \code
  $ export
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$VISP_WS/3rdparty/FT_SDK_01_4/linux/ubuntu16.04/ftSensorLibReleaseExamples/library/bin/lin-x86_64/release
  \endcode
  where `$VISP_WS/3rdparty/FT_SDK_01_4` contains IIT SDK.

  To configure the sensor, you may access the sensor through the web interface using your favorite browser.
  \image html vpForceTorqueIitSensor-ethernet.png

*/
class VISP_EXPORT vpForceTorqueIitSensor
{
public:
  vpForceTorqueIitSensor();
  ~vpForceTorqueIitSensor();

  void bias();
  bool connected(int timeout_ms = 0) const;
  vpColVector getForceTorque(bool filtered = false);

  void startStreaming();
  void stopStreaming();

protected:
  void acquisitionLoop();
  void close();
  void join();

protected:
  ftSensorLib m_ftLib;
  int m_numSensorsInLib;

  vpColVector m_ft;
  vpColVector m_ft_filt;

  ftSensorsConnected m_ftSensorsData {};

  std::atomic<bool> m_acquisitionEnabled;
  std::atomic<bool> m_dataValid;
  bool m_connected;

  std::thread m_acquisitionThread;
  std::chrono::time_point<std::chrono::system_clock> m_timeCur, m_timePrev;

  std::mutex m_mutex;
  int m_warmupMilliseconds;
};
END_VISP_NAMESPACE
#endif
#endif
