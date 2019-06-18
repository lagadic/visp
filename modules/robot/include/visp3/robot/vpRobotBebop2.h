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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef _vpRobotBebop2_h_
#define _vpRobotBebop2_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ARSDK

extern "C"
{
#include <libARController/ARController.h>
#include <libARSAL/ARSAL.h>
#include <libARDiscovery/ARDiscovery.h>
}

#include <string>
#include <signal.h>
#include <curses.h>
#include <iostream>

#include <visp3/io/vpKeyboard.h>
#include <visp3/core/vpIoTools.h>

#define TAG "vpRobotBebop2"


/*!
  \class vpRobotBebop2

  \ingroup group_robot_real_drone

  This class allows to control Parrot Bebop2 drone using ARSDK3.
*/
class VISP_EXPORT vpRobotBebop2
{
public:
  vpRobotBebop2(std::string ipAddress = "192.168.42.1", int discoveryPort = 44444,
                std::string fifo_dir = "/tmp/arsdk_XXXXXX", std::string fifo_name = "arsdk_fifo");
  virtual ~vpRobotBebop2();

  //*** Flight commands and parameters ***//

  std::string getIpAddress();
  int getDiscoveryPort();

  bool isRunning();
  bool isHovering();
  bool isFlying();
  bool isLanded();

  void takeOff();
  void land();


  void startStreaming();

  void handleKeyboardInput(int key);

private:
  //*** Attributes ***//

  std::string m_ipAddress; ///< Ip address of the drone to discover on the network
  int m_discoveryPort; ///< Port of the drone to discover on the network
  std::string m_fifo_dir; ///< Path of fifo file
  std::string m_fifo_name; ///< Name of fifo file

  FILE *m_videoOut; ///< File used for video output visualisation
  pid_t m_outputID; ///< ID for video output process
  ARSAL_Sem_t m_stateSem; ///< Semaphore

  bool m_running; ///< Used for checking if the programm is running

  ARDISCOVERY_Device_t *m_device;            ///< Used for drone discovery
  ARCONTROLLER_Device_t *m_deviceController; ///< Used for drone control

  eARCONTROLLER_ERROR m_errorController;    ///< Used for error handling
  eARCONTROLLER_DEVICE_STATE m_deviceState; ///< Used to store device state
  //*** ***//

  eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE getFlyingState();

  //*** Setup functions ***//
  void activateDisplay();
  void cleanUp();
  ARDISCOVERY_Device_t *discoverDrone();
  void createDroneController(ARDISCOVERY_Device_t *discoveredDrone);
  void setupCallbacks();
  void startController();
  //*** ***//

  //*** Callbacks ***//
  static void stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData);
  static eARCONTROLLER_ERROR decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void *customData);
  static eARCONTROLLER_ERROR didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData);

  static void cmdBatteryStateChangedRcv(ARCONTROLLER_Device_t *deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary);
  static void commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey,
                                      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData);
  //*** ***//
};

#endif
#endif
