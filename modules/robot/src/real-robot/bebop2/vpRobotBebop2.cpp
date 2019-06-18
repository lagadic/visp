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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ARSDK

#include <visp3/robot/vpRobotBebop2.h>

/*!
 * Default constructor.
 */
vpRobotBebop2::vpRobotBebop2(std::string ipAddress, int discoveryPort, std::string fifo_dir, std::string fifo_name)
  :m_ipAddress(ipAddress), m_discoveryPort(discoveryPort), m_fifo_dir(fifo_dir), m_fifo_name(fifo_name)
{
  m_outputID = 0;

  std::cout<<"Configuration"<<std::endl;

  m_errorController = ARCONTROLLER_OK;
  m_deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
  m_running = true;

  //Initialises a semaphore
  ARSAL_Sem_Init (&(m_stateSem), 0, 0);

  std::cout<<"Starting"<<std::endl;

  activateDisplay();
  std::cout<<"Display activated"<<std::endl<<std::endl;

  ARDISCOVERY_Device_t * discoverDevice = discoverDrone();
  std::cout<<"Device created"<<std::endl<<std::endl;

  createDroneController(discoverDevice);
  std::cout<<"Drone controller created"<<std::endl<<std::endl;

  setupCallbacks();
  std::cout<<"Callbacks created"<<std::endl<<std::endl;

  startController();
  std::cout<<"Controller started"<<std::endl<<std::endl;

}

/*!
 * Destructor.
 */
vpRobotBebop2::~vpRobotBebop2()
{
  cleanUp();
}

std::string vpRobotBebop2::getIpAddress()
{
  return m_ipAddress;
}

int vpRobotBebop2::getDiscoveryPort()
{
  return m_discoveryPort;
}

bool vpRobotBebop2::isRunning()
{
  return m_running;
}

bool vpRobotBebop2::isHovering()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING;
}

bool vpRobotBebop2::isFlying()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING;
}

bool vpRobotBebop2::isLanded()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED;
}

void vpRobotBebop2::takeOff()
{
  if (isLanded())
  {
    m_deviceController->aRDrone3->sendPilotingTakeOff(m_deviceController->aRDrone3);
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't take off : drone isn't landed.");
  }
}

void vpRobotBebop2::land()
{
  if (isFlying() || isHovering())
  {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't land : drone isn't flying or hovering.");
  }
}

void vpRobotBebop2::startStreaming()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
  m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable (m_deviceController->aRDrone3, 1);
  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }
}


void vpRobotBebop2::handleKeyboardInput(int key)
{
  switch (key) {
    case 'q':
      //Quit
      m_running = false;
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Quitting ... ");
      break;

    case 'e':
      //Emergency
      m_errorController = m_deviceController->aRDrone3->sendPilotingEmergency(m_deviceController->aRDrone3);
      m_running = false;
      break;

    case 't':
      //Takeoff
      takeOff();
      break;

    case ' ':
      //Landing
      land();
      break;

    case KEY_UP:
      //Up
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDGaz(m_deviceController->aRDrone3, 50);
      break;

    case KEY_DOWN:
      //Down
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDGaz(m_deviceController->aRDrone3, -50);
      break;

    case KEY_RIGHT:
      //Right
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDYaw(m_deviceController->aRDrone3, 50);
      break;

    case KEY_LEFT:
      //Left
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDYaw(m_deviceController->aRDrone3, -50);
      break;

    case 'r':
      //Forward
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDPitch(m_deviceController->aRDrone3, 50);
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
      break;

    case 'f':
      //Backward
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDPitch(m_deviceController->aRDrone3, -50);
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
      break;

    case 'd':
      //Roll left
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDRoll(m_deviceController->aRDrone3, -50);
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
      break;

    case 'g':
      //Roll right
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDRoll(m_deviceController->aRDrone3, 50);
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
      break;

    default:
      //No inputs -> drone stops moving
      m_errorController = m_deviceController->aRDrone3->setPilotingPCMD(m_deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
      break;
  }
  usleep(10);
}

//***                   ***//
//*** Private Functions ***//
//***                   ***//

eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE vpRobotBebop2::getFlyingState()
{
  eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
  eARCONTROLLER_ERROR error;

  ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED, &error);

  if (error == ARCONTROLLER_OK && elementDictionary != NULL)
    {
      ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
      ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

      #pragma GCC diagnostic ignored "-Wold-style-cast"
      #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
      #pragma GCC diagnostic ignored "-Wcast-qual"
      HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
      #pragma GCC diagnostic pop
      #pragma GCC diagnostic pop
      #pragma GCC diagnostic pop

      if (element != NULL)
      {
        //Suppress warnings
        #pragma GCC diagnostic ignored "-Wold-style-cast"
        #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
        #pragma GCC diagnostic ignored "-Wcast-qual"
        // Get the value
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE, arg);
        #pragma GCC diagnostic pop
        #pragma GCC diagnostic pop
        #pragma GCC diagnostic pop

        if (arg != NULL)
        {
          // Enums are stored as I32
          flyingState = static_cast<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE>(arg->value.I32);
        }
      }
    }
  return flyingState;
}

void vpRobotBebop2::activateDisplay()
{
  //Creates a temp directory to store video feed
  m_fifo_dir = vpIoTools::makeTempDirectory(m_fifo_dir);

  m_fifo_name = m_fifo_dir + "/" + m_fifo_name;

  if (mkfifo(const_cast<char *>(m_fifo_name.c_str()), 0666) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
    return;
  }

  //Launches the display
  if((m_outputID = fork()) == 0)
  {
    execlp("xterm", "xterm", "-e", "mplayer", "-demuxer",  "h264es", const_cast<char*>(m_fifo_name.c_str()), "-benchmark", "-really-quiet", NULL);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer and xterm.");
    return;
  }

  m_videoOut = fopen(const_cast<char*>(m_fifo_name.c_str()), "w");
}

ARDISCOVERY_Device_t * vpRobotBebop2::discoverDrone()
{
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  ARDISCOVERY_Device_t * device = ARDISCOVERY_Device_New(&errorDiscovery);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
  const char * charIpAddress = m_ipAddress.c_str();
  errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", charIpAddress, m_discoveryPort);

  if (errorDiscovery != ARDISCOVERY_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
  }

  return device;
}

void vpRobotBebop2::createDroneController(ARDISCOVERY_Device_t * discoveredDrone)
{
  m_deviceController = ARCONTROLLER_Device_New (discoveredDrone, &m_errorController);
  if (m_errorController != ARCONTROLLER_OK)
  {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
  }
  ARDISCOVERY_Device_Delete (&discoveredDrone);
}

void vpRobotBebop2::setupCallbacks()
{
  //Adding stateChanged callback, called when the state of the controller has changed
  m_errorController = ARCONTROLLER_Device_AddStateChangedCallback(m_deviceController, stateChanged, this);
  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
  }

  //Adding commendReceived callback, called when the a command has been received from the device
  m_errorController = ARCONTROLLER_Device_AddCommandReceivedCallback (m_deviceController, commandReceivedCallback, m_deviceController);

  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add Command callback failed.");
  }

  //Adding frame received callback, called when a streaming frame has been received from the device
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
  m_errorController = ARCONTROLLER_Device_SetVideoStreamCallbacks (m_deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , this);

  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error: %s", ARCONTROLLER_Error_ToString(m_errorController));
  }
}

void vpRobotBebop2::startController()
{
  //Starts the controller
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
  m_errorController = ARCONTROLLER_Device_Start (m_deviceController);

  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }

  ARSAL_Sem_Wait (&(m_stateSem));

  //Checks the device state
  m_deviceState = ARCONTROLLER_Device_GetState (m_deviceController, &m_errorController);

  if((m_errorController != ARCONTROLLER_OK) || (m_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING))
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", m_deviceState);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }
}

void vpRobotBebop2::cleanUp()
{
  //Lands the drone if not landed
  if(!isLanded()){
    land();
  }
  //Deletes the controller
  m_deviceState = ARCONTROLLER_Device_GetState (m_deviceController, &m_errorController);
  if ((m_errorController == ARCONTROLLER_OK) && (m_deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED))
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

    m_errorController = ARCONTROLLER_Device_Stop (m_deviceController);

    if (m_errorController == ARCONTROLLER_OK)
    {
      // wait state update update
      ARSAL_Sem_Wait (&(m_stateSem));
    }
  }

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
  ARCONTROLLER_Device_Delete (&m_deviceController);

  //Closes the display and deletes tmp files and processes
  fflush (m_videoOut);
  fclose (m_videoOut);

  if(m_outputID > 0)
  {
    kill(m_outputID, SIGKILL);
  }

  //Destroys the semaphore
  ARSAL_Sem_Destroy(&(m_stateSem));

  unlink(const_cast<char *>(m_fifo_name.c_str()));
  vpIoTools::remove(m_fifo_dir);
}

//***           ***//
//*** Callbacks ***//
//***           ***//

void vpRobotBebop2::stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void * customData)
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);
  (void)error;

  vpRobotBebop2 * drone = (vpRobotBebop2 *)customData;
  switch (newState)
  {
  case ARCONTROLLER_DEVICE_STATE_STOPPED:

    ARSAL_Sem_Post (&(drone->m_stateSem));
    //Stopping the programm
    drone->m_running = false;
    break;

  case ARCONTROLLER_DEVICE_STATE_RUNNING:
    ARSAL_Sem_Post (&(drone->m_stateSem));
    break;

  default:
    break;
  }
}

eARCONTROLLER_ERROR vpRobotBebop2::decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void * customData)
{
  vpRobotBebop2 * drone = (vpRobotBebop2 *)customData;

  if (drone->m_videoOut != NULL)
  {
    if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264)
    {
      fwrite(codec.parameters.h264parameters.spsBuffer, static_cast<size_t>(codec.parameters.h264parameters.spsSize), 1, drone->m_videoOut);
      fwrite(codec.parameters.h264parameters.ppsBuffer, static_cast<size_t>(codec.parameters.h264parameters.ppsSize), 1, drone->m_videoOut);

      fflush (drone->m_videoOut);
    }
  }
  else
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
  }

  return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR vpRobotBebop2::didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData)
{
  vpRobotBebop2 * drone = (vpRobotBebop2 *)customData;

  if (drone->m_videoOut != NULL)
  {
    if (frame != NULL)
    {
      fwrite(frame->data, frame->used, 1, drone->m_videoOut);

      fflush (drone->m_videoOut);
    }
    else
    {
      ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
    }
  }
  else
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
  }

  return ARCONTROLLER_OK;
}

void vpRobotBebop2::cmdBatteryStateChangedRcv(ARCONTROLLER_Device_t * deviceController, ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary)
{
  (void)deviceController;
  ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

  if (elementDictionary == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
    return;
  }

  //Suppress warnings
  #pragma GCC diagnostic ignored "-Wold-style-cast"
  #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
  #pragma GCC diagnostic ignored "-Wcast-qual"
  // get the command received in the device controller
  HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);
  #pragma GCC diagnostic pop
  #pragma GCC diagnostic pop
  #pragma GCC diagnostic pop

  if (singleElement == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
    return;
  }

  //Suppress warnings
  #pragma GCC diagnostic ignored "-Wold-style-cast"
  #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
  #pragma GCC diagnostic ignored "-Wcast-qual"
  // get the value
  HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);
  #pragma GCC diagnostic pop
  #pragma GCC diagnostic pop
  #pragma GCC diagnostic pop

  if (arg == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
    return;
  }
  //Prints the battery level
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Battery level changed : %d percent remaining.", arg->value.U8);
}

void vpRobotBebop2::commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary, void * customData)
{
  ARCONTROLLER_Device_t * deviceController = static_cast<ARCONTROLLER_Device_t *>(customData);

  if (deviceController == NULL)
      return;


  switch(commandKey) {
  case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
      //If the command received is a battery state changed
      cmdBatteryStateChangedRcv(deviceController, elementDictionary);
      break;
  default:
      break;
  }
}

#endif
