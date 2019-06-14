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

//Static parameters
bool vpRobotBebop2::m_running = false;
FILE * vpRobotBebop2::m_videoOut = nullptr;
ARSAL_Sem_t vpRobotBebop2::m_stateSem = nullptr;

/*!
 * Default constructor.
 */
vpRobotBebop2::vpRobotBebop2()
{

  m_outputID = 0;

  m_fifo_dir = new char[sizeof(FIFO_DIR_PATTERN)];
  m_fifo_name = new char[sizeof(FIFO_DIR_PATTERN) + sizeof(FIFO_NAME)];

  strcpy(m_fifo_dir, FIFO_DIR_PATTERN);
  strcpy(m_fifo_name, "");

  std::cout<<"Configuration"<<std::endl;

  m_errorController = ARCONTROLLER_OK;
  m_deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
  m_running = true;

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

  startStreaming();
  std::cout<<"Streaming started"<<std::endl<<std::endl;

}

/*!
 * Destructor.
 */
vpRobotBebop2::~vpRobotBebop2()
{
  cleanUp();
}

eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE vpRobotBebop2::getFlyingState()
{
  eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
  eARCONTROLLER_ERROR error;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED, &error);
  if (error == ARCONTROLLER_OK && elementDictionary != nullptr)
    {
      ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
      ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;
      HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);

      if (element != nullptr)
      {
        // Get the value
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE, arg);

        if (arg != nullptr)
        {
          // Enums are stored as I32
          flyingState = static_cast<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE>(arg->value.I32);
        }
      }
    }
  return flyingState;
}

bool vpRobotBebop2::isRunning()
{
  return m_running;
}

void vpRobotBebop2::takeOff()
{
  if (getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED)
  {
    m_deviceController->aRDrone3->sendPilotingTakeOff(m_deviceController->aRDrone3);
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't take off : drone isn't landed.");
  }
}

void vpRobotBebop2::land()
{
  eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState = getFlyingState();
  if (flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING || flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING)
  {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't land : drone isn't flying or hovering.");
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

int vpRobotBebop2::activateDisplay()
{
  //Creates a temp directory to store video feed
  if(mkdtemp(m_fifo_dir) == nullptr)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
    return 1;
  }
  snprintf(m_fifo_name, sizeof(FIFO_NAME)+sizeof(FIFO_DIR_PATTERN), "%s/%s", m_fifo_dir, FIFO_NAME);

  if(mkfifo(m_fifo_name, 0666) < 0)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
    return 1;
  }

  //Initialises the semaphore
  ARSAL_Sem_Init (&(m_stateSem), 0, 0);

  //Launches the display
  if((m_outputID = fork()) == 0)
  {
    execlp("xterm", "xterm", "-e", "mplayer", "-demuxer",  "h264es", m_fifo_name, "-benchmark", "-really-quiet", NULL);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer and xterm.");
    return -1;
  }

  m_videoOut = fopen(m_fifo_name, "w");

  return EXIT_SUCCESS;
}

ARDISCOVERY_Device_t * vpRobotBebop2::discoverDrone()
{
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  ARDISCOVERY_Device_t * device = ARDISCOVERY_Device_New(&errorDiscovery);

  if (errorDiscovery == ARDISCOVERY_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
    errorDiscovery = ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", BEBOP_IP_ADDRESS, BEBOP_DISCOVERY_PORT);

    if (errorDiscovery != ARDISCOVERY_OK)
    {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
    }
  }
  else {
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
  m_errorController = ARCONTROLLER_Device_AddStateChangedCallback(m_deviceController, stateChanged, m_deviceController);
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
  m_errorController = ARCONTROLLER_Device_SetVideoStreamCallbacks (m_deviceController, decoderConfigCallback, didReceiveFrameCallback, nullptr , nullptr);

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

void vpRobotBebop2::startStreaming()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
  m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable (m_deviceController->aRDrone3, 1);
  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }
}

void vpRobotBebop2::cleanUp()
{
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
  ARSAL_Sem_Destroy (&(m_stateSem));

  unlink(m_fifo_name);
  rmdir(m_fifo_dir);

  delete[] m_fifo_name;
  delete[] m_fifo_dir;
}

//***           ***//
//*** Callbacks ***//
//***           ***//

void vpRobotBebop2::stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void * customData)
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);
  (void)error;
  (void)customData;

  switch (newState)
  {
  case ARCONTROLLER_DEVICE_STATE_STOPPED:
    ARSAL_Sem_Post (&(m_stateSem));
    //Stopping the programm
    m_running = false;

    break;

  case ARCONTROLLER_DEVICE_STATE_RUNNING:
    ARSAL_Sem_Post (&(m_stateSem));
    break;

  default:
    break;
  }
}

eARCONTROLLER_ERROR vpRobotBebop2::decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void * customData)
{
  (void)customData;
  if (m_videoOut != nullptr)
  {
    if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264)
    {
      fwrite(codec.parameters.h264parameters.spsBuffer, static_cast<size_t>(codec.parameters.h264parameters.spsSize), 1, m_videoOut);
      fwrite(codec.parameters.h264parameters.ppsBuffer, static_cast<size_t>(codec.parameters.h264parameters.ppsSize), 1, m_videoOut);

      fflush (m_videoOut);
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
  (void)customData;
  if (m_videoOut != nullptr)
  {
    if (frame != nullptr)
    {
      fwrite(frame->data, frame->used, 1, m_videoOut);

      fflush (m_videoOut);
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
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = nullptr;

  if (elementDictionary == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
    return;
  }

  // get the command received in the device controller
  HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

  if (singleElement == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
    return;
  }

  // get the value
  HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);

  if (arg == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
    return;
  }
  //Prints the battery level
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Battery level changed : %d percent remaining.", arg->value.U8);
}

void vpRobotBebop2::commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t * elementDictionary, void * customData)
{
  ARCONTROLLER_Device_t * deviceController = static_cast<ARCONTROLLER_Device_t *>(customData);

  if (deviceController == nullptr)
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
