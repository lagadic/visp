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
 * Interface for Parrot's Bebop2 drone.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ARSDK

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif
#include <visp3/robot/vpRobotBebop2.h>

// ffmpeg is part of OpenCV
#ifdef VISP_HAVE_OPENCV
extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <visp/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>

#include <opencv2/core/core.hpp>

#endif

#define TAG "vpRobotBebop2"

bool vpRobotBebop2::m_running = false;
ARCONTROLLER_Device_t *vpRobotBebop2::m_deviceController = NULL;

/*!
 * Default constructor.
 */
vpRobotBebop2::vpRobotBebop2(bool verbose, float maxTilt, std::string ipAddress, int discoveryPort)
  : m_ipAddress(ipAddress), m_discoveryPort(discoveryPort), m_currentImage()
{

  if (verbose) {
    ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_INFO);
  } else {
    ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_WARNING);
  }

  // Setting up signal handling
  memset(&m_sigAct, 0, sizeof(m_sigAct));
  m_sigAct.sa_handler = vpRobotBebop2::sighandler;
  sigaction(SIGINT, &m_sigAct, 0);
  sigaction(SIGBUS, &m_sigAct, 0);
  sigaction(SIGSEGV, &m_sigAct, 0);
  sigaction(SIGKILL, &m_sigAct, 0);
  sigaction(SIGQUIT, &m_sigAct, 0);

  m_batteryLevel = 100;
  m_firstFrameHasBeenReceived = false;

  m_errorController = ARCONTROLLER_OK;
  m_deviceState = ARCONTROLLER_DEVICE_STATE_MAX;

  // Initialises a semaphore
  ARSAL_Sem_Init(&(m_stateSem), 0, 0);

  ARDISCOVERY_Device_t * discoverDevice = discoverDrone();
  std::cout<<"Device created"<<std::endl<<std::endl;

  createDroneController(discoverDevice);
  std::cout<<"Drone controller created"<<std::endl<<std::endl;

  setupCallbacks();
  std::cout<<"Callbacks created"<<std::endl<<std::endl;

  startController();
  std::cout << "Controller started" << std::endl << std::endl;

  if ((m_errorController != ARCONTROLLER_OK) || (m_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
    m_running = false;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "- Failed to setup drone control. Make sure your computer is connected to the drone Wifi spot before "
                "starting.");
  } else {
    m_relativeMoveEnded = true;
    m_videoDecodingStarted = false;
    m_running = true;
    setMaxTilt(maxTilt);
  }
}

/*!
 * Destructor.
 */
vpRobotBebop2::~vpRobotBebop2() { cleanUp(); }

std::string vpRobotBebop2::getIpAddress() { return m_ipAddress; }

int vpRobotBebop2::getDiscoveryPort() { return m_discoveryPort; }

float vpRobotBebop2::getMaxTilt() { return m_maxTilt; }

unsigned int vpRobotBebop2::getBatteryLevel() { return m_batteryLevel; }

bool vpRobotBebop2::isRunning() { return m_running; }

bool vpRobotBebop2::isStreaming() { return m_videoDecodingStarted; }

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
  if (m_running && isLanded()) {
    // We're doing a flat trim calibration of the drone before each takeoff
    m_deviceController->aRDrone3->sendPilotingFlatTrim(m_deviceController->aRDrone3);

    m_deviceController->aRDrone3->sendPilotingTakeOff(m_deviceController->aRDrone3);

    while (getFlyingState() != ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING) {
      vpTime::sleepMs(1);
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't take off : drone isn't landed.");
  }
}

void vpRobotBebop2::sighandler(int signo)
{
  std::cout << "Stopping Bebop2 because of detected signal (" << signo << "): " << (char)7;
  switch (signo) {
  case SIGINT:
    std::cout << "SIGINT (stopped by ^C) " << std::endl;
    break;
  case SIGBUS:
    std::cout << "SIGBUS (stopped due to a bus error) " << std::endl;
    break;
  case SIGSEGV:
    std::cout << "SIGSEGV (stopped due to a segmentation fault) " << std::endl;
    break;
  case SIGKILL:
    std::cout << "SIGKILL (stopped by CTRL \\) " << std::endl;
    break;
  case SIGQUIT:
    std::cout << "SIGQUIT " << std::endl;
    break;
  default:
    std::cout << signo << std::endl;
  }

  vpRobotBebop2::m_running = false;

  // Landing the drone
  if (m_deviceController != NULL) {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
}

void vpRobotBebop2::land()
{
  if (m_deviceController != NULL) {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
}

/*!

  Moves the drone by the given amounts \e dX, \e dY, \e dZ (meters) and rotate the heading by \e dPsi (radian).
  Doesn't do anything if the drone isn't flying or hovering.

  \param dX : displacement along X axis (meters).
  \param dY : displacement along Y axis (meters).
  \param dZ : displacement along Z axis (meters).
  \param dPsi : rotation of the heading (radians).
  \param blocking : specifies whether the function should be blocking or not.
    If blocking is true, the function will wait until the drone has finished moving.
    If blocking is false, the function will return even if the drone is still moving.

  */
void vpRobotBebop2::setPosition(float dX, float dY, float dZ, float dPsi, bool blocking)
{
  if (m_running && (isFlying() || isHovering())) {

    m_relativeMoveEnded = false;
    m_deviceController->aRDrone3->sendPilotingMoveBy(m_deviceController->aRDrone3, dX, dY, dZ, dPsi);

    if (blocking) {

      while (!m_relativeMoveEnded) {
        vpTime::sleepMs(1);
      }
    }
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : drone isn't flying or hovering.");
  }
}

#if 0
void vpRobotBebop2::setPosition(const vpColVector &pos, bool blocking)
{
  if (pos.size() != 4)
    exception
    setPosition(pos[0], ...)
}
#endif

/*!

Changes the position of the drone based on a homogeneous transformation matrix.

\param M : homogeneous matrix. Translation should be expressed in meters and rotation in radians.
\param blocking : specifies whether the function should be blocking or not.
  If blocking is true, the function will wait until the drone has finished moving.
  If blocking is false, the function will return even if the drone is still moving.

\warning The rotation around the X and Y axes should be equal to 0, as the drone cannot rotate around these axes.

*/
void vpRobotBebop2::setPosition(const vpHomogeneousMatrix &M, bool blocking)
{
  double epsilon = (std::numeric_limits<double>::min());
  if (std::abs(M.getRotationMatrix().getThetaUVector()[0]) >= epsilon) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : rotation around X axis should be 0.");
    return;
  }
  if (std::abs(M.getRotationMatrix().getThetaUVector()[1]) >= epsilon) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : rotation around Y axis should be 0.");
    return;
  }
  float dThetaZ = static_cast<float>(M.getRotationMatrix().getThetaUVector()[2]);
  vpTranslationVector t = M.getTranslationVector();
  setPosition(static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]), dThetaZ, blocking);
}

/*!

Sets the drone velocity.

\param vel : Velocity vector. Translation should be expressed in meters and rotation in radians.
\param delta_t : Sampling time (in seconds), time during which the velocity \e vel is applied.

\warning The rotation velocity around the X and Y axes should be equal to 0, as the drone cannot rotate around these
axes.

 */
void vpRobotBebop2::setVelocity(const vpColVector &vel, double delta_t)
{
  double epsilon = (std::numeric_limits<double>::min());

  if (vel.size() != 6) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR",
                "Can't set velocity : dimension of the velocity vector should be equal to 6.");
    return;
  }
  if (std::abs(vel[3]) >= epsilon) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : velocity of the rotation around X axis should be 0.");
    return;
  }
  if (std::abs(vel[4]) >= epsilon) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : velocity of the rotation around Y axis should be 0.");
    return;
  }

  vpHomogeneousMatrix M = vpExponentialMap::direct(vel, delta_t);
  setPosition(M, false);
}

/*!

Sets the max pitch and roll values for the drone.
These values are not taken into consideration by the drone when using
setPosition or setVelocity functions.
The smallest possible angle is 5, the maximum is 35.

\param maxTilt : new maximum pitch and roll value for the drone (degrees).

*/
void vpRobotBebop2::setMaxTilt(float maxTilt)
{
  if (isRunning()) {
    m_deviceController->aRDrone3->sendPilotingSettingsMaxTilt(m_deviceController->aRDrone3, maxTilt);
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set tilt value : drone isn't running.");
  }
}

/*!

Starts the video streaming from the drone camera. Every time a frame is received, it is decoded and stored into \e
m_currentImage, which can be obtained with getImage().

*/
void vpRobotBebop2::startStreaming()
{
  if (isRunning()) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Starting video streaming ... ");

    // Sending command to the drone to start the video stream
    m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable(m_deviceController->aRDrone3, 1);

    if (m_errorController == ARCONTROLLER_OK) {

      // Blocking until streaming is started
      while (getStreamingState() != ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_ENABLED) {
        vpTime::sleepMs(1);
      }
      startVideoDecoding();

    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't start streaming : drone isn't running.");
  }
}

void vpRobotBebop2::stopStreaming()
{
  if (m_videoDecodingStarted) {

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stopping video streaming ... ");

    // Sending command to the drone to stop the video stream
    m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable(m_deviceController->aRDrone3, 0);

    if (m_errorController == ARCONTROLLER_OK) {

      // Blocking until streaming is stopped
      while (getStreamingState() != ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_DISABLED) {
        vpTime::sleepMs(1);
      }

      stopVideoDecoding();
      std::cout << "VIDEO DECODING STOPPED" << std::endl;
    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't stop streaming : streaming already stopped.");
  }
}

void vpRobotBebop2::getImage(vpImage<unsigned char> &I)
{
  if (m_videoDecodingStarted) {
    I = m_currentImage;
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't get current image : video streaming isn't started.");
  }
}

void vpRobotBebop2::handleKeyboardInput(int key)
{
  switch (key) {
  case 'q':
    // Quit
    land();
    m_running = false;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Quitting ... ");
    break;

  case 'e':
    // Emergency
    m_errorController = m_deviceController->aRDrone3->sendPilotingEmergency(m_deviceController->aRDrone3);
    m_running = false;
    break;

  case 't':
    // Takeoff
    takeOff();
    break;

  case ' ':
    // Landing
    land();
    break;

  case KEY_UP:
    // Up
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDGaz(m_deviceController->aRDrone3, 50);
    break;

  case KEY_DOWN:
    // Down
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDGaz(m_deviceController->aRDrone3, -50);
    break;

  case KEY_RIGHT:
    // Right
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDYaw(m_deviceController->aRDrone3, 50);
    break;

  case KEY_LEFT:
    // Left
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDYaw(m_deviceController->aRDrone3, -50);
    break;

  case 'r':
    // Forward
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDPitch(m_deviceController->aRDrone3, 50);
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
    break;

  case 'f':
    // Backward
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDPitch(m_deviceController->aRDrone3, -50);
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
    break;

  case 'd':
    // Roll left
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDRoll(m_deviceController->aRDrone3, -50);
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
    break;

  case 'g':
    // Roll right
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDRoll(m_deviceController->aRDrone3, 50);
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);
    break;

  default:
    // No inputs -> drone stops moving
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

eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED vpRobotBebop2::getStreamingState()
{
  eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED streamingState =
      ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_MAX;
  eARCONTROLLER_ERROR error;

  ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(
      m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED,
      &error);

  if (error == ARCONTROLLER_OK && elementDictionary != NULL) {
    ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

    if (element != NULL) {
// Suppress warnings
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
      // Get the value
      HASH_FIND_STR(element->arguments,
                    ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED, arg);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

      if (arg != NULL) {
        // Enums are stored as I32
        streamingState =
            static_cast<eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED>(arg->value.I32);
      }
    }
  }
  return streamingState;
}

ARDISCOVERY_Device_t * vpRobotBebop2::discoverDrone()
{
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  ARDISCOVERY_Device_t * device = ARDISCOVERY_Device_New(&errorDiscovery);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Starting drone Wifi discovery ...");
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
  m_errorController = ARCONTROLLER_Device_AddCommandReceivedCallback(m_deviceController, commandReceivedCallback, this);

  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add Command callback failed.");
  }

  //Adding frame received callback, called when a streaming frame has been received from the device
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

void vpRobotBebop2::initCodec()
{
  av_register_all();
  avcodec_register_all();
  avformat_network_init();

  av_init_packet(&m_packet);

  AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Codec not found.");
    return;
  }

  m_codecContext = avcodec_alloc_context3(codec);
  if (!m_codecContext) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to allocate codec context.");
    return;
  }

  avcodec_get_context_defaults3(m_codecContext, codec);
  m_codecContext->flags |= CODEC_FLAG_LOW_DELAY;
  m_codecContext->flags2 |= CODEC_FLAG2_CHUNKS;
  m_codecContext->thread_count = 4;
  m_codecContext->thread_type = FF_THREAD_SLICE;
  m_codecContext->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;
  m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
  m_codecContext->skip_frame = AVDISCARD_DEFAULT;
  m_codecContext->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  m_codecContext->skip_loop_filter = AVDISCARD_DEFAULT;
  m_codecContext->workaround_bugs = FF_BUG_AUTODETECT;
  m_codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
  m_codecContext->codec_id = AV_CODEC_ID_H264;
  m_codecContext->skip_idct = AVDISCARD_DEFAULT;

  if (avcodec_open2(m_codecContext, codec, nullptr) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to open codec.");
    return;
  }
  m_packet.pts = AV_NOPTS_VALUE;
  m_packet.dts = AV_NOPTS_VALUE;
}

void vpRobotBebop2::cleanUpCodec()
{
  m_videoDecodingStarted = false;
  std::cout << "BEFORE FREEING CODEC" << std::endl;
  avcodec_free_context(&m_codecContext);
  std::cout << "AFTER FREEING CODEC" << std::endl;
}

void vpRobotBebop2::startVideoDecoding()
{
  if (!m_videoDecodingStarted) {
    initCodec();

    m_currentImage.resize(480, 856, 0);
    m_videoDecodingStarted = true;
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Video decoding is already started.");
  }
}

void vpRobotBebop2::stopVideoDecoding()
{
  if (m_videoDecodingStarted) {
    cleanUpCodec();
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Video decoding is already stopped.");
  }
}

void vpRobotBebop2::computeFrame(ARCONTROLLER_Frame_t *frame)
{
  AVFrame *picture;
  AVFrame *rgb_picture;
  picture = av_frame_alloc();
  rgb_picture = av_frame_alloc();
  AVPixelFormat pFormat = AV_PIX_FMT_BGR24;

  if (!picture) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to allocate video frame.");
    return;
  }

  m_codecContext->width = 856;
  m_codecContext->height = 480;
  int numBytes = av_image_get_buffer_size(pFormat, m_codecContext->width, m_codecContext->height, 1);

  m_packet.data = frame->data;
  m_packet.size = static_cast<int>(frame->used);

  uint8_t *buffer = (uint8_t *)av_malloc(static_cast<unsigned long>(numBytes) * sizeof(uint8_t));

  av_image_fill_arrays(rgb_picture->data, rgb_picture->linesize, buffer, pFormat, m_codecContext->width,
                       m_codecContext->height, 1);

  int ret = avcodec_send_packet(m_codecContext, &m_packet);
  if (ret < 0) {

    char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
    std::string err(errbuff);
    delete[] errbuff;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error sending a packet for decoding : %d, %s", ret, err.c_str());

  } else {

    ret = avcodec_receive_frame(m_codecContext, picture);

    if (ret < 0) {

      if (ret == AVERROR(EAGAIN)) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "AVERROR(EAGAIN)");
        return;
      }
      if (ret == AVERROR_EOF) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "AVERROR_EOF");
        return;
      }

      char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
      std::string err(errbuff);
      delete[] errbuff;
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error receiving a decoded frame : %d, %s", ret, err.c_str());

    } else {

      struct SwsContext *img_convert_ctx;
      img_convert_ctx =
          sws_getContext(m_codecContext->width, m_codecContext->height, m_codecContext->pix_fmt, m_codecContext->width,
                         m_codecContext->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
      sws_scale(img_convert_ctx, (picture)->data, (picture)->linesize, 0, m_codecContext->height, (rgb_picture)->data,
                (rgb_picture)->linesize);

      vpImageConvert::RGBToGrey(rgb_picture->data[0], m_currentImage.bitmap, m_currentImage.getWidth(),
                                m_currentImage.getHeight());
    }
  }

  av_free(picture);
  av_free(rgb_picture);
}

void vpRobotBebop2::cleanUp()
{
  if (m_deviceController != NULL) {
    // Lands the drone if not landed
    land();

    // Stops the streaming if not stopped
    stopStreaming();

    // Deletes the controller
    m_deviceState = ARCONTROLLER_Device_GetState(m_deviceController, &m_errorController);
    if ((m_errorController == ARCONTROLLER_OK) && (m_deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");
      m_errorController = ARCONTROLLER_Device_Stop(m_deviceController);

      if (m_errorController == ARCONTROLLER_OK) {
        // wait state update update
        ARSAL_Sem_Wait(&(m_stateSem));
      }
    }

    m_running = false;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Deleting device controller ...");
    ARCONTROLLER_Device_Delete(&m_deviceController);

    // Destroys the semaphore
    ARSAL_Sem_Destroy(&(m_stateSem));

    std::cout << "-- CLEANUP DONE --" << std::endl;
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while cleaning up memory.");
  }
}

//***           ***//
//*** Callbacks ***//
//***           ***//

void vpRobotBebop2::stateChanged(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void * customData)
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Controller state changed, newState: %d .....", newState);
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
  (void)codec;
  (void)customData;
  return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR vpRobotBebop2::didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData)
{
  vpRobotBebop2 *drone = (vpRobotBebop2 *)customData;

  if (frame != NULL) {

    if (drone->m_videoDecodingStarted) {
      drone->computeFrame(frame);
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
  }

  return ARCONTROLLER_OK;
}

void vpRobotBebop2::cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                              vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

  if (elementDictionary == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
    return;
  }

  // Suppress warnings
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
  // Get the command received in the device controller
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

  if (singleElement == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
    return;
  }

  // Suppress warnings
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
  // Get the value
  HASH_FIND_STR(singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
                arg);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

  if (arg == NULL) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
    return;
  }
  drone->m_batteryLevel = arg->value.U8;
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Battery level changed : %u percent remaining.", drone->m_batteryLevel);

  if (drone->m_batteryLevel <= 5) {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "    - WARNING, very low battery level, drone will stop soon !");
  } else if (drone->m_batteryLevel <= 10) {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "    - Warning, low battery level !");
  }
}

void vpRobotBebop2::cmdMaxPitchRollChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                              vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
  if (element != NULL) {
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_CURRENT,
                  arg);
    if (arg != NULL) {
      float current = arg->value.Float;
      drone->m_maxTilt = current;
    }
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
  }
}

void vpRobotBebop2::cmdRelativeMoveEndedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = NULL;

#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

  if (element != NULL) {
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wcast-qual"
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR, arg);
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

    if (arg != NULL) {
      eARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR error =
          static_cast<eARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR>(arg->value.I32);
      if ((error != ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_OK) &&
          (error != ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_INTERRUPTED)) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Relative move ended with error %d", error);
      }
      drone->m_relativeMoveEnded = true;
    }
  }
}
void vpRobotBebop2::commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey,
                                            ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData)
{
  vpRobotBebop2 *drone = (vpRobotBebop2 *)customData;

  if (drone == NULL)
    return;

  switch (commandKey) {
  case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
    // If the command received is a battery state changed
    cmdBatteryStateChangedRcv(elementDictionary, drone);
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED:
    // If the command receivend is a max pitch/roll changed
    cmdMaxPitchRollChangedRcv(elementDictionary, drone);
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND:
    // If the command received is a relative move ended
    cmdRelativeMoveEndedRcv(elementDictionary, drone);
    break;

  default:
    break;
  }
}

#endif
