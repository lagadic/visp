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
 * Interface for Parrot Bebop2 drone.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ARSDK

#include <visp3/robot/vpRobotBebop2.h>

#include <curses.h> // For keyboard inputs
#include <iostream>

// FFmpeg is part of OpenCV
#ifdef VISP_HAVE_OPENCV

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpImageConvert.h>

#endif // #ifdef VISP_HAVE_OPENCV

#define TAG "vpRobotBebop2" // For error messages of ARSDK

bool vpRobotBebop2::m_running = false;
ARCONTROLLER_Device_t *vpRobotBebop2::m_deviceController = NULL;

/*!

  Constructor (only one available).

  Initialises drone control, by discovering drones with IP \e ipAddress and port \e discoveryPort on the wifi network
  the computer is currently connected to.
  Sets up signal handling to safely land the drone when something bad happens.

  \warning This constructor should be called after the drone is turned on, and after the computer is connected to the
  drone wifi network.

  \warning If the connection to the drone failed, commands will not be accepted. After having called this constructor,
  it is recommanded to check if the drone is running with isRunning() before sending commands to the drone.


  \param[in] verbose : turn verbose on or off
    If verbose is true : info, warning, error and fatal error messages are displayed.
    If verbose is false : only warning, error and fatal error messages are displayed.
  \param[in] ipAddress : ip address used to discover the drone on the wifi network.
  \param[in] discoveryPort : port used to discover the drone on the wifi network.
 */
vpRobotBebop2::vpRobotBebop2(bool verbose, std::string ipAddress, int discoveryPort)
  : m_ipAddress(ipAddress), m_discoveryPort(discoveryPort), m_picture(NULL), m_rgb_picture(NULL),
    m_img_convert_ctx(NULL), m_buffer(NULL), m_currentImage()
{
  // Setting up signal handling
  memset(&m_sigAct, 0, sizeof(m_sigAct));
  m_sigAct.sa_handler = vpRobotBebop2::sighandler;
  sigaction(SIGINT, &m_sigAct, 0);
  sigaction(SIGBUS, &m_sigAct, 0);
  sigaction(SIGSEGV, &m_sigAct, 0);
  sigaction(SIGKILL, &m_sigAct, 0);
  sigaction(SIGQUIT, &m_sigAct, 0);

  m_batteryLevel = 100;

  m_relativeMoveEnded = true;
  m_videoDecodingStarted = false;

  setVerbose(verbose);

  m_errorController = ARCONTROLLER_OK;
  m_deviceState = ARCONTROLLER_DEVICE_STATE_MAX;

  // Initialises a semaphore
  ARSAL_Sem_Init(&(m_stateSem), 0, 0);

  // Creates a discovery device to find the drone on the wifi network
  ARDISCOVERY_Device_t *discoverDevice = discoverDrone();

  // Creates a drone controller with the discovery device
  createDroneController(discoverDevice);

  // Sets up callbacks
  setupCallbacks();

  // Start the drone controller, connects to the drone. If an error occurs, it will set m_errorController to the error.
  startController();

  // We check if the drone was actually found and connected to the controller
  if ((m_errorController != ARCONTROLLER_OK) || (m_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
    m_running = false;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "- Failed to setup drone control. Make sure your computer is connected to the drone Wifi spot before "
                "starting.");
  } else {
    m_running = true;
  }
}

/*!

  Destructor. Lands the drone if not landed, safely disconnects everything, and deactivates video streaming and
  decoding if needed.
*/
vpRobotBebop2::~vpRobotBebop2() { cleanUp(); }

/*!

  Sends a flat trim command to the robot, to calibrate accelerometer and gyro.

  \warning Should be executed only if the drone is landed and on a flat surface.
*/
void vpRobotBebop2::doFlatTrim()
{
  if (isRunning() && m_deviceController != NULL && isLanded()) {

    m_flatTrimFinished = false;

    m_deviceController->aRDrone3->sendPilotingFlatTrim(m_deviceController->aRDrone3);

    // m_flatTrimFinished is set back to true when the drone has finished the calibration, via a callback
    while (!m_flatTrimFinished) {
      vpTime::sleepMs(1);
    }
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't do a flat trim : drone isn't landed.");
  }
}

/*!

  Gets the drone IP address used during wifi discovery.
*/
std::string vpRobotBebop2::getIpAddress() { return m_ipAddress; }

/*!

  Gets the drone port used during wifi discovery.
*/
int vpRobotBebop2::getDiscoveryPort() { return m_discoveryPort; }

/*!

  Gets the current max pitch and roll values allowed for the drone.
*/
float vpRobotBebop2::getMaxTilt() { return m_maxTilt; }

/*!

  Gets current battery level.
*/
unsigned int vpRobotBebop2::getBatteryLevel() { return m_batteryLevel; }

#ifdef VISP_HAVE_OPENCV
/*!
  \warning This function is only available if ViSP is build with OpenCV support.

  Gets the last streamed and decoded image from the drone camera feed in grayscale.
  The image obtained has a width of 856 and a height of 480.

  \param[in,out] I : grayscale image that will contain last streamed and decoded image after the function ends.
*/
void vpRobotBebop2::getGrayscaleImage(vpImage<unsigned char> &I)
{
  if (m_videoDecodingStarted) {
    I.resize(m_currentImage.getHeight(), m_currentImage.getWidth());
    vpImageConvert::RGBaToGrey(reinterpret_cast<unsigned char *>(m_currentImage.bitmap), I.bitmap,
                               m_currentImage.getSize());
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't get current image : video streaming isn't started.");
  }
}

/*!
  \warning This function is only available if ViSP is build with OpenCV support.

  Gets the last streamed and decoded image from the drone camera feed in RGBa.
  The image obtained has a width of 856 and a height of 480.

  \param[in,out] I : RGBa image that will contain last streamed and decoded image after the function ends.
*/
void vpRobotBebop2::getRGBaImage(vpImage<vpRGBa> &I)
{
  if (m_videoDecodingStarted) {
    I = m_currentImage;
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't get current image : video streaming isn't started.");
  }
}
#endif // #ifdef VISP_HAVE_OPENCV

/*!

  Checks if the drone is running, ie if the drone is connected and ready to receive commands.
 */
bool vpRobotBebop2::isRunning() { return m_running; }

/*!

  Checks if the drone is currently streaming and decoding the video from its camera.
 */
bool vpRobotBebop2::isStreaming() { return m_videoDecodingStarted; }

/*!

  Checks if the drone is currently hovering, ie if the drone is up in the air but not moving.
 */
bool vpRobotBebop2::isHovering()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING;
}

/*!

  Checks if the drone is currently flying, ie if the drone is up in the air and moving.
 */
bool vpRobotBebop2::isFlying()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING;
}

/*!

  Checks if the drone is currently landed.
 */
bool vpRobotBebop2::isLanded()
{
  return getFlyingState() == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED;
}

/*!

  Sends take off command.

  \param[in] blocking : If true, the function return when take off is achieved. If false, returns immediately. You can
  check if take off is finished using isHovering().
 */
void vpRobotBebop2::takeOff(bool blocking)
{
  if (isRunning() && isLanded() && m_deviceController != NULL) {

    m_deviceController->aRDrone3->sendPilotingTakeOff(m_deviceController->aRDrone3);

    if (blocking) {
      while (!isHovering()) {
        vpTime::sleepMs(1);
      }
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't take off : drone isn't landed.");
  }
}

/*!

  Sends landing command.

  \warning This function is static because it needs to be called by the signal handler in case of a detected signal.
 */
void vpRobotBebop2::land()
{
  if (m_deviceController != NULL) {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
}

/*!

  Moves the drone by the given amounts \e dX, \e dY, \e dZ (meters) and rotate the heading by \e dPsi (radian).
  Doesn't do anything if the drone isn't flying or hovering.

  \param[in] dX : displacement along X axis (meters).
  \param[in] dY : displacement along Y axis (meters).
  \param[in] dZ : displacement along Z axis (meters).
  \param[in] dPsi : rotation of the heading (radians).
  \param[in] blocking : specifies whether the function should be blocking or not.
    If blocking is true, the function will wait until the drone has finished moving.
    If blocking is false, the function will return even if the drone is still moving.

  \sa setPosition(const vpHomogeneousMatrix &M, bool blocking)
*/
void vpRobotBebop2::setPosition(float dX, float dY, float dZ, float dPsi, bool blocking)
{
  if (isRunning() && m_deviceController != NULL && (isFlying() || isHovering())) {

    m_relativeMoveEnded = false;
    m_deviceController->aRDrone3->sendPilotingMoveBy(m_deviceController->aRDrone3, dX, dY, dZ, dPsi);

    if (blocking) {

      // m_relativeMoveEnded is set back to true when the drone has finished his move, via a callback
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

  \param[in] M : homogeneous matrix. Translation should be expressed in meters and rotation in radians.
  \param[in] blocking : specifies whether the function should be blocking or not.
    If blocking is true, the function will wait until the drone has finished moving.
    If blocking is false, the function will return even if the drone is still moving.

  \warning The rotation around the X and Y axes should be equal to 0, as the drone cannot rotate around these axes.

  \sa setPosition(float dX, float dY, float dZ, float dPsi, bool blocking)
*/
void vpRobotBebop2::setPosition(const vpHomogeneousMatrix &M, bool blocking)
{
  double epsilon = (std::numeric_limits<double>::epsilon());
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

  \param[in] vel_cmd : Drone velocity commands, vx, vy, vz, wz. Translation velocities (vx, vy, vz) should be expressed
  in meters and rotation velocity (wz) in radians.
  \param[in] delta_t : Sampling time (in seconds), time during which the
  velocity \e vel_cmd is applied.

  \warning The dimension of the velocity vector should be equal to 4, as the drone cannot rotate around X and Y axes.
 */
void vpRobotBebop2::setVelocity(const vpColVector &vel_cmd, double delta_t)
{

  if (vel_cmd.size() != 4) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR",
                "Can't set velocity : dimension of the velocity vector should be equal to 4.");
    stopMoving();
    return;
  }

  vpColVector ve(6);
  ve[0] = vel_cmd[0];
  ve[1] = vel_cmd[1];
  ve[2] = vel_cmd[2];
  ve[5] = vel_cmd[3];
  vpHomogeneousMatrix M = vpExponentialMap::direct(ve, delta_t);
  setPosition(M, false);
}

/*!

  Sets the verbose level for console prints.

  \param[in] verbose : specifies the desired verbose level.
    If verbose is true : info, warning, error and fatal error messages are displayed.
    If verbose is false : only warning, error and fatal error messages are displayed.
*/
void vpRobotBebop2::setVerbose(bool verbose)
{
  if (verbose) {
    ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_INFO);
  } else {
    ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_WARNING);
  }
}

/*!

  Sets the max pitch and roll values for the drone.
  The smallest possible angle is 5, the maximum is 35.

  \param[in] maxTilt : new maximum pitch and roll value for the drone (degrees).

  \warning This value is only taken into account by the drone when issuing percentage-of-max-tilt-value-based commands.
  Currently only used by handleKeyboardInput.

  \warning This value is not taken into account by the drone when using setPosition or setVelocity functions.
*/
void vpRobotBebop2::setMaxTilt(float maxTilt)
{
  if (isRunning() && m_deviceController != NULL) {
    m_deviceController->aRDrone3->sendPilotingSettingsMaxTilt(m_deviceController->aRDrone3, maxTilt);
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set tilt value : drone isn't running.");
  }
}

/*!

  Starts the video streaming from the drone camera. Every time a frame is received, it is decoded and stored into \e
  m_currentImage, which can be obtained with getImage().

  \sa getImage(vpImage<unsigned char> &I)
*/
void vpRobotBebop2::startStreaming()
{
  if (isRunning() && m_deviceController != NULL) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Starting video streaming ... ");

    // Sending command to the drone to start the video stream
    m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable(m_deviceController->aRDrone3, 1);

    if (m_errorController == ARCONTROLLER_OK) {

      // Blocking until streaming is started
      while (getStreamingState() != ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_ENABLED) {
        vpTime::sleepMs(1);
      }
      startVideoDecoding();
      vpTime::sleepMs(1000); // We wait for the streaming to actually start (it has a delay before actually sending
                             // frames, even if it is indicated as started by the drone).
    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't start streaming : drone isn't running.");
  }
}

/*!

  Stops any drone movement.
*/
void vpRobotBebop2::stopMoving()
{
  if (isRunning() && !isLanded() && m_deviceController != NULL) {
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMD(m_deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
  }
}

/*!

  Stops the streaming and decoding of the drone camera video
*/
void vpRobotBebop2::stopStreaming()
{
  if (m_videoDecodingStarted && m_deviceController != NULL) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stopping video streaming ... ");

    // Sending command to the drone to stop the video stream
    m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable(m_deviceController->aRDrone3, 0);

    if (m_errorController == ARCONTROLLER_OK) {

      // Blocking until streaming is stopped
      while (getStreamingState() != ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_DISABLED) {
        vpTime::sleepMs(1);
      }
      vpTime::sleepMs(500); // We wait for the streaming to actually stops or else it causes segmentation fault.
      stopVideoDecoding();

    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't stop streaming : streaming already stopped.");
  }
}

/*!

  Sends predefined movement commands to the drone based on a keyboard input \e key.
  See keyboard control example.

  \warning Can handle directionnal arrows if Curses is used to detect input.

  \param[in] key : key input to handle.
*/
void vpRobotBebop2::handleKeyboardInput(int key)
{
  if (isRunning()) {
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
      takeOff(false);
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
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error when handling keyboard input : drone isn't running.");
  }
}

//***                   ***//
//*** Private Functions ***//
//***                   ***//

/*!

  Signal handler. Lands the drone if possible in order to prevent accidents.
*/
void vpRobotBebop2::sighandler(int signo)
{
  std::cout << "Stopping Bebop2 because of detected signal (" << signo << "): " << static_cast<char>(7);
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
  std::exit(EXIT_FAILURE);
}

/*!

  Gets the current flying state of the drone.
*/
eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE vpRobotBebop2::getFlyingState()
{
  if (m_deviceController != NULL) {
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState =
        ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
    eARCONTROLLER_ERROR error;

    ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(
        m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED, &error);

    if (error == ARCONTROLLER_OK && elementDictionary != NULL) {
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
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error when checking flying state : drone isn't connected.");
    return ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
  }
}

/*!

  Gets the current streaming state of the drone.
*/
eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED vpRobotBebop2::getStreamingState()
{
  if (m_deviceController != NULL) {
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
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error when checking streaming state : drone isn't connected.");
    return ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_MAX;
  }
}

/*!

  Discovers the drone on the wifi network and returns the detected device.
*/
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
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Drone controller created.");
  return device;
}

/*!

Create a drone controller based on a discovered device.

\param[in] discoveredDrone : discovered drone to control with the controller. Deleted at the end of this function.
*/
void vpRobotBebop2::createDroneController(ARDISCOVERY_Device_t * discoveredDrone)
{
  m_deviceController = ARCONTROLLER_Device_New (discoveredDrone, &m_errorController);
  if (m_errorController != ARCONTROLLER_OK)
  {
      ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
  }
  ARDISCOVERY_Device_Delete (&discoveredDrone);
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Device created.");
}

/*!

  Sets up callbacks, which are functions called by the drone for different kinds of events.
*/
void vpRobotBebop2::setupCallbacks()
{
  //Adding stateChanged callback, called when the state of the controller has changed
  m_errorController = ARCONTROLLER_Device_AddStateChangedCallback(m_deviceController, stateChangedCallback, this);
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
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Callbacks set up.");
}

/*!

  Starts the drone controller, which is then ready to receive commands.
*/
void vpRobotBebop2::startController()
{
  //Starts the controller
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
  m_errorController = ARCONTROLLER_Device_Start (m_deviceController);

  if(m_errorController != ARCONTROLLER_OK)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }

  // Waits for the stateChangedCallback to unclock the semaphore
  ARSAL_Sem_Wait (&(m_stateSem));

  //Checks the device state
  m_deviceState = ARCONTROLLER_Device_GetState (m_deviceController, &m_errorController);

  if((m_errorController != ARCONTROLLER_OK) || (m_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING))
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", m_deviceState);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Controller started.");
}

/*!

  Initialises the codec used to decode the drone H264 video stream.
*/
void vpRobotBebop2::initCodec()
{
  av_register_all();
  avcodec_register_all();
  avformat_network_init();

  // Finds the correct codec
  AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Codec not found.");
    return;
  }

  // Allocates memory for codec
  m_codecContext = avcodec_alloc_context3(codec);

  if (!m_codecContext) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to allocate codec context.");
    return;
  }

  // Sets codec parameters (TODO : should be done automaticaly by drone callback decoderConfigCallback
  m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
  m_codecContext->skip_frame = AVDISCARD_DEFAULT;
  m_codecContext->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  m_codecContext->skip_loop_filter = AVDISCARD_DEFAULT;
  m_codecContext->workaround_bugs = AVMEDIA_TYPE_VIDEO;
  m_codecContext->codec_id = AV_CODEC_ID_H264;
  m_codecContext->skip_idct = AVDISCARD_DEFAULT;
  m_codecContext->width = 856;
  m_codecContext->height = 480;
  if (codec->capabilities & CODEC_CAP_TRUNCATED) {
    m_codecContext->flags |= CODEC_FLAG_TRUNCATED;
  }
  m_codecContext->flags2 |= CODEC_FLAG2_CHUNKS;

  // Opens the codec
  if (avcodec_open2(m_codecContext, codec, NULL) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to open codec.");
    return;
  }

  AVPixelFormat pFormat = AV_PIX_FMT_BGR24;
  m_codecContext->width = 856;
  m_codecContext->height = 480;
  int numBytes = av_image_get_buffer_size(pFormat, m_codecContext->width, m_codecContext->height, 1);
  m_buffer = (uint8_t *)av_malloc(static_cast<unsigned long>(numBytes) * sizeof(uint8_t));

  av_init_packet(&m_packet);        // Packed used to send data to the decoder
  m_picture = av_frame_alloc();     // Frame used to receive data from the decoder
  m_rgb_picture = av_frame_alloc(); // Frame used to store rescaled frame received from the decoder

  m_img_convert_ctx = sws_getContext(m_codecContext->width, m_codecContext->height, m_codecContext->pix_fmt,
                                     m_codecContext->width, m_codecContext->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL,
                                     NULL, NULL); // Used to rescale frame received from the decoder
}

/*!

  Safely frees any memory that was allocated by the codec.
*/
void vpRobotBebop2::cleanUpCodec()
{
  m_videoDecodingStarted = false;
  av_packet_unref(&m_packet);

  if (m_codecContext) {
    avcodec_flush_buffers(m_codecContext);
    avcodec_free_context(&m_codecContext);
  }

  if (m_picture) {
    av_frame_free(&m_picture);
  }

  if (m_rgb_picture) {
    av_frame_free(&m_rgb_picture);
  }

  if (m_img_convert_ctx) {
    sws_freeContext(m_img_convert_ctx);
  }
  if (m_buffer) {
    av_free(m_buffer);
  }
}

/*!

  Starts the video decoding : initialises the codec and the image stored by the drone.
*/
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

/*!

  Stops the video decoding : safely cleans up memory allocated by the codec.
*/
void vpRobotBebop2::stopVideoDecoding()
{
  if (m_videoDecodingStarted) {
    cleanUpCodec();
  } else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Video decoding is already stopped.");
  }
}

/*!

  Decodes a H264 frame received from the drone.

  \param[in] frame : pointer to the frame sent by the drone. Called during didReceiveFrameCallback.
*/
void vpRobotBebop2::computeFrame(ARCONTROLLER_Frame_t *frame)
{

  m_packet.data = frame->data;
  m_packet.size = static_cast<int>(frame->used);

  av_image_fill_arrays(m_rgb_picture->data, m_rgb_picture->linesize, m_buffer, AV_PIX_FMT_BGR24, m_codecContext->width,
                       m_codecContext->height, 1);

  int ret = avcodec_send_packet(m_codecContext, &m_packet);
  if (ret < 0) {

    char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
    std::string err(errbuff);
    delete[] errbuff;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error sending a packet for decoding : %d, %s", ret, err.c_str());

  } else {

    ret = avcodec_receive_frame(m_codecContext, m_picture);

    if (ret < 0) {

      if (ret == AVERROR(EAGAIN)) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "AVERROR(EAGAIN)");
      } else if (ret == AVERROR_EOF) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "AVERROR_EOF");
      } else {

        char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
        std::string err(errbuff);
        delete[] errbuff;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error receiving a decoded frame : %d, %s", ret, err.c_str());
      }
    } else {
      sws_scale(m_img_convert_ctx, (m_picture)->data, (m_picture)->linesize, 0, m_codecContext->height,
                (m_rgb_picture)->data, (m_rgb_picture)->linesize);
      // TODO ; faire std::mutex sur m_picture et faire le décodage au moment de l'appel de getRGB ou gray image
      vpImageConvert::BGRToRGBa(m_rgb_picture->data[0], reinterpret_cast<unsigned char *>(m_currentImage.bitmap),
                                m_currentImage.getWidth(), m_currentImage.getHeight());
    }
  }

  av_packet_unref(&m_packet);

  av_frame_unref(m_picture);
  av_frame_unref(m_rgb_picture);
}

/*!

  Safely stops the drone controller and everything needed during drone control. Called by the destructor.
  The drone is not running anymore after this is called.
*/
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

/*!

  Callback. Called by the drone when the drone controller changes state.

  \param[in] newState : new controller state to handle.
  \param[in] error : not used.
  \param[in] customData : pointer to custom data, here used to point to the drone.
*/
void vpRobotBebop2::stateChangedCallback(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error,
                                         void *customData)
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

/*!

  Callback. Called when streaming is started, allows to get codec parameters for the H264 video streammed by the drone.
  Currently not used, as the codec parameter are currently manually set up.

  \param[in] codec : codec used to stream the drone camera video.
  \param[in] customData : pointer to custom data.
*/
eARCONTROLLER_ERROR vpRobotBebop2::decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void * customData)
{
  (void)codec;
  (void)customData;
  return ARCONTROLLER_OK;
}

/*!

  Callback. Called when the drone sends a frame to decode.
  Sends the frame to the computeFrame() function.

  \param[in] frame : the frame coded in H264 to decode.
  \param[in] customData : pointer to custom data, here used to point to the drone.
*/
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

/*!

  Gets the current battery level of the drone when a battery-level-changed callback is called.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
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

/*!

  Gets the current max pitch and roll values of the drone when a maxPitchRoll-changed callback is called.
  Used to save the value in a attribute of the drone.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
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

/*!

  Called when the drone sends a relativeMoveEnded callback.
  Used to know when the drone has finished a relative move.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
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

/*!

  Callback. Computes a command received from the drone.

  \param[in] : the type of command received from the drone.
  \param[in] elementDictionary : the object containing the data received.
  \param[in] customData : pointer to custom data, here used to point to the drone.

*/
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
  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLATTRIMCHANGED:
    // If the command received is a flat trim finished
    drone->m_flatTrimFinished = true;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Flat trim finished ...");
    break;
  default:
    break;
  }
}

#undef TAG

#endif // VISP_HAVE_ARSDK
