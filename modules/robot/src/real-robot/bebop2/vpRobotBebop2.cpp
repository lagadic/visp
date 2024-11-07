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
 * Interface for Parrot Bebop 2 drone.
 *
 * Authors:
 * Gatien Gaumerais
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ARSDK

#include <visp3/robot/vpRobotBebop2.h>

#include <visp3/core/vpExponentialMap.h> // For velocity computation

#ifdef VISP_HAVE_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}
#include <visp3/core/vpImageConvert.h>
#endif

#include <iostream>
#include <math.h>

#define TAG "vpRobotBebop2" // For error messages of ARSDK

/*!
  Bebop coordinates system :

  Camera reference :
    + x : right
    + y : down
    + z : forward

  Effective reference :
    + x : forward
    + y : right
    + z : down
*/

BEGIN_VISP_NAMESPACE
bool vpRobotBebop2::m_running = false;
ARCONTROLLER_Device_t *vpRobotBebop2::m_deviceController = nullptr;

/*!

  Constructor (only one available).

  Initialises drone control, by discovering drones with IP \e ipAddress and port \e discoveryPort on the wifi network
  the computer is currently connected to.
  Sets up signal handling to safely land the drone when something bad happens.

  \warning This constructor should be called after the drone is turned on, and after the computer is connected to the
  drone wifi network.

  \warning If the connection to the drone failed, the program will throw an exception.

  After having called this constructor, it is recommended to check if the drone is running with isRunning() before
  sending commands to the drone.

  \param[in] verbose : turn verbose on or off
    If verbose is true : info, warning, error and fatal error messages are displayed.
    If verbose is false : only warning, error and fatal error messages are displayed.

  \param[in] setDefaultSettings : set default settings or not
    If setDefaultSettings is true : the drone is reset to factory settings and the following parameters are set :
      - Resolution of streamed video to 480p (856x480).
      - Max roll and pitch to 10 degrees.
      - Video stabilization to 0 (no stabilization).
      - Video exposure compensation to 0.
      - Video streaming mode to 0 (lowest latency, average reliability).
      - Camera orientation to 0 degrees for tilt and 0 degrees for pan
    If setDefaultSettings is false : the current settings are unchanged.

  \param[in] ipAddress : ip address used to discover the drone on the wifi network.
  \param[in] discoveryPort : port used to discover the drone on the wifi network.

  \exception vpException::fatalError : If the program failed to connect to the drone.
*/
vpRobotBebop2::vpRobotBebop2(bool verbose, bool setDefaultSettings, std::string ipAddress, int discoveryPort)
  : m_ipAddress(ipAddress), m_discoveryPort(discoveryPort)
{
  // Setting up signal handling
  memset(&m_sigAct, 0, sizeof(m_sigAct));
  m_sigAct.sa_handler = vpRobotBebop2::sighandler;
  sigaction(SIGINT, &m_sigAct, 0);
  sigaction(SIGBUS, &m_sigAct, 0);
  sigaction(SIGSEGV, &m_sigAct, 0);
  sigaction(SIGKILL, &m_sigAct, 0);
  sigaction(SIGQUIT, &m_sigAct, 0);

#ifdef VISP_HAVE_FFMPEG
  m_codecContext = nullptr;
  m_packet = AVPacket();
  m_picture = nullptr;
  m_bgr_picture = nullptr;
  m_img_convert_ctx = nullptr;
  m_buffer = nullptr;
  m_videoDecodingStarted = false;
#endif

  m_batteryLevel = 100;

  m_exposureSet = true;
  m_flatTrimFinished = true;
  m_relativeMoveEnded = true;
  m_videoResolutionSet = true;
  m_streamingStarted = false;
  m_streamingModeSet = false;
  m_settingsReset = false;

  m_update_codec_params = false;
  m_codec_params_data = std::vector<uint8_t>();

  m_maxTilt = -1;

  m_cameraHorizontalFOV = -1;
  m_currentCameraTilt = -1;
  m_minCameraTilt = -1;
  m_maxCameraTilt = -1;
  m_currentCameraPan = -1;
  m_minCameraPan = -1;
  m_maxCameraPan = -1;

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
    cleanUp();
    m_running = false;

    throw(vpException(vpException::fatalError,
                      "Failed to connect to bebop2 with ip %s and port %d. Make sure that the ip address is correct "
                      "and that your computer is connected to the drone Wifi spot before starting",
                      ipAddress.c_str(), discoveryPort));
  }
  else {
    m_running = true;

#ifdef VISP_HAVE_FFMPEG
    setVideoResolution(0);
#endif
    if (setDefaultSettings) {
      resetAllSettings();

      setMaxTilt(10);

#ifdef VISP_HAVE_FFMPEG
      setVideoStabilisationMode(0);
      setExposure(0);
      setStreamingMode(0);
#endif
      setCameraOrientation(0, 0, true);
    }
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
  if (isRunning() && m_deviceController != nullptr && isLanded()) {

    m_flatTrimFinished = false;

    m_deviceController->aRDrone3->sendPilotingFlatTrim(m_deviceController->aRDrone3);

    // m_flatTrimFinished is set back to true when the drone has finished the calibration, via a callback
    while (!m_flatTrimFinished) {
      vpTime::sleepMs(1);
    }
  }
  else {
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
double vpRobotBebop2::getMaxTilt() { return m_maxTilt; }

/*!

  Gets current battery level.

  \warning When the drone battery gets to 0, it will automatically land and won't process any command.
*/
unsigned int vpRobotBebop2::getBatteryLevel() { return m_batteryLevel; }

/*!

  Gets camera horizontal FOV (degrees).
*/
double vpRobotBebop2::getCameraHorizontalFOV() const { return m_cameraHorizontalFOV; }

/*!

  Gets current camera tilt (degrees).
*/
double vpRobotBebop2::getCurrentCameraTilt() const { return m_currentCameraTilt; }

/*!

  Gets minimum camera tilt (degrees).
*/
double vpRobotBebop2::getMinCameraTilt() const { return m_minCameraTilt; }

/*!

  Gets maximum camera tilt (degrees).
*/
double vpRobotBebop2::getMaxCameraTilt() const { return m_maxCameraTilt; }

/*!

  Gets current camera pan (degrees).
*/
double vpRobotBebop2::getCurrentCameraPan() const { return m_currentCameraPan; }

/*!

  Gets minimum camera pan (degrees).
*/
double vpRobotBebop2::getMinCameraPan() const { return m_minCameraPan; }

/*!

  Gets maximum camera pan (degrees).
*/
double vpRobotBebop2::getMaxCameraPan() const { return m_maxCameraPan; }

/*!

  Sets camera orientation : tilt and pan (degrees).

  \warning The camera movement is gradual. It takes some time for the camera to reach the desired orientation.

  \param[in] tilt : The desired tilt.
  \param[in] pan : The desired pan.
  \param[in] blocking : If true, the function returns when the camera reached the desired position.
                        If false, returns immediately. You can check if the camera reached the desired position
                        using getCurrentCameraTilt() and getCurrentCameraPan().
*/
void vpRobotBebop2::setCameraOrientation(double tilt, double pan, bool blocking)
{
  if (isRunning() && m_deviceController != nullptr) {

    m_deviceController->aRDrone3->sendCameraOrientationV2(m_deviceController->aRDrone3, static_cast<float>(tilt),
                                                          static_cast<float>(pan));

    if (blocking) {
      while (std::abs(tilt - m_currentCameraTilt) > 0.01 || std::abs(pan - m_currentCameraPan) > 0.01) {
        vpTime::sleepMs(1);
      }
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set camera orientation : drone isn't running.");
  }
}

/*!

  Sets camera tilt (degrees).

  \warning The camera movement is gradual. It takes some time for the camera to reach the desired tilt.

  \param[in] tilt : The desired tilt.
  \param[in] blocking : If true, the function returns when the camera reached the desired position.
                        If false, returns immediately. You can check if the camera reached the desired position
                        using getCurrentCameraTilt().
*/
void vpRobotBebop2::setCameraTilt(double tilt, bool blocking)
{
  if (isRunning() && m_deviceController != nullptr) {

    m_deviceController->aRDrone3->sendCameraOrientationV2(m_deviceController->aRDrone3, static_cast<float>(tilt),
                                                          static_cast<float>(getCurrentCameraPan()));

    if (blocking) {
      while (std::abs(tilt - m_currentCameraTilt) > 0.01) {
        vpTime::sleepMs(1);
      }
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set camera tilt value : drone isn't running.");
  }
}

/*!

  Sets camera pan (degrees).

  \warning The camera movement is gradual. It takes some time for the camera to reach the desired pan.

  \param[in] pan : The desired pan.
  \param[in] blocking : If true, the function returns when the camera reached the desired position.
                        If false, returns immediately. You can check if the camera reached the desired position
                        using getCurrentCameraPan().
*/
void vpRobotBebop2::setCameraPan(double pan, bool blocking)
{
  if (isRunning() && m_deviceController != nullptr) {

    m_deviceController->aRDrone3->sendCameraOrientationV2(
        m_deviceController->aRDrone3, static_cast<float>(getCurrentCameraTilt()), static_cast<float>(pan));

    if (blocking) {
      while (std::abs(pan - m_currentCameraPan) > 0.01) {
        vpTime::sleepMs(1);
      }
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set camera pan value : drone isn't running.");
  }
}

/*!

  Checks if the drone is running, ie if the drone is connected and ready to receive commands.
*/
bool vpRobotBebop2::isRunning()
{
  if (m_deviceController == nullptr) {
    return false;
  }
  else {
    return m_running;
  }
}

/*!

  Checks if the drone is currently streaming and decoding the video from its camera.
*/
bool vpRobotBebop2::isStreaming()
{
#ifdef VISP_HAVE_FFMPEG
  return m_videoDecodingStarted;
#else
  return false;
#endif
}

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

  \param[in] blocking : If true, the function returns when take off is achieved. If false, returns immediately. You can
  check if take off is finished using isHovering().
*/
void vpRobotBebop2::takeOff(bool blocking)
{
  if (isRunning() && isLanded() && m_deviceController != nullptr) {

    m_deviceController->aRDrone3->sendPilotingTakeOff(m_deviceController->aRDrone3);

    if (blocking) {
      while (!isHovering()) {
        vpTime::sleepMs(1);
      }
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't take off : drone isn't landed.");
  }
}

/*!

  Sends landing command.

  \warning This function is static because it needs to be called by the signal handler in case of a detected signal.
*/
void vpRobotBebop2::land()
{
  if (m_deviceController != nullptr) {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
}

/*!

  Sets the vertical speed, expressed as signed percentage of the maximum vertical speed.

  \warning The drone will not stop moving in that direction until you send another motion command.

  \param[in] value : desired vertical speed in signed percentage, between 100 and -100.
    Positive values will make the drone go up
    Negative values will make the drone go down
*/
void vpRobotBebop2::setVerticalSpeed(int value)
{
  if (isRunning() && m_deviceController != nullptr && (isFlying() || isHovering())) {
    m_errorController =
      m_deviceController->aRDrone3->setPilotingPCMDGaz(m_deviceController->aRDrone3, static_cast<char>(value));

    if (m_errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error when sending move command : %s",
                  ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set vertical speed : drone isn't flying or hovering.");
  }
}

/*!

  Sets the yaw speed, expressed as signed percentage of the maximum yaw speed.

  \warning The drone will not stop moving in that direction until you send another motion command.

  \param[in] value : desired yaw speed in signed percentage, between 100 and -100.
    Positive values will make the drone turn to its right / clockwise
    Negative values will make the drone turn to its left / counterclockwise
*/
void vpRobotBebop2::setYawSpeed(int value)
{
  if (isRunning() && m_deviceController != nullptr && (isFlying() || isHovering())) {

    m_errorController =
      m_deviceController->aRDrone3->setPilotingPCMDYaw(m_deviceController->aRDrone3, static_cast<char>(value));

    if (m_errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error when sending move command : %s",
                  ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set yaw speed : drone isn't flying or hovering.");
  }
}

/*!

  Sets the pitch angle, expressed as signed percentage of the maximum pitch angle.

  \warning The drone will not stop moving in that direction until you send another motion command.

  \param[in] value : desired pitch in signed percentage, between 100 and -100.
    Positive values will make the drone tilt and go forward
    Negative values will make the drone tilt and go backward
*/
void vpRobotBebop2::setPitch(int value)
{
  if (isRunning() && m_deviceController != nullptr && (isFlying() || isHovering())) {

    m_errorController =
      m_deviceController->aRDrone3->setPilotingPCMDPitch(m_deviceController->aRDrone3, static_cast<char>(value));
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);

    if (m_errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error when sending move command : %s",
                  ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set pitch value : drone isn't flying or hovering.");
  }
}

/*!

  Sets the roll angle, expressed as signed percentage of the maximum roll angle.

  \warning The drone will not stop moving in that direction until you send another motion command.

  \param[in] value : desired roll in signed percentage, between 100 and -100.
    Positive values will make the drone tilt and go to its right
    Negative values will make the drone tilt and go to its left
*/
void vpRobotBebop2::setRoll(int value)
{
  if (isRunning() && m_deviceController != nullptr && (isFlying() || isHovering())) {

    m_errorController =
      m_deviceController->aRDrone3->setPilotingPCMDRoll(m_deviceController->aRDrone3, static_cast<char>(value));
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMDFlag(m_deviceController->aRDrone3, 1);

    if (m_errorController != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error when sending move command : %s",
                  ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set roll value : drone isn't flying or hovering.");
  }
}

/*!

  Cuts the motors. Should only be used in emergency cases.

  \warning The drone will fall.
*/
void vpRobotBebop2::cutMotors()
{
  if (m_deviceController != nullptr) {
    m_errorController = m_deviceController->aRDrone3->sendPilotingEmergency(m_deviceController->aRDrone3);
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
  if (isRunning() && m_deviceController != nullptr && (isFlying() || isHovering())) {

    m_relativeMoveEnded = false;
    m_deviceController->aRDrone3->sendPilotingMoveBy(m_deviceController->aRDrone3, dX, dY, dZ, dPsi);

    if (blocking) {

      // m_relativeMoveEnded is set back to true when the drone has finished his move, via a callback
      while (!m_relativeMoveEnded) {
        vpTime::sleepMs(1);
      }
    }
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't move : drone isn't flying or hovering.");
  }
}

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
  }
  else {
    ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_WARNING);
  }
}

/*!

  Resets drone settings (like max tilt or streaming mode) to factory defaults.
*/
void vpRobotBebop2::resetAllSettings()
{
  if (isRunning() && m_deviceController != nullptr) {

    m_settingsReset = false;
    m_deviceController->common->sendSettingsReset(m_deviceController->common);

    while (!m_settingsReset) {
      vpTime::sleepMs(1);
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't reset drone settings : drone isn't running.");
  }
}

/*!

  Sets the max pitch and roll values for the drone.
  The smallest possible angle is 5, the maximum is 35.

  \param[in] maxTilt : new maximum pitch and roll value for the drone (degrees).

  \warning This value is only taken into account by the drone when issuing percentage-of-max-tilt-value-based commands.

  \warning This value is not taken into account by the drone when using setPosition or setVelocity functions.
*/
void vpRobotBebop2::setMaxTilt(double maxTilt)
{
  if (isRunning() && m_deviceController != nullptr) {
    m_deviceController->aRDrone3->sendPilotingSettingsMaxTilt(m_deviceController->aRDrone3,
                                                              static_cast<float>(maxTilt));
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set tilt value : drone isn't running.");
  }
}

/*!

  Stops any drone movement.

  \warning Depending on the speed of the drone when the function is called, the drone may still move a bit until it
    stabilizes.
*/
void vpRobotBebop2::stopMoving()
{
  if (isRunning() && !isLanded() && m_deviceController != nullptr) {
    m_errorController = m_deviceController->aRDrone3->setPilotingPCMD(m_deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
  }
}

//***                     ***//
//*** Streaming functions ***//
//***                     ***//

#ifdef VISP_HAVE_FFMPEG // Functions related to video streaming and decoding requiers FFmpeg

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Gets the last streamed and decoded image from the drone camera feed in grayscale.
  The image obtained has a width of 856 and a height of 480.

  \param[in,out] I : grayscale image that will contain last streamed and decoded image after the function ends.
*/
void vpRobotBebop2::getGrayscaleImage(vpImage<unsigned char> &I)
{
  if (m_videoDecodingStarted) {

    if (m_bgr_picture->data[0] != nullptr) {
      I.resize(static_cast<unsigned int>(m_videoHeight), static_cast<unsigned int>(m_videoWidth));

      m_bgr_picture_mutex.lock();
      vpImageConvert::BGRToGrey(m_bgr_picture->data[0], reinterpret_cast<unsigned char *>(I.bitmap), I.getWidth(),
                                I.getHeight());
      m_bgr_picture_mutex.unlock();
    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Error while getting current grayscale image : image data is nullptr");
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't get current image : video streaming isn't started.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Gets the last streamed and decoded image from the drone camera feed in RGBa.
  The image obtained has a width of 856 and a height of 480.

  \param[in,out] I : RGBa image that will contain last streamed and decoded image after the function ends.
*/
void vpRobotBebop2::getRGBaImage(vpImage<vpRGBa> &I)
{
  if (m_videoDecodingStarted) {

    if (m_bgr_picture->data[0] != nullptr) {
      I.resize(static_cast<unsigned int>(m_videoHeight), static_cast<unsigned int>(m_videoWidth));

      m_bgr_picture_mutex.lock();
      vpImageConvert::BGRToRGBa(m_bgr_picture->data[0], reinterpret_cast<unsigned char *>(I.bitmap), I.getWidth(),
                                I.getHeight());
      m_bgr_picture_mutex.unlock();
    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Error while getting current RGBa image : image data is nullptr");
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't get current image : video streaming isn't started.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Gets the height of the video streamed by the drone (pixels).
*/
int vpRobotBebop2::getVideoHeight() { return m_videoHeight; }

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Gets the width of the video streamed by the drone (pixels).
*/
int vpRobotBebop2::getVideoWidth() { return m_videoWidth; }

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Sets the exposure compensation of the video (min : -1.5, max : 1.5).

  \param[in] expo : desired video exposure compensation.
*/
void vpRobotBebop2::setExposure(float expo)
{
  if (isRunning() && m_deviceController != nullptr) {
    expo = std::min<float>(1.5f, std::max<float>(-1.5f, expo));

    m_exposureSet = false;
    m_deviceController->aRDrone3->sendPictureSettingsExpositionSelection(m_deviceController->aRDrone3, expo);

    // m_exposureSet is set back to true when the drone has finished his move, via a callback
    while (!m_exposureSet) {
      vpTime::sleepMs(1);
    }
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set exposure : drone isn't running.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Sets the streaming mode.

  \warning This function should be called only if the drone isn't flying and the streaming isn't started.

  \param[in] mode : desired streaming mode.
    If mode = 0 (default mode), the streaming minimizes latency with average reliability (best for piloting).
    If mode = 1, the streaming maximizes the reliability with an average latency (best when streaming quality is
  important but not the latency).
    If mode = 2, the streaming maximizes the reliability using a framerate decimation with an average latency (best when
  streaming quality is important but not the latency).
*/
void vpRobotBebop2::setStreamingMode(int mode)
{
  if (isRunning() && m_deviceController != nullptr) {

    if (!isStreaming() && isLanded()) {
      eARCOMMANDS_ARDRONE3_MEDIASTREAMING_VIDEOSTREAMMODE_MODE cmd_mode =
        ARCOMMANDS_ARDRONE3_MEDIASTREAMING_VIDEOSTREAMMODE_MODE_LOW_LATENCY;
      switch (mode) {
      case 0:
        cmd_mode = ARCOMMANDS_ARDRONE3_MEDIASTREAMING_VIDEOSTREAMMODE_MODE_LOW_LATENCY;
        break;
      case 1:
        cmd_mode = ARCOMMANDS_ARDRONE3_MEDIASTREAMING_VIDEOSTREAMMODE_MODE_HIGH_RELIABILITY;
        break;
      case 2:
        cmd_mode = ARCOMMANDS_ARDRONE3_MEDIASTREAMING_VIDEOSTREAMMODE_MODE_HIGH_RELIABILITY_LOW_FRAMERATE;
        break;
      default:
        break;
      }
      m_streamingModeSet = false;
      m_deviceController->aRDrone3->sendMediaStreamingVideoStreamMode(m_deviceController->aRDrone3, cmd_mode);

      // m_streamingModeSet is set back to true when the drone has finished setting the stream mode, via a callback
      while (!m_streamingModeSet) {
        vpTime::sleepMs(1);
      }

    }
    else {
      ARSAL_PRINT(
          ARSAL_PRINT_ERROR, "ERROR",
          "Can't set streaming mode : drone has to be landed and not streaming in order to set streaming mode.");
    }
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set streaming mode : drone isn't running.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Sets the streaming mode.

  \warning This function should be called only if the drone isn't flying and the streaming isn't started.

  \param[in] mode : desired streaming mode.
    If mode = 0 (default mode), the resolution is 480p (856x480).
    If mode = 1, the resolution is 720p (1280x720).
*/
void vpRobotBebop2::setVideoResolution(int mode)
{
  if (isRunning() && m_deviceController != nullptr) {

    if (!isStreaming() && isLanded()) {

      eARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEORESOLUTIONS_TYPE cmd_mode;

      switch (mode) {

      case 0:
      default:
        cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEORESOLUTIONS_TYPE_REC1080_STREAM480;
        m_videoWidth = 856;
        m_videoHeight = 480;
        break;

      case 1:
        cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEORESOLUTIONS_TYPE_REC720_STREAM720;
        m_videoWidth = 1280;
        m_videoHeight = 720;
        break;
      }

      m_videoResolutionSet = false;
      m_deviceController->aRDrone3->sendPictureSettingsVideoResolutions(m_deviceController->aRDrone3, cmd_mode);

      // m_videoResolutionSet is set back to true when the drone has finished setting the resolution, via a callback
      while (!m_videoResolutionSet) {
        vpTime::sleepMs(1);
      }

    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR",
                  "Can't set video resolution : drone has to be landed and not streaming in order to set streaming "
                  "parameters.");
    }
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set video resolution : drone isn't running.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Sets the video stabilisation mode.

  \param[in] mode : desired stabilisation mode.
    If mode = 0, the video follows drone angles (no stabilisation, default).
    If mode = 1, the video is stabilised on roll only.
    If mode = 2, the video is stabilised on pitch only.
    If mode = 3, the video is stabilised on both pitch and roll.
*/
void vpRobotBebop2::setVideoStabilisationMode(int mode)
{
  if (isRunning() && m_deviceController != nullptr) {

    eARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE cmd_mode =
      ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE_NONE;
    switch (mode) {

    case 0:
      cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE_NONE;
      break;
    case 1:
      cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE_ROLL;
      break;
    case 2:
      cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE_PITCH;
      break;
    case 3:
      cmd_mode = ARCOMMANDS_ARDRONE3_PICTURESETTINGS_VIDEOSTABILIZATIONMODE_MODE_ROLL_PITCH;
      break;

    default:
      break;
    }
    m_deviceController->aRDrone3->sendPictureSettingsVideoStabilizationMode(m_deviceController->aRDrone3, cmd_mode);

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't set video stabilisation mode : drone isn't running.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Starts the video streaming from the drone camera. Every time a frame is received, it is decoded and stored into \e
  m_currentImage, which can be obtained with getImage().

  \sa getImage(vpImage<unsigned char> &I)
*/
void vpRobotBebop2::startStreaming()
{
  if (isRunning() && m_deviceController != nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Starting video streaming ... ");

    // Sending command to the drone to start the video stream
    m_errorController = m_deviceController->aRDrone3->sendMediaStreamingVideoEnable(m_deviceController->aRDrone3, 1);

    if (m_errorController == ARCONTROLLER_OK) {
      m_streamingStarted = false;
      // Blocking until streaming is started
      while (!m_streamingStarted) {
        vpTime::sleepMs(1);
      }
      startVideoDecoding();

      /* We wait for the streaming to actually start (it has a delay before actually
      sending frames, even if it is indicated as started by the drone), or else the
      decoder is doesn't have time to synchronize with the stream */
      vpTime::sleepMs(4000);

    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't start streaming : drone isn't running.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Stops the streaming and decoding of the drone camera video
*/
void vpRobotBebop2::stopStreaming()
{
  if (m_videoDecodingStarted && m_deviceController != nullptr) {
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

    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Can't stop streaming : streaming already stopped.");
  }
}

#endif // #ifdef VISP_HAVE_FFMPEG

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
  if (m_deviceController != nullptr) {
    m_deviceController->aRDrone3->sendPilotingLanding(m_deviceController->aRDrone3);
  }
  std::exit(EXIT_FAILURE);
}

/*!

  Gets the current flying state of the drone.
*/
eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE vpRobotBebop2::getFlyingState()
{
  if (m_deviceController != nullptr) {
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState =
      ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
    eARCONTROLLER_ERROR error;

    ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(
        m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED, &error);

    if (error == ARCONTROLLER_OK && elementDictionary != nullptr) {
      ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
      ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;

      HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);

      if (element != nullptr) {
        // Suppress warnings
        //  Get the value
        HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE,
                      arg);

        if (arg != nullptr) {
          // Enums are stored as I32
          flyingState = static_cast<eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE>(arg->value.I32);
        }
      }
    }
    return flyingState;
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error when checking flying state : drone isn't connected.");
    return ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
  }
}

/*!

  Gets the current streaming state of the drone.
*/
eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED vpRobotBebop2::getStreamingState()
{
  if (m_deviceController != nullptr) {
    eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED streamingState =
      ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_MAX;
    eARCONTROLLER_ERROR error;

    ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary = ARCONTROLLER_ARDrone3_GetCommandElements(
        m_deviceController->aRDrone3, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED,
        &error);

    if (error == ARCONTROLLER_OK && elementDictionary != nullptr) {
      ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
      ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;

      HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);

      if (element != nullptr) {
        // Get the value
        HASH_FIND_STR(element->arguments,
                      ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED, arg);

        if (arg != nullptr) {
          // Enums are stored as I32
          streamingState =
            static_cast<eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED>(arg->value.I32);
        }
      }
    }
    return streamingState;
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error when checking streaming state : drone isn't connected.");
    return ARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED_MAX;
  }
}

/*!

  Discovers the drone on the wifi network and returns the detected device.
*/
ARDISCOVERY_Device_t *vpRobotBebop2::discoverDrone()
{
  eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

  ARDISCOVERY_Device_t *device = ARDISCOVERY_Device_New(&errorDiscovery);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Starting drone Wifi discovery ...");
  const char *charIpAddress = m_ipAddress.c_str();
  errorDiscovery =
    ARDISCOVERY_Device_InitWifi(device, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", charIpAddress, m_discoveryPort);

  if (errorDiscovery != ARDISCOVERY_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
  }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Drone controller created.");
  return device;
}

/*!

Create a drone controller based on a discovered device.

\param[in] discoveredDrone : discovered drone to control with the controller. Deleted at the end of this function.
*/
void vpRobotBebop2::createDroneController(ARDISCOVERY_Device_t *discoveredDrone)
{
  m_deviceController = ARCONTROLLER_Device_New(discoveredDrone, &m_errorController);
  if (m_errorController != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
  }
  ARDISCOVERY_Device_Delete(&discoveredDrone);
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Device created.");
}

/*!

  Sets up callbacks, which are functions called by the drone for different kinds of events.
*/
void vpRobotBebop2::setupCallbacks()
{
  // Adding stateChanged callback, called when the state of the controller has changed
  m_errorController = ARCONTROLLER_Device_AddStateChangedCallback(m_deviceController, stateChangedCallback, this);
  if (m_errorController != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
  }

  // Adding commendReceived callback, called when the a command has been received from the device
  m_errorController = ARCONTROLLER_Device_AddCommandReceivedCallback(m_deviceController, commandReceivedCallback, this);

  if (m_errorController != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "add Command callback failed.");
  }

#ifdef VISP_HAVE_FFMPEG
  // Adding frame received callback, called when a streaming frame has been received from the device
  m_errorController = ARCONTROLLER_Device_SetVideoStreamCallbacks(m_deviceController, decoderConfigCallback,
                                                                  didReceiveFrameCallback, nullptr, this);

  if (m_errorController != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error: %s", ARCONTROLLER_Error_ToString(m_errorController));
  }
#endif
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Callbacks set up.");
}

/*!

  Starts the drone controller, which is then ready to receive commands.
*/
void vpRobotBebop2::startController()
{
  // Starts the controller
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
  m_errorController = ARCONTROLLER_Device_Start(m_deviceController);

  if (m_errorController != ARCONTROLLER_OK) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }

  // Waits for the stateChangedCallback to unclock the semaphore
  ARSAL_Sem_Wait(&(m_stateSem));

  // Checks the device state
  m_deviceState = ARCONTROLLER_Device_GetState(m_deviceController, &m_errorController);

  if ((m_errorController != ARCONTROLLER_OK) || (m_deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING)) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", m_deviceState);
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(m_errorController));
  }
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Controller started.");
}

#ifdef VISP_HAVE_FFMPEG
/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

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

  // Sets codec parameters (TODO : should be done automaticaly by drone callback decoderConfigCallback)
  m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
  m_codecContext->skip_frame = AVDISCARD_DEFAULT;
  m_codecContext->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  m_codecContext->skip_loop_filter = AVDISCARD_DEFAULT;
  m_codecContext->workaround_bugs = AVMEDIA_TYPE_VIDEO;
  m_codecContext->codec_id = AV_CODEC_ID_H264;
  m_codecContext->skip_idct = AVDISCARD_DEFAULT;

  m_codecContext->width = m_videoWidth;
  m_codecContext->height = m_videoHeight;

  if (codec->capabilities & AV_CODEC_CAP_TRUNCATED) {
    m_codecContext->flags |= AV_CODEC_FLAG_TRUNCATED;
  }
  m_codecContext->flags2 |= AV_CODEC_FLAG2_CHUNKS;

  // Opens the codec
  if (avcodec_open2(m_codecContext, codec, nullptr) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to open codec.");
    return;
  }

  AVPixelFormat pFormat = AV_PIX_FMT_BGR24;
  int numBytes = av_image_get_buffer_size(pFormat, m_codecContext->width, m_codecContext->height, 1);
  m_buffer = static_cast<uint8_t *>(av_malloc(static_cast<unsigned long>(numBytes) * sizeof(uint8_t)));

  av_init_packet(&m_packet);    // Packed used to send data to the decoder
  m_picture = av_frame_alloc(); // Frame used to receive data from the decoder

  m_bgr_picture_mutex.lock();
  m_bgr_picture = av_frame_alloc(); // Frame used to store rescaled frame received from the decoder
  m_bgr_picture_mutex.unlock();

  m_img_convert_ctx = sws_getContext(m_codecContext->width, m_codecContext->height, m_codecContext->pix_fmt,
                                     m_codecContext->width, m_codecContext->height, pFormat, SWS_BICUBIC, nullptr, nullptr,
                                     nullptr); // Used to rescale frame received from the decoder
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

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

  if (m_bgr_picture) {
    m_bgr_picture_mutex.lock();
    av_frame_free(&m_bgr_picture);
    m_bgr_picture_mutex.unlock();
  }

  if (m_img_convert_ctx) {
    sws_freeContext(m_img_convert_ctx);
  }
  if (m_buffer) {
    av_free(m_buffer);
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Starts the video decoding : initialises the codec and the image stored by the drone.
*/
void vpRobotBebop2::startVideoDecoding()
{
  if (!m_videoDecodingStarted) {
    initCodec();
    m_videoDecodingStarted = true;
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Video decoding is already started.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Stops the video decoding : safely cleans up memory allocated by the codec.
*/
void vpRobotBebop2::stopVideoDecoding()
{
  if (m_videoDecodingStarted) {
    cleanUpCodec();
  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Video decoding is already stopped.");
  }
}

/*!
  \warning This function is only available if ViSP is build with ffmpeg support.

  Decodes a H264 frame received from the drone.

  \param[in] frame : pointer to the frame sent by the drone. Called during didReceiveFrameCallback.
*/
void vpRobotBebop2::computeFrame(ARCONTROLLER_Frame_t *frame)
{

  // Updating codec parameters from SPS and PPS buffers received from the drone in decoderConfigCallback
  if (m_update_codec_params && m_codec_params_data.size()) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Updating H264 codec parameters (Buffer Size: %lu) ...",
                m_codec_params_data.size());

    m_packet.data = &m_codec_params_data[0];
    m_packet.size = static_cast<int>(m_codec_params_data.size());

    int ret = avcodec_send_packet(m_codecContext, &m_packet);

    if (ret == 0) {

      ret = avcodec_receive_frame(m_codecContext, m_picture);

      if (ret == 0 || ret == AVERROR(EAGAIN)) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "H264 codec parameters updated.");
      }
      else {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unexpected error while updating H264 parameters.");
      }
    }
    else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unexpected error while sending H264 parameters.");
    }
    m_update_codec_params = false;
    av_packet_unref(&m_packet);
    av_frame_unref(m_picture);
  }

  // Decoding frame coming from the drone
  m_packet.data = frame->data;
  m_packet.size = static_cast<int>(frame->used);

  int ret = avcodec_send_packet(m_codecContext, &m_packet);
  if (ret < 0) {

    char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
    std::string err(errbuff);
    delete[] errbuff;
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error sending a packet for decoding : %d, %s", ret, err.c_str());

  }
  else {

    ret = avcodec_receive_frame(m_codecContext, m_picture);

    if (ret < 0) {

      if (ret == AVERROR(EAGAIN)) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "AVERROR(EAGAIN)"); // Not an error, need to send data again
      }
      else if (ret == AVERROR_EOF) {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "AVERROR_EOF"); // End of file reached, should not happen with drone footage
      }
      else {

        char *errbuff = new char[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuff, AV_ERROR_MAX_STRING_SIZE);
        std::string err(errbuff);
        delete[] errbuff;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error receiving a decoded frame : %d, %s", ret, err.c_str());
      }
    }
    else {
      m_bgr_picture_mutex.lock();
      av_frame_unref(m_bgr_picture);
      av_image_fill_arrays(m_bgr_picture->data, m_bgr_picture->linesize, m_buffer, AV_PIX_FMT_BGR24,
                           m_codecContext->width, m_codecContext->height, 1);

      sws_scale(m_img_convert_ctx, (m_picture)->data, (m_picture)->linesize, 0, m_codecContext->height,
                (m_bgr_picture)->data, (m_bgr_picture)->linesize);

      m_bgr_picture_mutex.unlock();
    }
  }

  av_packet_unref(&m_packet);

  av_frame_unref(m_picture);
}
#endif // #ifdef VISP_HAVE_FFMPEG
/*!

  Safely stops the drone controller and everything needed during drone control. Called by the destructor.
  The drone is not running anymore after this is called.
*/
void vpRobotBebop2::cleanUp()
{
  if (m_deviceController != nullptr) {
    // Lands the drone if not landed
    land();

#ifdef VISP_HAVE_FFMPEG
    // Stops the streaming if not stopped
    stopStreaming();
#endif

    // Deletes the controller
    m_deviceState = ARCONTROLLER_Device_GetState(m_deviceController, &m_errorController);
    if ((m_errorController == ARCONTROLLER_OK) && (m_deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED)) {

      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");
      m_errorController = ARCONTROLLER_Device_Stop(m_deviceController);

      if (m_errorController == ARCONTROLLER_OK) {
        // Wait for the semaphore to increment, it will when the controller changes its state to 'stopped'
        ARSAL_Sem_Wait(&(m_stateSem));
      }
    }
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Deleting device controller ...");
    ARCONTROLLER_Device_Delete(&m_deviceController);

    // Destroys the semaphore
    ARSAL_Sem_Destroy(&(m_stateSem));

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Cleanup done.");
  }
  else {

    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while cleaning up memory.");
  }
  m_running = false;
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
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Controller state changed, new state: %d.", newState);
  (void)error;

  vpRobotBebop2 *drone = static_cast<vpRobotBebop2 *>(customData);
  switch (newState) {
  case ARCONTROLLER_DEVICE_STATE_STOPPED:
    // Stopping the programm
    drone->m_running = false;
    // Incrementing semaphore
    ARSAL_Sem_Post(&(drone->m_stateSem));
    break;

  case ARCONTROLLER_DEVICE_STATE_RUNNING:
    // Incrementing semaphore
    ARSAL_Sem_Post(&(drone->m_stateSem));
    break;

  default:
    break;
  }
}

#ifdef VISP_HAVE_FFMPEG
/*!

  Callback. Called when streaming is started, allows to get codec parameters for the H264 video streammed by the drone.
  Currently not used, as the codec parameter are currently manually set up. Function only available when ffmpeg
  available.

  \param[in] codec : codec used to stream the drone camera video.
  \param[in] customData : pointer to custom data.
*/
eARCONTROLLER_ERROR vpRobotBebop2::decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void *customData)
{
  vpRobotBebop2 *drone = static_cast<vpRobotBebop2 *>(customData);

  uint8_t *sps_buffer_ptr = codec.parameters.h264parameters.spsBuffer;
  uint32_t sps_buffer_size = static_cast<uint32_t>(codec.parameters.h264parameters.spsSize);
  uint8_t *pps_buffer_ptr = codec.parameters.h264parameters.ppsBuffer;
  uint32_t pps_buffer_size = static_cast<uint32_t>(codec.parameters.h264parameters.ppsSize);

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "H264 configuration packet received: #SPS: %d #PPS: %d", sps_buffer_size,
              pps_buffer_size);

  drone->m_update_codec_params = (sps_buffer_ptr && pps_buffer_ptr && sps_buffer_size && pps_buffer_size &&
                                  (pps_buffer_size < 32) && (sps_buffer_size < 32));

  if (drone->m_update_codec_params) {
    // If codec parameters where received from the drone, we store them to pass them to the decoder in the next call of
    // computeFrame
    drone->m_codec_params_data.resize(sps_buffer_size + pps_buffer_size);
    std::copy(sps_buffer_ptr, sps_buffer_ptr + sps_buffer_size, drone->m_codec_params_data.begin());
    std::copy(pps_buffer_ptr, pps_buffer_ptr + pps_buffer_size, drone->m_codec_params_data.begin() + sps_buffer_size);
  }
  else {
 // If data is invalid, we clear the vector
    drone->m_codec_params_data.clear();
  }
  return ARCONTROLLER_OK;
}

/*!

  Callback. Called when the drone sends a frame to decode. Function only available when ffmpeg available.
  Sends the frame to the computeFrame() function.

  \param[in] frame : the frame coded in H264 to decode.
  \param[in] customData : pointer to custom data, here used to point to the drone.
*/
eARCONTROLLER_ERROR vpRobotBebop2::didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData)
{
  vpRobotBebop2 *drone = static_cast<vpRobotBebop2 *>(customData);

  if (frame != nullptr) {

    if (drone->m_videoDecodingStarted) {
      drone->computeFrame(frame);
    }

  }
  else {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is nullptr.");
  }

  return ARCONTROLLER_OK;
}
#endif // #ifdef VISP_HAVE_FFMPEG

/*!

  Gets the current battery level of the drone when a battery-level-changed callback is called.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
void vpRobotBebop2::cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                              vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = nullptr;

  if (elementDictionary == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is nullptr");
    return;
  }

  // Get the command received in the device controller
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

  if (singleElement == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is nullptr");
    return;
  }

  // Get the value
  HASH_FIND_STR(singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
                arg);

  if (arg == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is nullptr");
    return;
  }
  drone->m_batteryLevel = arg->value.U8;
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Battery level changed : %u percent remaining.", drone->m_batteryLevel);

  if (drone->m_batteryLevel <= 5) {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "    - WARNING, very low battery level, drone will stop soon !");
  }
  else if (drone->m_batteryLevel <= 10) {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "    - Warning, low battery level !");
  }
}

/*!

  Gets the camera pan and tilt values when a camera-orientation-changed callback is called.
  Stores the values in m_currentCameraPan and m_currentCameraTilt.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
void vpRobotBebop2::cmdCameraOrientationChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                                   vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
  if (element != nullptr) {
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2_TILT, arg);

    if (arg != nullptr) {
      drone->m_currentCameraTilt = static_cast<double>(arg->value.Float);
    }

    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2_PAN, arg);
    if (arg != nullptr) {
      drone->m_currentCameraPan = static_cast<double>(arg->value.Float);
    }
  }
}

/*!

  Gets the camera orientation values : min and max pan and tilt values which are sent via a callback when the drone
  connects.
    Stores the values in m_minCameraTilt, m_maxCameraTilt, m_minCameraPan and m_maxCameraPan.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
void vpRobotBebop2::cmdCameraSettingsRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;
  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
  if (element != nullptr) {
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_FOV,
                  arg);
    if (arg != nullptr) {
      drone->m_cameraHorizontalFOV = static_cast<double>(arg->value.Float);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Camera horizontal FOV : %f degrees.",
                  static_cast<double>(drone->m_cameraHorizontalFOV));
    }
    HASH_FIND_STR(element->arguments,
                  ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMAX, arg);
    if (arg != nullptr) {
      drone->m_maxCameraPan = static_cast<double>(arg->value.Float);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Max camera pan : %f degrees.",
                  static_cast<double>(drone->m_maxCameraPan));
    }
    HASH_FIND_STR(element->arguments,
                  ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMIN, arg);
    if (arg != nullptr) {
      drone->m_minCameraPan = static_cast<double>(arg->value.Float);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Min camera pan : %f degrees.",
                  static_cast<double>(drone->m_minCameraPan));
    }
    HASH_FIND_STR(element->arguments,
                  ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMAX, arg);
    if (arg != nullptr) {
      drone->m_maxCameraTilt = static_cast<double>(arg->value.Float);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Max camera tilt : %f degrees.",
                  static_cast<double>(drone->m_maxCameraTilt));
    }
    HASH_FIND_STR(element->arguments,
                  ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMIN, arg);
    if (arg != nullptr) {
      drone->m_minCameraTilt = static_cast<double>(arg->value.Float);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - Min camera tilt : %f degrees.",
                  static_cast<double>(drone->m_minCameraTilt));
    }
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
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;

  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);
  if (element != nullptr) {
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED_CURRENT,
                  arg);
    if (arg != nullptr) {
      drone->m_maxTilt = static_cast<double>(arg->value.Float);
    }
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
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;

  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);

  if (element != nullptr) {
    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR, arg);

    if (arg != nullptr) {
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

  Called when the drone sends a exposureChanged callback.
    Used to know when the drone has changed its video exposure level.

  \param[in] elementDictionary : the object containing the data received.
  \param[in] drone : pointer to the drone who called the callback.

  \sa commandReceivedCallback()
*/
void vpRobotBebop2::cmdExposureSetRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone)
{
  ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
  ARCONTROLLER_DICTIONARY_ELEMENT_t *element = nullptr;

  HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, element);

  if (element != nullptr) {

    HASH_FIND_STR(element->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_EXPOSITIONCHANGED_VALUE,
                  arg);

    if (arg != nullptr) {
      drone->m_exposureSet = true;
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
  vpRobotBebop2 *drone = static_cast<vpRobotBebop2 *>(customData);

  if (drone == nullptr)
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
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Flat trim finished ...");
    drone->m_flatTrimFinished = true;
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_EXPOSITIONCHANGED:
    // If the command received is a exposition changed
    cmdExposureSetRcv(elementDictionary, drone);
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEORESOLUTIONSCHANGED:
    // If the command received is a resolution changed
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Video resolution set ...");
    drone->m_videoResolutionSet = true;
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED:
    // If the command received is a streaming started
    drone->m_streamingStarted = true;
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOSTREAMMODECHANGED:
    // If the command received is a streaming mode changed
    drone->m_streamingModeSet = true;
    break;

  case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_RESETCHANGED:
    // If the command received is a settings reset
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Settings reset ...");
    drone->m_settingsReset = true;
    break;

  case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATIONV2:
    // If the command received is a camera orientation changed
    cmdCameraOrientationChangedRcv(elementDictionary, drone);
    break;

  case ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED:
    // If the command received is a camera information sent
    cmdCameraSettingsRcv(elementDictionary, drone);
    break;

  default:
    break;
  }
}

#undef TAG
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpRobotBebop2.cpp.o) has
// no symbols
void dummy_vpRobotBebop2() { };
#endif // VISP_HAVE_ARSDK
