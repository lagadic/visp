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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Gatien Gaumerais
 *
*****************************************************************************/

#ifndef _vpRobotBebop2_h_
#define _vpRobotBebop2_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_ARSDK) && defined(VISP_HAVE_THREADS)

#include <visp3/core/vpImage.h>

extern "C" {
#include <libARController/ARController.h> // For drone control
#include <libARSAL/ARSAL.h>               // For semaphore

#ifdef VISP_HAVE_FFMPEG
#include <libavcodec/avcodec.h> // For H264 video decoding
#include <libswscale/swscale.h> // For rescaling decoded frames
#endif
}

#include <mutex>
#include <signal.h>
#include <string>

BEGIN_VISP_NAMESPACE
/*!
  \class vpRobotBebop2

  \ingroup group_robot_real_drone

  Interface for Parrot ARSDK3, allowing to control the Bebop 2 drone and get images from the camera (if ViSP was built
  with FFMpeg support).

  \sa \ref tutorial-bebop2-vs
*/
class VISP_EXPORT vpRobotBebop2
{
public:
  vpRobotBebop2(bool verbose = false, bool setDefaultSettings = true, std::string ipAddress = "192.168.42.1",
                int discoveryPort = 44444);
  virtual ~vpRobotBebop2();

  //! @name Drone networking information
  //@{
  std::string getIpAddress();
  int getDiscoveryPort();
  //@}

  //! @name General drone information
  //@{
  void doFlatTrim();
  unsigned int getBatteryLevel();
  void setVerbose(bool verbose);
  void resetAllSettings();
  //@}

  //! @name Drone state checking
  //@{
  bool isFlying();
  bool isHovering();
  bool isLanded();
  bool isRunning();
  bool isStreaming();
  //@}

  //*** Motion commands ***//
  //! @name Motion commands and parameters
  //@{
  void cutMotors();
  double getMaxTilt();
  void setMaxTilt(double maxTilt);
  void setPitch(int value);
  void setPosition(float dX, float dY, float dZ, float dPsi, bool blocking);
  void setPosition(const vpHomogeneousMatrix &M, bool blocking);
  void setRoll(int value);
  void setVelocity(const vpColVector &vel, double delta_t);
  void setVerticalSpeed(int value);
  void setYawSpeed(int value);
  void stopMoving();
  void takeOff(bool blocking = true);
  //@}
  static void land();
  //*** ***//

  //*** Streaming commands ***//
#ifdef VISP_HAVE_FFMPEG
  /** @name Streaming commands and parameters (only available if ViSP was built with FFMpeg support) */
  //@{
  void getGrayscaleImage(vpImage<unsigned char> &I);
  void getRGBaImage(vpImage<vpRGBa> &I);
  int getVideoHeight();
  int getVideoWidth();
  void setExposure(float expo);
  void setStreamingMode(int mode);
  void setVideoResolution(int mode);
  void setVideoStabilisationMode(int mode);
  void startStreaming();
  void stopStreaming();
  //@}
#endif
  //*** ***//

  //*** Camera control commands ***//
  /** @name Camera control commands and parameters */
  //@{
  double getCameraHorizontalFOV() const;
  double getCurrentCameraPan() const;
  double getMaxCameraPan() const;
  double getMinCameraPan() const;
  double getCurrentCameraTilt() const;
  double getMaxCameraTilt() const;
  double getMinCameraTilt() const;
  void setCameraOrientation(double tilt, double pan, bool blocking = false);
  void setCameraPan(double pan, bool blocking = false);
  void setCameraTilt(double tilt, bool blocking = false);
  //@}
  //*** ***//

private:
  //*** Attributes ***//
  std::string m_ipAddress; ///< Ip address of the drone to discover on the network
  int m_discoveryPort;     ///< Port of the drone to discover on the network

  ARSAL_Sem_t m_stateSem;    ///< Semaphore
  struct sigaction m_sigAct; ///< Signal handler

#ifdef VISP_HAVE_FFMPEG
  AVCodecContext *m_codecContext; ///< Codec context for video stream decoding
  AVPacket m_packet;              ///< Packed used to send data to the decoder
  AVFrame *m_picture;             ///< Frame used to receive data from the decoder
  std::mutex m_bgr_picture_mutex; ///< Mutex to protect m_bgr_picture
  AVFrame *m_bgr_picture;         ///< Frame used to store rescaled frame received from the decoder
  SwsContext *m_img_convert_ctx;  ///< Used to rescale frame received from the decoder
  uint8_t *m_buffer;              ///< Buffer used to fill frame arrays

  bool m_videoDecodingStarted; ///< Used to know if the drone is currently streaming and decoding its camera video feed

  int m_videoWidth;  ///< Width of the video streamed from the camera
  int m_videoHeight; ///< Height of the video streamed from the camera
#endif

  static bool m_running; ///< Used for checking if the drone is running ie if successfully connected and ready to
                         ///< receive commands

  bool m_exposureSet;        ///< Used to know if exposure compensation has been set
  bool m_flatTrimFinished;   ///< Used to know when the drone has finished a flat trim
  bool m_relativeMoveEnded;  ///< Used to know when the drone has ended a relative move
  bool m_videoResolutionSet; ///< Used to know if video resolution has been set
  bool m_streamingStarted;   ///< Used to know if the streaming has been started
  bool m_streamingModeSet;   ///< Used to know if the streaming mode has been set
  bool m_settingsReset;      ///< Used to know when the drone a finished the settings reset

  bool m_update_codec_params;               ///< Used to know if codec parameters need to be updated
  std::vector<uint8_t> m_codec_params_data; ///< Used to store codec parameters when they need to be updated

  unsigned int m_batteryLevel; ///< Percentage of battery remaining
  double m_maxTilt;            ///< Max pitch and roll value of the drone

  double m_cameraHorizontalFOV; ///< Camera horizontal FOV

  double m_currentCameraTilt; ///< Current tilt of the camera
  double m_minCameraTilt;     ///< Minimum possible tilt of the camera
  double m_maxCameraTilt;     ///< Maximum possible tilt of the camera

  double m_currentCameraPan; ///< Current pan of the camera
  double m_minCameraPan;     ///< Minimum possible tilt of the camera
  double m_maxCameraPan;     ///< Maximum possible tilt of the camera

  static ARCONTROLLER_Device_t *m_deviceController; ///< Used for drone control

  eARCONTROLLER_ERROR m_errorController;    ///< Used for error handling
  eARCONTROLLER_DEVICE_STATE m_deviceState; ///< Used to store device state
  //*** ***//

  [[noreturn]] static void sighandler(int signo);

  eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE getFlyingState();
  eARCOMMANDS_ARDRONE3_MEDIASTREAMINGSTATE_VIDEOENABLECHANGED_ENABLED getStreamingState();

  //*** Setup functions ***//
  void cleanUp();
  ARDISCOVERY_Device_t *discoverDrone();
  void createDroneController(ARDISCOVERY_Device_t *discoveredDrone);
  void setupCallbacks();
  void startController();

#ifdef VISP_HAVE_FFMPEG
  //*** Video streaming functions ***//
  void initCodec();
  void cleanUpCodec();

  void startVideoDecoding();
  void stopVideoDecoding();
  void computeFrame(ARCONTROLLER_Frame_t *frame);
  //*** ***//
#endif

  //*** Callbacks ***//
  static void stateChangedCallback(eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData);
#ifdef VISP_HAVE_FFMPEG
  static eARCONTROLLER_ERROR decoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void *customData);
  static eARCONTROLLER_ERROR didReceiveFrameCallback(ARCONTROLLER_Frame_t *frame, void *customData);
#endif

  static void cmdBatteryStateChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone);
  static void cmdCameraOrientationChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary,
                                             vpRobotBebop2 *drone);
  static void cmdCameraSettingsRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone);
  static void cmdExposureSetRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone);
  static void cmdMaxPitchRollChangedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone);
  static void cmdRelativeMoveEndedRcv(ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, vpRobotBebop2 *drone);
  static void commandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY commandKey,
                                      ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData);
  //*** ***//
};
END_VISP_NAMESPACE
#endif //#ifdef VISP_HAVE_ARSDK
#endif //#ifndef _vpRobotBebop2_h_
