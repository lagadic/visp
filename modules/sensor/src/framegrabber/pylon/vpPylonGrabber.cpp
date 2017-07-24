/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Description: Class allows to grab images from a Basler camera
 * using Pylon SDK.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

/*!
  \file vpPylonGrabber.cpp
  \brief Wrapper over Basler Pylon SDK to capture images from Basler
  cameras.
*/

#include <visp3/core/vpException.h>
#include <visp3/sensor/vpPylonGrabber.h>

#ifdef VISP_HAVE_PYLON

#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>

#include <visp3/core/vpTime.h>

/*!
   Default constructor that consider the first camera found on the bus as
   active.
 */
vpPylonGrabber::vpPylonGrabber()
  : m_camera(), m_index(0), m_numCameras(0), m_connected(false)
{
  m_numCameras = this->getNumCameras();
}

/*!
   Default destructor that closes the connection with the camera.
 */
vpPylonGrabber::~vpPylonGrabber()
{
  close();
  // It looks like that ~CInstantCamera can't destroy or remove the
  // attached device properly.
  m_camera.DestroyDevice();
}

/*!
  \return Return the number of cameras connected on the bus.
*/
unsigned int vpPylonGrabber::getNumCameras()
{
  Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t lstDevices;
  TlFactory.EnumerateDevices(lstDevices);

  return lstDevices.size();
}

/*!
  Print to the output stream active camera information (serial number,
  camera model, camera vendor, sensor, resolution, firmware version,
  ...).
  */
std::ostream &vpPylonGrabber::getCameraInfo(std::ostream &os)
{
  this->connect();

  Pylon::CDeviceInfo deviceInfo = m_camera.GetDeviceInfo();
  // Get the camera control object.
  GenApi::INodeMap &control = m_camera.GetNodeMap();

  GenApi::CIntegerPtr widthMax = control.GetNode("WidthMax");
  GenApi::CIntegerPtr heightMax = control.GetNode("HeightMax");

  os << "Camera information:   " << std::endl;
  os << " Serial number      : " << deviceInfo.GetSerialNumber() << std::endl;
  os << " Camera model       : " << deviceInfo.GetModelName() << std::endl;
  os << " Camera vendor      : " << deviceInfo.GetVendorName() << std::endl;
  os << " Resolution         : " << widthMax->GetValue() << "x"
     << heightMax->GetValue() << std::endl;
  os << " Firmware version   : " << deviceInfo.GetDeviceVersion()
     << std::endl;

  return os;
}

/*!
  Return the handler to the active camera or NULL if the camera is not
  connected. This function was designed to provide a direct access to
  the Pylon SDK to get access to advanced functionalities that are not
  implemented in this class.
*/
Pylon::CInstantCamera *vpPylonGrabber::getCameraHandler()
{
  this->connect();

  if (m_connected == true) {
    return &m_camera;
  } else {
    return NULL;
  }
}

/*!
  Return camera capture framerate.
  If the camera doesn't support framerate property, return an exception.

  \sa setFrameRate()
 */
float vpPylonGrabber::getFrameRate()
{
  this->connect();

  float frame_rate;
  if (this->getProperty<float, GenApi::IFloat>("AcquisitionFrameRate",
                                               frame_rate))
    return frame_rate;
  else if (this->getProperty<float, GenApi::IFloat>("AcquisitionFrameRateAbs",
                                                    frame_rate))
    return frame_rate;
  else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get frame rate.");
}

/*!
  Return camera gain value in dB or raw value.
  If the camera doesn't support gain property, return an exception.

  \sa setGain()
 */
float vpPylonGrabber::getGain()
{
  this->connect();

  float gain;
  if (this->getProperty<float, GenApi::IFloat>("Gain", gain))
    return gain;
  else if (this->getProperty<float, GenApi::IFloat>("GainAbs", gain))
    return gain;
  else if (this->getProperty<float, GenApi::IInteger>("GainRaw", gain))
    return gain;
  else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get gain.");
}

/*!
  Return blacklevel value in % or raw value.
  If the camera doesn't support blacklevel property, return an exception.

  According to SFNC (standard feature naming convention) of GenICam
  standard, Black level is used instead of brightness.

  See "Terminology Changes" section of the page:
  https://www.ptgrey.com/kb/11020?countryid=237
  \sa setBlackLevel()
 */
float vpPylonGrabber::getBlackLevel()
{
  this->connect();

  float black_level;
  if (this->getProperty<float, GenApi::IFloat>("BlackLevel", black_level))
    return black_level;
  else if (this->getProperty<float, GenApi::IFloat>("BlackLevelAbs",
                                                    black_level))
    return black_level;
  else if (this->getProperty<float, GenApi::IInteger>("BlackLevelRaw",
                                                      black_level))
    return black_level;
  else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get blacklevel.");
}

/*!
  Return sharpness value.
  If the camera doesn't support sharpness property, return an exception.

  \sa setSharpness()
 */
float vpPylonGrabber::getSharpness()
{
  this->connect();

  float sharpness;
  if (this->getProperty<float, GenApi::IFloat>("SharpnessEnhancement",
                                               sharpness))
    return sharpness;
  else if (this->getProperty<float, GenApi::IFloat>("SharpnessEnhancementAbs",
                                                    sharpness))
    return sharpness;
  else if (this->getProperty<float, GenApi::IInteger>(
               "SharpnessEnhancementRaw", sharpness))
    return sharpness;
  else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get sharpness.");
}

/*!
  Return exposure value in ms.
  If the camera doesn't support exposure property, return an exception.

  According to SFNC (standard feature naming convention) of GenICam
  standard, Exposure is used and deprecates shutter.

  See "Terminology Changes" section of the page:
  https://www.ptgrey.com/kb/11020?countryid=237
  \sa setExposure()
 */
float vpPylonGrabber::getExposure()
{
  this->connect();

  float exposure_us;
  if (this->getProperty<float, GenApi::IFloat>("ExposureTime", exposure_us))
    return exposure_us * 0.001;
  else if (this->getProperty<float, GenApi::IFloat>("ExposureTimeAbs",
                                                    exposure_us))
    return exposure_us * 0.001;
  else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get exposure.");
}

/*!
  Return gamma correction value.
  If the camera doesn't support gamma property, return an exception.

  \sa setGamma()
 */
float vpPylonGrabber::getGamma()
{
  this->connect();

  float gamma;
  if (m_camera.IsGigE() || m_camera.IsUsb()) {
    this->getProperty<float, GenApi::IFloat>("Gamma", gamma);
    return gamma;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get gamma.");
}

/*!
  Return the serial id of a camera with \e index.
  \param index : Camera index.

  \sa setCameraSerial()
 */
std::string vpPylonGrabber::getCameraSerial(unsigned int index)
{
  unsigned int num_cameras = vpPylonGrabber::getNumCameras();

  if (index >= num_cameras) {
    throw(vpException(
        vpException::badValue,
        "The camera with index %u is not present. Only %d cameras connected.",
        index, num_cameras));
  }

  Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t lstDevices; // List of connected cameras
  TlFactory.EnumerateDevices(lstDevices);

  std::ostringstream os;
  os << lstDevices[index].GetSerialNumber();
  return os.str();
}

/*!
  \brief Loads the selected configuration into the camera's volatile
  memory and makes it the active configuration set.

  \param user_set 0 for factory default; 1, 2 and 3 for UserSet1,
  UserSet2 and UserSet3.
  \return true for finished, false otherwise.

  \sa saveUserSet(unsigned int, bool)
 */
bool vpPylonGrabber::loadUserSet(unsigned int user_set)
{
  this->connect();

  bool success = false;

  success = selectUserSet(user_set);

  if (success) {
    // Get the camera control object.
    GenApi::INodeMap &control = m_camera.GetNodeMap();
    GenApi::CPointer<GenApi::ICommand> prop = control.GetNode("UserSetLoad");
    prop->Execute();
    vpTime::wait(200); // How long you have to wait?
    success = prop->IsDone();
  }

  return success;
}

/*!
  \brief Gets the configuration set being used as the default startup
  set.

  \return 0 for factory default; 1, 2 and 3 for UserSet1,
  UserSet2 and UserSet3. -1 for other user sets.

  \sa setUserSetDefault()
 */
unsigned int vpPylonGrabber::getUserSetDefault()
{
  this->connect();

  if (m_camera.IsGigE()) {
    Basler_GigECamera::UserSetDefaultSelectorEnums user_set =
        Basler_GigECamera::UserSetDefaultSelector_Default;
    this->getProperty<Basler_GigECamera::UserSetDefaultSelectorEnums,
                      GenApi::IEnumerationT<
                          Basler_GigECamera::UserSetDefaultSelectorEnums>>(
        "UserSetDefaultSelector", user_set);
    switch (user_set) {
    case Basler_GigECamera::UserSetDefaultSelector_Default:
      return 0;
      break;
    case Basler_GigECamera::UserSetDefaultSelector_UserSet1:
      return 1;
      break;
    case Basler_GigECamera::UserSetDefaultSelector_UserSet2:
      return 2;
      break;
    case Basler_GigECamera::UserSetDefaultSelector_UserSet3:
      return 3;
      break;
    default:
      return -1;
    }
  } else if (m_camera.IsUsb()) {
    Basler_UsbCameraParams::UserSetDefaultEnums user_set =
        Basler_UsbCameraParams::UserSetDefault_Default;
    this->getProperty<
        Basler_UsbCameraParams::UserSetDefaultEnums,
        GenApi::IEnumerationT<Basler_UsbCameraParams::UserSetDefaultEnums>>(
        "UserSetDefault", user_set);
    switch (user_set) {
    case Basler_UsbCameraParams::UserSetDefault_Default:
      return 0;
      break;
    case Basler_UsbCameraParams::UserSetDefault_UserSet1:
      return 1;
      break;
    case Basler_UsbCameraParams::UserSetDefault_UserSet2:
      return 2;
      break;
    case Basler_UsbCameraParams::UserSetDefault_UserSet3:
      return 3;
      break;
    default:
      return -1;
    }
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to get default user set.");
}

/*!
  If multiples cameras are connected on the bus, select the camera to
  dial with.

  \param index : Current camera index, a value comprised between 0 (the
  first camera found on the bus) and the number of cameras found and
  returned by getNumCameras() minus 1.

  If two cameras are connected on the bus, setting \e index to one
  allows to communicate with the second one. This identifier is not
  unique. That is why, it is also possible to select a camera by its
  serial number, which is unique using setCameraSerial().

  \exception vpException::badValue : If the index is greater or equal to
  the number of cameras connected to the bus.
  */
void vpPylonGrabber::setCameraIndex(unsigned int index)
{
  if (index >= m_numCameras) {
    throw(vpException(
        vpException::badValue,
        "The camera with index %u is not present. Only %d cameras connected.",
        index, m_numCameras));
  }

  m_index = index;
}

/*!
  Set the current camera from its unique serial identifier.
  \param serial : Camera serial string.

  \sa getCameraSerial()
 */
void vpPylonGrabber::setCameraSerial(std::string &serial)
{
  m_numCameras = this->getNumCameras();
  for (unsigned int i = 0; i < m_numCameras; i++) {
    if (vpPylonGrabber::getCameraSerial(i) == serial) {
      m_index = i;
      return;
    }
  }
  throw(vpException(vpException::badValue,
                    "The camera with serial id %u is not present.", serial));
}

/*!
  Set camera frame rate.
  \param frame_rate : Camera frame rate (fps) to set.
  \return The camera current framerate.

  \sa getFramerate()
 */
float vpPylonGrabber::setFrameRate(float frame_rate)
{
  this->connect();

  if (this->setProperty<float, GenApi::IFloat>("AcquisitionFrameRate",
                                               frame_rate)) {
    this->getProperty<float, GenApi::IFloat>("AcquisitionFrameRate",
                                             frame_rate);
    return frame_rate;
  } else if (this->setProperty<float, GenApi::IFloat>(
                 "AcquisitionFrameRateAbs", frame_rate)) {
    this->getProperty<float, GenApi::IFloat>("AcquisitionFrameRateAbs",
                                             frame_rate);
    return frame_rate;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set frame rate.");
}

/*!
  Set camera gain mode and value.
  \param gain_auto : If true set auto gain, if false set manual gain
  applying \e gain_value parameter.
  \param gain_value : The amount of amplification that is applied to a
  pixel in manual mode. An increase in gain can result in an increase in
  noise.

  \return The measured gain after applying the new setting.

  \sa getGain()

 */
float vpPylonGrabber::setGain(bool gain_auto, float gain_value)
{
  this->connect();

  if (m_camera.IsGigE()) {
    if (gain_auto)
      this->setProperty<
          Basler_GigECamera::GainAutoEnums,
          GenApi::IEnumerationT<Basler_GigECamera::GainAutoEnums>>(
          "GainAuto", Basler_GigECamera::GainAuto_Continuous);
    else
      this->setProperty<
          Basler_GigECamera::GainAutoEnums,
          GenApi::IEnumerationT<Basler_GigECamera::GainAutoEnums>>(
          "GainAuto", Basler_GigECamera::GainAuto_Off);
  } else if (m_camera.IsUsb()) {
    if (gain_auto)
      this->setProperty<
          Basler_UsbCameraParams::GainAutoEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::GainAutoEnums>>(
          "GainAuto", Basler_UsbCameraParams::GainAuto_Continuous);
    else
      this->setProperty<
          Basler_UsbCameraParams::GainAutoEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::GainAutoEnums>>(
          "GainAuto", Basler_UsbCameraParams::GainAuto_Off);
  }

  if (this->setProperty<float, GenApi::IFloat>("GainAbs", gain_value)) {
    this->getProperty<float, GenApi::IFloat>("GainAbs", gain_value);
    return gain_value;
  } else if (this->setProperty<float, GenApi::IFloat>("Gain", gain_value)) {
    this->getProperty<float, GenApi::IFloat>("Gain", gain_value);
    return gain_value;
  } else if (this->setProperty<float, GenApi::IFloat>("GainRaw",
                                                      gain_value)) {
    this->getProperty<float, GenApi::IFloat>("GainRaw", gain_value);
    return gain_value;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set gain.");
}

/*!
  Set camera blacklevel mode and parameter.
  \param blacklevel_value : This is the level of black in an image. A
  high blacklevel will result in a low amount of black in the image.

  \return The measured blacklevel after applying the new setting.

  According to SFNC (standard feature naming convention) of GenICam
  standard, Black level is used instead of brightness.

  See "Terminology Changes" section of the page:
  https://www.ptgrey.com/kb/11020?countryid=237
  \sa getBlackLevel()
 */
float vpPylonGrabber::setBlackLevel(float blacklevel_value)
{
  this->connect();

  if (this->setProperty<float, GenApi::IFloat>("BlackLevelAbs",
                                               blacklevel_value)) {
    this->getProperty<float, GenApi::IFloat>("BlackLevelAbs",
                                             blacklevel_value);
    return blacklevel_value;
  } else if (this->setProperty<float, GenApi::IFloat>("BlackLevel",
                                                      blacklevel_value)) {
    this->getProperty<float, GenApi::IFloat>("BlackLevel", blacklevel_value);
    return blacklevel_value;
  } else if (this->setProperty<float, GenApi::IInteger>("BlackLevelRaw",
                                                        blacklevel_value)) {
    this->getProperty<float, GenApi::IFloat>("BlackLevelRaw",
                                             blacklevel_value);
    return blacklevel_value;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set blacklevel.");
}

/*!
  Set camera exposure mode and parameter.

  \param exposure_on : If true turn exposure on, otherwise turn off.
  \param exposure_auto : If true set auto exposure, if false set manual
exposure applying \e exposure_value parameter.
  \param exposure_value : This is the exposure value in ms.

  \return The measured exposure time in ms after applying the new setting.

  According to SFNC (standard feature naming convention) of GenICam
  standard, Exposure is used and deprecates shutter.

  See "Terminology Changes" section of the page:
  https://www.ptgrey.com/kb/11020?countryid=237
  \sa getExposure()
 */
float vpPylonGrabber::setExposure(bool exposure_on, bool exposure_auto,
                                  float exposure_value)
{
  this->connect();

  if (m_camera.IsGigE()) {
    if (exposure_on)
      this->setProperty<
          Basler_GigECamera::ExposureModeEnums,
          GenApi::IEnumerationT<Basler_GigECamera::ExposureModeEnums>>(
          "ExposureMode", Basler_GigECamera::ExposureMode_Timed);
    else
      this->setProperty<
          Basler_GigECamera::ExposureModeEnums,
          GenApi::IEnumerationT<Basler_GigECamera::ExposureModeEnums>>(
          "ExposureMode", Basler_GigECamera::ExposureMode_Off);

    if (exposure_auto)
      this->setProperty<
          Basler_GigECamera::ExposureAutoEnums,
          GenApi::IEnumerationT<Basler_GigECamera::ExposureAutoEnums>>(
          "ExposureAuto", Basler_GigECamera::ExposureAuto_Continuous);
    else
      this->setProperty<
          Basler_GigECamera::ExposureAutoEnums,
          GenApi::IEnumerationT<Basler_GigECamera::ExposureAutoEnums>>(
          "ExposureAuto", Basler_GigECamera::ExposureAuto_Off);
  } else if (m_camera.IsUsb()) {
    if (exposure_on)
      this->setProperty<
          Basler_UsbCameraParams::ExposureModeEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::ExposureModeEnums>>(
          "ExposureMode", Basler_UsbCameraParams::ExposureMode_Timed);
    else
      throw vpException(vpException::notImplementedError,
                        "Can't set exposure off.");

    if (exposure_auto)
      this->setProperty<
          Basler_UsbCameraParams::ExposureAutoEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::ExposureAutoEnums>>(
          "ExposureAuto", Basler_UsbCameraParams::ExposureAuto_Continuous);
    else
      this->setProperty<
          Basler_UsbCameraParams::ExposureAutoEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::ExposureAutoEnums>>(
          "ExposureAuto", Basler_UsbCameraParams::ExposureAuto_Off);
  }

  if (this->setProperty<float, GenApi::IFloat>("ExposureTimeAbs",
                                               exposure_value * 1000)) {
    this->getProperty<float, GenApi::IFloat>("ExposureTimeAbs",
                                             exposure_value);
    return exposure_value * 0.001;
  } else if (this->setProperty<float, GenApi::IFloat>(
                 "ExposureTime", exposure_value * 1000)) {
    this->getProperty<float, GenApi::IFloat>("ExposureTime", exposure_value);
    return exposure_value * 0.001;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set exposure.");
}

/*!
  Set camera sharpness mode and parameter.

  \param sharpness_on : If true turn sharpness on, otherwise turn off.
  \param sharpness_value : Parameter used to tune the filter applied on
  the image to reduce blurred edges in an image.

  \return The measured sharpness after applying the new setting.

  \sa getSharpness()
 */
float vpPylonGrabber::setSharpness(bool sharpness_on, float sharpness_value)
{
  this->connect();

  if (m_camera.IsGigE()) {
    if (sharpness_on)
      this->setProperty<
          Basler_GigECamera::DemosaicingModeEnums,
          GenApi::IEnumerationT<Basler_GigECamera::DemosaicingModeEnums>>(
          "DemosaicingMode", Basler_GigECamera::DemosaicingMode_BaslerPGI);
    else
      this->setProperty<
          Basler_GigECamera::DemosaicingModeEnums,
          GenApi::IEnumerationT<Basler_GigECamera::DemosaicingModeEnums>>(
          "DemosaicingMode", Basler_GigECamera::DemosaicingMode_Simple);
  } else if (m_camera.IsUsb()) {
    if (sharpness_on)
      this->setProperty<Basler_UsbCameraParams::DemosaicingModeEnums,
                        GenApi::IEnumerationT<
                            Basler_UsbCameraParams::DemosaicingModeEnums>>(
          "DemosaicingMode",
          Basler_UsbCameraParams::DemosaicingMode_BaslerPGI);
    else
      this->setProperty<Basler_UsbCameraParams::DemosaicingModeEnums,
                        GenApi::IEnumerationT<
                            Basler_UsbCameraParams::DemosaicingModeEnums>>(
          "DemosaicingMode", Basler_UsbCameraParams::DemosaicingMode_Simple);
  }

  if (this->setProperty<float, GenApi::IFloat>("SharpnessEnhancementAbs",
                                               sharpness_value)) {
    this->getProperty<float, GenApi::IFloat>("SharpnessEnhancementAbs",
                                             sharpness_value);
    return sharpness_value;
  } else if (this->setProperty<float, GenApi::IFloat>("SharpnessEnhancement",
                                                      sharpness_value)) {
    this->getProperty<float, GenApi::IFloat>("SharpnessEnhancement",
                                             sharpness_value);
    return sharpness_value;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set sharpness.");
}

/*!
  Set camera gamma correction mode and parameter.

  \param gamma_on : If true turn gamma correction on, otherwise turn off.
  \param gamma_value : Parameter used to perform gamma correction of
  pixel intensity.

  \return The measured gamma correction value after applying the new
  setting.

  \sa getGamma()
 */
float vpPylonGrabber::setGamma(bool gamma_on, float gamma_value)
{
  this->connect();

  if (m_camera.IsGigE()) {
    this->setProperty<bool, GenApi::IBoolean>("GammaEnable", gamma_on);
    this->setProperty<float, GenApi::IFloat>("Gamma", gamma_value);
    this->getProperty<float, GenApi::IFloat>("Gamma", gamma_value);
    return gamma_value;
  } else if (m_camera.IsUsb()) {
    this->setProperty<float, GenApi::IFloat>("Gamma", gamma_value);
    this->getProperty<float, GenApi::IFloat>("Gamma", gamma_value);
    return gamma_value;
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set gamma.");
}

/*!
  \brief Saves the current active configuration set into the selected
  user set.

  \param user_set 0 for factory default; 1, 2 and 3 for UserSet1,
  UserSet2 and UserSet3.
  \param set_default Whether to set the configuration set to be used as
  the default startup set.
  \return true for finished, false otherwise.

  \sa loadUserSet(unsigned int)
 */
bool vpPylonGrabber::saveUserSet(unsigned int user_set, bool set_default)
{
  this->connect();

  bool success = false;

  success = selectUserSet(user_set);

  if (success) {
    // Get the camera control object.
    GenApi::INodeMap &control = m_camera.GetNodeMap();
    GenApi::CPointer<GenApi::ICommand> prop = control.GetNode("UserSetSave");
    prop->Execute();
    vpTime::wait(200); // How long you have to wait?
    success = prop->IsDone();
  }

  if (success)
    success = setUserSetDefault(user_set);

  return success;
}

/*!
  \brief Sets the configuration set to be used as the default startup
  set.

  \param 0 for factory default; 1, 2 and 3 for UserSet1, UserSet2 and
  UserSet3. -1 for other user sets.
  \return true for finished, false otherwise.

  \sa getUserSetDefault()
 */
bool vpPylonGrabber::setUserSetDefault(unsigned int user_set)
{
  this->connect();

  if (m_camera.IsGigE()) {
    switch (user_set) {
    case 0:
      return this
          ->setProperty<Basler_GigECamera::UserSetDefaultSelectorEnums,
                        GenApi::IEnumerationT<
                            Basler_GigECamera::UserSetDefaultSelectorEnums>>(
              "UserSetDefaultSelector",
              Basler_GigECamera::UserSetDefaultSelector_Default);
      break;
    case 1:
      return this
          ->setProperty<Basler_GigECamera::UserSetDefaultSelectorEnums,
                        GenApi::IEnumerationT<
                            Basler_GigECamera::UserSetDefaultSelectorEnums>>(
              "UserSetDefaultSelector",
              Basler_GigECamera::UserSetDefaultSelector_UserSet1);
      break;
    case 2:
      return this
          ->setProperty<Basler_GigECamera::UserSetDefaultSelectorEnums,
                        GenApi::IEnumerationT<
                            Basler_GigECamera::UserSetDefaultSelectorEnums>>(
              "UserSetDefaultSelector",
              Basler_GigECamera::UserSetDefaultSelector_UserSet2);
      break;
    case 3:
      return this
          ->setProperty<Basler_GigECamera::UserSetDefaultSelectorEnums,
                        GenApi::IEnumerationT<
                            Basler_GigECamera::UserSetDefaultSelectorEnums>>(
              "UserSetDefaultSelector",
              Basler_GigECamera::UserSetDefaultSelector_UserSet3);
      break;
    default:
      return false;
    }
  } else if (m_camera.IsUsb()) {
    switch (user_set) {
    case 0:
      return this->setProperty<
          Basler_UsbCameraParams::UserSetDefaultEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::UserSetDefaultEnums>>(
          "UserSetDefault", Basler_UsbCameraParams::UserSetDefault_Default);
      break;
    case 1:
      return this->setProperty<
          Basler_UsbCameraParams::UserSetDefaultEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::UserSetDefaultEnums>>(
          "UserSetDefault", Basler_UsbCameraParams::UserSetDefault_UserSet1);
      break;
    case 2:
      return this->setProperty<
          Basler_UsbCameraParams::UserSetDefaultEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::UserSetDefaultEnums>>(
          "UserSetDefault", Basler_UsbCameraParams::UserSetDefault_UserSet2);
      break;
    case 3:
      return this->setProperty<
          Basler_UsbCameraParams::UserSetDefaultEnums,
          GenApi::IEnumerationT<Basler_UsbCameraParams::UserSetDefaultEnums>>(
          "UserSetDefault", Basler_UsbCameraParams::UserSetDefault_UserSet3);
      break;
    default:
      return false;
    }
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to set default user set.");
}

/*!
   Start active camera capturing images.

   \sa stopCapture()
 */
void vpPylonGrabber::startCapture()
{
  this->connect();

  if (!m_camera.IsGrabbing()) {
    m_camera.StartGrabbing(1);
  }
  if (m_connected && m_camera.IsGrabbing())
    init = true;
  else
    init = false;
}

/*!
   Stop active camera capturing images.

   \sa startCapture()
 */
void vpPylonGrabber::stopCapture()
{
  if (m_camera.IsGrabbing()) {
    m_camera.StopGrabbing();
  }
  if (m_connected && m_camera.IsGrabbing())
    init = true;
  else
    init = false;
}

/*!
   Connect the active camera.

   \sa disconnect()
 */
void vpPylonGrabber::connect()
{
  if (m_connected == false) {
    m_numCameras = this->getNumCameras();
    if (m_numCameras == 0) {
      throw(vpException(vpException::fatalError, "No camera found"));
    }

    if (!m_camera.IsPylonDeviceAttached()) {
      Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
      Pylon::DeviceInfoList_t lstDevices;
      TlFactory.EnumerateDevices(lstDevices);

      m_camera.Attach(TlFactory.CreateDevice(lstDevices[m_index]));
    }
    // Connect to a camera
    m_camera.Open();
    m_connected = true;
  }
  if (m_connected && m_camera.IsGrabbing())
    init = true;
  else
    init = false;
}

/*!
   Disconnect the active camera.

   \sa connect()
 */
void vpPylonGrabber::disconnect()
{
  if (m_connected == true) {
    m_camera.Close();
    m_connected = false;
  }
  if (m_connected && m_camera.IsGrabbing())
    init = true;
  else
    init = false;
}

/*!
   \brief Stop active camera capturing images and disconnect the active
   camera.

   If you want to use again this camera, you may call
   setCameraIndex(const unsigned int &) and open(vpImage<unsigned char>
   &) or open(vpImage<vpRGBa> &) to connect again the camera.
 */
void vpPylonGrabber::close()
{
  this->stopCapture();
  this->disconnect();
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).
*/
void vpPylonGrabber::acquire(vpImage<unsigned char> &I)
{
  this->open();

  Pylon::CGrabResultPtr grabResult;
  // Retrieve an image
  if (!m_camera.RetrieveResult(2000, grabResult)) {
    throw(vpException(vpException::fatalError,
                      "Cannot retrieve image from camera with serial %u",
                      getCameraSerial(m_index)));
  }

  if (grabResult->GrabSucceeded()) {
    height = grabResult->GetHeight();
    width = grabResult->GetWidth();
    I.resize(height, width);

    Pylon::CImageFormatConverter imageConvert;
    imageConvert.OutputPixelFormat = Pylon::PixelType_Mono8;
    imageConvert.OutputPaddingX = 0;
    // Create a converted image
    imageConvert.Convert(I.bitmap, sizeof(unsigned char) * width * height,
                         (Pylon::IImage &)grabResult);
  }
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).
*/
void vpPylonGrabber::acquire(vpImage<vpRGBa> &I)
{
  this->open();

  Pylon::CGrabResultPtr grabResult;
  // Retrieve an image
  if (!m_camera.RetrieveResult(2000, grabResult)) {
    throw(vpException(vpException::fatalError,
                      "Cannot retrieve image from camera with serial %u",
                      getCameraSerial(m_index)));
  }

  if (grabResult->GrabSucceeded()) {
    height = grabResult->GetHeight();
    width = grabResult->GetWidth();
    I.resize(height, width);

    Pylon::CImageFormatConverter imageConvert;
    imageConvert.OutputPixelFormat = Pylon::PixelType_BGRA8packed;
    imageConvert.OutputPaddingX = 0;
    // Create a converted image
    Pylon::CPylonImage destImage;
    imageConvert.Convert(destImage, (Pylon::IImage &)grabResult);
    Pylon::SBGRA8Pixel *pixel = (Pylon::SBGRA8Pixel *)destImage.GetBuffer();
    for (unsigned int i = 0; i < height; i++) {
      for (unsigned int j = 0; j < width; j++) {
        unsigned int p_index = i * width + j;
        I[i][j].R = pixel[p_index].R;
        I[i][j].G = pixel[p_index].G;
        I[i][j].B = pixel[p_index].B;
        I[i][j].A = pixel[p_index].A;
      }
    }
  }
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpPylonGrabber::open(vpImage<unsigned char> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpPylonGrabber::open(vpImage<vpRGBa> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera and start capture.

   Similar then calling:
   \code
   vpPylonGrabber g;
   ...
   g.connect();
   g.startCapture();
   \endcode
 */
void vpPylonGrabber::open()
{
  this->connect();
  this->startCapture();
}

/*!
  \brief Selects the configuration set to load, save, or configure.

  \param user_set 0 for factory default; 1, 2 and 3 for UserSet1,
  UserSet2 and UserSet3.
  \return true for success, false for failure.

  Default User Set is read-only and cannot be modified.
 */
bool vpPylonGrabber::selectUserSet(unsigned int user_set)
{
  this->connect();

  bool success = false;

  if (m_camera.IsGigE()) {
    switch (user_set) {
    case 0:
      success = this->setProperty<
          Basler_GigECamera::UserSetSelectorEnums,
          GenApi::IEnumerationT<Basler_GigECamera::UserSetSelectorEnums>>(
          "UserSetSelector", Basler_GigECamera::UserSetSelector_Default);
      break;
    case 1:
      success = this->setProperty<
          Basler_GigECamera::UserSetSelectorEnums,
          GenApi::IEnumerationT<Basler_GigECamera::UserSetSelectorEnums>>(
          "UserSetSelector", Basler_GigECamera::UserSetSelector_UserSet1);
      break;
    case 2:
      success = this->setProperty<
          Basler_GigECamera::UserSetSelectorEnums,
          GenApi::IEnumerationT<Basler_GigECamera::UserSetSelectorEnums>>(
          "UserSetSelector", Basler_GigECamera::UserSetSelector_UserSet2);
      break;
    case 3:
      success = this->setProperty<
          Basler_GigECamera::UserSetSelectorEnums,
          GenApi::IEnumerationT<Basler_GigECamera::UserSetSelectorEnums>>(
          "UserSetSelector", Basler_GigECamera::UserSetSelector_UserSet3);
      break;
    default:
      throw vpException(vpException::notImplementedError,
                        "Unsupported user set.");
    }
  } else if (m_camera.IsUsb()) {
    switch (user_set) {
    case 0:
      success = this->setProperty<
          Basler_UsbCameraParams::UserSetSelectorEnums,
          GenApi::IEnumerationT<
              Basler_UsbCameraParams::UserSetSelectorEnums>>(
          "UserSetSelector", Basler_UsbCameraParams::UserSetSelector_Default);
      break;
    case 1:
      success = this->setProperty<
          Basler_UsbCameraParams::UserSetSelectorEnums,
          GenApi::IEnumerationT<
              Basler_UsbCameraParams::UserSetSelectorEnums>>(
          "UserSetSelector",
          Basler_UsbCameraParams::UserSetSelector_UserSet1);
      break;
    case 2:
      success = this->setProperty<
          Basler_UsbCameraParams::UserSetSelectorEnums,
          GenApi::IEnumerationT<
              Basler_UsbCameraParams::UserSetSelectorEnums>>(
          "UserSetSelector",
          Basler_UsbCameraParams::UserSetSelector_UserSet2);
      break;
    case 3:
      success = this->setProperty<
          Basler_UsbCameraParams::UserSetSelectorEnums,
          GenApi::IEnumerationT<
              Basler_UsbCameraParams::UserSetSelectorEnums>>(
          "UserSetSelector",
          Basler_UsbCameraParams::UserSetSelector_UserSet3);
      break;
    default:
      throw vpException(vpException::notImplementedError,
                        "Unsupported user set.");
    }
  } else
    throw vpException(vpException::notImplementedError,
                      "Don't know how to select user set.");

  return success;
}

/*!

   Operator that allows to capture a grey level image.
   \param I : The captured image.
 */
vpPylonGrabber &vpPylonGrabber::operator>>(vpImage<unsigned char> &I)
{
  this->acquire(I);
  return *this;
}

/*!

   Operator that allows to capture a color image.
   \param I : The captured image.
 */
vpPylonGrabber &vpPylonGrabber::operator>>(vpImage<vpRGBa> &I)
{
  this->acquire(I);
  return *this;
}

#else
// Work arround to avoid warning:
// libvisp_pylon.a(vpPylonGrabber.cpp.o) has no symbols
void dummy_vpPylonGrabber(){};
#endif // #ifdef VISP_HAVE_PYLON
