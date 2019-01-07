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
 * Description: Implementation of vpPylonGrabberGigE class.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

/*!
  \file vpPylonGrabberGigE.cpp
  \brief Subclass of vpPylonGrabber, implements Basler GigE cameras
  supporting.
*/

#include "vpPylonGrabberGigE.h"

#ifdef VISP_HAVE_PYLON

#include <visp3/core/vpException.h>
#include <visp3/core/vpTime.h>

/*!
   Default constructor that consider the first camera found on the bus as
   active.
 */
vpPylonGrabberGigE::vpPylonGrabberGigE() : m_camera(), m_index(0), m_numCameras(0), m_connected(false)
{
  getNumCameras();
}

/*!
   Default destructor that closes the connection with the camera.
 */
vpPylonGrabberGigE::~vpPylonGrabberGigE() { close(); }

/*!
  \return Return the number of cameras connected on the bus.
*/
unsigned int vpPylonGrabberGigE::getNumCameras()
{
  Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t lstDevices;
  Pylon::DeviceInfoList_t filter; // Filter for GigE cameras.
  Pylon::CBaslerGigEDeviceInfo gige_devinfo;
  filter.push_back(gige_devinfo);
  TlFactory.EnumerateDevices(lstDevices, filter);

  m_numCameras = lstDevices.size();
  return m_numCameras;
}

/*!
  Print to the output stream active camera information (serial number,
  camera model, camera vendor, sensor, resolution, firmware version,
  ...).
  */
std::ostream &vpPylonGrabberGigE::getCameraInfo(std::ostream &os)
{
  connect();

  Pylon::CDeviceInfo deviceInfo = m_camera.GetDeviceInfo();
  // Get the camera control object.
  GenApi::INodeMap &control = m_camera.GetNodeMap();

  GenApi::CIntegerPtr widthMax = control.GetNode("WidthMax");
  GenApi::CIntegerPtr heightMax = control.GetNode("HeightMax");

  os << "Camera information:   " << std::endl;
  os << " Serial number      : " << deviceInfo.GetSerialNumber() << std::endl;
  os << " Camera model       : " << deviceInfo.GetModelName() << std::endl;
  os << " Camera vendor      : " << deviceInfo.GetVendorName() << std::endl;
  os << " Resolution         : " << widthMax->GetValue() << "x" << heightMax->GetValue() << std::endl;
  os << " Firmware version   : " << deviceInfo.GetDeviceVersion() << std::endl;

  return os;
}

/*!
  Return the handler to the active camera or NULL if the camera is not
  connected. This function was designed to provide a direct access to
  the Pylon SDK to get access to advanced functionalities that are not
  implemented in this class.
*/
Pylon::CInstantCamera *vpPylonGrabberGigE::getCameraHandler()
{
  connect();

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
float vpPylonGrabberGigE::getFrameRate()
{
  connect();

  float frame_rate = m_camera.AcquisitionFrameRateAbs.GetValue();
  return frame_rate;
}

/*!
  Return camera gain value in dB or raw value.
  If the camera doesn't support gain property, return an exception.

  \sa setGain()
 */
float vpPylonGrabberGigE::getGain()
{
  connect();

  if (GenApi::IsReadable(m_camera.GainAbs))
    return m_camera.GainAbs.GetValue();
  else if (GenApi::IsReadable(m_camera.GainRaw))
    return m_camera.GainRaw.GetValue();
  else
    throw vpException(vpException::notImplementedError, "Don't know how to get gain.");
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
float vpPylonGrabberGigE::getBlackLevel()
{
  connect();

  if (GenApi::IsReadable(m_camera.BlackLevelAbs))
    return m_camera.BlackLevelAbs.GetValue();
  else if (GenApi::IsReadable(m_camera.BlackLevelRaw))
    return m_camera.BlackLevelRaw.GetValue();
  else
    throw vpException(vpException::notImplementedError, "Don't know how to get blacklevel.");
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
float vpPylonGrabberGigE::getExposure()
{
  connect();

  if (GenApi::IsReadable(m_camera.ExposureTimeAbs))
    return m_camera.ExposureTimeAbs.GetValue() * 0.001;
  else if (GenApi::IsReadable(m_camera.ExposureTimeRaw))
    return m_camera.ExposureTimeRaw.GetValue();
  else
    throw vpException(vpException::notImplementedError, "Don't know how to get exposure.");
}

/*!
  Return gamma correction value.
  If the camera doesn't support gamma property, return an exception.

  \sa setGamma()
 */
float vpPylonGrabberGigE::getGamma()
{
  connect();

  float gamma = m_camera.Gamma.GetValue();
  return gamma;
}

/*!
  Return the serial id of a camera with \e index.
  \param index : Camera index.

  \sa setCameraSerial()
 */
std::string vpPylonGrabberGigE::getCameraSerial(unsigned int index)
{
  getNumCameras();

  if (index >= m_numCameras) {
    throw(vpException(vpException::badValue, "The camera with index %u is not present. Only %d cameras connected.",
                      index, m_numCameras));
  }

  Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
  Pylon::DeviceInfoList_t lstDevices; // List of connected cameras
  Pylon::DeviceInfoList_t filter;     // Filter for GigE cameras.
  Pylon::CBaslerGigEDeviceInfo gige_devinfo;
  filter.push_back(gige_devinfo);
  TlFactory.EnumerateDevices(lstDevices, filter);

  std::ostringstream os;
  os << lstDevices[index].GetSerialNumber();
  return os.str();
}

/*!
  \brief Loads the selected configuration into the camera's volatile
  memory and makes it the active configuration set.

  \param user_set See vpPylonGrabber::UserSetName for valid values.
  \return true for finished, false otherwise.

  \sa saveUserSet()
 */
bool vpPylonGrabberGigE::loadUserSet(UserSetName user_set)
{
  connect();

  bool success = selectUserSet(user_set);

  if (success) {
    m_camera.UserSetLoad.Execute();
    vpTime::wait(200); // How long you have to wait?
    success = m_camera.UserSetLoad.IsDone();
  }

  return success;
}

/*!
  \brief Gets the configuration set being used as the default startup
  set.

  \sa setUserSetDefault()
 */
vpPylonGrabber::UserSetName vpPylonGrabberGigE::getUserSetDefault()
{
  connect();

  Basler_GigECamera::UserSetDefaultSelectorEnums user_set = m_camera.UserSetDefaultSelector.GetValue();

  switch (user_set) {
  case Basler_GigECamera::UserSetDefaultSelector_Default:
    return USERSET_DEFAULT;
    break;
  case Basler_GigECamera::UserSetDefaultSelector_UserSet1:
    return USERSET_USERSET1;
    break;
  case Basler_GigECamera::UserSetDefaultSelector_UserSet2:
    return USERSET_USERSET2;
    break;
  case Basler_GigECamera::UserSetDefaultSelector_UserSet3:
    return USERSET_USERSET3;
    break;
  default:
    return USERSET_UNKNOWN;
  }
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
void vpPylonGrabberGigE::setCameraIndex(unsigned int index)
{
  if (index >= m_numCameras) {
    throw(vpException(vpException::badValue, "The camera with index %u is not present. Only %d cameras connected.",
                      index, m_numCameras));
  }

  m_index = index;
}

/*!
  Set the current camera from its unique serial identifier.
  \param serial : Camera serial string.

  \sa getCameraSerial()
 */
void vpPylonGrabberGigE::setCameraSerial(const std::string &serial)
{
  m_numCameras = getNumCameras();
  for (unsigned int i = 0; i < m_numCameras; i++) {
    if (getCameraSerial(i) == serial) {
      m_index = i;
      return;
    }
  }
  throw(vpException(vpException::badValue, "The camera with serial id %s is not present.", serial.c_str()));
}

/*!
  Set camera frame rate.
  \param frame_rate : Camera frame rate (fps) to set.
  \return The camera current framerate.

  \sa getFramerate()
 */
float vpPylonGrabberGigE::setFrameRate(float frame_rate)
{
  connect();

  m_camera.AcquisitionFrameRateAbs.SetValue(frame_rate);

  return m_camera.AcquisitionFrameRateAbs.GetValue();
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
float vpPylonGrabberGigE::setGain(bool gain_auto, float gain_value)
{
  connect();

  if (gain_auto)
    m_camera.GainAuto.SetValue(Basler_GigECamera::GainAuto_Continuous);
  else
    m_camera.GainAuto.SetValue(Basler_GigECamera::GainAuto_Off);

  if (GenApi::IsWritable(m_camera.GainAbs)) {
    m_camera.GainAbs.SetValue(gain_value);
    return m_camera.GainAbs.GetValue();
  } else if (GenApi::IsWritable(m_camera.GainRaw)) {
    m_camera.GainRaw.SetValue(gain_value);
    return m_camera.GainRaw.GetValue();
  } else
    throw vpException(vpException::notImplementedError, "Don't know how to set gain.");
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
float vpPylonGrabberGigE::setBlackLevel(float blacklevel_value)
{
  connect();

  if (GenApi::IsWritable(m_camera.BlackLevelAbs)) {
    m_camera.BlackLevelAbs.SetValue(blacklevel_value);
    return m_camera.BlackLevelAbs.GetValue();
  } else if (GenApi::IsWritable(m_camera.BlackLevelRaw)) {
    m_camera.BlackLevelRaw.SetValue(blacklevel_value);
    return m_camera.BlackLevelRaw.GetValue();
  } else
    throw vpException(vpException::notImplementedError, "Don't know how to set blacklevel.");
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
float vpPylonGrabberGigE::setExposure(bool exposure_on, bool exposure_auto, float exposure_value)
{
  connect();

  if (exposure_on)
    m_camera.ExposureMode.SetValue(Basler_GigECamera::ExposureMode_Timed);
  else
    m_camera.ExposureMode.SetValue(Basler_GigECamera::ExposureMode_Off);

  if (exposure_auto)
    m_camera.ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Continuous);
  else
    m_camera.ExposureAuto.SetValue(Basler_GigECamera::ExposureAuto_Off);

  if (GenApi::IsWritable(m_camera.ExposureTimeAbs)) {
    m_camera.ExposureTimeAbs.SetValue(exposure_value * 1000);
    return m_camera.ExposureTimeAbs.GetValue() * 0.001;
  } else if (GenApi::IsWritable(m_camera.ExposureTimeRaw)) {
    m_camera.ExposureTimeRaw.SetValue(exposure_value);
    return m_camera.ExposureTimeRaw.GetValue();
  } else
    throw vpException(vpException::notImplementedError, "Don't know how to set exposure.");
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
float vpPylonGrabberGigE::setGamma(bool gamma_on, float gamma_value)
{
  connect();

  if (GenApi::IsWritable(m_camera.GammaEnable))
    m_camera.GammaEnable.SetValue(gamma_on);

  if (GenApi::IsWritable(m_camera.Gamma)) {
    m_camera.Gamma.SetValue(gamma_value);
    return m_camera.Gamma.GetValue();
  } else
    throw vpException(vpException::notImplementedError, "Don't know how to set gamma.");
}

/*!
  \brief Saves the current active configuration set into the selected
  user set.

  \param user_set See vpPylonGrabber::UserSetName for valid values.
  \param set_default Whether to set the configuration set to be used as
  the default startup set.
  \return true for finished, false otherwise.

  \sa loadUserSet()
 */
bool vpPylonGrabberGigE::saveUserSet(UserSetName user_set, bool set_default)
{
  connect();

  bool success = selectUserSet(user_set);

  if (success) {
    m_camera.UserSetSave.Execute();
    vpTime::wait(200); // How long you have to wait?
    success = m_camera.UserSetSave.IsDone();
  }

  if (success && set_default)
    success = setUserSetDefault(user_set);

  return success;
}

/*!
  \brief Sets the configuration set to be used as the default startup
  set.

  \param user_set See vpPylonGrabber::UserSetName for valid values.
  \return true for finished, false otherwise.

  \sa getUserSetDefault()
 */
bool vpPylonGrabberGigE::setUserSetDefault(UserSetName user_set)
{
  connect();

  switch (user_set) {
  case USERSET_DEFAULT:
    m_camera.UserSetDefaultSelector.SetValue(Basler_GigECamera::UserSetDefaultSelector_Default);
    return true;
    break;
  case USERSET_USERSET1:
    m_camera.UserSetDefaultSelector.SetValue(Basler_GigECamera::UserSetDefaultSelector_UserSet1);
    return true;
    break;
  case USERSET_USERSET2:
    m_camera.UserSetDefaultSelector.SetValue(Basler_GigECamera::UserSetDefaultSelector_UserSet2);
    return true;
    break;
  case USERSET_USERSET3:
    m_camera.UserSetDefaultSelector.SetValue(Basler_GigECamera::UserSetDefaultSelector_UserSet3);
    return true;
    break;
  default:
    return false;
  }
}

/*!
   Start active camera capturing images.

   \sa stopCapture()
 */
void vpPylonGrabberGigE::startCapture()
{
  connect();

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
void vpPylonGrabberGigE::stopCapture()
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
void vpPylonGrabberGigE::connect()
{
  if (m_connected == false) {
    m_numCameras = getNumCameras();
    if (m_numCameras == 0) {
      throw(vpException(vpException::fatalError, "No camera found"));
    }

    if (!m_camera.IsPylonDeviceAttached()) {
      Pylon::CTlFactory &TlFactory = Pylon::CTlFactory::GetInstance();
      Pylon::DeviceInfoList_t lstDevices;
      Pylon::DeviceInfoList_t filter; // Filter for GigE cameras.
      Pylon::CBaslerGigEDeviceInfo gige_devinfo;
      filter.push_back(gige_devinfo);
      TlFactory.EnumerateDevices(lstDevices, filter);

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
void vpPylonGrabberGigE::disconnect()
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
void vpPylonGrabberGigE::close()
{
  stopCapture();
  disconnect();
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).
*/
void vpPylonGrabberGigE::acquire(vpImage<unsigned char> &I)
{
  open();

  Pylon::CGrabResultPtr grabResult;
  // Retrieve an image
  if (!m_camera.RetrieveResult(2000, grabResult)) {
    throw(vpException(vpException::fatalError, "Cannot retrieve image from camera with serial %s",
                      getCameraSerial(m_index).c_str()));
  }

  if (grabResult->GrabSucceeded()) {
    height = grabResult->GetHeight();
    width = grabResult->GetWidth();
    I.resize(height, width);

    Pylon::CImageFormatConverter imageConvert;
    imageConvert.OutputPixelFormat = Pylon::PixelType_Mono8;
    imageConvert.OutputPaddingX = 0;
    // Create a converted image
    imageConvert.Convert(I.bitmap, sizeof(unsigned char) * width * height, (Pylon::IImage &)grabResult);
  }
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).
*/
void vpPylonGrabberGigE::acquire(vpImage<vpRGBa> &I)
{
  open();

  Pylon::CGrabResultPtr grabResult;
  // Retrieve an image
  if (!m_camera.RetrieveResult(2000, grabResult)) {
    throw(vpException(vpException::fatalError, "Cannot retrieve image from camera with serial %s",
                      getCameraSerial(m_index).c_str()));
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
void vpPylonGrabberGigE::open(vpImage<unsigned char> &I)
{
  open();
  acquire(I);
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpPylonGrabberGigE::open(vpImage<vpRGBa> &I)
{
  open();
  acquire(I);
}

/*!
   Connect to the active camera and start capture.

   Similar then calling:
   \code
   vpPylonGrabberGigE g;
   ...
   g.connect();
   g.startCapture();
   \endcode
 */
void vpPylonGrabberGigE::open()
{
  connect();
  startCapture();
}

/*!
  \brief Selects the configuration set to load, save, or configure.

  \param user_set See vpPylonGrabber::UserSetName for valid values.
  \return true for success, false for failure.

  Default User Set is read-only and cannot be modified.
 */
bool vpPylonGrabberGigE::selectUserSet(UserSetName user_set)
{
  connect();

  switch (user_set) {
  case USERSET_DEFAULT:
    m_camera.UserSetSelector.SetValue(Basler_GigECamera::UserSetSelector_Default);
    return true;
    break;
  case USERSET_USERSET1:
    m_camera.UserSetSelector.SetValue(Basler_GigECamera::UserSetSelector_UserSet1);
    return true;
    break;
  case USERSET_USERSET2:
    m_camera.UserSetSelector.SetValue(Basler_GigECamera::UserSetSelector_UserSet2);
    return true;
    break;
  case USERSET_USERSET3:
    m_camera.UserSetSelector.SetValue(Basler_GigECamera::UserSetSelector_UserSet3);
    return true;
    break;
  default:
    return false;
  }
}

/*!
   Operator that allows to capture a grey level image.
   \param I : The captured image.
 */
vpPylonGrabber &vpPylonGrabberGigE::operator>>(vpImage<unsigned char> &I)
{
  acquire(I);
  return *this;
}

/*!
   Operator that allows to capture a color image.
   \param I : The captured image.
 */
vpPylonGrabber &vpPylonGrabberGigE::operator>>(vpImage<vpRGBa> &I)
{
  acquire(I);
  return *this;
}

#else
// Work arround to avoid warning:
// libvisp_pylon.a(vpPylonGrabberGigE.cpp.o) has no symbols
void dummy_vpPylonGrabberGigE(){};
#endif // #ifdef VISP_HAVE_PYLON
