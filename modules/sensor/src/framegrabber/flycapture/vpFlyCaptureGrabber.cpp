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
 * Description: Class which enables to project an image in the 3D space
 * and get the view of a virtual camera.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFlyCaptureGrabber.cpp
  \brief Wrapper over PointGrey FlyCapture SDK to capture images from
  PointGrey cameras.
*/

#include <visp3/core/vpException.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

#ifdef VISP_HAVE_FLYCAPTURE

#include <visp3/core/vpTime.h>

/*!
   Default constructor that consider the first camera found on the bus as
   active.
 */
vpFlyCaptureGrabber::vpFlyCaptureGrabber()
  : m_camera(), m_guid(), m_index(0), m_numCameras(0), m_rawImage(), m_connected(false), m_capture(false)
{
  m_numCameras = this->getNumCameras();
}

/*!
   Default destructor that closes the connection with the camera.
 */
vpFlyCaptureGrabber::~vpFlyCaptureGrabber() { close(); }
/*!
  \return Return the number of cameras connected on the bus.
*/
unsigned int vpFlyCaptureGrabber::getNumCameras()
{
  unsigned int numCameras;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error = busMgr.GetNumOfCameras(&numCameras);
  if (error != FlyCapture2::PGRERROR_OK) {
    numCameras = 0;
  }
  return numCameras;
}

/*!
  Print to the output stream active camera information (serial number, camera
  model, camera vendor, sensor, resolution, firmaware version, ...).
  */
std::ostream &vpFlyCaptureGrabber::getCameraInfo(std::ostream &os)
{
  this->connect();

  FlyCapture2::CameraInfo camInfo;
  FlyCapture2::Error error = m_camera.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  os << "Camera information:   " << std::endl;
  os << " Serial number      : " << camInfo.serialNumber << std::endl;
  os << " Camera model       : " << camInfo.modelName << std::endl;
  os << " Camera vendor      : " << camInfo.vendorName << std::endl;
  os << " Sensor             : " << camInfo.sensorInfo << std::endl;
  os << " Resolution         : " << camInfo.sensorResolution << std::endl;
  os << " Firmware version   : " << camInfo.firmwareVersion << std::endl;
  os << " Firmware build time: " << camInfo.firmwareBuildTime << std::endl;
  return os;
}

/*!
  Return the handler to the active camera or NULL if the camera is not
connected. This function was designed to provide a direct access to the
FlyCapture SDK to get access to advanced functionalities that are not
implemented in this class.

  We provide here after and example that shows how to use this function to
access to the camera and check if a given video mode and framerate are
supported by the camera.

\code
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.connect();
  FlyCapture2::Camera *handler = g.getCameraHandler();
  bool supported = false;
  handler->GetVideoModeAndFrameRateInfo(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60, &supported);
  if (supported)
    g.setVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60); g.startCapture();

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
\endcode


  The following code shows how to use this function to check if a given
format7 (here MODE_0, PIXEL_FORMAT_MONO8) is supported by the camera:
\code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;
  g.connect();
  FlyCapture2::Camera *handler = g.getCameraHandler();

  // Query for available Format 7 modes
  const FlyCapture2::Mode k_fmt7Mode = FlyCapture2::MODE_0;
  const FlyCapture2::PixelFormat k_fmt7PixFmt = FlyCapture2::PIXEL_FORMAT_MONO8;

  FlyCapture2::Format7Info fmt7Info;
  bool supported;
  fmt7Info.mode = k_fmt7Mode;
  FlyCapture2::Error error = handler->GetFormat7Info( &fmt7Info, &supported );
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    return -1;
  }
  if (supported) {
    std::cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << std::endl;
     std::cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")"
               << std::endl;
     std::cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")"
               << std::endl;
     std::cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << std::endl;

    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 ) {
      // Pixel format not supported!
      std::cout << "Pixel format is not supported" << std::endl;
      return -1;
    }
  }
#endif
}
  \endcode

*/
FlyCapture2::Camera *vpFlyCaptureGrabber::getCameraHandler()
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

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Frame rate: " << g.getFrameRate() << " fps" << std::endl;
  \endcode

  \sa setFrameRate()
 */
float vpFlyCaptureGrabber::getFrameRate()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::FRAME_RATE);
  return prop.absValue;
}

/*!
  Return camera shutter value in ms.
  If the camera doesn't support shutter property, return an exception.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Shutter value: " << g.getShutter() << " ms" << std::endl;
  \endcode

  \sa setShutter()
 */
float vpFlyCaptureGrabber::getShutter()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::SHUTTER);
  return prop.absValue;
}

/*!
  Return camera gain value in db.
  If the camera doesn't support gain property, return an exception.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Gain value: " << g.getGain() << " ms" << std::endl;
  \endcode

  \sa setGain()
 */
float vpFlyCaptureGrabber::getGain()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::GAIN);
  return prop.absValue;
}

/*!
  Return brightness value in %.
  If the camera doesn't support brightness property, return an exception.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Brightness value: " << g.getBrightness() << " %" << std::endl;
  \endcode

  \sa setBrightness()
 */
float vpFlyCaptureGrabber::getBrightness()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::BRIGHTNESS);
  return prop.absValue;
}

/*!
  Return sharpness value.
  If the camera doesn't support sharpness property, return an exception.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Sharpness value: " << g.getSharpness() << std::endl;
  \endcode

  \sa setSharpness()
 */
unsigned int vpFlyCaptureGrabber::getSharpness()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::SHARPNESS);
  return prop.valueA;
}

/*!
  Return exposure value.
  If the camera doesn't support exposure property, return an exception.

  \code
  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);
  std::cout << "Exposure value: " << g.getExposure() << std::endl;
  \endcode

  \sa setExposure()
 */
float vpFlyCaptureGrabber::getExposure()
{
  this->connect();

  FlyCapture2::Property prop = this->getProperty(FlyCapture2::AUTO_EXPOSURE);
  return prop.absValue;
}

/*!
  Return the serial id of a camera with \e index.
  \param index : Camera index.

  The following code shows how to retrieve the serial id of all the cameras
that are connected on the bus.
\code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;
  unsigned int num_cameras = vpFlyCaptureGrabber::getNumCameras();
  for (unsigned int i=0; i<num_cameras; i++) {
    unsigned int serial_id = vpFlyCaptureGrabber::getCameraSerial(i);
    std::cout << "Camera with index " << i << " has serial id: " << serial_id << std::endl;
  }
#endif
}
  \endcode

  When two cameras are connected (PGR Flea3 in our case), we get the
 following:
  \code
Camera with index 0 has serial id: 15372913
Camera with index 1 has serial id: 15290004
\endcode

  \sa setCameraSerial()
 */
unsigned int vpFlyCaptureGrabber::getCameraSerial(unsigned int index)
{
  unsigned int num_cameras = vpFlyCaptureGrabber::getNumCameras();
  if (index >= num_cameras) {
    throw(vpException(vpException::badValue, "The camera with index %u is not present. Only %d cameras connected.",
                      index, num_cameras));
  }
  unsigned int serial_id;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error;
  error = busMgr.GetCameraSerialNumberFromIndex(index, &serial_id);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get camera with index %d serial id.", index));
  }
  return serial_id;
}

/*!

  If multiples cameras are connected on the bus, select the camero to dial
  with.

  \param index : Current camera index, a value comprised between 0 (the first
  camera found on the bus) and the number of cameras found on the bus and
  returned by getNumCameras() minus 1. If two cameras are connected on the
  bus, setting \e index to one allows to communicate with the second one. This
  identifier is not unique. That is why, it is also possible to select a
  camera by its serial number, which is unique using setCameraSerial().

  \exception vpException::badValue : If the index is greater or equal to the
  number of cameras connected to the bus.
  */
void vpFlyCaptureGrabber::setCameraIndex(unsigned int index)
{
  if (index >= m_numCameras) {
    throw(vpException(vpException::badValue, "The camera with index %u is not present. Only %d cameras connected.",
                      index, m_numCameras));
  }

  m_index = index;
}

/*!
   Set the current camera from its unique serial identifier.
   \param serial_id : Camera serial id.

   The following example shows how to capture images from a camera that has
seial id 15290004.
\code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraSerial(15290004); // Set camera with serial id
  g.open(I);
  g.getCameraInfo(std::cout);

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
  \endcode

  \sa getCameraSerial()
 */
void vpFlyCaptureGrabber::setCameraSerial(unsigned int serial_id)
{
  FlyCapture2::BusManager busMgr;
  FlyCapture2::Error error;
  m_numCameras = this->getNumCameras();
  for (unsigned int i = 0; i < m_numCameras; i++) {
    if (vpFlyCaptureGrabber::getCameraSerial(i) == serial_id) {
      m_index = i;
      return;
    }
  }
  throw(vpException(vpException::badValue, "The camera with serial id %u is not present.", serial_id));
}

/*!
  Set camera property.

  \param prop_type : Property type.
  \param on : true to turn property on.
  \param auto_on : true to turn auto mode on, false to turn manual mode.
  \param value : value to set.
  \param prop_value : Switch to affect value to the corresponding variable.
 */
void vpFlyCaptureGrabber::setProperty(const FlyCapture2::PropertyType &prop_type, bool on, bool auto_on, float value,
                                      PropertyValue prop_value)
{
  this->connect();

  FlyCapture2::PropertyInfo propInfo;
  propInfo = this->getPropertyInfo(prop_type);

  if (propInfo.present) {
    FlyCapture2::Property prop;
    prop.type = prop_type;
    prop.onOff = on && propInfo.onOffSupported;
    prop.autoManualMode = auto_on && propInfo.autoSupported;
    prop.absControl = propInfo.absValSupported;
    switch (prop_value) {
    case ABS_VALUE: {
      float value_ = (std::max)((std::min)((float)value, (float)propInfo.absMax), (float)propInfo.absMin);
      prop.absValue = value_;
      break;
    }
    case VALUE_A: {
      unsigned int value_ =
          (std::max)((std::min)((unsigned int)value, (unsigned int)propInfo.max), (unsigned int)propInfo.min);
      prop.valueA = value_;
      break;
    }
    }

    FlyCapture2::Error error;
    error = m_camera.SetProperty(&prop);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot set property %d.", (int)prop_type));
    }
  }
}

/*!
  Set camera frame rate.
  \param frame_rate : Camera frame rate (fps) to set.
  \return The camera current framerate.

  The following example shows how to use this function.
  \code
#include <iomanip>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float framerate = g.getFrameRate();
  std::cout << "Cur frame rate: " << std::fixed << std::setprecision(3) << framerate << " fps" << std::endl;
  framerate = g.setFrameRate(30); // Set framerate to 30 fps
  std::cout << "New frame rate: " << std::fixed << std::setprecision(3) << framerate << " fps" << std::endl;

  g.open(I);
  while (1)
    g.acquire(I);
#endif
}
  \endcode

  \sa getFramerate()
 */
float vpFlyCaptureGrabber::setFrameRate(float frame_rate)
{
  this->connect();

  this->setProperty(FlyCapture2::FRAME_RATE, true, false, frame_rate);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::FRAME_RATE);
  return prop.absValue;
}

/*!
  Set camera shutter mode and parameter.
  \param auto_shutter : If true set auto shutter, if false set manual shutter
applying \e shutter_ms parameter. \param shutter_ms : This is the speed at
which the camera shutter opens and closes in manual mode. \return The measured
shutter after applying the new setting.

  The following example shows how to use this function:
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float shutter_ms = g.getShutter();
  std::cout << "Shutter       : " << shutter_ms << " ms" << std::endl;
  shutter_ms = g.setShutter(false, 10); // Turn manual shutter on to 10ms
  std::cout << "Shutter manual: " << shutter_ms << " ms" << std::endl;
  shutter_ms = g.setShutter(true); // Turn auto shutter on
  std::cout << "Shutter auto  : " << shutter_ms << " ms" << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  \sa getShutter()
 */
float vpFlyCaptureGrabber::setShutter(bool auto_shutter, float shutter_ms)
{
  this->connect();

  this->setProperty(FlyCapture2::SHUTTER, true, auto_shutter, shutter_ms);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::SHUTTER);
  return prop.absValue;
}

/*!
  Set camera gain mode and value.
  \param gain_auto : If true set auto gain, if false set manual gain applying
\e gain_value parameter. \param gain_value : The amount of amplification that
is applied to a pixel in manual mode. An increase in gain can result in an
increase in noise. \return The measured gain after applying the new setting.

  The following example shows how to use this function:
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float gain_db = g.getGain();
  std::cout << "Gain: " << gain_db << " db" << std::endl;
  gain_db = g.setGain(false, 5); // Turn manual gain on to 5db
  std::cout << "Gain manual: " << gain_db << " db" << std::endl;
  gain_db = g.setGain(true); // Turn auto shutter on
  std::cout << "Gain auto  : " << gain_db << " db" << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  \sa getGain()

 */
float vpFlyCaptureGrabber::setGain(bool gain_auto, float gain_value)
{
  this->connect();

  this->setProperty(FlyCapture2::GAIN, true, gain_auto, gain_value);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::GAIN);
  return prop.absValue;
}

/*!
  Set camera brightness mode and parameter.
  \param brightness_auto : If true set auto brightness, if false set manual
brightness applying \e brightness_value parameter. \param brightness_value :
This is the level of black in an image. A high brightness will result in a low
amount of black in the image. \return The measured brightness after applying
the new setting.

  The following example shows how to use this function:
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float brightness = g.getBrightness();
  std::cout << "Brightness       : " << brightness << " %" << std::endl;
  brightness = g.setBrightness(false, 2); // Turn manual brightness on to 2%
  std::cout << "Brightness manual: " << brightness << " %" << std::endl;
  brightness = g.setBrightness(true); // Turn auto brightness on
  std::cout << "Brightness auto  : " << brightness << " %" << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  \sa getBrightness()
 */
float vpFlyCaptureGrabber::setBrightness(bool brightness_auto, float brightness_value)
{
  this->connect();

  this->setProperty(FlyCapture2::BRIGHTNESS, true, brightness_auto, brightness_value);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::BRIGHTNESS);
  return prop.absValue;
}

/*!
  Set camera exposure mode and parameter.

  \param exposure_on : If true turn exposure on, otherwise turn off.
  \param exposure_auto : If true set auto exposure, if false set manual
exposure applying \e exposure_value parameter. \param exposure_value : This is
the average intensity of the image. It will use other available (non-manually
adjustable) controls to adjust the image. Specifically, when shutter and gain
are both in auto mode, manually adjusting the exposure is actually adjusting
the auto-exposure, which tries to make the average intensity of the image 1/4
of the auto-exposure value e.g. exposure is 400, the camera will try to adjust
shutter and gain so that the average image intensity is 100. When the
  auto-exposure mode is enabled for exposure, the camera tries to manipulate
shutter and gain such that 5% of the image is saturated (pixel value of 255).

  \return The measured brightness after applying the new setting.

  The following example shows how to use this function:
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float exposure = g.getExposure();
  std::cout << "Exposure       : " << exposure << std::endl;
  exposure = g.setExposure(true, false, 1); // Turn manual exposure on to 1
  std::cout << "Exposure manual: " << exposure << std::endl;
  exposure = g.setExposure(true, true); // Turn auto exposure on
  std::cout << "Exposure auto  : " << exposure << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  \sa getExposure()
 */
float vpFlyCaptureGrabber::setExposure(bool exposure_on, bool exposure_auto, float exposure_value)
{
  this->connect();

  this->setProperty(FlyCapture2::AUTO_EXPOSURE, exposure_on, exposure_auto, exposure_value);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::AUTO_EXPOSURE);
  return prop.absValue;
}

/*!
  Set camera sharpness mode and parameter.

  \param sharpness_on : If true turn sharpness on, otherwise turn off.
  \param sharpness_auto : If true set auto sharpness, if false set manual
sharpness applying \e sharpness_value parameter. \param sharpness_value :
Parameter used to tune the filter applyed on the image to reduce blurred edges
in an image.

  \return The measured sharpness after applying the new setting.

  The following example shows how to use this function:
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  float sharpness = g.getSharpness();
  std::cout << "Sharpness       : " << sharpness << std::endl;
  sharpness = g.setSharpness(true, false, 1000); // Turn manual sharpness on to 1000
  std::cout << "Sharpness manual: " << sharpness << std::endl;
  sharpness = g.setSharpness(true, true); // Turn auto sharpness on
  std::cout << "Sharpness auto  : " << sharpness << std::endl;

  g.open(I);
  ...
#endif
}
  \endcode

  \sa getSharpness()
 */
unsigned int vpFlyCaptureGrabber::setSharpness(bool sharpness_on, bool sharpness_auto, unsigned int sharpness_value)
{
  this->connect();

  this->setProperty(FlyCapture2::SHARPNESS, sharpness_on, sharpness_auto, (float)sharpness_value, VALUE_A);
  FlyCapture2::Property prop = this->getProperty(FlyCapture2::SHARPNESS);
  return prop.valueA;
}

/*!
  Return property values.
  \param prop_type : Property type.
 */
FlyCapture2::Property vpFlyCaptureGrabber::getProperty(FlyCapture2::PropertyType prop_type)
{
  this->connect();

  FlyCapture2::Property prop;
  prop.type = prop_type;
  FlyCapture2::Error error;
  error = m_camera.GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get property %d value.", (int)prop_type));
  }
  return prop;
}

/*!
  Return information concerning a given property type.
  \param prop_type : Property type.
  \exception vpException::fatalError : If property type doesn't exist.
 */
FlyCapture2::PropertyInfo vpFlyCaptureGrabber::getPropertyInfo(FlyCapture2::PropertyType prop_type)
{
  this->connect();

  FlyCapture2::PropertyInfo propInfo;
  propInfo.type = prop_type;

  FlyCapture2::Error error;
  error = m_camera.GetPropertyInfo(&propInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get property %d info.", (int)prop_type));
  }
  return propInfo;
}

/*!
  Set video mode and framerate of the active camera.
  \param video_mode : Camera video mode.
  \param frame_rate : Camera frame rate.

  The following example shows how to use this function to set the camera image
resolution to 1280 x 960, pixel format to Y8 and capture framerate to 60 fps.
  \code
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  vpImage<unsigned char> I;
  vpFlyCaptureGrabber g;

  g.setCameraIndex(0); // Default camera is the first on the bus
  g.setVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60);
  g.open(I);
  g.getCameraInfo(std::cout);

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
  }
#endif
}
  \endcode
 */
void vpFlyCaptureGrabber::setVideoModeAndFrameRate(FlyCapture2::VideoMode video_mode, FlyCapture2::FrameRate frame_rate)
{
  this->connect();

  FlyCapture2::Error error;
  error = m_camera.SetVideoModeAndFrameRate(video_mode, frame_rate);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot set video mode and framerate."));
  }
}

/*!
  Return true if video mode and framerate is supported.
  */
bool vpFlyCaptureGrabber::isVideoModeAndFrameRateSupported(FlyCapture2::VideoMode video_mode,
                                                           FlyCapture2::FrameRate frame_rate)
{
  this->connect();

  FlyCapture2::Error error;
  bool supported = false;
  error = m_camera.GetVideoModeAndFrameRateInfo(video_mode, frame_rate, &supported);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get video mode and framerate."));
  }
  return supported;
}

/*!
  Return size and offset corresponding to a centered roi.
  \param size : Horizontal or vertical roi size. If set to 0, use the max
  allowed size. \param max_size : Allowed max size. \param step : Step.
 */
std::pair<unsigned int, unsigned int> vpFlyCaptureGrabber::centerRoi(unsigned int size, unsigned int max_size,
                                                                     unsigned int step)
{
  if (size == 0 || size > max_size)
    size = max_size;
  // size should be a multiple of step
  size = size / step * step;
  const unsigned int offset = (max_size - size) / 2;
  // Return offset for centering roi
  return std::make_pair(size, offset);
}

/*!
  Set format7 video mode.
  \param format7_mode : Format 7 mode.
  \param pixel_format : Pixel format.
  \param width,height : Size of the centered roi. If set to 0, use the max
allowed size.

  If the format7 video mode and pixel format are not supported, return an
exception.

  The following example shows how to use this fonction to capture a 640x480
roi:

  \code
#include <iomanip>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpImage<unsigned char> I;

  vpFlyCaptureGrabber g;
  g.setCameraIndex(0);

  g.setFormat7VideoMode(FlyCapture2::MODE_0, FlyCapture2::PIXEL_FORMAT_MONO8, 640, 480);

  g.open(I);
  ...
#endif
}
  \endcode
 */
void vpFlyCaptureGrabber::setFormat7VideoMode(FlyCapture2::Mode format7_mode, FlyCapture2::PixelFormat pixel_format,
                                              unsigned int width, unsigned int height)
{
  this->connect();

  FlyCapture2::Format7Info fmt7_info;
  bool fmt7_supported;
  FlyCapture2::Error error;

  fmt7_info.mode = format7_mode;
  error = m_camera.GetFormat7Info(&fmt7_info, &fmt7_supported);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get format7 info."));
  }
  if (!fmt7_supported) {
    throw(vpException(vpException::fatalError, "Format7 mode %d not supported.", (int)format7_mode));
  }

  FlyCapture2::Format7ImageSettings fmt7_settings;
  fmt7_settings.mode = format7_mode;
  fmt7_settings.pixelFormat = pixel_format;
  // Set centered roi
  std::pair<unsigned int, unsigned int> roi_w = this->centerRoi(width, fmt7_info.maxWidth, fmt7_info.imageHStepSize);
  std::pair<unsigned int, unsigned int> roi_h = this->centerRoi(height, fmt7_info.maxHeight, fmt7_info.imageVStepSize);
  fmt7_settings.width = roi_w.first;
  fmt7_settings.offsetX = roi_w.second;
  fmt7_settings.height = roi_h.first;
  fmt7_settings.offsetY = roi_h.second;

  // Validate the settings
  FlyCapture2::Format7PacketInfo fmt7_packet_info;
  bool valid = false;
  error = m_camera.ValidateFormat7Settings(&fmt7_settings, &valid, &fmt7_packet_info);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot validate format7 settings."));
  }
  if (!valid) {
    throw(vpException(vpException::fatalError, "Format7 settings are not valid."));
  }
  error = m_camera.SetFormat7Configuration(&fmt7_settings, fmt7_packet_info.recommendedBytesPerPacket);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot set format7 settings."));
  }
}

/*!
  Return true if format7 mode is supported.
  */
bool vpFlyCaptureGrabber::isFormat7Supported(FlyCapture2::Mode format7_mode)
{
  this->connect();

  FlyCapture2::Format7Info fmt7_info;
  bool supported = false;
  FlyCapture2::Error error;

  fmt7_info.mode = format7_mode;
  error = m_camera.GetFormat7Info(&fmt7_info, &supported);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot get format7 info."));
  }

  return supported;
}
/*!
   Start active camera capturing images.

   \sa stopCapture()
 */
void vpFlyCaptureGrabber::startCapture()
{
  this->connect();

  if (m_capture == false) {

    FlyCapture2::Error error;
    error = m_camera.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot start capture for camera with serial %u",
                        getCameraSerial(m_index)));
    }
    m_capture = true;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Stop active camera capturing images.

   \sa startCapture()
 */
void vpFlyCaptureGrabber::stopCapture()
{
  if (m_capture == true) {

    FlyCapture2::Error error;
    error = m_camera.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot stop capture."));
    }
    m_capture = false;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Connect the active camera.

   \sa disconnect()
 */
void vpFlyCaptureGrabber::connect()
{
  if (m_connected == false) {
    FlyCapture2::Error error;
    m_numCameras = this->getNumCameras();
    if (m_numCameras == 0) {
      throw(vpException(vpException::fatalError, "No camera found on the bus"));
    }

    FlyCapture2::BusManager busMgr;

    error = busMgr.GetCameraFromIndex(m_index, &m_guid);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot retrieve guid of camera with index %u.", m_index));
    }
    // Connect to a camera
    error = m_camera.Connect(&m_guid);
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot connect to camera with serial %u", getCameraSerial(m_index)));
    }
    m_connected = true;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Disconnect the active camera.

   \sa connect()
 */
void vpFlyCaptureGrabber::disconnect()
{
  if (m_connected == true) {

    FlyCapture2::Error error;
    error = m_camera.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot stop capture."));
    }
    m_connected = false;
  }
  if (m_connected && m_capture)
    init = true;
  else
    init = false;
}

/*!
   Stop active camera capturing images and disconnect the active camera.
   If you want to use again this camera, you may call setCamera(const unsigned
   int &) and open(vpImage<unsigned char> &) or open(vpImage<vpRGBa> &) to
   connect again the camera.

   Similar then calling stopCapture() and disconnect():
   \code
   vpFlyCaptureGrabber g;
   ...
   g.stopCapture();
   g.disconnect();
   \endcode
 */
void vpFlyCaptureGrabber::close()
{
  this->stopCapture();
  this->disconnect();
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).
*/
void vpFlyCaptureGrabber::acquire(vpImage<unsigned char> &I)
{
  FlyCapture2::TimeStamp timestamp;
  this->acquire(I, timestamp);
}

/*!
  Acquire a gray level image from the active camera.

  \param I : Image data structure (8 bits image).

  \param timestamp : The acquisition timestamp.
*/
void vpFlyCaptureGrabber::acquire(vpImage<unsigned char> &I, FlyCapture2::TimeStamp &timestamp)
{
  this->open();

  FlyCapture2::Error error;
  // Retrieve an image
  error = m_camera.RetrieveBuffer(&m_rawImage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot retrieve image from camera with serial %u",
                      getCameraSerial(m_index)));
  }
  timestamp = m_rawImage.GetTimeStamp();

  height = m_rawImage.GetRows();
  width = m_rawImage.GetCols();
  I.resize(height, width);

  // Create a converted image using a stride equals to `sizeof(unsigned
  // char) * width`, which makes sure there is no paddings or holes
  // between pixel data. And the convertedImage object is sharing the
  // same data buffer with vpImage object `I`.
  FlyCapture2::Image convertedImage(height, width, sizeof(unsigned char) * width, I.bitmap,
                                    sizeof(unsigned char) * I.getSize(), FlyCapture2::PIXEL_FORMAT_MONO8);

  // Convert the raw image
  error = m_rawImage.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &convertedImage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot convert image from camera with serial %u",
                      getCameraSerial(m_index)));
  }
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).
*/
void vpFlyCaptureGrabber::acquire(vpImage<vpRGBa> &I)
{
  FlyCapture2::TimeStamp timestamp;
  this->acquire(I, timestamp);
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (RGBa image).

  \param timestamp : The acquisition timestamp.
*/
void vpFlyCaptureGrabber::acquire(vpImage<vpRGBa> &I, FlyCapture2::TimeStamp &timestamp)
{
  this->open();

  FlyCapture2::Error error;
  // Retrieve an image
  error = m_camera.RetrieveBuffer(&m_rawImage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot retrieve image from camera with serial %u",
                      getCameraSerial(m_index)));
  }
  timestamp = m_rawImage.GetTimeStamp();

  // Create a converted image
  FlyCapture2::Image convertedImage;

  // Convert the raw image
  error = m_rawImage.Convert(FlyCapture2::PIXEL_FORMAT_RGBU, &convertedImage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot convert image from camera with serial %u",
                      getCameraSerial(m_index)));
  }
  height = convertedImage.GetRows();
  width = convertedImage.GetCols();
  I.resize(height, width);

  unsigned char *data = convertedImage.GetData();
  unsigned int stride = convertedImage.GetStride();
  unsigned int Bps = convertedImage.GetBitsPerPixel() / 8; // Bytes per pixel
  // `I.bitmap` and `I[i]` are pointers to `vpRGBa` objects. While
  // `data` is a pointer to an array of 32-bit RGBU values with each
  // value a byte in the order of R, G, B and U and goes on.
  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      unsigned char *pp = data + i * stride + j * Bps;
      I[i][j].R = pp[0];
      I[i][j].G = pp[1];
      I[i][j].B = pp[2];
      I[i][j].A = pp[3];
    }
  }
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpFlyCaptureGrabber::open(vpImage<unsigned char> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera, start capture and retrieve an image.
   \param I : Captured image.
 */
void vpFlyCaptureGrabber::open(vpImage<vpRGBa> &I)
{
  this->open();
  this->acquire(I);
}

/*!
   Connect to the active camera and start capture.

   Similar then calling:
   \code
   vpFlyCaptureGrabber g;
   ...
   g.connect();
   g.startCapture();
   \endcode
 */
void vpFlyCaptureGrabber::open()
{
  this->connect();
  this->startCapture();
}

/*!
   Return true if camera power is available, false otherwise.

   \sa getCameraPower(), setCameraPowerOn(), setCameraPowerOff()
 */
bool vpFlyCaptureGrabber::isCameraPowerAvailable()
{
  this->connect();

  const unsigned int powerReg = 0x400;
  unsigned int powerRegVal = 0;

  FlyCapture2::Error error;
  error = m_camera.ReadRegister(powerReg, &powerRegVal);
  if (error != FlyCapture2::PGRERROR_OK) {
    return false;
  }

  return ((powerRegVal & 0x00008000) != 0);
}

/*!
  Return true if the camera is powered on, false otherwise

  \sa setCameraPower()
 */
bool vpFlyCaptureGrabber::getCameraPower()
{
  if (!isCameraPowerAvailable())
    return false;
  const unsigned int powerReg = 0x610;
  unsigned int powerRegVal = 0;

  FlyCapture2::Error error;
  error = m_camera.ReadRegister(powerReg, &powerRegVal);
  if (error != FlyCapture2::PGRERROR_OK) {
    return false;
  }

  return ((powerRegVal & (0x1 << 31)) != 0);
}

/*!
  Power on/off the camera.

  \param on : true to power on the camera, false to power off the camera.

  The following example shows how to turn off a camera.
  \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  vpFlyCaptureGrabber g;

  g.setCameraIndex(0);
  g.connect();

  bool power = g.getCameraPower();
  std::cout << "Camera is powered: " << ((power == true) ? "on" : "off") << std::endl;

  if (power)
    g.setCameraPower(false); // Power off the camera
#endif
}
  \endcode

  \sa getCameraPower()
 */
void vpFlyCaptureGrabber::setCameraPower(bool on)
{
  this->connect();

  if (!isCameraPowerAvailable()) {
    throw(vpException(vpException::badValue, "Cannot power on camera. Feature not available"));
  }

  // Power on the camera
  const unsigned int powerReg = 0x610;
  unsigned int powerRegVal = 0;

  powerRegVal = (on == true) ? 0x80000000 : 0x0;

  FlyCapture2::Error error;
  error = m_camera.WriteRegister(powerReg, powerRegVal);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot power on the camera."));
  }

  const unsigned int millisecondsToSleep = 100;
  unsigned int regVal = 0;
  unsigned int retries = 10;

  // Wait for camera to complete power-up
  do {
    vpTime::wait(millisecondsToSleep);
    error = m_camera.ReadRegister(powerReg, &regVal);
    if (error == FlyCapture2::PGRERROR_TIMEOUT) {
      // ignore timeout errors, camera may not be responding to
      // register reads during power-up
    } else if (error != FlyCapture2::PGRERROR_OK) {
      error.PrintErrorTrace();
      throw(vpException(vpException::fatalError, "Cannot power on the camera."));
    }

    retries--;
  } while ((regVal & powerRegVal) == 0 && retries > 0);

  // Check for timeout errors after retrying
  if (error == FlyCapture2::PGRERROR_TIMEOUT) {
    error.PrintErrorTrace();
    throw(vpException(vpException::fatalError, "Cannot power on the camera. Timeout occur"));
  }
}

/*!

   Operator that allows to capture a grey level image.
   \param I : The captured image.

   \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
  vpImage<unsigned char> I;
  vpFlyCaptureGrabber g;
  g >> I;
}
   \endcode
 */
vpFlyCaptureGrabber &vpFlyCaptureGrabber::operator>>(vpImage<unsigned char> &I)
{
  this->acquire(I);
  return *this;
}

/*!

   Operator that allows to capture a grey level image.
   \param I : The captured image.

   \code
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
  vpImage<vpRGBa> I;
  vpFlyCaptureGrabber g;
  g >> I;
}
   \endcode
 */
vpFlyCaptureGrabber &vpFlyCaptureGrabber::operator>>(vpImage<vpRGBa> &I)
{
  this->acquire(I);
  return *this;
}

#else
// Work arround to avoid warning:
// libvisp_flycapture.a(vpFlyCaptureGrabber.cpp.o) has no symbols
void dummy_vpFlyCaptureGrabber(){};
#endif
