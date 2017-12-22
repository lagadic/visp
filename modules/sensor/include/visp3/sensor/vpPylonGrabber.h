/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Description: Classes allow to grab images from a Basler camera using
 * Pylon SDK.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

#ifndef __vpPylonGrabber_h_
#define __vpPylonGrabber_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFrameGrabber.h>

#ifdef VISP_HAVE_PYLON

// Work arround: /usr/X11R6/include/X11/X.h:115:30: note: expanded from macro
// 'None'
#ifdef None
#undef None
#endif
// Work arround: /usr/X11R6/include/X11/Xlib.h:83:16: note: expanded from
// macro 'Status'
#ifdef Status
#undef Status
#endif

#include <pylon/PylonIncludes.h>

/*!
  \file vpPylonGrabber.h
  \brief Wrapper over Basler Pylon SDK to capture images from Basler
  cameras.
*/

/*!
  \class vpPylonGrabber
  \ingroup group_sensor_camera

  Allows to grab images from a Basler camera using Pylon SDK. This is an
  abstract base class which can't be instantiated. Use vpPylonFactory
  functions to create class instances instead. Currently only GigE and
  USB3 cameras are supported.

  To get a vpPylonGrabber, use the following code.
  \code
  vpPylonFactory &factory = vpPylonFactory::instance();
  vpPylonGrabber *g = factory.createPylonGrabber(vpPylonFactory::BASLER_GIGE);
  \endcode

  To use this class install first Pylon SDK\n
  https://www.baslerweb.com/en/support/downloads/software-downloads/#type=pylonsoftware;version=all
  \n Installation instructions included.

  Once installed, configure ViSP using cmake to detect Pylon SDK and
  build ViSP to include Pylon SDK support.

  This class was tested under Ubuntu with pylon 5.0.9 and the following
  cameras:
  - acA640-90gm
  - acA1600-60gm

  This class is inspired by vpFlyCaptureGrabber with much simplified methods.
 */
class VISP_EXPORT vpPylonGrabber : public vpFrameGrabber
{
public:
  /*!
     Default destructor.
   */
  virtual ~vpPylonGrabber(){};

  /*! Valid values for user set names.
   */
  enum UserSetName {
    USERSET_DEFAULT,  //!< The default user set.
    USERSET_USERSET1, //!< User set 1.
    USERSET_USERSET2, //!< User set 2.
    USERSET_USERSET3, //!< User set 3.
    USERSET_UNKNOWN   //!< User set not supported.
  };

  /*!
    Acquire a gray level image from the active camera.

    \param I : Image data structure (8 bits image).
  */
  virtual void acquire(vpImage<unsigned char> &I) = 0;
  /*!
    Acquire a color image from the active camera.

    \param I : Image data structure (RGBa image).
  */
  virtual void acquire(vpImage<vpRGBa> &I) = 0;

  /*!
     \brief Stop active camera capturing images and disconnect the active
     camera.

     If you want to use again this camera, you may call
     setCameraIndex(const unsigned int &) and open(vpImage<unsigned char>
     &) or open(vpImage<vpRGBa> &) to connect again the camera.
   */
  virtual void close() = 0;
  /*!
     Connect the active camera.

     \sa disconnect()
   */
  virtual void connect() = 0;
  /*!
     Disconnect the active camera.

     \sa connect()
   */
  virtual void disconnect() = 0;

  /*!
    Return blacklevel value in % or raw value.
    If the camera doesn't support blacklevel property, return an exception.

    According to SFNC (standard feature naming convention) of GenICam
    standard, Black level is used instead of brightness.

    See "Terminology Changes" section of the page:
    https://www.ptgrey.com/kb/11020?countryid=237
    \sa setBlackLevel()
   */
  virtual float getBlackLevel() = 0;
  /*!
    Print to the output stream active camera information (serial number,
    camera model, camera vendor, sensor, resolution, firmware version,
    ...).
    */
  virtual std::ostream &getCameraInfo(std::ostream &os) = 0;
  /*!
    Return the handler to the active camera or NULL if the camera is not
    connected. This function was designed to provide a direct access to
    the Pylon SDK to get access to advanced functionalities that are not
    implemented in this class.
  */
  virtual Pylon::CInstantCamera *getCameraHandler() = 0;
  /*! Return the index of the active camera. */
  virtual unsigned int getCameraIndex() const = 0;
  /*!
    Return the serial id of a camera with \e index.
    \param index : Camera index.

    \sa setCameraSerial()
   */
  virtual std::string getCameraSerial(unsigned int index) = 0;
  /*!
    Return exposure value in ms.
    If the camera doesn't support exposure property, return an exception.

    According to SFNC (standard feature naming convention) of GenICam
    standard, Exposure is used and deprecates shutter.

    See "Terminology Changes" section of the page:
    https://www.ptgrey.com/kb/11020?countryid=237
    \sa setExposure()
   */
  virtual float getExposure() = 0;
  /*!
    Return camera capture framerate.
    If the camera doesn't support framerate property, return an exception.

    \sa setFrameRate()
   */
  virtual float getFrameRate() = 0;
  /*!
    Return camera gain value in dB or raw value.
    If the camera doesn't support gain property, return an exception.

    \sa setGain()
   */
  virtual float getGain() = 0;
  /*!
    \brief Get the number of cameras of specific subclasses. GigE, USB,
    etc.
    \return Return the number of cameras connected on the bus GigE or USB.
  */
  virtual unsigned int getNumCameras() = 0;
  /*!
    Return gamma correction value.
    If the camera doesn't support gamma property, return an exception.

    \sa setGamma()
   */
  virtual float getGamma() = 0;
  /*!
    \brief Loads the selected configuration into the camera's volatile
    memory and makes it the active configuration set.

    \param user_set See vpPylonGrabber::UserSetName for valid values.
    \return true for finished, false otherwise.

    \sa saveUserSet()
   */
  virtual bool loadUserSet(UserSetName user_set) = 0;
  /*!
    \brief Gets the configuration set being used as the default startup
    set.

    \sa setUserSetDefault()
   */
  virtual UserSetName getUserSetDefault() = 0;

  //! Return true if the camera is connected.
  virtual bool isConnected() const = 0;
  //! Return true if the camera capture is started.
  virtual bool isCaptureStarted() const = 0;
  /*!
     Connect to the active camera, start capture and retrieve an image.
     \param I : Captured image.
   */
  virtual void open(vpImage<unsigned char> &I) = 0;
  /*!
     Connect to the active camera, start capture and retrieve an image.
     \param I : Captured image.
   */
  virtual void open(vpImage<vpRGBa> &I) = 0;

  /*!
     Operator that allows to capture a grey level image.
     \param I : The captured image.
   */
  virtual vpPylonGrabber &operator>>(vpImage<unsigned char> &I) = 0;
  /*!
     Operator that allows to capture a color image.
     \param I : The captured image.
   */
  virtual vpPylonGrabber &operator>>(vpImage<vpRGBa> &I) = 0;

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
  virtual float setBlackLevel(float blacklevel_value) = 0;
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
  virtual void setCameraIndex(unsigned int index) = 0;
  /*!
    Set the current camera from its unique serial identifier.
    \param serial : Camera serial string.

    \sa getCameraSerial()
   */
  virtual void setCameraSerial(const std::string &serial) = 0;
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
  virtual float setExposure(bool exposure_on, bool exposure_auto, float exposure_value) = 0;
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
  virtual float setGain(bool gain_auto, float gain_value) = 0;
  /*!
    Set camera frame rate.
    \param frame_rate : Camera frame rate (fps) to set.
    \return The camera current framerate.

    \sa getFramerate()
   */
  virtual float setFrameRate(float frame_rate) = 0;
  /*!
    Set camera gamma correction mode and parameter.

    \param gamma_on : If true turn gamma correction on, otherwise turn off.
    \param gamma_value : Parameter used to perform gamma correction of
    pixel intensity.

    \return The measured gamma correction value after applying the new
    setting.

    \sa getGamma()
   */
  virtual float setGamma(bool gamma_on, float gamma_value = 1) = 0;
  /*!
    \brief Saves the current active configuration set into the selected
    user set.

    \param user_set See vpPylonGrabber::UserSetName for valid values.
    \param set_default Whether to set the configuration set to be used as
    the default startup set.
    \return true for finished, false otherwise.

    \sa loadUserSet()
   */
  virtual bool saveUserSet(UserSetName user_set, bool set_default = false) = 0;
  /*!
    \brief Sets the configuration set to be used as the default startup
    set.

    \param user_set See vpPylonGrabber::UserSetName for valid values.
    \return true for finished, false otherwise.

    \sa getUserSetDefault()
   */
  virtual bool setUserSetDefault(UserSetName user_set) = 0;

  /*!
     Start active camera capturing images.

     \sa stopCapture()
   */
  virtual void startCapture() = 0;
  /*!
     Stop active camera capturing images.

     \sa startCapture()
   */
  virtual void stopCapture() = 0;
};

#endif // #ifdef VISP_HAVE_PYLON
#endif // #ifndef __vpPylonGrabber_h_
