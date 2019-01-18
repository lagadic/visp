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
 * Description: Header of vpPylonGrabberUsb class.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

/*!
  \file vpPylonGrabberUsb.h
  \brief Subclass of vpPylonGrabber, implements Basler USB3 cameras
  supporting.
*/

#ifndef _vpPylonGrabberUsb_h_
#define _vpPylonGrabberUsb_h_

#include <visp3/core/vpConfig.h>

#include <visp3/sensor/vpPylonGrabber.h>

#ifdef VISP_HAVE_PYLON

#include <pylon/usb/BaslerUsbInstantCamera.h>

/*!
  \class vpPylonGrabberUsb

  Allows to grab images from a Basler USB camera using Pylon SDK.

  This class should not be instantiated directly. Use
  vpPylonFactory::createPylonGrabber() instead.

  \headerfile vpPylonGrabberUsb.h ""
 */
class VISP_EXPORT vpPylonGrabberUsb : public vpPylonGrabber
{
public:
  vpPylonGrabberUsb();
  virtual ~vpPylonGrabberUsb();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);

  void close();
  void connect();
  void disconnect();

  float getBlackLevel();
  std::ostream &getCameraInfo(std::ostream &os);
  Pylon::CInstantCamera *getCameraHandler();
  /*! Return the index of the active camera. */
  unsigned int getCameraIndex() const { return m_index; };
  std::string getCameraSerial(unsigned int index);
  float getExposure();
  float getFrameRate();
  float getGain();
  unsigned int getNumCameras();
  float getGamma();
  bool loadUserSet(UserSetName user_set);
  UserSetName getUserSetDefault();

  //! Return true if the camera is connected.
  bool isConnected() const { return m_connected; }
  //! Return true if the camera capture is started.
  bool isCaptureStarted() const { return m_camera.IsGrabbing(); }
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  vpPylonGrabber &operator>>(vpImage<unsigned char> &I);
  vpPylonGrabber &operator>>(vpImage<vpRGBa> &I);

  float setBlackLevel(float blacklevel_value = 0);
  void setCameraIndex(unsigned int index);
  void setCameraSerial(const std::string &serial);
  float setExposure(bool exposure_on, bool exposure_auto, float exposure_value = 0);
  float setGain(bool gain_auto, float gain_value = 0);
  float setFrameRate(float frame_rate);
  float setGamma(bool gamma_on, float gamma_value = 1);
  bool saveUserSet(UserSetName user_set, bool set_default = false);
  bool setUserSetDefault(UserSetName user_set);

  void startCapture();
  void stopCapture();

protected:
  void open();
  bool selectUserSet(UserSetName user_set);

private:
  Pylon::CBaslerUsbInstantCamera m_camera; //!< Pointer to each camera
  unsigned int m_index;                    //!< Active camera index
  unsigned int m_numCameras;               //!< Number of connected USB cameras
  bool m_connected;                        //!< true if camera connected
};

#endif // #ifdef VISP_HAVE_PYLON
#endif // #ifndef _vpPylonGrabberUsb_h_
