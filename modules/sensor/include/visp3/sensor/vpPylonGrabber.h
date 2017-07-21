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

#ifndef __vpPylonGrabber_h_
#define __vpPylonGrabber_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFrameGrabber.h>

#ifdef VISP_HAVE_PYLON

#include <pylon/PylonIncludes.h>

/*!
  \file vpPylonGrabber.h
  \brief Wrapper over Basler Pylon SDK to capture images from Basler
  cameras.
*/

/*!
  \class vpPylonGrabber
  \ingroup group_sensor_camera

  Allows to grab images from a Basler camera using Pylon SDK.

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
  vpPylonGrabber();
  virtual ~vpPylonGrabber();

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
  static std::string getCameraSerial(unsigned int index);
  float getExposure();
  float getFrameRate();
  float getGain();
  static unsigned int getNumCameras();
  float getSharpness();
  float getGamma();

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
  void setCameraSerial(std::string &serial);
  float setExposure(bool exposure_on, bool exposure_auto,
                    float exposure_value = 0);
  float setGain(bool gain_auto, float gain_value = 0);
  float setFrameRate(float frame_rate);
  float setSharpness(bool sharpness_on, float sharpness_value = 0);
  float setGamma(bool gamma_on, float gamma_value);

  void startCapture();
  void stopCapture();

protected:
  void open();
  /*!
    \brief  Get camera property.
    \param[in]  name Property name.
    \param[out]  value The returned value.
    \return true for success, false for failure.

    The typename N is used internally by Pylon SDK to get GenApi::INode,
    and it depends on T. The most used possible values for N are
    IBoolean, IFloat, IInteger and IString. See GenApi namespace for
    other I initiated structures.
   */
  template <typename T, typename N>
  bool getProperty(const GenICam::gcstring &name, T &value)
  {
    this->connect();

    // Get the camera control object.
    GenApi::INodeMap &control = m_camera.GetNodeMap();

    GenApi::CPointer<N> prop = control.GetNode(name);

    if (GenApi::IsReadable(prop)) {
      value = prop->GetValue();
    }

    return GenApi::IsReadable(prop);
  }
  /*!
    \brief  Set camera property.

    \param  name Property name.
    \param  value value to set.
    \return true for success, false for failure.

    The typename N is used internally by Pylon SDK to get GenApi::INode,
    and it depends on T. The most used possible values for N are
    IBoolean, IFloat, IInteger and IString. See GenApi namespace for
    other I initiated structures.
   */
  template <typename T, typename N>
  bool setProperty(const GenICam::gcstring &name, const T &value)
  {
    this->connect();

    // Get the camera control object.
    GenApi::INodeMap &control = m_camera.GetNodeMap();

    GenApi::CPointer<N> prop = control.GetNode(name);

    if (GenApi::IsWritable(prop)) {
      prop->SetValue(value);
    }

    return GenApi::IsWritable(prop);
  }

protected:
  Pylon::CInstantCamera m_camera; //!< Pointer to each camera
  unsigned int m_index;           //!< Active camera index
  unsigned int m_numCameras;      //!< Number of connected cameras
  bool m_connected;               //!< true if camera connected

private:
  Pylon::PylonAutoInitTerm
      m_autoInitTerm; //!< Auto initialize and terminate object.
};

#endif // #ifdef VISP_HAVE_PYLON
#endif // #ifndef __vpPylonGrabber_h_
