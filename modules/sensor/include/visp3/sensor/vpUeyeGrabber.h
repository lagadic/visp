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
 * IDS uEye interface.
 *
*****************************************************************************/

#ifndef _vpUeyeGrabber_h_
#define _vpUeyeGrabber_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_UEYE

BEGIN_VISP_NAMESPACE
/*!
 * \class vpUeyeGrabber
 * \ingroup group_sensor_camera
 *
 * Allows to grab images from an IDS camera using uEye SDK.
 *
 * This class was tested with IDS UI328xCP-C and UI328xCP-C camera models.
 *
 * By default, the first connected camera is the active one.
 * The following code shows how to get information about all the connected cameras
 * (index, id, model, serial number):
 * \snippet tutorial-grabber-ids-ueye.cpp List camera info
 *
 * If multiple cameras are connected, to select a specific one use the following code:
 * \snippet tutorial-grabber-ids-ueye.cpp Active camera info
 *
 * Create an image container, either for gray level or color images:
 * \snippet tutorial-grabber-ids-ueye.cpp Create image
 *
 * Then call open() to connect the active camera:
 * \snippet tutorial-grabber-ids-ueye.cpp Open connection
 *
 * From here you can modify camera default settings,
 * - either by loading camera parameters from a config file created by `ueyedemo` binary:
 * \snippet tutorial-grabber-ids-ueye.cpp Load settings from file
 * - either using setter like setColorMode(), setExposure(), setFrameRate(), setGain(), setSubsampling(),
 * setWhiteBalance()
 *
 * Depending on the settings, you may update image container size, especially if you want to create a window to display
 * the image:
 * \snippet tutorial-grabber-ids-ueye.cpp Update image size
 *
 * Now you can create an infinite loop to grab images:
 * \code
 * while (1) {
 *   g.acquire(I);
 * }
 * \endcode
*/
class VISP_EXPORT vpUeyeGrabber
{
public:
  vpUeyeGrabber();
  virtual ~vpUeyeGrabber();

  void acquire(vpImage<unsigned char> &I, double *timestamp_camera = nullptr, std::string *timestamp_system = nullptr);
  void acquire(vpImage<vpRGBa> &I, double *timestamp_camera = nullptr, std::string *timestamp_system = nullptr);

  std::string getActiveCameraModel() const;
  std::string getActiveCameraSerialNumber() const;

  std::vector<unsigned int> getCameraIDList() const;
  std::vector<std::string> getCameraModelList() const;
  std::vector<std::string> getCameraSerialNumberList() const;
  double getFramerate() const;
  unsigned int getFrameHeight() const;
  unsigned int getFrameWidth() const;

  bool isConnected() const;
  void loadParameters(const std::string &filename);
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  bool setActiveCamera(unsigned int cam_index);
  bool setColorMode(const std::string &color_mode);
  bool setExposure(bool auto_exposure, double exposure_ms = -1);
  bool setFrameRate(bool auto_frame_rate, double manual_frame_rate_hz = -1);
  bool setGain(bool auto_gain, int master_gain = -1, bool gain_boost = false);
  void setSubsampling(int factor);
  void setVerbose(bool verbose);
  void setWhiteBalance(bool auto_wb);

private:
  vpUeyeGrabber(const vpUeyeGrabber &);            // noncopyable
  vpUeyeGrabber &operator=(const vpUeyeGrabber &); //

  class vpUeyeGrabberImpl;
  vpUeyeGrabberImpl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
