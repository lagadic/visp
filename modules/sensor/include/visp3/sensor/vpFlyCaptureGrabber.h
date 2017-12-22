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
 * Description: Class which enables to project an image in the 3D space
 * and get the view of a virtual camera.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpFlyCaptureGrabber_h_
#define __vpFlyCaptureGrabber_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFrameGrabber.h>

#ifdef VISP_HAVE_FLYCAPTURE

#include <FlyCapture2.h>

/*!
  \file vpFlyCaptureGrabber.h
  \brief Wrapper over PointGrey FlyCapture SDK to capture images from
  PointGrey cameras.
*/
/*!
  \class vpFlyCaptureGrabber
  \ingroup group_sensor_camera

  Allows to grab images from a PointGrey camera using FlyCapture SDK.

  To use this class install first FlyCapture SDK
https://www.ptgrey.com/flycapture-sdk. Installation instructions are provide
here https://visp.inria.fr/3rd_flycapture. \note To install FlyCapture SDK on
linux follow https://www.ptgrey.com/tan/10548. \note For specific details
about using FlyCapture and Linux with a USB 3.0 camera, see
http://www.ptgrey.com/KB/10685.

  Once installed configure ViSP using cmake to detect FlyCapture SDK and build
ViSP to include FlyCapture SDK support.

  This class was tested under Ubuntu and Windows with the following cameras:
  - Flea3 USB 3.0 cameras (FL3-U3-32S2M-CS, FL3-U3-13E4C-C)
  - Flea2 firewire camera (FL2-03S2C)
  - Dragonfly2 firewire camera (DR2-COL)

  The following example shows how to use this class to capture images
  from the first camera that is found.
  \code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  try {
  int nframes = 100;
  vpImage<unsigned char> I;
  char filename[255];
  vpFlyCaptureGrabber g;
  std::cout << "Number of cameras detected: " << g.getNumCameras() << std::endl;

  g.setCameraIndex(0); // Default camera is the first on the bus
  g.getCameraInfo(std::cout);
  g.open(I);

  for(int i=0; i< nframes; i++) {
    g.acquire(I);
    sprintf(filename, "image%04d.pgm", i);
    vpImageIo::write(I, filename);
  }
#endif
}
  \endcode

  If more than one camera is detected, you can use setCamera(const unsigned
int &) to select the camera of interest.

  It is also possible to capture images from multiple cameras. The following
example shows how to capture simultaneously images from multiple cameras.

  \code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>

int main()
{
#if defined(VISP_HAVE_FLYCAPTURE)
  int nframes = 100;
  char filename[255];
  unsigned int numCameras = vpFlyCaptureGrabber::getNumCameras();

  std::cout << "Number of cameras detected: " << numCameras << std::endl;

  vpFlyCaptureGrabber *g = new vpFlyCaptureGrabber [numCameras];
  std::vector< vpImage<unsigned char> > I(numCameras);

  for(unsigned int cam=0; cam < numCameras; cam++) {
    g[cam].setCameraIndex(cam); // Default camera is the first on the bus
    g[cam].getCameraInfo(std::cout);
    g[cam].open(I[cam]);
  }

  for(int i=0; i< nframes; i++) {
    for(unsigned int cam=0; cam < numCameras; cam++) {
      g[cam].acquire(I[cam]);
      sprintf(filename, "image-camera%d-%04d.pgm", cam, i);
      vpImageIo::write(I[cam], filename);
    }
  }
  delete [] g;
#endif
}
  \endcode
 */
class VISP_EXPORT vpFlyCaptureGrabber : public vpFrameGrabber
{
public:
  vpFlyCaptureGrabber();
  virtual ~vpFlyCaptureGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, FlyCapture2::TimeStamp &timestamp);
  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<vpRGBa> &I, FlyCapture2::TimeStamp &timestamp);

  void close();
  void connect();
  void disconnect();

  float getBrightness();
  std::ostream &getCameraInfo(std::ostream &os); // Cannot be const since
                                                 // FlyCapture2::Camera::GetCameraInfo()
                                                 // isn't
  FlyCapture2::Camera *getCameraHandler();
  /*! Return the index of the active camera. */
  unsigned int getCameraIndex() const { return m_index; };
  bool getCameraPower();
  static unsigned int getCameraSerial(unsigned int index);
  float getExposure();
  float getFrameRate();
  float getGain();
  static unsigned int getNumCameras();
  unsigned int getSharpness();
  float getShutter();

  bool isCameraPowerAvailable();
  //! Return true if the camera is connected.
  bool isConnected() const { return m_connected; }
  //! Return true if the camera capture is started.
  bool isCaptureStarted() const { return m_capture; }
  bool isFormat7Supported(FlyCapture2::Mode format7_mode);
  bool isVideoModeAndFrameRateSupported(FlyCapture2::VideoMode video_mode, FlyCapture2::FrameRate frame_rate);
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  vpFlyCaptureGrabber &operator>>(vpImage<unsigned char> &I);
  vpFlyCaptureGrabber &operator>>(vpImage<vpRGBa> &I);

  float setBrightness(bool brightness_auto, float brightness_value = 0);
  void setCameraIndex(unsigned int index);
  void setCameraPower(bool on);
  void setCameraSerial(unsigned int serial);
  float setExposure(bool exposure_on, bool exposure_auto, float exposure_value = 0);
  float setGain(bool gain_auto, float gain_value = 0);
  void setFormat7VideoMode(FlyCapture2::Mode format7_mode, FlyCapture2::PixelFormat pixel_format, unsigned int width,
                           unsigned int height);
  float setFrameRate(float frame_rate);
  unsigned int setSharpness(bool sharpness_on, bool sharpness_auto, unsigned int sharpness_value = 0);
  float setShutter(bool auto_shutter, float shutter_ms = 10);
  void setVideoModeAndFrameRate(FlyCapture2::VideoMode video_mode, FlyCapture2::FrameRate frame_rate);

  void startCapture();
  void stopCapture();

protected:
  typedef enum {
    ABS_VALUE, //!< Consider FlyCapture2::Property::absValue
    VALUE_A,   //!< Consider FlyCapture2::Property::valueA
  } PropertyValue;
  std::pair<unsigned int, unsigned int> centerRoi(unsigned int size, unsigned int max_size, unsigned int step);
  FlyCapture2::Property getProperty(FlyCapture2::PropertyType prop_type);
  FlyCapture2::PropertyInfo getPropertyInfo(FlyCapture2::PropertyType prop_type);
  void open();
  void setProperty(const FlyCapture2::PropertyType &prop_type, bool on, bool auto_on, float value,
                   PropertyValue prop_value = ABS_VALUE);

protected:
  FlyCapture2::Camera m_camera;  //!< Pointer to each camera
  FlyCapture2::PGRGuid m_guid;   //!< Active camera guid
  unsigned int m_index;          //!< Active camera index
  unsigned int m_numCameras;     //!< Number of connected cameras
  FlyCapture2::Image m_rawImage; //!< Image buffer
  bool m_connected;              //!< true if camera connected
  bool m_capture;                //!< true is capture started
};

#endif
#endif
