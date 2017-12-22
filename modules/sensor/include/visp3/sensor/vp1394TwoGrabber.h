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
 * Description:
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vp1394TwoGrabber.h
  \brief class for firewire cameras video capture using libdc1394-2.x api

  \warning This class is a wrapper over libdc1394-2 optional 3rd party.

  Thus to be enabled, this class needs the optional libdc1394-2 3rd party.
  Installation instruction are provided here https://visp.inria.fr/3rd_dc1394.

  vp1394TwoGrabber was tested with the following cameras:
  - Marlin F033C, F131B
  - PointGrey Dragonfly 2, Flea 2
  - APPLE iSight extern webcam

*/

#ifndef vp1394TwoGrabber_h
#define vp1394TwoGrabber_h

#include <visp3/core/vpConfig.h>

#include <list>

#if defined(VISP_HAVE_DC1394)

/*!
  \class vp1394TwoGrabber

  \ingroup group_sensor_camera

  \brief Class for firewire ieee1394 video devices using libdc1394-2.x api.


  Needs libraw1394-1.2.0 and libdc1394-2.0.0 or more recent versions
  available on http://sourceforge.net.

  This class was tested with Marlin F033C and F131B cameras and with
  Point Grey Dragonfly 2, Flea 2 and Flea 3 cameras.

  \ingroup libdevice

  This grabber allows single or multi camera acquisition.

  - Here you will find an example of single capture from the first camera
found on the bus. This example is available in tutorial-grabber-1394.cpp:
    \include tutorial-grabber-1394.cpp
    A line by line explanation of this example is provided in \ref
tutorial-grabber. An other example that shows how to use format 7 and the
auto-shutter is provided in vp1394TwoGrabber() constructor:

  - If more than one camera is connected, it is also possible to select a
specific camera by its GUID:
\code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394)
  vpImage<unsigned char> I; // Create a gray level image container
  bool reset = false; // Disable bus reset during construction
  vp1394TwoGrabber g(reset); // Create a grabber based on libdc1394-2.x third party lib

  unsigned int ncameras; // Number of cameras on the bus
  ncameras = g.getNumCameras();
  std::cout << ncameras << " cameras found:" << std::endl;

  for(unsigned int i=0; i< ncameras; i++)
  {
    g.setCamera(i);
    uint64_t guid = g.getGuid();
    printf("camera %d with guid 0x%lx\n", i, (long unsigned int)guid);
  }

  // produce:
  // 2 cameras found:
  // camera 0 with guid 0xb09d01009b329c
  // camera 1 with guid 0xb09d01007e0ee7
  g.setCamera( (uint64_t)0xb09d01009b329cULL );

  printf("Use camera with GUID: 0x%lx\n", (long unsigned int)g.getGuid());
  g.acquire(I); // Acquire an image from the camera with GUID 0xb09d01009b329c

  vpImageIo::write(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

  - Here an example of multi camera capture.  An other example is available in
setCamera():
\code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394)
  bool reset = false; // Disable bus reset during construction
  vp1394TwoGrabber g(reset);    // Creation of a grabber instance based on libdc1394-2.x third party lib.
  unsigned int ncameras; // Number of cameras on the bus
  ncameras = g.getNumCameras();

  // Create an image container for each camera
  vpImage<unsigned char> *I = new vpImage<unsigned char> [ncameras];
  char filename[FILENAME_MAX];

  // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
  g.setCamera(0);
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV422);

  // If the second camera support 30 fps acquisition
  g.setCamera(1);
  g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);

  // Acquire an image from each camera
  for (unsigned int camera=0; camera < ncameras; camera ++) {
    g.setCamera(camera);
    g.acquire(I[camera]);
    sprintf(filename, "image-cam%d.pgm", camera);
    vpImageIo::write(I[camera], filename);
  }
  delete [] I;
#endif
}
  \endcode

  \author  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

*/

/*
 * Interface with libdc1394 2.x
 */
#include <string.h>

#include <dc1394/control.h>
#include <dc1394/utils.h>
#include <dc1394/vendor/avt.h>

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

class VISP_EXPORT vp1394TwoGrabber : public vpFrameGrabber
{

public:
  static const char *strVideoMode[DC1394_VIDEO_MODE_NUM];
  static const char *strFramerate[DC1394_FRAMERATE_NUM];
  static const char *strColorCoding[DC1394_COLOR_CODING_NUM];

  /*!
    Enumeration of iso speed. See libdc1394 2.x header file
    dc1394/control.h
  */
  typedef enum {
    vpISO_SPEED_100 = DC1394_ISO_SPEED_100,
    vpISO_SPEED_200 = DC1394_ISO_SPEED_200,
    vpISO_SPEED_400 = DC1394_ISO_SPEED_400,
    vpISO_SPEED_800 = DC1394_ISO_SPEED_800,
    vpISO_SPEED_1600 = DC1394_ISO_SPEED_1600,
    vpISO_SPEED_3200 = DC1394_ISO_SPEED_3200
  } vp1394TwoIsoSpeedType;

  /*!
    Enumeration of video modes. See libdc1394 2.x header file
    dc1394/control.h
  */
  typedef enum {
    vpVIDEO_MODE_160x120_YUV444 = DC1394_VIDEO_MODE_160x120_YUV444,
    vpVIDEO_MODE_320x240_YUV422 = DC1394_VIDEO_MODE_320x240_YUV422,
    vpVIDEO_MODE_640x480_YUV411 = DC1394_VIDEO_MODE_640x480_YUV411,
    vpVIDEO_MODE_640x480_YUV422 = DC1394_VIDEO_MODE_640x480_YUV422,
    vpVIDEO_MODE_640x480_RGB8 = DC1394_VIDEO_MODE_640x480_RGB8,
    vpVIDEO_MODE_640x480_MONO8 = DC1394_VIDEO_MODE_640x480_MONO8,
    vpVIDEO_MODE_640x480_MONO16 = DC1394_VIDEO_MODE_640x480_MONO16,
    vpVIDEO_MODE_800x600_YUV422 = DC1394_VIDEO_MODE_800x600_YUV422,
    vpVIDEO_MODE_800x600_RGB8 = DC1394_VIDEO_MODE_800x600_RGB8,
    vpVIDEO_MODE_800x600_MONO8 = DC1394_VIDEO_MODE_800x600_MONO8,
    vpVIDEO_MODE_1024x768_YUV422 = DC1394_VIDEO_MODE_1024x768_YUV422,
    vpVIDEO_MODE_1024x768_RGB8 = DC1394_VIDEO_MODE_1024x768_RGB8,
    vpVIDEO_MODE_1024x768_MONO8 = DC1394_VIDEO_MODE_1024x768_MONO8,
    vpVIDEO_MODE_800x600_MONO16 = DC1394_VIDEO_MODE_800x600_MONO16,
    vpVIDEO_MODE_1024x768_MONO16 = DC1394_VIDEO_MODE_1024x768_MONO16,
    vpVIDEO_MODE_1280x960_YUV422 = DC1394_VIDEO_MODE_1280x960_YUV422,
    vpVIDEO_MODE_1280x960_RGB8 = DC1394_VIDEO_MODE_1280x960_RGB8,
    vpVIDEO_MODE_1280x960_MONO8 = DC1394_VIDEO_MODE_1280x960_MONO8,
    vpVIDEO_MODE_1600x1200_YUV422 = DC1394_VIDEO_MODE_1600x1200_YUV422,
    vpVIDEO_MODE_1600x1200_RGB8 = DC1394_VIDEO_MODE_1600x1200_RGB8,
    vpVIDEO_MODE_1600x1200_MONO8 = DC1394_VIDEO_MODE_1600x1200_MONO8,
    vpVIDEO_MODE_1280x960_MONO16 = DC1394_VIDEO_MODE_1280x960_MONO16,
    vpVIDEO_MODE_1600x1200_MONO16 = DC1394_VIDEO_MODE_1600x1200_MONO16,
    vpVIDEO_MODE_EXIF = DC1394_VIDEO_MODE_EXIF,
    vpVIDEO_MODE_FORMAT7_0 = DC1394_VIDEO_MODE_FORMAT7_0,
    vpVIDEO_MODE_FORMAT7_1 = DC1394_VIDEO_MODE_FORMAT7_1,
    vpVIDEO_MODE_FORMAT7_2 = DC1394_VIDEO_MODE_FORMAT7_2,
    vpVIDEO_MODE_FORMAT7_3 = DC1394_VIDEO_MODE_FORMAT7_3,
    vpVIDEO_MODE_FORMAT7_4 = DC1394_VIDEO_MODE_FORMAT7_4,
    vpVIDEO_MODE_FORMAT7_5 = DC1394_VIDEO_MODE_FORMAT7_5,
    vpVIDEO_MODE_FORMAT7_6 = DC1394_VIDEO_MODE_FORMAT7_6,
    vpVIDEO_MODE_FORMAT7_7 = DC1394_VIDEO_MODE_FORMAT7_7
  } vp1394TwoVideoModeType;

  /*!
    Enumeration of framerates. See libdc1394 2.x header file
    dc1394/control.h
  */
  typedef enum {
    vpFRAMERATE_1_875 = DC1394_FRAMERATE_1_875,
    vpFRAMERATE_3_75 = DC1394_FRAMERATE_3_75,
    vpFRAMERATE_7_5 = DC1394_FRAMERATE_7_5,
    vpFRAMERATE_15 = DC1394_FRAMERATE_15,
    vpFRAMERATE_30 = DC1394_FRAMERATE_30,
    vpFRAMERATE_60 = DC1394_FRAMERATE_60,
    vpFRAMERATE_120 = DC1394_FRAMERATE_120,
    vpFRAMERATE_240 = DC1394_FRAMERATE_240
  } vp1394TwoFramerateType;

  /*!
    Enumeration of color codings. See libdc1394 2.x header file
    dc1394/control.h
  */
  typedef enum {
    vpCOLOR_CODING_MONO8 = DC1394_COLOR_CODING_MONO8,
    vpCOLOR_CODING_YUV411 = DC1394_COLOR_CODING_YUV411,
    vpCOLOR_CODING_YUV422 = DC1394_COLOR_CODING_YUV422,
    vpCOLOR_CODING_YUV444 = DC1394_COLOR_CODING_YUV444,
    vpCOLOR_CODING_RGB8 = DC1394_COLOR_CODING_RGB8,
    vpCOLOR_CODING_MONO16 = DC1394_COLOR_CODING_MONO16,
    vpCOLOR_CODING_RGB16 = DC1394_COLOR_CODING_RGB16,
    vpCOLOR_CODING_MONO16S = DC1394_COLOR_CODING_MONO16S,
    vpCOLOR_CODING_RGB16S = DC1394_COLOR_CODING_RGB16S,
    vpCOLOR_CODING_RAW8 = DC1394_COLOR_CODING_RAW8,
    vpCOLOR_CODING_RAW16 = DC1394_COLOR_CODING_RAW16
  } vp1394TwoColorCodingType;

  /*!
    Enumeration of the parameters that can be modified. See libdc1394 2.x
    header file dc1394/control.h
  */
  typedef enum {
    vpFEATURE_BRIGHTNESS = DC1394_FEATURE_BRIGHTNESS,
    vpFEATURE_EXPOSURE = DC1394_FEATURE_EXPOSURE,
    vpFEATURE_SHARPNESS = DC1394_FEATURE_SHARPNESS,
    //    vpFEATURE_WHITE_BALANCE   = DC1394_FEATURE_WHITE_BALANCE,
    vpFEATURE_HUE = DC1394_FEATURE_HUE,
    vpFEATURE_SATURATION = DC1394_FEATURE_SATURATION,
    vpFEATURE_GAMMA = DC1394_FEATURE_GAMMA,
    vpFEATURE_SHUTTER = DC1394_FEATURE_SHUTTER,
    vpFEATURE_GAIN = DC1394_FEATURE_GAIN,
    vpFEATURE_IRIS = DC1394_FEATURE_IRIS,
    //    vpFEATURE_FOCUS           = DC1394_FEATURE_FOCUS,
    //    vpFEATURE_TEMPERATURE     = DC1394_FEATURE_TEMPERATURE,
    //    vpFEATURE_TRIGGER         = DC1394_FEATURE_TRIGGER,
    //    vpFEATURE_TRIGGER_DELAY   = DC1394_FEATURE_TRIGGER_DELAY,
    //    vpFEATURE_WHITE_SHADING   = DC1394_FEATURE_WHITE_SHADING,
    //    vpFEATURE_FRAME_RATE      = DC1394_FEATURE_FRAME_RATE,
    //    vpFEATURE_ZOOM            = DC1394_FEATURE_ZOOM,
    //    vpFEATURE_PAN             = DC1394_FEATURE_PAN,
    //    vpFEATURE_TILT            = DC1394_FEATURE_TILT,
    //    vpFEATURE_OPTICAL_FILTER  = DC1394_FEATURE_OPTICAL_FILTER,
    //    vpFEATURE_CAPTURE_SIZE    = DC1394_FEATURE_CAPTURE_SIZE,
    //    vpFEATURE_CAPTURE_QUALITY = DC1394_FEATURE_CAPTURE_QUALITY
  } vp1394TwoParametersType;

private:
  /*!
    Control structure of the values that can be modified during the execution.
  */
  typedef struct {
    uint32_t brightness;
    uint32_t exposure;
    uint32_t sharpness;
    //    uint32_t whiteBalance;
    uint32_t hue;
    uint32_t saturation;
    uint32_t gamma;
    uint32_t shutter;
    uint32_t gain;
    uint32_t iris;
    //    uint32_t focus;
    //    uint32_t temperature ;
    //    uint32_t trigger ;
    //    uint32_t triggerDelay ;
    //    uint32_t whiteShadding ;
    //    uint32_t frameRate ;
    //    uint32_t zoom;
    //    uint32_t pan;
    //    uint32_t tilt ;
    //    uint32_t opticalFilter ;
    //    uint32_t captureSize;
    //    uint32_t captureQuality ;
  } vpDc1394TwoCameraParametersData;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vp1394TwoGrabber(const vp1394TwoGrabber &)
  //    : camera(NULL), cameras(NULL), num_cameras(0), camera_id(0),
  //    verbose(false), camIsOpen(NULL),
  //      num_buffers(4), // ring buffer size
  //      isDataModified(NULL), initialShutterMode(NULL), dataCam(NULL)
  //    #ifdef VISP_HAVE_DC1394_CAMERA_ENUMERATE // new API >
  //    libdc1394-2.0.0-rc7 , d(NULL),
  //      list(NULL)
  //    #endif
  //  {
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!");
  //  }
  //  vp1394TwoGrabber &operator=(const vp1394TwoGrabber &){
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  explicit vp1394TwoGrabber(bool reset = true);
  virtual ~vp1394TwoGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, uint64_t &timestamp, uint32_t &id);
  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<vpRGBa> &I, uint64_t &timestamp, uint32_t &id);

  void close();

  static std::string colorCoding2string(vp1394TwoColorCodingType colorcoding);

  dc1394video_frame_t *dequeue();
  dc1394video_frame_t *dequeue(vpImage<unsigned char> &I);
  dc1394video_frame_t *dequeue(vpImage<unsigned char> &I, uint64_t &timestamp, uint32_t &id);
  dc1394video_frame_t *dequeue(vpImage<vpRGBa> &I);
  dc1394video_frame_t *dequeue(vpImage<vpRGBa> &I, uint64_t &timestamp, uint32_t &id);
  void enqueue(dc1394video_frame_t *frame);

  static std::string framerate2string(vp1394TwoFramerateType fps);

  void getAutoGain(unsigned int &minvalue, unsigned int &maxvalue);
  void getAutoShutter(unsigned int &minvalue, unsigned int &maxvalue);

  uint64_t getCamera();
  void getCamera(uint64_t &camera);
  void getColorCoding(vp1394TwoColorCodingType &coding);
  uint32_t getColorCodingSupported(vp1394TwoVideoModeType videomode, std::list<vp1394TwoColorCodingType> &codings);
  void getFramerate(vp1394TwoFramerateType &fps);
  uint32_t getFramerateSupported(vp1394TwoVideoModeType videomode, std::list<vp1394TwoFramerateType> &fps);
  uint64_t getGuid();
  void getGuid(uint64_t &guid);
  void getHeight(unsigned int &height);
  unsigned int getHeight();
  void getNumCameras(unsigned int &ncameras) const;
  unsigned int getNumCameras() const;
  unsigned int getParameterValue(vp1394TwoParametersType param);
  unsigned int getRingBufferSize() const;
  void getVideoMode(vp1394TwoVideoModeType &videomode);
  uint32_t getVideoModeSupported(std::list<vp1394TwoVideoModeType> &videomodes);
  void getWidth(unsigned int &width);
  unsigned int getWidth();

  bool isColor();
  bool isColorCodingSupported(vp1394TwoVideoModeType videomode, vp1394TwoColorCodingType coding);
  bool isFramerateSupported(vp1394TwoVideoModeType videomode, vp1394TwoFramerateType fps);
  bool isVideoModeSupported(vp1394TwoVideoModeType videomode);
  bool isVideoModeFormat7(vp1394TwoVideoModeType videomode);

  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  vp1394TwoGrabber &operator>>(vpImage<unsigned char> &I);
  vp1394TwoGrabber &operator>>(vpImage<vpRGBa> &I);

  void printCameraInfo();

  void resetBus();

  void setAutoGain(bool enable = true);
  void setAutoGain(unsigned int minvalue, unsigned int maxvalue);
  void setAutoShutter(bool enable = true);
  void setAutoShutter(unsigned int minvalue, unsigned int maxvalue);
  void setCamera(uint64_t camera);
  void setColorCoding(vp1394TwoColorCodingType coding);
  void setFormat7ROI(unsigned int left = 0, unsigned int top = 0, unsigned int width = 0, unsigned int height = 0);
  void setFramerate(vp1394TwoFramerateType fps);
  void setIsoTransmissionSpeed(vp1394TwoIsoSpeedType isospeed);
  void setPanControl(unsigned int panControlValue);
  void setParameterValue(vp1394TwoParametersType param, unsigned int val);
  void setRingBufferSize(unsigned int size);
  void setVideoMode(vp1394TwoVideoModeType videomode);

  static vp1394TwoVideoModeType string2videoMode(std::string videomode);
  static vp1394TwoFramerateType string2framerate(std::string fps);
  static vp1394TwoColorCodingType string2colorCoding(std::string colorcoding);

  static std::string videoMode2string(vp1394TwoVideoModeType videomode);

private:
  void open();
  void initialize(bool reset);
  void setCapture(dc1394switch_t _switch);
  void setTransmission(dc1394switch_t _switch);
  inline void updateDataCamToStruct();
  inline void updateDataStructToCam();

private:
  dc1394camera_t *camera, **cameras;
  unsigned int num_cameras;
  unsigned int camera_id;

  bool verbose;
  bool *camIsOpen;
  unsigned int num_buffers;

  /* parameters for the cameras */
  bool *isDataModified;
  dc1394feature_mode_t *initialShutterMode;
  vpDc1394TwoCameraParametersData *dataCam;

#ifdef VISP_HAVE_DC1394_CAMERA_ENUMERATE
  dc1394_t *d;
  dc1394camera_list_t *list;
#endif
};

#endif
#endif
