/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vp1394Grabber.h
  \brief class for firewire cameras video capture.

  \warning This class needs at least libdc1394-1.0.0 and
  libraw1394-1.1.0. These libraries are available from
  http://sourceforge.net/projects/libdc1394 and
  http://sourceforge.net/projects/libraw1394 .

  vp1394Grabber was tested with a Marlin F033C camera. This grabber is
  not working with a PointGrey DragonFly 2 camera. Since libdc1394-1.x
  is deprecated, you should better use vp1394TwoGrabber based on
  libdc1394-2.x.

*/

#ifndef vp1394Grabber_h
#define vp1394Grabber_h

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_DC1394_1)

#include <string>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>
#include <visp/vpList.h>



/*!
  \class vp1394Grabber

  \ingroup Framegrabber CameraDriver

  \brief Class for firewire ieee1394 video devices using libdc1394-1.x api

  Needs libraw1394-1.2.0 and libdc1394-1.1.0 or more recent versions
  available on http://sourceforge.net.

  This class was tested with Marlin F033C and F131B cameras. This grabber is
  not working with a PointGrey DragonFly 2 camera. Since libdc1394-1.x
  is deprecated, you should better use vp1394TwoGrabber based on
  libdc1394-2.x.

  The code below shows how to use this class.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vp1394Grabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394_1)
  vpImage<unsigned char> I; // Create a gray level image container
  vp1394Grabber g;          // Create a grabber based on libdc1394-1.x third party lib
  g.setFormat(FORMAT_VGA_NONCOMPRESSED); // Format_0
  g.setMode(MODE_640x480_MONO);          // Mode 5
  g.setFramerate(FRAMERATE_15);          // 15 fps

  g.open(I);                           // Open the framegrabber
  g.acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

  \author  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

*/
class VISP_EXPORT vp1394Grabber : public vpFrameGrabber
{
public:
  /*! \enum vp1394ImageFormatType
  Supported image coding format.
  */
  typedef enum {
    YUV444, /*!< YUV 4:4:4 coding format (24 bits/pixel) */
    YUV422, /*!< YUV 4:2:2 coding format (16 bits/pixel) */
    YUV411, /*!< YUV 4:1:1 coding format (12 bits/pixel) */
    RGB,    /*!< RGB coding format (24 bits/pixel) */
    RGBa,   /*!< RGBa coding format (32 bits/pixel) */
    MONO,   /*!< MONO8 coding format (8 bits/pixel) */
    MONO16  /*!< MONO16 coding format (16 bits/pixel) */
  } vp1394ImageFormatType;

  static const int DROP_FRAMES; // Number of frames to drop
  static const int NUM_BUFFERS; // Number of buffers
  static const int MAX_PORTS;   // Port maximal number
  static const int MAX_CAMERAS; // Maximal number of cameras on the bus

  static const char * strFormats[NUM_FORMATS];
  static const char * strModesInFormat0[NUM_FORMAT0_MODES];
  static const char * strModesInFormat1[NUM_FORMAT1_MODES];
  static const char * strModesInFormat2[NUM_FORMAT2_MODES];
  static const char * strModesInFormat6[NUM_FORMAT6_MODES];
  static const char * strModesInFormat7[NUM_MODE_FORMAT7];
  static const char * strColorsInFormat7[NUM_COLOR_FORMAT7];
  static const char * strFramerates[NUM_FRAMERATES];


public:
  vp1394Grabber();
  vp1394Grabber(vpImage<unsigned char> &I);
  virtual ~vp1394Grabber();

  void setCamera(unsigned int camera);
  void getCamera(unsigned int &camera);

  void setFormat(int format);
  void getFormat(int & format);
  int  getFormatSupported(vpList<int> & formats);

  void setMode(int mode);
  void getMode(int & mode);
  int  getModeSupported(int format, vpList<int> & modes);

  void setFramerate(int framerate);
  void getFramerate(int & framerate);
  int  getFramerateSupported(int format, int mode, vpList<int> & framerates);

  int  convertFormat   (std::string format);
  int  convertMode     (std::string mode);
  int  convertFramerate(std::string framerate);

  std::string convertFormat   (int format);
  std::string convertMode     (int mode);
  std::string convertFramerate(int framerate);

  void setShutter(unsigned int shutter);
  void getShutter(unsigned int &min_shutter,
		  unsigned int &shutter,
		  unsigned int &max_shutter);

  void setGain(unsigned int gain);
  void getGain(unsigned int &min_gain,
	       unsigned int &gain,
	       unsigned int &max_gain);

  void open(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);
  void acquire(vpImage<vpRGBa> &I);
  void close();

  void getWidth (unsigned int &width);
  void getHeight(unsigned int &height);
  void getNumCameras(unsigned int &cameras);


private:

  void open();
  void setup();

  void getImageCharacteristics(int _format, int _mode,
			       int &width, int &height,
			       vp1394ImageFormatType &imageformat);
  int* dmaCapture(bool waiting = true);
  void dmaDoneWithBuffer();

  void startIsoTransmission();
  void stopIsoTransmission();

private:
  bool iso_transmission_started;
  bool handle_created;
  bool camera_found;
  bool camera_nodes_allocated;
  bool dma_started;
  unsigned int  num_cameras;
  /* declarations for libdc1394 */
  raw1394handle_t      *handles; // MAX_CAMERAS
  dc1394_cameracapture *cameras; // MAX_CAMERAS
  nodeid_t             *camera_nodes;
  dc1394_miscinfo      miscinfo;
  int                  *cam_count; // MAX_CAMERAS

  /* declarations for video1394 */
  char device_name[FILENAME_MAX];

  unsigned int camera;
  // Camera settings
  int  *pformat;    // MAX_CAMERAS
  int  *pmode;      // MAX_CAMERAS
  int  *pframerate; // MAX_CAMERAS
  bool verbose;

  // Image settings
  int *_width;  // MAX_CAMERAS
  int *_height; // MAX_CAMERAS
  vp1394ImageFormatType *image_format; // MAX_CAMERAS
} ;

#endif
#endif

