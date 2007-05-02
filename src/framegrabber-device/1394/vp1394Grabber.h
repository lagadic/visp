/****************************************************************************
 *
 * $Id: vp1394Grabber.h,v 1.14 2007-05-02 13:29:40 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
  \brief class for firewire cameras video capture

  \warning This class needs at least libdc1394-1.0.0 and
  libraw1394-1.1.0. These libraries are available from
  http://sourceforge.net/projects/libdc1394 and
  http://sourceforge.net/projects/libraw1394 .

  vp1394Grabber was tested with a Marlin F033C camera.

  \ingroup libdevice
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
  \brief class for firewire ieee1394 video devices.

  Needs libraw1394-1.2.0 and libdc1394-1.1.0 or more recent versions
  available on http://sourceforge.net.

  This class was tested with Marlin F033C and F131B cameras.

  \ingroup libdevice

  \author  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

*/
class VISP_EXPORT vp1394Grabber : public vpFrameGrabber
{
public:
  /*! \enum ImageFormatEnum
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
  } ImageFormatEnum;

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
			       ImageFormatEnum &imageformat);
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
  ImageFormatEnum *image_format; // MAX_CAMERAS
} ;

#endif
#endif

