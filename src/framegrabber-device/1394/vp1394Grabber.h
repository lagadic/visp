/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 2005  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Fabien Spindler
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: fspindle@irisa.fr
#    www  : http://www.irisa.fr/lagadic
#
#----------------------------------------------------------------------------
*/

/*!
  \file vp1394Grabber.h
  \brief class for firewire cameras video capture
  \ingroup libdevice
*/

#ifndef vp1394Grabber_h
#define vp1394Grabber_h

#include <visp/vpConfig.h>

#if ( defined (HAVE_LIBDC1394_CONTROL) & defined(HAVE_LIBRAW1394) )

#include <string>

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>



/*!
  \class vp1394Grabber
  \brief class for firewire ieee1394 video devices. 

  Needs libraw1394-1.2.0 and libdc1394-1.1.0 or more recent versions
  available on http://sourceforge.net.

  This class was tested with Marlin F033C and F131B cameras.

  \ingroup libdevice

  \author  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

*/
class vp1394Grabber : public vpFrameGrabber
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


public:
  vp1394Grabber();
  vp1394Grabber(vpImage<unsigned char> &I);
  ~vp1394Grabber();

  void setFormat(int format, int camera=0);
  void setMode(int mode, int camera=0);
  void setFramerate(int framerate, int camera=0);

  void open(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I) { ; };
  void acquire(vpImage<vpRGBa> &I) { ; };
  void close();

  void getWidth (int &width, int camera=0);
  void getHeight(int &height, int camera=0);

private:

  void open();
  void setup();

  void getImageCharacteristics(int _format, int _mode,
			       int &width, int &height,
			       ImageFormatEnum &imageformat,
			       int camera=0);
  int* dmaCapture(bool waiting = true, int camera=0);
  void dmaDoneWithBuffer(int camera=0);

  void startIsoTransmission();
  void stopIsoTransmission();

private:
  bool iso_transmission_started;
  bool handle_created;
  bool camera_found;
  bool camera_nodes_allocated;
  int  num_cameras;
  /* declarations for libdc1394 */
  raw1394handle_t      *handles; // MAX_CAMERAS
  dc1394_cameracapture *cameras; // MAX_CAMERAS
  nodeid_t             *camera_nodes;
  dc1394_miscinfo      miscinfo;
  int                  *cam_count; // MAX_CAMERAS

  /* declarations for video1394 */
  char device_name[FILENAME_MAX];

  // Camera settings
  int  *format;    // MAX_CAMERAS
  int  *mode;      // MAX_CAMERAS
  int  *framerate; // MAX_CAMERAS
  bool verbose;

  // Image settings
  int *width;  // MAX_CAMERAS
  int *height; // MAX_CAMERAS
  ImageFormatEnum *image_format; // MAX_CAMERAS
} ;

#endif
#endif

