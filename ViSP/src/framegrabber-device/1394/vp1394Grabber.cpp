/****************************************************************************
 *
 * $Id: vp1394Grabber.cpp,v 1.12 2006-06-28 08:52:32 fspindle Exp $
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
  \file vp1394Grabber.cpp
  \brief member functions for firewire cameras
  \ingroup libdevice
*/
#include <iostream>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_DC1394) && (VISP_HAVE_DC1394_VERSION == 1)



#include <visp/vp1394Grabber.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

const int vp1394Grabber::DROP_FRAMES = 1; /*!< With libdc1394-1.0.0 or more rencent versions, DROP_FRAMES has to be set to 1 in order to suppress the latency. */
const int vp1394Grabber::NUM_BUFFERS = 8; /*!< Number of buffers */
const int vp1394Grabber::MAX_PORTS   = 4; /*!< Maximal number of ports */
const int vp1394Grabber::MAX_CAMERAS = 8; /*!< Maximal number of cameras */

const char * vp1394Grabber::strFormats[NUM_FORMATS]= {
  "Format_0",
  "Format_1",
  "Format_2",
  "Reserved", // 3 reserved formats
  "Reserved",
  "Reserved",
  "Format_6",
  "Format_7"
};

const char * vp1394Grabber::strModesInFormat0[NUM_FORMAT0_MODES]= {
  "Format_0, Mode_0: 160x120 YUV (4:4:4)",
  "Format_0, Mode_1: 320x240 YUV (4:2:2)",
  "Format_0, Mode_2: 640x480 YUV (4:1:1)",
  "Format_0, Mode_3: 640x480 YUV (4:2:2)",
  "Format_0, Mode_4: 640x480 RGB 24bpp",
  "Format_0, Mode_5: 640x480 Mono 8bpp",
  "Format_0, Mode_6: 640x480 Mono 16bpp"
};

const char * vp1394Grabber::strModesInFormat1[NUM_FORMAT1_MODES]= {
  "Format_1, Mode_0: 800x600 YUV (4:2:2)",
  "Format_1, Mode_1: 800x600 RGB 24bpp",
  "Format_1, Mode_2: 800x600 Mono 8bpp",
  "Format_1, Mode_3: 1024x768 YUV (4:2:2)",
  "Format_1, Mode_4: 1024x768 RGB 24bpp",
  "Format_1, Mode_5: 1024x768 Mono 8bpp",
  "Format_1, Mode_6: 800x600 Mono 16bpp",
  "Format_1, Mode_7: 1024x768 Mono 16bpp"
};

const char * vp1394Grabber::strModesInFormat2[NUM_FORMAT2_MODES]= {
  "Format_2, Mode_0: 1280x960 YUV (4:2:2)",
  "Format_2, Mode_1: 1280x960 RGB 24bpp",
  "Format_2, Mode_2: 1280x960 Mono 8bpp",
  "Format_2, Mode_3: 1600x1200 YUV (4:2:2)",
  "Format_2, Mode_4: 1600x1200 RGB 24bpp",
  "Format_2, Mode_5: 1600x1200 Mono 8bpp",
  "Format_2, Mode_6: 1280x960 Mono 16bpp",
  "Format_2, Mode_7: 1600x1200 Mono 16bpp"
};

const char * vp1394Grabber::strModesInFormat6[NUM_FORMAT6_MODES]= {
  "Format6, Mode_0: EXIT (not supported)"
};

const char * vp1394Grabber::strModesInFormat7[NUM_MODE_FORMAT7]= {
  "Format_7, Mode_0",
  "Format_7, Mode_1",
  "Format_7, Mode_2",
  "Format_7, Mode_3",
  "Format_7, Mode_4",
  "Format_7, Mode_5",
  "Format_7, Mode_6",
  "Format_7, Mode_7"
};

const char * vp1394Grabber::strColorsInFormat7[NUM_COLOR_FORMAT7] = {
  "Format_7 Color, Mono 8bpp",
  "Format_7 Color, YUV 4:1:1",
  "Format_7 Color, YUV 4:2:2",
  "Format_7 Color, YUV 4:4:4",
  "Format_7 Color, RGB 24bpp",
  "Format_7 Color, Mono 16bpp",
  "Format_7 Color, RGB 48bpp"
};

const char * vp1394Grabber::strFramerates[NUM_FRAMERATES] = {
  "1.875 Hz",
  "3.75 Hz",
  "7.5 Hz",
  "15 Hz",
  "30 Hz",
  "60 Hz"
};

/*!
  Default constructor. Using this constructor, you have explicitly to call
  open().

  By default:
  - the camera is the first found on the bus,

  Current camera settings can be changed using setCamera(),
  setFormat(), setMode() or setFramerate() after a call to open(). The
  list of supported formats, modes and framerates is available using
  respectively getFormatSupported(), getModeSupported(),
  getFramerateSupported().

  \code
  vpImage<unsigned char> I;
  vp1394Grabber g;
  g.open(I);
  g.setFormat(FORMAT_VGA_NONCOMPRESSED); // Format_0
  g.setMode(MODE_640x480_MONO); // Mode 5
  g.setFramerate(FRAMERATE_15); //  15 fps
  \endcode

  \sa open(), setCamera(), setFormat(), setMode(), setFramerate()

*/
vp1394Grabber::vp1394Grabber( )
{
  sprintf(device_name, "/dev/video1394/0");

  // allocations
  handles      = new raw1394handle_t      [vp1394Grabber::MAX_CAMERAS];
  cameras      = new dc1394_cameracapture [vp1394Grabber::MAX_CAMERAS];
  cam_count    = new int [vp1394Grabber::MAX_CAMERAS];
  pformat      = new int [vp1394Grabber::MAX_CAMERAS];
  pmode        = new int [vp1394Grabber::MAX_CAMERAS];
  pframerate   = new int [vp1394Grabber::MAX_CAMERAS];
  width        = new int [vp1394Grabber::MAX_CAMERAS];
  height       = new int [vp1394Grabber::MAX_CAMERAS];
  image_format = new ImageFormatEnum [vp1394Grabber::MAX_CAMERAS];

  // Camera settings initialisation
  for (int i=0; i < vp1394Grabber::MAX_CAMERAS; i ++) {
    pformat[i]    = 0;
    pmode[i]      = 0;
    pframerate[i] = 0;
  }
  verbose = false;

  iso_transmission_started = false;
  handle_created           = false;
  camera_found             = false;
  dma_started              = false;
  num_cameras              = 0;
  camera                   = 0; // the first camera on the bus

  // Image settings
  for (int i=0; i < vp1394Grabber::MAX_CAMERAS; i ++) {
    width[i]        = 0;
    height[i]       = 0;
    image_format[i] = RGB;
  }

  init = false ;
}

/*!

  Constructor which initialize the grabber and call the open() method.

  \param I  Image data structure (8 bits image)

  By default:
  - the camera is the first found on the bus,

  Current camera settings can be changed using setCamera(),
  setFormat(), setMode() or setFramerate() after a call to open(). The
  list of supported formats, modes and framerates is available using
  respectively getFormatSupported(), getModeSupported(),
  getFramerateSupported().

  \code
  vpImage<unsigned char> I;
  vp1394Grabber g(I);
  g.setFormat(FORMAT_VGA_NONCOMPRESSED); // Format_0
  g.setMode(MODE_640x480_MONO); // Mode 5
  g.setFramerate(FRAMERATE_15); //  15 fps
  \endcode

  \sa setCamera(), setFormat(), setMode(), setFramerate()
*/
vp1394Grabber::vp1394Grabber(vpImage<unsigned char> &I)
{
  sprintf(device_name, "/dev/video1394/0");

  // allocations
  handles      = new raw1394handle_t      [vp1394Grabber::MAX_CAMERAS];
  cameras      = new dc1394_cameracapture [vp1394Grabber::MAX_CAMERAS];
  cam_count    = new int [vp1394Grabber::MAX_CAMERAS];
  pformat      = new int [vp1394Grabber::MAX_CAMERAS];
  pmode        = new int [vp1394Grabber::MAX_CAMERAS];
  pframerate   = new int [vp1394Grabber::MAX_CAMERAS];
  width        = new int [vp1394Grabber::MAX_CAMERAS];
  height       = new int [vp1394Grabber::MAX_CAMERAS];
  image_format = new ImageFormatEnum [vp1394Grabber::MAX_CAMERAS];

  // Camera settings
  for (int i=0; i < vp1394Grabber::MAX_CAMERAS; i ++) {
    pformat[i]    = 0;
    pmode[i]      = 0;
    pframerate[i] = 0;
  }
  verbose = false;

  iso_transmission_started = false;
  handle_created           = false;
  camera_found             = false;
  num_cameras              = 0;
  camera                   = 0; // the first camera on the bus

  // Image settings
  for (int i=0; i < vp1394Grabber::MAX_CAMERAS; i ++) {
    width[i]        = 0;
    height[i]       = 0;
    image_format[i] = RGB;
  }

  init = false ;

  open ( I ) ;
}

/*!

  Destructor.

  Stops the iso transmission and close the device.

  \sa close()

*/
vp1394Grabber::~vp1394Grabber()
{
  close();
}

/*!

  If multiples cameras are connected on the bus, select the camero to dial
  with.

  \param camera A camera. The value must be comprised between 0 (the first
  camera) and the number of cameras found on the bus and returned by
  getNumCameras(). If two cameras are connected on the bus, setting \e camera
  to one allows to communicate with the second one.

  \exception settingError If the required camera is not reachable.

  \warning Has to be called after open() to be sure that a camera is detected.

  \code
  vpImage<unsigned char> I;
  vp1394Grabber g;
  g.open(I);
  unsigned int num_cameras; // Number of cameras on the bus
  g.getNumCameras(num_cameras);
  g.setCamera(num_cameras-1); // To dial with the last camera on the bus
  acquire(I); // I contains the frame captured by the last camera on the bus
  \endcode

  \sa setFormat(), setMode(), setFramerate(), open(), getNumCameras()

*/
void
vp1394Grabber::setCamera(unsigned int camera)
{
  if (camera >= num_cameras) {
    close();
    vpERROR_TRACE("The required camera is not present");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "The required camera is not present") );
  }

  this->camera = camera;

}
/*!

  Get the active camera on the bus.

  \param camera A camera id. The value is comprised between 0 (the first
  camera) and the number of cameras found on the bus and returned by
  getNumCameras().

  \sa setCamera(), getNumCameras()

*/
void
vp1394Grabber::getCamera(unsigned int &camera)
{
  camera = this->camera;

}
/*!

  Set the capture format for a given camera on the bus. To set the format for a
  specific camera, call setCamera() before.

  \param format The camera image format. The current camera format is given by
  getFormat(). The camera supported formats are given by
  getFormatSupported(). The allowed values given in
  libdc1394/dc1394_control.h are:
  - FORMAT_VGA_NONCOMPRESSED for the Format_0
  - FORMAT_SVGA_NONCOMPRESSED_1 for the Format_1
  - FORMAT_SVGA_NONCOMPRESSED_2 for the Format_2
  - FORMAT_STILL_IMAGE for the Format_6
  - FORMAT_SCALABLE_IMAGE_SIZE for the  Format_7

  \warning The requested format is sent to the camera only after a call to
  close(). Depending on the format and the camera mode, image size can differ.

  \exception settingError If the required camera is not present.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa setMode(), setFramerate(), open(), setCamera(), getNumCameras()

*/
void
vp1394Grabber::setFormat(int format)
{
  if (iso_transmission_started  == true) {

    stopIsoTransmission();

    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_dma_unlisten( handles[i], &cameras[i] );
    iso_transmission_started = false;
  }

  this->pformat[camera] = format;

  setup();
  startIsoTransmission();
}

/*!

  Query the actual capture format of a given camera. The camera supported
  formats are given by getFormatSupported().

  \warning Before requerying the actual format a handle must
  be created by calling open(), and a camera must be connected.

  \param format The camera capture format (see file
  libdc1394/dc1394_control.h), either :

  - FORMAT_VGA_NONCOMPRESSED for the Format_0
  - FORMAT_SVGA_NONCOMPRESSED_1 for the Format_1
  - FORMAT_SVGA_NONCOMPRESSED_2 for the Format_2
  - FORMAT_STILL_IMAGE for the Format_6
  - FORMAT_SCALABLE_IMAGE_SIZE for the  Format_7

  \exception settingError If the required camera is not present or if an error occurs.

  \sa setFormat(), getFormatSupported(), open(), setCamera(), getNumCameras()

*/
void
vp1394Grabber::getFormat(int & format)
{
  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  if(dc1394_get_camera_misc_info(handles[camera],
				 cameras[camera].node,
				 &miscinfo) !=DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to get misc info");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get misc info") );
  }

  format = miscinfo.format;
}

/*!

  Query the available camera image formats.

  \warning Before requerying supported formats a handle must be created by
  calling open(), and a camera must be connected.

  \param  formats The list of supported camera image formats.

  \return The number of supported camera image formats, 0 if an error occurs.

  \exception settingError If the required camera is not present or if an error occurs.

  \sa getModeSupported(), getFramerateSupported(), open(),
  getFormat(), setCamera()

*/
int
vp1394Grabber::getFormatSupported(vpList<int> & formats)
{
  int nb = 0; // Number of supported formats

  // Refresh the list of supported formats
  formats.kill();

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  quadlet_t value;
  if (dc1394_query_supported_formats(handles[camera],
				     cameras[camera].node,
				     &value) != DC1394_SUCCESS) {
    vpERROR_TRACE("Could not query supported formats");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Could not query supported formats") );
  }

  if (value & (0x1<<31)) {
    formats.addRight(FORMAT_VGA_NONCOMPRESSED);
    nb ++;
  }
  if (value & (0x1<<30)) {
    formats.addRight(FORMAT_SVGA_NONCOMPRESSED_1);
    nb ++;
  }
  if (value & (0x1<<29)) {
    formats.addRight(FORMAT_SVGA_NONCOMPRESSED_2);
    nb ++;
  }
  if (value & (0x1<<25)) {
    formats.addRight(FORMAT_STILL_IMAGE);
    nb ++;
  }
  if (value & (0x1<<24)) {
    formats.addRight(FORMAT_SCALABLE_IMAGE_SIZE);
    nb ++;
  }

  return nb;
}

/*!

  Set the camera capture mode for a given camera on the bus.

  \param mode The camera capture mode. The current camera mode is
  given by getMode(). The camera supported modes are given by
  getSupportedModes(). The allowed values are given in
  libdc1394/dc1394_control.h file:

  - Allowed mode for Format 0: MODE_160x120_YUV444 = 64, MODE_320x240_YUV422, MODE_640x480_YUV411, MODE_640x480_YUV422, MODE_640x480_RGB, MODE_640x480_MONO, MODE_640x480_MONO16

  - Allowed modes for Format 1: MODE_800x600_YUV422 = 96, MODE_800x600_RGB, MODE_800x600_MONO, MODE_1024x768_YUV422, MODE_1024x768_RGB, MODE_1024x768_MONO, MODE_800x600_MONO16, MODE_1024x768_MONO16

  - Allowed modes for Format 2: MODE_1280x960_YUV422 = 128, MODE_1280x960_RGB, MODE_1280x960_MONO, MODE_1600x1200_YUV422, MODE_1600x1200_RGB, MODE_1600x1200_MONO, MODE_1280x960_MONO16, MODE_1600x1200_MONO16

  - Allowed modes for Format 6: MODE_EXIF= 256

  - Allowed modes for Format 7: MODE_FORMAT7_0= 288, MODE_FORMAT7_1, MODE_FORMAT7_2, MODE_FORMAT7_3, MODE_FORMAT7_4, MODE_FORMAT7_5, MODE_FORMAT7_6, MODE_FORMAT7_7


  All these modes are not supported by your camera. The camera supported modes
  are given by getModeSupported().

  \warning The requested format is sent to the camera only after a call to
  close(). Depending on the format and the camera mode, image size can differ.

  \exception settingError If the required camera is not present.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa setFormat(), setFramerate(), open(), getNumCameras(), setCamera()

*/
void
vp1394Grabber::setMode(int mode)
{
  if (iso_transmission_started  == true) {

    stopIsoTransmission();

    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_dma_unlisten( handles[i], &cameras[i] );
    iso_transmission_started = false;
  }

  this->pmode[camera] = mode;

  setup();
  startIsoTransmission();
}

/*!

  Query the actual capture mode of a given camera. Given a format, all
  the camera supported modes are given by getModeSupported().

  \warning Before requerying the actual mode a handle must
  be created by calling open(), and a camera must be connected.

  \param mode The camera capture mode.

  \exception settingError If the required camera is not present.

  \sa setMode(), getModeSupported(), open(), setCamera()

*/
void
vp1394Grabber::getMode(int & mode)
{

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }
  if(dc1394_get_camera_misc_info(handles[camera],
				 cameras[camera].node,
				 &miscinfo) !=DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to get misc info");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get misc info") );
  }

  mode = miscinfo.mode;
}

/*!

  Query the available camera modes for the given format.

  \warning Before requerying supported modes for a given format a handle must
  be created by calling open(), and a camera must be connected.

  \param format Camera image format. Values for format are parts of the list
  (see file libdc1394/dc1394_control.h):

  - FORMAT_VGA_NONCOMPRESSED = 384 for the Format_0
  - FORMAT_SVGA_NONCOMPRESSED_1 for the Format_1
  - FORMAT_SVGA_NONCOMPRESSED_2 for the Format_2
  - FORMAT_STILL_IMAGE = 390 for the Format_6
  - FORMAT_SCALABLE_IMAGE_SIZE for the Format_7

  \param modes The list of supported camera modes.

  \return The number of supported camera modes, 0 if an error occurs.

  \exception settingError If the required camera is not present.

  \sa getMode(), getFramerateSupported(), open(), setCamera()
*/
int
vp1394Grabber::getModeSupported(int format, vpList<int> & modes)
{
  int nb = 0; // Number of supported modes

  // Refresh the list of supported modes
  modes.kill();

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  quadlet_t value;
  if (dc1394_query_supported_modes(handles[camera],
				   cameras[camera].node,
				   format, &value) != DC1394_SUCCESS) {
    vpERROR_TRACE("Could not query supported modes for format %d\n", format);
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Could not query supported mode") );
  }

  switch (format) {
  case FORMAT_VGA_NONCOMPRESSED:
    for (int m = MODE_FORMAT0_MIN; m <= MODE_FORMAT0_MAX; m ++) {
      if (value & (0x1<<(31-(m-MODE_FORMAT0_MIN)))) {
	modes.addRight(m);
	nb ++;
      }
    }
    break;

  case FORMAT_SVGA_NONCOMPRESSED_1:
    for (int m = MODE_FORMAT1_MIN; m <= MODE_FORMAT1_MAX; m ++) {
      if (value & (0x1<<(31-(m-MODE_FORMAT1_MIN)))) {
	modes.addRight(m);
	nb ++;
      }
    }
    break;

  case FORMAT_SVGA_NONCOMPRESSED_2:
    for (int m = MODE_FORMAT2_MIN; m <= MODE_FORMAT2_MAX; m ++) {
      if (value & (0x1<<(31-(m-MODE_FORMAT2_MIN)))) {
	modes.addRight(m);
	nb ++;
      }
    }
    break;

  case FORMAT_STILL_IMAGE:
    for (int m = MODE_FORMAT6_MIN; m <= MODE_FORMAT6_MAX; m ++) {
      if (value & (0x1<<(31-(m-MODE_FORMAT6_MIN)))) {
	modes.addRight(m);
	nb ++;
      }
    }
    break;

  case FORMAT_SCALABLE_IMAGE_SIZE:
    for (int m = MODE_FORMAT7_MIN; m <= MODE_FORMAT7_MAX; m ++) {
      if (value & (0x1<<(31-(m-MODE_FORMAT7_MIN)))) {
	modes.addRight(m);
	nb ++;
      }
    }
    break;
  }

  return nb;
}

/*!

  Set the capture framerate for a given camera on the bus.

  \param framerate The camera framerate. The current framerate of the camera is
  given by getFramerate(). The camera supported framerates are given by
  getFramerateSupported(). The allowed values given in
  libdc1394/dc1394_control.h are:
  - FRAMERATE_1_875 = 32 for 1.875 fps,
  - FRAMERATE_3_75 for 3.75 fps,
  - FRAMERATE_7_5 for 7.5 fps,
  - FRAMERATE_15 for 15 fps,
  - FRAMERATE_30 for 30 fps,
  - FRAMERATE_60 for 60 fps,
  - FRAMERATE_120 for 120 fps,
  - FRAMERATE_240 for 240 fps.

  \exception settingError If the required camera is not present.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa setFormat(), setMode(), open(), getNumCameras(), setCamera()

*/
void
vp1394Grabber::setFramerate(int framerate)
{
  if (iso_transmission_started  == true) {

    stopIsoTransmission();

    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_dma_unlisten( handles[i], &cameras[i] );
    iso_transmission_started = false;
  }

  this->pframerate[camera] = framerate;

  setup();
  startIsoTransmission();
}

/*!

  Query the actual capture framerate of a given camera. The camera supported
  framerates are given by getFramerateSupported().

  \warning Before requerying the actual framerate a handle must
  be created by calling open(), and a camera must be connected.

  \param framerate The camera capture framerate.

  \return true if the framerate was obtained, false if an error occurs.

  \exception settingError If the required camera is not present.

  \sa setFramerate(), getFramerateSupported(), open(), getNumCameras(),
  setCamera()

*/
void
vp1394Grabber::getFramerate(int & framerate)
{
  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  if(dc1394_get_camera_misc_info(handles[camera],
				 cameras[camera].node,
				 &miscinfo) !=DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to get misc info");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get misc info") );
  }

  framerate = miscinfo.framerate;
}

/*!

  Query the available framerates for the given camera image format and mode
  (see file libdc1394/dc1394_control.h). The framerate is only available if
  format is either FORMAT_VGA_NONCOMPRESSED (format 0),
  FORMAT_SVGA_NONCOMPRESSED_1 (format 1) or FORMAT_SVGA_NONCOMPRESSED_2 (format
  2). If format is equal to FORMAT_STILL_IMAGE (format 6) or
  FORMAT_SCALABLE_IMAGE_SIZE (format 7), the framerate is not avalaible. In
  this case, we return 0.

  \warning Before requerying supported framerates for a given format and mode a
  handle must be created by calling open(), and a camera must be connected.

  \param format Camera image format.
  \param mode Camera mode.
  \param framerates The list of supported camera framerates.

  \return The number of supported camera image framerates, 0 if an error
  occurs.

  \exception settingError If the required camera is not present.

  \sa getFormatSupported(), getModeSupported(), open(), getFramerate(),
  setCamera()
*/
int
vp1394Grabber::getFramerateSupported(int format, int mode,
				     vpList<int> & framerates)
{
  int nb = 0; // Number of supported framerates

  // Refresh the list of supported framerates
  framerates.kill();

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  switch(format) {
    case FORMAT_VGA_NONCOMPRESSED:
    case FORMAT_SVGA_NONCOMPRESSED_1:
    case FORMAT_SVGA_NONCOMPRESSED_2:
      {

	quadlet_t value;
	if (dc1394_query_supported_framerates(handles[camera],
					      cameras[camera].node,
					      format, mode,
					      &value) != DC1394_SUCCESS) {
	  vpERROR_TRACE("Could not query supported frametates for format %d\n"
		      "and mode %d\n", format, mode);
	  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
					 "Could not query supported framerates") );
	}

	for (int f = FRAMERATE_MIN; f <= FRAMERATE_MAX; f ++) {
	  if (value & (0x1<<(31-(f-FRAMERATE_MIN)))) {
	    framerates.addRight(f);
	    nb ++;
	  }
	}
      }
      break;
  default:
    // Framerate not avalaible for:
    //  - FORMAT_STILL_IMAGE for the Format_6
    //  - FORMAT_SCALABLE_IMAGE_SIZE for the Format_7
    return 0;
  }

  return nb;
}

/*!

  Converts the string containing the description of the format into the format
  dentifier.

  \param format The string describing the format given by Format(int)

  \return The camera capture format identifier, either :
  - FORMAT_VGA_NONCOMPRESSED for the Format_0
  - FORMAT_SVGA_NONCOMPRESSED_1 for the Format_1
  - FORMAT_SVGA_NONCOMPRESSED_2 for the Format_2
  - FORMAT_STILL_IMAGE for the Format_6
  - FORMAT_SCALABLE_IMAGE_SIZE for the Format_7
  .
  This method returns 0 if the string does not match to a format string.

  \sa convertFormat(int), convertMode(string), convertFramerate(string&)

*/
int vp1394Grabber::convertFormat(string format)
{
  for (int i = FORMAT_MIN; i <= FORMAT_MAX; i ++) {
    //    if (format.compare(0, format(i).length(), format(i)) == 0) {
    if (format.compare(0, convertFormat(i).length(), convertFormat(i)) == 0) {
       return i;
    }
  };

  return 0;
}
/*!

  Converts the string containing the description of the mode into the mode
  dentifier.

  \param mode The string describing the mode given by Mode(int)

  \return The camera capture mode identifier.

  This method returns 0 if the string does not match to a mode string.

  \sa convertMode(int), convertFormat(string), convertFramerate(string)

*/
int vp1394Grabber::convertMode(string mode)
{
  for (int i = MODE_FORMAT0_MIN; i <= MODE_FORMAT0_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };
  for (int i = MODE_FORMAT1_MIN; i <= MODE_FORMAT1_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };
  for (int i = MODE_FORMAT2_MIN; i <= MODE_FORMAT2_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };
  for (int i = MODE_FORMAT6_MIN; i <= MODE_FORMAT6_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };
  for (int i = MODE_FORMAT7_MIN; i <= MODE_FORMAT7_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };
  for (int i = COLOR_FORMAT7_MIN; i <= COLOR_FORMAT7_MAX; i ++) {
    if (mode.compare(convertMode(i)) == 0)
      return i;
  };

  return 0;
}
/*!

  Converts the string containing the description of the framerate into the
  framerate dentifier.

  \param framerate The string describing the framerate given by Framerate(int)

  \return The camera capture framerate identifier.

  This method returns 0 if the string does not match to a framerate string.

  \sa convertFramerate(int), convertFormat(string &), convertMode(string)

*/
int vp1394Grabber::convertFramerate(string framerate)
{
  for (int i = FRAMERATE_MIN; i <= FRAMERATE_MAX; i ++) {
    if (framerate.compare(convertFramerate(i)) == 0)
      return i;
  };

  return 0;
}


/*!

  Converts the format identifier into a string containing the description of
  the format.

  \param format The camera capture format, either :
  - FORMAT_VGA_NONCOMPRESSED for the Format_0
  - FORMAT_SVGA_NONCOMPRESSED_1 for the Format_1
  - FORMAT_SVGA_NONCOMPRESSED_2 for the Format_2
  - FORMAT_STILL_IMAGE for the Format_6
  - FORMAT_SCALABLE_IMAGE_SIZE for the Format_7

  \return A string describing the format, an empty string if the format is not
  supported.

  \sa convertMode(), convertFramerate()

*/
string vp1394Grabber::convertFormat(int format)
{
  string _format;
  if ((format >= FORMAT_MIN) && (format <= FORMAT_MAX)) {
    switch (format) {
    case FORMAT_VGA_NONCOMPRESSED:    _format = strFormats[0]; break;
    case FORMAT_SVGA_NONCOMPRESSED_1: _format = strFormats[1]; break;
    case FORMAT_SVGA_NONCOMPRESSED_2: _format = strFormats[2]; break;
      // 3 reserved formats
    case FORMAT_SVGA_NONCOMPRESSED_2 + 1: _format = strFormats[3]; break;
    case FORMAT_SVGA_NONCOMPRESSED_2 + 2: _format = strFormats[4]; break;
    case FORMAT_SVGA_NONCOMPRESSED_2 + 3: _format = strFormats[5]; break;

    case FORMAT_STILL_IMAGE:          _format = strFormats[6]; break;
    case FORMAT_SCALABLE_IMAGE_SIZE:  _format = strFormats[7]; break;
    default:
      break;
    }
  }
  else {
    cout << "The format " << format
	 << " is not supported by the camera" << endl;
    _format = "Not Valid";
  }

  return _format;
}
/*!

  Converts the mode identifier into a string containing the description of the
  mode.

  \param mode The camera capture mode.

  \return A string describing the mode, an empty string if the mode is not
  supported.

  \sa convertFormat(), convertFramerate()

*/
string vp1394Grabber::convertMode(int mode)
{
  string _mode;

  if ((mode >= MODE_FORMAT0_MIN) && (mode <= MODE_FORMAT0_MAX)) {
    switch (mode) {
    case MODE_160x120_YUV444: _mode = strModesInFormat0[0]; break;
    case MODE_320x240_YUV422: _mode = strModesInFormat0[1]; break;
    case MODE_640x480_YUV411: _mode = strModesInFormat0[2]; break;
    case MODE_640x480_YUV422: _mode = strModesInFormat0[3]; break;
    case MODE_640x480_RGB:    _mode = strModesInFormat0[4]; break;
    case MODE_640x480_MONO:   _mode = strModesInFormat0[5]; break;
    case MODE_640x480_MONO16: _mode = strModesInFormat0[6]; break;
    default: break;
    }
  }
  else if ((mode >= MODE_FORMAT1_MIN) && (mode <= MODE_FORMAT1_MAX)) {
    switch (mode) {
    case MODE_800x600_YUV422:  _mode = strModesInFormat1[0]; break;
    case MODE_800x600_RGB:     _mode = strModesInFormat1[1]; break;
    case MODE_800x600_MONO:    _mode = strModesInFormat1[2]; break;
    case MODE_1024x768_YUV422: _mode = strModesInFormat1[3]; break;
    case MODE_1024x768_RGB:    _mode = strModesInFormat1[4]; break;
    case MODE_1024x768_MONO:   _mode = strModesInFormat1[5]; break;
    case MODE_800x600_MONO16:  _mode = strModesInFormat1[6]; break;
    case MODE_1024x768_MONO16: _mode = strModesInFormat1[7]; break;
    default: break;
    }
  }
  else if ((mode >= MODE_FORMAT2_MIN) && (mode <= MODE_FORMAT2_MAX)) {
    switch (mode) {
    case MODE_1280x960_YUV422:  _mode = strModesInFormat2[0]; break;
    case MODE_1280x960_RGB:     _mode = strModesInFormat2[1]; break;
    case MODE_1280x960_MONO:    _mode = strModesInFormat2[2]; break;
    case MODE_1600x1200_YUV422: _mode = strModesInFormat2[3]; break;
    case MODE_1600x1200_RGB:    _mode = strModesInFormat2[4]; break;
    case MODE_1600x1200_MONO:   _mode = strModesInFormat2[5]; break;
    case MODE_1280x960_MONO16:  _mode = strModesInFormat2[6]; break;
    case MODE_1600x1200_MONO16: _mode = strModesInFormat2[7]; break;
    default: break;
    }
  }
  else if ((mode >= MODE_FORMAT6_MIN) && (mode <= MODE_FORMAT6_MAX)) {
    switch (mode) {
    case MODE_EXIF: _mode = strModesInFormat6[0]; break;
    default: break;
    }
  }
  else if ((mode >= MODE_FORMAT7_MIN) && (mode <= MODE_FORMAT7_MAX)) {
    switch (mode) {
    case MODE_FORMAT7_0: _mode = strModesInFormat7[0]; break;
    case MODE_FORMAT7_1: _mode = strModesInFormat7[1]; break;
    case MODE_FORMAT7_2: _mode = strModesInFormat7[2]; break;
    case MODE_FORMAT7_3: _mode = strModesInFormat7[3]; break;
    case MODE_FORMAT7_4: _mode = strModesInFormat7[4]; break;
    case MODE_FORMAT7_5: _mode = strModesInFormat7[5]; break;
    case MODE_FORMAT7_6: _mode = strModesInFormat7[6]; break;
    case MODE_FORMAT7_7: _mode = strModesInFormat7[7]; break;
    default: break;
    }
  }
  else if ((mode >= COLOR_FORMAT7_MIN) && (mode <= COLOR_FORMAT7_MAX)) {
    switch (mode) {
    case COLOR_FORMAT7_MONO8:  _mode = strColorsInFormat7[0]; break;
    case COLOR_FORMAT7_YUV411: _mode = strColorsInFormat7[1]; break;
    case COLOR_FORMAT7_YUV422: _mode = strColorsInFormat7[2]; break;
    case COLOR_FORMAT7_YUV444: _mode = strColorsInFormat7[3]; break;
    case COLOR_FORMAT7_RGB8:   _mode = strColorsInFormat7[4]; break;
    case COLOR_FORMAT7_MONO16: _mode = strColorsInFormat7[5]; break;
    case COLOR_FORMAT7_RGB16:  _mode = strColorsInFormat7[6]; break;
    default: break;
    }
  }
  else {
    cout << "The mode " << mode << " is not supported by the camera" << endl;
    _mode = "Not valid";
  }

  return _mode;
}

/*!

  Converts the framerate identifier into a string
  containing the description of the framerate.

  \param framerate The camera capture framerate.

  \return A string describing the framerate, an empty string if the framerate
  is not supported.

  \sa convertFormat(), convertMode()

*/
string vp1394Grabber::convertFramerate(int framerate)
{
  string _framerate;

  if ((framerate >= FRAMERATE_MIN) && (framerate <= FRAMERATE_MAX)) {
    switch (framerate) {
    case FRAMERATE_1_875: _framerate = strFramerates[0]; break;
    case FRAMERATE_3_75:  _framerate = strFramerates[1]; break;
    case FRAMERATE_7_5:   _framerate = strFramerates[2]; break;
    case FRAMERATE_15:    _framerate = strFramerates[3]; break;
    case FRAMERATE_30:    _framerate = strFramerates[4]; break;
    case FRAMERATE_60:    _framerate = strFramerates[5]; break;
    default: break;
    }
  }
  else {
    cout << "The framerate " << framerate
	 << " is not supported by the camera" << endl;
    _framerate = "Not valid";
  }

  return _framerate;

}

/*!

  Set the shutter for a given camera.

  \param shutter The shutter value to apply to the camera.


  \exception settingError If the required camera is not present or if an error
  occurs.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa getShutter(), open(), getNumCameras(), setCamera()
*/
void
vp1394Grabber::setShutter(unsigned int shutter)
{
  unsigned int min_shutter = 0;
  unsigned int max_shutter = 0;


  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  if(dc1394_get_min_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_SHUTTER, &min_shutter) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get min shutter value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get min shutter value") );
  }

  if(dc1394_get_max_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_SHUTTER, &max_shutter) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get max shutter value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get max shutter value") );
  }

  if (shutter < min_shutter || shutter > max_shutter) {
    vpCERROR << "The requested shutter " << shutter
	   << " must be comprised between " << min_shutter
	   << " and " << max_shutter << endl;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Cannot set shutter: bad value") );
  }

  if ( dc1394_set_shutter(handles[camera],
			  cameras[camera].node,
			  shutter) != DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to set shutter");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Cannot set shutter") );
  }
}

/*!

  Query the actual shutter value of a given camera and the bounded shutter
  values.

  \warning Before requerying the shutter a handle must
  be created by calling open(), and a camera must be connected.

  \param min_shutter Minimal autorized shutter value.

  \param shutter The current camera shutter value. This value is comprised
  between \e min_shutter and \e max_shutter.

  \param max_shutter Maximal autorized shutter value.


  \exception settingError If the required camera is not present or if an error occurs.

  \sa setShutter(), open(), getNumCameras(), setCamera()

*/
void
vp1394Grabber::getShutter(unsigned int &min_shutter,
			  unsigned int &shutter,
			  unsigned int &max_shutter)
{
  shutter = 0;
  min_shutter = 0;
  max_shutter = 0;

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }
  if(dc1394_get_shutter(handles[camera],
			cameras[camera].node,
			&shutter) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get shutter value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get shutter value") );

  }

  if(dc1394_get_min_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_SHUTTER, &min_shutter) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get min shutter value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get min shutter value") );
  }

  if(dc1394_get_max_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_SHUTTER, &max_shutter) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get max shutter value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get max shutter value") );
  }
}

/*!

  Set the gain for a given camera.

  \warning Before setting the gain a handle must
  be created by calling open(), and a camera must be connected.

  \param gain The gain value to apply to the camera.

  \exception settingError If the required camera is not present or if an error occurs.

  \sa getGain(), getNumCameras(), setCamera()
*/
void
vp1394Grabber::setGain(unsigned int gain)
{
  unsigned int min_gain = 0;
  unsigned int max_gain = 0;


  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }
  if(dc1394_get_min_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_GAIN, &min_gain) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get min gain value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get min gain value") );
  }

  if(dc1394_get_max_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_GAIN, &max_gain) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get max gain value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get max gain value") );
  }

  if (gain < min_gain || gain > max_gain) {
    vpCERROR << "The requested gain " << gain
	   << " must be comprised between " << min_gain
	   << " and " << max_gain << endl;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Cannot set shutter: bad value") );
  }

  if ( dc1394_set_gain(handles[camera],
		       cameras[camera].node,
		       gain) != DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to set gain");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Cannot set gain") );
  }
}

/*!

  Query the actual gain value of a given camera and the bounded gain
  values.

  \warning Before requerying the gain a handle must
  be created by calling open(), and a camera must be connected.

  \param min_gain Minimal autorized gain value.

  \param gain The current camera gain value. This value is comprised
  between \e min_gain and \e max_gain.

  \param max_gain Maximal autorized gain value.

  \exception settingError If the required camera is not present or if an error
  occurs.

  \sa setGain(), open(), getNumCameras(), setCamera()

*/
void
vp1394Grabber::getGain(unsigned int &min_gain,
		       unsigned int &gain,
		       unsigned int &max_gain)
{
  gain = 0;
  min_gain = 0;
  max_gain = 0;

  if (handle_created == false) {
    close();
    vpERROR_TRACE("To set the shutter the handle must be created");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter the handle must be created") );
  }
  if (camera_found == false) {
    close();
    vpERROR_TRACE("To set the shutter a camera must be connected");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "To set the shutter a camera must be connected") );
  }

  if(dc1394_get_gain(handles[camera],
		     cameras[camera].node,
		     &gain) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get gain value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get gain value") );

  }

  if(dc1394_get_min_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_GAIN, &min_gain) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get min gain value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get min gain value") );
  }

  if(dc1394_get_max_value(handles[camera],
			  cameras[camera].node,
			  FEATURE_GAIN, &max_gain) !=DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("Unable to get max gain value");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Unable to get max gain value") );

  }
}

/*!

  Get the image width. It depends on the camera format setFormat() and mode
  setMode(). The width of the images is only available after a call to open().

  \param width The image width, zero if the required camera is not avalaible.

  \exception settingError If the required camera is not present.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa getHeight(), setCamera()

*/
void vp1394Grabber::getWidth(int &width)
{
  if (camera >= num_cameras) {
    width = 0;
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "The required camera is not present") );
  }

  width = this->width[camera];
  nrows = width;
}

/*!

  get the image height. It depends on the camera format setFormat() and mode
  setMode(). The height of the images is only available after a call to
  open().

  \param height The image width.
  \param camera A camera. The value must be comprised between 0 and the
  number of cameras found on the bus and returned by getNumCameras().

  \exception settingError If the required camera is not present.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa getWidth(), getImageFormat(), close(), getNumCameras(), setCamera()

*/
void vp1394Grabber::getHeight(int &height)
{
  if (camera >= num_cameras) {
    height = 0;
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "The required camera is not present") );
  }

  height = this->height[camera];
  ncols = height;
}

/*!

  Query the number of cameras on the bus.

  \param cameras The number of cameras found on the bus.


*/
void
vp1394Grabber::getNumCameras(unsigned int &cameras)
{
  if (camera_found == false) {
    vpCTRACE << "No camera found..."<< endl;
    cameras = 0;
  }

  cameras = num_cameras;
}


/*!
  Initialize grey level image acquisition

  \param I : Image data structure (8 bits image)

*/
void
vp1394Grabber::open(vpImage<unsigned char> &I)
{

  open();

  // Get the actual camera format, mode and framerate
  getFormat(pformat[camera]);
  getMode(pmode[camera]);
  getFramerate(pframerate[camera]);

  setup();
  startIsoTransmission();

  int _ncols, _nrows;
  getWidth(_ncols) ;
  getHeight(_nrows) ;

  vpDEBUG_TRACE(10, "%d %d", _nrows, _ncols ) ;

  I.resize(_nrows, _ncols) ;

  init = true ;

}

/*!
  Initialize color image (in RGBa format) acquisition

  \param I : Image data structure (RGBa format)

*/
void
vp1394Grabber::open(vpImage<vpRGBa> &I)
{

  open();

  // Get the actual camera format, mode and framerate
  getFormat(pformat[camera]);
  getMode(pmode[camera]);
  getFramerate(pframerate[camera]);

  setup();
  startIsoTransmission();

  int _ncols, _nrows;
  getWidth(_ncols) ;
  getHeight(_nrows) ;

  vpERROR_TRACE("%d %d", _nrows, _ncols ) ;

  I.resize(_nrows, _ncols) ;

  init = true ;

}

/*!
  Acquire a grey level image from a given camera.

  \param I  Image data structure (8 bits image)
  \param camera A camera. The value must be comprised between 0 and the
  number of cameras found on the bus and returned by getNumCameras().

  \exception initializationError If the device is not openned.

  \sa getField(), setCamera()
*/
void
vp1394Grabber::acquire(vpImage<unsigned char> &I)
{

  if (init==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Initialization not done") );
  }

  int  *bitmap = NULL ;
  bitmap = dmaCapture();

  int _ncols, _nrows;
  getWidth(_ncols) ;
  getHeight(_nrows) ;

  if ((I.getCols() != _ncols)||(I.getRows() != _nrows))
    I.resize(_nrows, _ncols) ;

  int size  = I.getRows()*I.getCols();
  switch (image_format[camera])
  {
  case MONO:
    memcpy(I.bitmap, (unsigned char *) bitmap, size*sizeof(unsigned char));
    break;
  case YUV411: {
    vpImageConvert::YUV411ToGrey( (unsigned char *) bitmap, I.bitmap, size);
    break;
  }
  case YUV422: {
    vpImageConvert::YUV422ToGrey( (unsigned char *) bitmap, I.bitmap, size);
    break;
  }
  case RGB: {
    vpImageConvert::RGBToGrey((unsigned char *) bitmap, I.bitmap, size);
    break;
  }
  case RGBa: {
    vpImageConvert::RGBaToGrey((unsigned char *) bitmap, I.bitmap, size);
    break;
  }
  default:
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Format conversion not implemented. Acquisition failed.") );
    break;
  };

  dmaDoneWithBuffer();

}

/*!
  Acquire a color image from a given camera.

  \param I  Image data structure (RGBa image)
  \param camera A camera. The value must be comprised between 0 and the
  number of cameras found on the bus and returned by getNumCameras().

  \exception initializationError If the device is not openned.

  \sa getField(), setCamera()
*/
void
vp1394Grabber::acquire(vpImage<vpRGBa> &I)
{

  if (init==false)
  {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Initialization not done") );
  }

  int  *bitmap = NULL ;
  bitmap = dmaCapture();

  int _ncols, _nrows;
  getWidth(_ncols) ;
  getHeight(_nrows) ;

  if ((I.getCols() != _ncols)||(I.getRows() != _nrows))
    I.resize(_nrows, _ncols) ;

  int size  = I.getRows()*I.getCols();
  switch (image_format[camera])
  {
  case MONO:
    vpImageConvert::GreyToRGBa((unsigned char *) bitmap,
			       (unsigned char *) I.bitmap, size);
    break;
  case YUV411: {
    vpImageConvert::YUV411ToRGBa( (unsigned char *) bitmap,
				  (unsigned char *) I.bitmap, size);
    break;
  }
  case YUV422: {
    vpImageConvert::YUV422ToRGBa( (unsigned char *) bitmap,
				  (unsigned char *) I.bitmap, size);
    break;
  }
  case RGB: {
    vpImageConvert::RGBToRGBa((unsigned char *) bitmap,
			      (unsigned char *) I.bitmap, size);
    break;
  }
  case RGBa: {
    memcpy((unsigned char *) I.bitmap, (unsigned char *) bitmap, size);
    break;
  }
  default:
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Format conversion not implemented. Acquisition failed.") );
    break;
  };

  dmaDoneWithBuffer();

}


/*!

  Open ohci and asign handle to it and get the camera nodes and
  describe them as we find them.

  \exception initializationError If a raw1394 handle can't be aquired.

  \sa close()
*/
void
vp1394Grabber::open()
{
  //int num_nodes;
  int num_ports = vp1394Grabber::MAX_PORTS;
  struct raw1394_portinfo ports[vp1394Grabber::MAX_PORTS];
  raw1394handle_t raw_handle;

  if (handles == NULL)
    handles      = new raw1394handle_t      [vp1394Grabber::MAX_CAMERAS];
  if (cameras == NULL)
    cameras      = new dc1394_cameracapture [vp1394Grabber::MAX_CAMERAS];
  if (cam_count == NULL)
    cam_count    = new int [vp1394Grabber::MAX_CAMERAS];
  if (pformat == NULL)
    pformat      = new int [vp1394Grabber::MAX_CAMERAS];
  if (pmode == NULL)
    pmode        = new int [vp1394Grabber::MAX_CAMERAS];
  if (pframerate == NULL)
    pframerate   = new int [vp1394Grabber::MAX_CAMERAS];
  if (width == NULL)
    width        = new int [vp1394Grabber::MAX_CAMERAS];
  if (height == NULL)
    height       = new int [vp1394Grabber::MAX_CAMERAS];
  if (image_format == NULL)
    image_format = new ImageFormatEnum [vp1394Grabber::MAX_CAMERAS];

  raw_handle = raw1394_new_handle();

  if (raw_handle==NULL) {
    close();
    vpERROR_TRACE("Unable to aquire a raw1394 handle\n\n"
		"Please check \n"
		"  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
		"  - if you have read/write access to /dev/raw1394\n\n");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Unable to aquire a raw1394 handle") );
  }

  num_ports = raw1394_get_port_info(raw_handle, ports, num_ports);
  raw1394_destroy_handle(raw_handle);
  if (verbose)
    printf("number of ports detected: %d\n", num_ports);

  if (num_ports < 1) {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "no ports found") );
  }

  //num_nodes = raw1394_get_nodecount(raw_handle);
  num_cameras = 0;

  /* get dc1394 handle to each port */
  for (int p = 0; p < num_ports; p++)  {

    /* get the camera nodes and describe them as we find them */
    raw_handle = raw1394_new_handle();
    raw1394_set_port( raw_handle, p );

    camera_nodes = NULL;
    camera_nodes = dc1394_get_camera_nodes(raw_handle, &cam_count[p], 0);
    raw1394_destroy_handle(raw_handle);
    if (verbose)
      fprintf(stdout, "%d camera(s) on port %d\n", cam_count[p], p);

    for (int i = 0; i < cam_count[p]; i++, num_cameras ++) {
      handles[num_cameras] = dc1394_create_handle(p);
      if (handles[num_cameras]==NULL) {
	close();
	vpERROR_TRACE("Unable to aquire a raw1394 handle\n");
	vpERROR_TRACE("did you load the drivers?\n");
	throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				       "Unable to aquire a raw1394 handle.") );
      }
      cameras[num_cameras].node = camera_nodes[i];

    }
    if (cam_count[p])
      dc1394_free_camera_nodes(camera_nodes);
  }

  if (num_cameras < 1) {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "no cameras found") );
  }
  vpDEBUG_TRACE(10, "%d cameras detected\n", num_cameras);

  camera_found = true;

  handle_created = true;

  // Get the actual camera format, mode and framerate
  getFormat(pformat[camera]);
  getMode(pmode[camera]);
  getFramerate(pframerate[camera]);
}

/*!

  Get the feature and set the dma capture.

  Set the camera to the specified format setFormat(), mode setMode() and
  framerate setFramerate(). Considering the camera format and mode, updates
  the captured image size.

  \exception otherError If unable to get feature set or setup the camera.

  \sa setFormat(), setMode(), setFramerate(), getWidth(), getHeight()

*/
void
vp1394Grabber::setup()
{
  unsigned int channel;
  unsigned int speed;

  if ( handle_created == true && camera_found == true) {

    for (unsigned int i = 0; i < num_cameras; i++) {
      if (verbose) {
	dc1394_feature_set features;

	if(dc1394_get_camera_feature_set(handles[i],
					 cameras[i].node,
					 &features) != DC1394_SUCCESS) {
	  close();

	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "Unable to get feature set") );
	}
	else {
	  dc1394_print_feature_set(&features);
	}
      }

      // After setting the camera format and mode we update the image size
      getImageCharacteristics(pformat[i],
			      pmode[i],
			      width[i],
			      height[i],
			      image_format[i]);

      if (dc1394_get_iso_channel_and_speed(handles[i],
					   cameras[i].node,
					   &channel,
					   &speed) != DC1394_SUCCESS) {
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Unable to get the iso channel number") );
      }

      if (pformat[i] == FORMAT_SCALABLE_IMAGE_SIZE) {
	if( dc1394_dma_setup_format7_capture(handles[i],
					     cameras[i].node,
					     channel,
					     pmode[i],
					     speed,
					     USE_MAX_AVAIL, /*max packet size*/
					     0, 0, /* left, top */
					     width[i],
					     height[i],
					     vp1394Grabber::NUM_BUFFERS,
					     vp1394Grabber::DROP_FRAMES,
					     device_name,
					     &cameras[i]) != DC1394_SUCCESS) {
	  close();

	  vpERROR_TRACE("Unable to setup camera in format 7 mode 0-\n"
		      "check line %d of %s to"
		      "make sure that the video mode,framerate and format are "
		      "supported by your camera.\n",
		      __LINE__,__FILE__);

	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Unable to setup camera in format 7 ") );

	}
	if (verbose) {
	  unsigned int qpp; // packet bytes
	  if (dc1394_query_format7_byte_per_packet(handles[i],
						   cameras[i].node,
						   pmode[i],
						   &qpp) != DC1394_SUCCESS) {
	    close();

	    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					   "Unable to query format7 byte_per_packet ") );
	  }
	  //cout << "Format 7: byte per packet : " << qpp << endl;
	}

      } else {

	if (dc1394_dma_setup_capture(handles[i],
				     cameras[i].node,
				     channel,
				     pformat[i],
				     pmode[i],
				     speed,
				     pframerate[i],
				     vp1394Grabber::NUM_BUFFERS,
				     vp1394Grabber::DROP_FRAMES,
				     device_name,
				     &cameras[i]) != DC1394_SUCCESS) {
	  vpERROR_TRACE("Unable to setup camera- check line %d of %s to"
		      "make sure that the video mode,framerate and format are "
		      "supported by your camera.\n",
		      __LINE__,__FILE__);
	  close();

	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "Unable to setup camera ") );
	}
      }
    }
  }
}

/*!

  Gets the image size and coding format, depending on the camera image format
  and the camera mode.

  \param _format The camera capture format.
  \param _mode The camera capture mode.
  \param width Width of the image for the given camera capture format and mode.

  \param height Height of the image for the given camera capture format and
  mode.

  \param image_format Coding image format for the given camera capture
  format and mode.

  \exception otherError If camera mode (see setMode()) and the image format
  (see setFormat()) are incompatible.

  \sa setFormat(), setMode(), getWidth(), getHeight(), setCamera()
*/
void
vp1394Grabber::getImageCharacteristics(int _format, int _mode,
				       int &_width, int &_height,
				       ImageFormatEnum &_image_format)
{
  switch(_format)
  {
  case FORMAT_VGA_NONCOMPRESSED:
    switch(_mode)
    {
    case MODE_160x120_YUV444:
      _width = 160; _height = 120;
      _image_format = YUV444;
      break;
    case MODE_320x240_YUV422:
      _width = 320; _height = 240;
      _image_format = YUV422;
      break;
    case MODE_640x480_YUV411:
      _width = 640; _height = 480;
      _image_format = YUV411;
      break;
    case MODE_640x480_YUV422:
      _width = 640; _height = 480;
      _image_format = YUV422;
      break;
    case MODE_640x480_RGB:
      _width = 640; _height = 480;
      _image_format = RGB;
      break;
    case MODE_640x480_MONO:
      _width = 640; _height = 480;
      _image_format = MONO;
      break;
    case MODE_640x480_MONO16:
      _width = 640; _height = 480;
      _image_format = MONO;
      break;
    default:
      close();
      vpERROR_TRACE("Error: camera image format and camera mode are uncompatible...\n");
      vpERROR_TRACE("format: %d and mode: %d\n", _format, _mode);
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Wrong mode for format 0") );
      break;
    }
    break;
  case FORMAT_SVGA_NONCOMPRESSED_1:
    switch(_mode)
    {
    case MODE_800x600_YUV422:
      _width = 800; _height = 600;
      _image_format = YUV422;
      break;
    case MODE_800x600_RGB:
      _width = 800; _height = 600;
      _image_format = RGB;
      break;
    case MODE_800x600_MONO:
      _width = 800; _height = 600;
      _image_format = MONO;
      break;
    case MODE_800x600_MONO16:
      _width = 800; _height = 600;
      _image_format = MONO16;
      break;
    case MODE_1024x768_YUV422:
      _width = 1024; _height = 768;
      _image_format = YUV422;
      break;
    case MODE_1024x768_RGB:
      _width = 1024; _height = 768;
      _image_format = RGB;
      break;
    case MODE_1024x768_MONO:
      _width = 1024; _height = 768;
      _image_format = MONO;
      break;
    case MODE_1024x768_MONO16:
      _width = 1024; _height = 768;
      _image_format = MONO16;
      break;
    default:
      close();
      vpERROR_TRACE("Error: camera image format and camera mode are uncompatible...\n");
      vpERROR_TRACE("format: %d and mode: %d\n", _format, _mode);
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Wrong mode for format 1") );
      break;
    }
    break;
  case FORMAT_SVGA_NONCOMPRESSED_2:
    switch(_mode)
    {
    case MODE_1280x960_YUV422:
      _width = 1280; _height = 960;
      _image_format = YUV422;
      break;
    case MODE_1280x960_RGB:
      _width = 1280; _height = 960;
      _image_format = RGB;
      break;
    case MODE_1280x960_MONO:
      _width = 1280; _height = 960;
      _image_format = MONO;
      break;
   case MODE_1280x960_MONO16:
      _width = 1280; _height = 960;
      _image_format = MONO16;
      break;
    case MODE_1600x1200_YUV422:
      _width = 1600; _height = 1200;
      _image_format = YUV422;
      break;
    case MODE_1600x1200_RGB:
      _width = 1600; _height = 1200;
      _image_format = RGB;
      break;
    case MODE_1600x1200_MONO:
      _width = 1600; _height = 1200;
      _image_format = MONO;
      break;
    case MODE_1600x1200_MONO16:
      _width = 1600; _height = 1200;
      _image_format = MONO16;
     break;
    default:
      close();
      vpERROR_TRACE("Error: camera image format and camera mode are uncompatible...\n");
      vpERROR_TRACE("format: %d and mode: %d\n", _format, _mode);
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Wrong mode for format 2") );
      break;
    }
    break;
  case FORMAT_SCALABLE_IMAGE_SIZE:
#if 1
    switch(_mode)
    {
    case MODE_FORMAT7_0:
      _width = 656; _height = 492;
      _image_format = YUV422;
     break;

    case MODE_FORMAT7_1:
      _width = 328; _height = 492;
      _image_format = MONO;
     break;
    case MODE_FORMAT7_2:
      _width = 656; _height = 244;
      _image_format = MONO;
     break;
    case MODE_FORMAT7_3:
    case MODE_FORMAT7_4:
    case MODE_FORMAT7_5:
    case MODE_FORMAT7_6:
    case MODE_FORMAT7_7:
      _width = 656; _height = 244;
      _image_format = MONO;
     break;
    }


#else
    // Did not work. Even if subsampling is activated in MODE_FORMAT7_1, image
    // max size is the size of the image without considering the subsampling
    switch(_mode)
    {
    case MODE_FORMAT7_0:
      //      _width = 656; _height = 492;
      _image_format = YUV422;
     break;

    case MODE_FORMAT7_1:
      //      _width = 328; _height = 492;
      _image_format = MONO;
     break;
    case MODE_FORMAT7_2:
      //      _width = 656; _height = 244;
      _image_format = MONO;
     break;
    }

    // In format 7 we query set the image size to the maximal image size
    if (dc1394_query_format7_max_image_size(handles[camera],
					    cameras[camera].node,
					    _mode,
					    &_width,
					    &_height) != DC1394_SUCCESS) {
      close();
      vpERROR_TRACE("Unable to get maximal image size for format 7\n");
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Unable to get maximal image size for format 7 ") );
    }
    cout << "max width=" << _width << " height: " << _height << endl;
#endif

    break;
  default:
    close();
    vpERROR_TRACE("Error: camera image format and camera mode are uncompatible...\n");
    vpERROR_TRACE("format: %d and mode: %d\n", _format, _mode);
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Wrong format") );
    break;
  }
}


/*!

  Captures a frame from the given camera using DMA (direct memory acces). Two
  policies are available, either this fonction waits for a frame (waiting
  mode), either it returns if no frame is available (polling mode).

  After you have finished with the frame, you must return the buffer to the
  pool by calling dmaDoneWithBuffer().

  \param waiting Capture mode; true if you want to wait for an available image
  (waiting mode), false to activate the polling mode.

  \return NULL if no frame is available, the address of the image buffer
  otherwise.

  \exception otherError If no frame is available.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa dmaDoneWithBuffer(), getNumCameras(), setCamera()
*/

int*
vp1394Grabber::dmaCapture(bool waiting)
{

  if (camera >= num_cameras) {
    cout << "The required camera is not present..."
	 << endl;
    return NULL;
  }

  if ( handle_created == true && camera_found == true) {
    if (waiting) {
      if (num_cameras == 1) {
	// Only one camera on the bus
	if (dc1394_dma_single_capture(&cameras[camera]) != DC1394_SUCCESS) {
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "No frame is available...") );
	  return NULL;
	}
      }
      else {
	// More than one camera on the bus.
	if (dc1394_dma_multi_capture(cameras, num_cameras) != DC1394_SUCCESS) {
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "No frame is available...") );
	  return NULL;
	}
	//	cout << "-";
      }
    }
    else {
      if (num_cameras == 1) {
	// Only one camera on the bus
	if (dc1394_dma_single_capture_poll(&cameras[camera]) != DC1394_SUCCESS) {
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "No frame is available...") );
	  return NULL;
	}
      }
      else {
	// More than one camera on the bus.
	if (dc1394_dma_multi_capture_poll(cameras, num_cameras)
	    != DC1394_SUCCESS) {
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "No frame is available...") );
	  return NULL;
	}

      }
    }
  }

  return cameras[camera].capture_buffer;
}

/*!

  Return the buffer to the pool for the given camera. This allows the driver to
  use the buffer previously handed to the user.

  \exception settingError If the required camera is not present.
  \exception otherError If can't stop the dma access.

  \warning Has to be called after open() to be sure that a camera is detected.

  \sa dmaCapture(), setCamera()

*/
void
vp1394Grabber::dmaDoneWithBuffer()
{

  if (camera >= num_cameras) {
    close();
    vpERROR_TRACE("The required camera is not present");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "The required camera is not present") );
  }

  if ( handle_created == true && camera_found == true) {
    if (dc1394_dma_done_with_buffer(&cameras[camera]) != DC1394_SUCCESS) {
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Can't done the dma") );
    }
  }
}



/*!

  Close the link between the camera and the acquisition program

*/
void
vp1394Grabber::close()
{
  if (iso_transmission_started  == true) {

    stopIsoTransmission();

    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_dma_unlisten( handles[i], &cameras[i] );
    iso_transmission_started = false;
  }
  if ((camera_found  == true) && (dma_started == true)) {
    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_dma_release_camera( handles[i], &cameras[i]);
    camera_found = false;
    dma_started  = false;
  }
  if (handle_created == true) {
    for (unsigned int i=0; i < num_cameras; i++)
      dc1394_destroy_handle(handles[i]);
    handle_created = false;
  }

  if (handles != NULL)      { delete [] handles;      handles = NULL;      }
  if (cameras != NULL)      { delete [] cameras;      cameras = NULL;      }
  if (cam_count != NULL)    { delete [] cam_count;    cam_count = NULL;    }
  if (pformat != NULL)      { delete [] pformat;      pformat = NULL;       }
  if (pmode != NULL)        { delete [] pmode;        pmode = NULL;         }
  if (pframerate != NULL)   { delete [] pframerate;   pframerate = NULL;    }
  if (width != NULL)        { delete [] width;        width = NULL;        }
  if (height != NULL)       { delete [] height;       height = NULL;       }
  if (image_format != NULL) { delete [] image_format; image_format = NULL; }

}
/*!
  Start the transmission of the images for all the cameras on the bus.

  \exception otherError If Unable to start camera iso transmission.

  \sa stopIsoTransmission()

*/
void vp1394Grabber::startIsoTransmission()
{
  if ( handle_created == true && camera_found == true)  {

    for (unsigned int i = 0; i < num_cameras; i ++) {
      if (dc1394_start_iso_transmission(handles[i],
					cameras[i].node) !=DC1394_SUCCESS) {
	close();
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Unable to start camera iso transmission") );
      }
    }
    iso_transmission_started = true;
  }
}

/*!

  Stop the Iso transmission for all the cameras on the bus.

  \return true on success, false otherwise.

  \sa StartIsoTransmission()

*/
void vp1394Grabber::stopIsoTransmission()
{

  if (iso_transmission_started == true)  {
    if (handle_created == true && camera_found == true) {
      for (unsigned int i = 0; i < num_cameras; i ++) {
	if (dc1394_stop_iso_transmission(handles[i],
					 cameras[i].node) != DC1394_SUCCESS) {
	  close();
	  throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
					 "Can't stop the camera") );
	}

      }
      iso_transmission_started = false;
    }
  }
}


#endif
