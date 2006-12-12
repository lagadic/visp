/****************************************************************************
 *
 * $Id: vp1394TwoGrabber.cpp,v 1.3 2006-12-12 17:04:34 fspindle Exp $
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
  \file vp1394TwoGrabber.cpp
  \brief member functions for firewire cameras
  \ingroup libdevice
*/
#include <iostream>

#include <visp/vpConfig.h>

/*
 * Interface with libdc1394 2.x
 */
#if defined(VISP_HAVE_DC1394_2)

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

const int vp1394TwoGrabber::NUM_BUFFERS = 8; /*!< Number of buffers */

const char * vp1394TwoGrabber::strVideoMode[DC1394_VIDEO_MODE_NUM]= {
  "MODE_160x120_YUV444",
  "MODE_320x240_YUV422",
  "MODE_640x480_YUV411",
  "MODE_640x480_YUV422",
  "MODE_640x480_RGB8",
  "MODE_640x480_MONO8",
  "MODE_640x480_MONO16",
  "MODE_800x600_YUV422",
  "MODE_800x600_RGB8",
  "MODE_800x600_MONO8",
  "MODE_1024x768_YUV422",
  "MODE_1024x768_RGB8",
  "MODE_1024x768_MONO8",
  "MODE_800x600_MONO16",
  "MODE_1024x768_MONO16",
  "MODE_1280x960_YUV422",
  "MODE_1280x960_RGB8",
  "MODE_1280x960_MONO8",
  "MODE_1600x1200_YUV422",
  "MODE_1600x1200_RGB8",
  "MODE_1600x1200_MONO8",
  "MODE_1280x960_MONO16",
  "MODE_1600x1200_MONO16",
  "MODE_EXIF",
  "MODE_FORMAT7_0",
  "MODE_FORMAT7_1",
  "MODE_FORMAT7_2",
  "MODE_FORMAT7_3",
  "MODE_FORMAT7_4",
  "MODE_FORMAT7_5",
  "MODE_FORMAT7_6",
  "MODE_FORMAT7_7"
};

const char * vp1394TwoGrabber::strFramerate[DC1394_FRAMERATE_NUM]= {
  "FRAMERATE_1_875",
  "FRAMERATE_3_75",
  "FRAMERATE_7_5",
  "FRAMERATE_15",
  "FRAMERATE_30",
  "FRAMERATE_60",
  "FRAMERATE_120",
  "FRAMERATE_240"
};



/*!
  Default constructor.

  By default:
  - the camera is the first found on the bus.

  Current camera settings can be changed using setCamera() to select the active
  camera on the bus and than setVideoMode() or setFramerate() to fix the active
  camera settings. The list of supported video modes and framerates is
  available using respectively getVideoModeSupported() and
  getFramerateSupported().

  \code
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
  g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_15);
  while(1)
    g.acquire(I);
  \endcode

  \sa setCamera(), setVideoMode(), setFramerate()

*/
vp1394TwoGrabber::vp1394TwoGrabber( )
{
  // protected members
  ncols = nrows = 0;

  // private members
  num_cameras = 0;
  cameras = NULL;
  camera_id = 0;
  verbose = false;//true;

  open();
  getNumCameras(num_cameras);

  camInUse = new bool [num_cameras];
  for (int i=0; i < num_cameras; i ++)
    camInUse[i] = false;

  setCamera(0);
  setCapture(DC1394_OFF);

  init = true;
}

/*!

  Destructor.

  Close the firewire grabber.

  \sa close()

*/
vp1394TwoGrabber::~vp1394TwoGrabber()
{
  close();
}


/*!

  If multiples cameras are connected on the bus, select the camero to dial
  with.

  \param camera_id : A camera identifier. The value must be comprised
  between 0 (the first camera) and the number of cameras found on the
  bus and returned by getNumCameras() minus 1. If two cameras are
  connected on the bus, setting \e camera to one allows to communicate
  with the second one.

  \exception vpFrameGrabberException::settingError : If the required camera is
  not reachable.

  Here an example of single capture from the last camera found on the bus:
  \code
  unsigned ncameras; // Number of cameras on the bus
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.getNumCameras(ncameras);
  g.setCamera(ncameras-1); // To dial with the last camera on the bus
  while(1)
    g.acquire(I);// I contains the frame captured by the last camera on the bus
  \endcode

  Here an example of multi camera capture:
  \code
  unsigned ncameras; // Number of cameras on the bus
  vp1394TwoGrabber g;
  g.getNumCameras(ncameras);
  vpImage<unsigned char> *I = new vpImage<unsigned char> [ncameras];

  // If the first camera supports vpVIDEO_MODE_640x480_YUV422 video mode
  g.setCamera(0);
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV422);

  // If the second camera support 30 fps acquisition
  g.setCamera(1);
  g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);

  while(1) {
    for (unsigned camera=0; camera < ncameras; camera ++) {
      g.setCamera(camera);
      g.acquire(I[camera]);
    }
  }
  delete [] I;
  \endcode


  \sa setFormat(), setVideoMode(), setFramerate(), getNumCameras()

*/
void
vp1394TwoGrabber::setCamera(unsigned camera_id)
{
  if (camera_id >= num_cameras) {
    close();
    vpERROR_TRACE("The required camera %u is not present", camera_id);
    vpERROR_TRACE("Only %u camera on the bus.", num_cameras);
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "The required camera is not present") );
  }

  this->camera_id =  camera_id;

  camInUse[camera_id] = true;

  // create a pointer to the working camera
  camera = cameras[camera_id];

  dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);

}

/*!

  Get the active camera identifier on the bus.

  \param camera_id : The active camera identifier. The value is
  comprised between 0 (the first camera) and the number of cameras
  found on the bus returned by getNumCameras() minus 1.

  \exception vpFrameGrabberException::initializationError : If no
  camera is found.

  \sa setCamera(), getNumCameras()

*/
void
vp1394TwoGrabber::getCamera(unsigned &camera_id)
{
  if (! num_cameras) {
    camera_id = this->camera_id;
  }
  else {
    close();
    vpERROR_TRACE("No cameras found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No cameras found") );
  }
}

/*!

  Return the number of cameras connected on the bus.

  \param cameras : The number of cameras found on the bus.


*/
void
vp1394TwoGrabber::getNumCameras(unsigned &cameras)
{
  if (! num_cameras) {
    vpCTRACE << "No camera found..."<< endl;
    cameras = 0;
  }

  cameras = num_cameras;
}

/*!

  Set the camera video capture mode.

  \param videomode : The camera video capture mode. The current camera mode is
  given by getVideoMode(). The camera supported modes are given by
  getVideoModeSupported().

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't set
  the video mode.

  \sa getVideoMode(), getVideoModeSupported(), setCamera()

*/
void
vp1394TwoGrabber::setVideoMode(vp1394TwoVideoMode videomode)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  if (dc1394_video_set_mode(camera, (dc1394video_mode_t) videomode) != DC1394_SUCCESS) {

    close();
    vpERROR_TRACE("Can't set video mode");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Can't set video mode") );
    }

}

/*!

  Query the actual capture video mode of the active camera. All
  the active camera supported modes are given by getVideoModeSupported().

  \param videomode : The camera capture video mode.

  \exception vpFrameGrabberException::initializationError : If the
  required camera is not present.

  \exception vpFrameGrabberException::settingError : If we can't get
  the camera actual video mode.

  \sa setVideoMode(), getVideoModeSupported(), setCamera()

*/
void
vp1394TwoGrabber::getVideoMode(vp1394TwoVideoMode & videomode)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  dc1394video_mode_t _videomode;
  if (dc1394_video_get_mode(camera, &_videomode) != DC1394_SUCCESS) {

    close();
    vpERROR_TRACE("Can't get current video mode");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Can't get current video mode") );
  }
  videomode = (vp1394TwoVideoMode) _videomode;

}



/*!

  Query the available active camera video modes.


  \param videomodes : The list of supported camera video modes.

  \return The number of supported camera modes, 0 if an error occurs.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't get
  video modes.

  \sa setVideoMode(), getVideoMode(), getCamera()
*/
int
vp1394TwoGrabber::getVideoModeSupported(vpList<vp1394TwoVideoMode> & videomodes)
{
  int nb = 0; // Number of supported modes

  // Refresh the list of supported modes
  videomodes.kill();

  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }
  dc1394video_modes_t _videomodes;

  // get video modes:
  if (dc1394_video_get_supported_modes(camera, &_videomodes)!=DC1394_SUCCESS) {

    close();
    vpERROR_TRACE("Can't get video modes");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Can't get video modes") );
  }

  // parse the video modes to add in the list
  for (int i=0; i < _videomodes.num; i++) {
    vp1394TwoVideoMode _mode = (vp1394TwoVideoMode) _videomodes.modes[i];
    videomodes.addRight( _mode );
  }

  // return the number of available video modes
  return _videomodes.num;
}

/*!

  Set the active camera framerate.

  \param fps : The camera framerate. The current framerate of the camera is
  given by getFramerate(). The camera supported framerates are given by
  getFramerateSupported().

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't set
  the framerate.


  \sa getFramerate(), getFramerateSupported() , setCamera()

*/
void
vp1394TwoGrabber::setFramerate(vp1394TwoFramerate fps)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }
  if (dc1394_video_set_framerate(camera, (dc1394framerate_t) fps) != DC1394_SUCCESS) {

    close();
    vpERROR_TRACE("Can't set framerate");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Can't set framerate") );
  }
}

/*!

  Query the actual camera framerate of the active camera. The camera supported
  framerates are given by getFramerateSupported().

  \param fps : The camera capture framerate.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't get
  the framerate.

  \sa setFramerate(), getFramerateSupported(), setCamera()

*/
void
vp1394TwoGrabber::getFramerate(vp1394TwoFramerate & fps)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }
  dc1394framerate_t _fps;
  if (dc1394_video_get_framerate(camera, &_fps) != DC1394_SUCCESS) {

    close();
    vpERROR_TRACE("Can't get current framerate");
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Can't get current framerate") );
  }
  fps = (vp1394TwoFramerate) _fps;

}

/*!

  Query the available framerates for the given camera image mode (see
  file dc1394/control.h). No framerate is associated to the following
  camera modes :

  - vp1394TwoGrabber::vpVIDEO_MODE_EXIF (format 6),
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_0 (format 7):
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_1 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_2 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_3 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_4 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_5 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_6 (format 7)
  - vp1394TwoGrabber::vpVIDEO_MODE_FORMAT7_7 (format 7)

  \param mode : Camera video mode.

  \param fps : The list of supported camera framerates for the given camera
  video mode.

  \return The number of supported framerates, 0 if no framerate is available.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't get
  the supported framerates.

  \sa getVideoModeSupported(), setCamera()
*/
int
vp1394TwoGrabber::getFramerateSupported(vp1394TwoVideoMode mode,
					vpList<vp1394TwoFramerate> & fps)
{
  int nb = 0; // Number of supported framerates

  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  // Refresh the list of supported framerates
  fps.kill();

  switch(mode) {
    // Framerate not avalaible for:
    //  - vpVIDEO_MODE_EXIF ie Format_6
    //  - vpVIDEO_MODE_FORMAT7... ie the Format_7
  case vpVIDEO_MODE_EXIF:
  case vpVIDEO_MODE_FORMAT7_0:
  case vpVIDEO_MODE_FORMAT7_1:
  case vpVIDEO_MODE_FORMAT7_2:
  case vpVIDEO_MODE_FORMAT7_3:
  case vpVIDEO_MODE_FORMAT7_4:
  case vpVIDEO_MODE_FORMAT7_5:
  case vpVIDEO_MODE_FORMAT7_6:
  case vpVIDEO_MODE_FORMAT7_7:
    return 0;
    break;
  default:
    {
      dc1394framerates_t _fps;
      if (dc1394_video_get_supported_framerates(camera,
						(dc1394video_mode_t)mode,
						&_fps) != DC1394_SUCCESS) {
	close();
	vpERROR_TRACE("Could not query supported frametates for mode %d\n",
		      mode);
	throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				       "Could not query supported framerates") );
      }
      if (_fps.num == 0)
	return 0;

      for (int i = 0; i < _fps.num; i ++)
	fps.addRight((vp1394TwoFramerate)_fps.framerates[i]);

      return _fps.num;
    }
    break;
  }
}

/*!

  Open ohci and asign handle to it and get the camera nodes and
  describe them as we find them.

  \param camera_id : The camera identifier to dial with. By default,
  we use the first camera found on the bus. For the first camera on
  the bus, camera_id is 0.

  \exception initializationError : If a raw1394 handle can't be aquired,
  or if no camera is found.

  \sa close()
*/
void
vp1394TwoGrabber::open()
{
  // Find cameras
  int err = dc1394_find_cameras(&cameras, &num_cameras);
  if (err!=DC1394_SUCCESS && err != DC1394_NO_CAMERA) {
    close();
    vpERROR_TRACE("Unable to look for cameras\n\n"
             "Please check \n"
	     "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
	     "  - if you have read/write access to /dev/raw1394\n\n");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Unable to look for cameras") );

  }
  /*-----------------------------------------------------------------------
   *  get the camera nodes and describe them as we find them
   *-----------------------------------------------------------------------*/
  if (num_cameras == 0) {
    close();
    vpERROR_TRACE("No cameras found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No cameras found") );
  }

  if (camera_id >= num_cameras) {
    // Bad camera id
    close();
    vpERROR_TRACE("Bad camera id: %u", camera_id);
    vpERROR_TRACE("Only %u camera on the bus.", num_cameras);
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Bad camera id") );
  }

  if (verbose) {
    cout << "------ Bus information ------" << endl;
    cout << "Number of camera(s) on the bus : " << num_cameras <<endl;
    cout << "-----------------------------" << endl;
  }
}

/*!

  Close the firewire grabber.

  Stops the capture and the iso transmission of the active cameras and than
  release the cameras.

*/
void
vp1394TwoGrabber::close()
{
  if (num_cameras) {
    for (unsigned i = 0; i < num_cameras;i++) {
      if (camInUse[i]) {
	camera = cameras[i];

	setCapture(DC1394_OFF);
	setTransmission(DC1394_OFF);
	dc1394_cleanup_iso_channels_and_bandwidth(camera);

	dc1394_free_camera(camera);
      }
    }
    free(cameras);

    delete [] camInUse;
  }

  camInUse = NULL;
  num_cameras = 0;

  init = false;

}

/*!

  Setup camera capture using dma.

  \param _switch : Camera capture switch:
  - DC1394_ON to start dma capture,
  - DC1394_OFF to stop camera capture.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't set
  dma capture.

  \sa setVideoMode(), setFramerate()
*/
void
vp1394TwoGrabber::setCapture(dc1394switch_t _switch)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  if (_switch == DC1394_ON) {
    if (dc1394_capture_setup(camera, NUM_BUFFERS) != DC1394_SUCCESS) {
      vpERROR_TRACE("Unable to setup camera capture-\n"
		    "make sure that the video mode and framerate are "
		    "supported by your camera.\n");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Could not setup dma capture") );
    }
  }
  else { // _switch == DC1394_OFF
    dc1394error_t code = dc1394_capture_stop(camera);

    if (code != DC1394_SUCCESS && code != DC1394_CAPTURE_IS_NOT_SET) {
      vpERROR_TRACE("Unable to stop camera capture\n");
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				     "Could not setup dma capture") );
    }

  }

}


/*!

  Setup camera transmission.

  \param _switch : Transmission switch:
  - DC1394_ON to start iso transmission,
  - DC1394_OFF to stop iso transmission.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::settingError : If we can't set
  the video mode.

  \sa getTransmission()
*/
void
vp1394TwoGrabber::setTransmission(dc1394switch_t _switch)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  if (dc1394_video_set_transmission(camera, _switch) != DC1394_SUCCESS) {
    vpERROR_TRACE("Unable to setup camera capture-\n"
		  "make sure that the video mode and framerate are "
		  "supported by your camera.\n");
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Could not setup dma capture") );
  }

}


/*!
  Exist only for compatibility with other grabbing devices.

  Call acquire(vpImage<unsigned char> &I)

  \param I : Image data structure (8 bits image)

  \sa acquire(vpImage<unsigned char> &I)

*/
void
vp1394TwoGrabber::open(vpImage<unsigned char> &I)
{
  acquire(I);
}

/*!
  Exist only for compatibility with other grabbing devices.

  Call acquire(vpImage<vpRGBa> &I)

  \param I : Image data structure (RGBa format)

  \sa acquire(vpImage<vpRGBa> &I)

*/
void
vp1394TwoGrabber::open(vpImage<vpRGBa> &I)
{
  acquire(I);
}

/*!

  Get an image from the active camera frame buffer. This buffer neads to be
  released by enqueue().

  \return Pointer to the libdc1394-2.x image data structure.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \code
  vp1394TwoGrabber g;
  dc1394video_frame_t *frame;
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
  g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_15);
  while(1) {
    frame = g.dequeue();
    // Current image is now in frame structure
    g.enqueue(frame);
  }

  \endcode

  \sa enqueue()
*/
dc1394video_frame_t *
vp1394TwoGrabber::dequeue()
{

  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  if (camera->is_iso_on == DC1394_OFF) {
    setTransmission(DC1394_ON);
  }

  // Start dma capture if halted
  if (! camera->capture_is_set)
    setCapture(DC1394_ON);

  dc1394video_frame_t *frame;

  if (dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame)
      !=DC1394_SUCCESS) {
    vpERROR_TRACE ("Error: Failed to capture from camera %d\n", camera_id);
  }

  unsigned width  = frame->size[0];
  unsigned height = frame->size[1];
  unsigned size   = width * height;
}

/*!
  Release the frame buffer used by the active camera.

  \param frame : Pointer to the libdc1394-2.x image data structure.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \sa dequeue()
*/
void
vp1394TwoGrabber::enqueue(dc1394video_frame_t *frame)
{

  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  if (frame)
    dc1394_capture_enqueue(camera, frame);

}

/*!
  Acquire a grey level image from the active camera.

  \param I : Image data structure (8 bits image).

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::otherError : If format
  conversion to return a 8 bits image is not implemented.

  \sa setCamera(), setVideoMode(), setFramerate(), dequeue(), enqueue()
*/
void
vp1394TwoGrabber::acquire(vpImage<unsigned char> &I)
{
  dc1394video_frame_t *frame;
  unsigned width, height, size;

  frame = dequeue();

  getWidth(width);
  getHeight(height);

  size = width * height;

  if ((I.getCols() != width)||(I.getRows() != height))
    I.resize(height, width);

  switch(frame->color_coding) {
  case DC1394_COLOR_CODING_MONO8:
    memcpy(I.bitmap, (unsigned char *) frame->image,
	   size*sizeof(unsigned char));
    break;

  case DC1394_COLOR_CODING_YUV411:
    vpImageConvert::YUV411ToGrey( (unsigned char *) frame->image,
				  I.bitmap, size);
    break;

  case DC1394_COLOR_CODING_YUV422:
    vpImageConvert::YUV422ToGrey( (unsigned char *) frame->image,
				  I.bitmap, size);
    break;

  case DC1394_COLOR_CODING_RGB8:
    vpImageConvert::RGBToGrey((unsigned char *) frame->image, I.bitmap, size);
    break;


  default:
    close();
    vpERROR_TRACE("Format conversion not implemented. Acquisition failed.");
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Format conversion not implemented. "
				   "Acquisition failed.") );
    break;
  };

  enqueue(frame);
}

/*!
  Acquire a color image from the active camera.

  \param I : Image data structure (32 bits RGBa image).

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \exception vpFrameGrabberException::otherError : If format
  conversion to return a RGBa bits image is not implemented.

  \sa setCamera(), setVideoMode(), setFramerate(), dequeue(), enqueue()
*/
void
vp1394TwoGrabber::acquire(vpImage<vpRGBa> &I)
{
  dc1394video_frame_t *frame;
  unsigned width, height, size;

  frame = dequeue();

  getWidth(width);
  getHeight(height);

  size = width * height;

  if ((I.getCols() != width)||(I.getRows() != height))
    I.resize(height, width);

  switch(frame->color_coding) {
  case DC1394_COLOR_CODING_MONO8:
    vpImageConvert::GreyToRGBa((unsigned char *) frame->image,
			       (unsigned char *) I.bitmap, size);
    break;

  case DC1394_COLOR_CODING_YUV411:
    vpImageConvert::YUV411ToRGBa( (unsigned char *) frame->image,
				  (unsigned char *) I.bitmap, size);
    break;

  case DC1394_COLOR_CODING_YUV422:
    vpImageConvert::YUV422ToRGBa( (unsigned char *) frame->image,
				  (unsigned char *) I.bitmap, size);
    break;

  case DC1394_COLOR_CODING_RGB8:
    vpImageConvert::RGBToRGBa((unsigned char *) frame->image,
			      (unsigned char *) I.bitmap, size);
    break;


  default:
    close();
    vpERROR_TRACE("Format conversion not implemented. Acquisition failed.");
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Format conversion not implemented. "
				   "Acquisition failed.") );
    break;
  };

  enqueue(frame);

}

/*!

  Get the image width. It depends on the camera video mode setVideoMode(). The
  image size is only available after a call to open() or acquire().

  \param width : The image width, zero if the required camera is not avalaible.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \warning Has to be called after open() or acquire() to be sure that camera
  settings are send to the camera.

  \sa getHeight(), open(), acquire()

*/
void vp1394TwoGrabber::getWidth(unsigned &width)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  width = ncols;

}

/*!

  Get the image height. It depends on the camera vide mode
  setVideoMode(). The image size is only available after a call to
  open() or acquire().

  \param height : The image width.

  \exception vpFrameGrabberException::initializationError : If no
  camera found on the bus.

  \warning Has to be called after open() or acquire() to be sure that camera
  settings are send to the camera.

  \sa getWidth()

*/
void vp1394TwoGrabber::getHeight(unsigned &height)
{
  if (! num_cameras) {
    close();
    vpERROR_TRACE("No camera found");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "No camera found") );
  }

  height = ncols;
}

/*!
  Display camera information for the active camera.

  \sa setCamera()
*/
void
vp1394TwoGrabber::printCameraInfo()
{
  cout << "----------------------------------------------------------"
       << endl
       << "-----            Information for camera " << camera_id
       << "            -----" << endl
       << "----------------------------------------------------------" << endl;

  dc1394_print_camera_info( camera);

  dc1394featureset_t features;
  if(dc1394_get_camera_feature_set(camera, &features) != DC1394_SUCCESS) {
    close();
    vpERROR_TRACE("unable to get feature set for camera %d\n", camera_id);
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Cannot get camera features") );

  } else {
    dc1394_print_feature_set(&features);
  }
  cout << "----------------------------------------------------------" << endl;
}

/*!

  Converts the video mode identifier into a string containing the description
  of the mode.

  \param videomode : The camera capture video mode.

  \return A string describing the mode, an empty string if the mode is not
  supported.

  \sa string2videoMode()
*/
string vp1394TwoGrabber::videoMode2string(vp1394TwoVideoMode videomode)
{
  string _str = "";
  dc1394video_mode_t _videomode = (dc1394video_mode_t) videomode;

  if ((_videomode >= DC1394_VIDEO_MODE_MIN)
      && (_videomode <= DC1394_VIDEO_MODE_MAX)) {
    switch (_videomode) {
    case DC1394_VIDEO_MODE_160x120_YUV444: _str = strVideoMode[0]; break;
    case DC1394_VIDEO_MODE_320x240_YUV422: _str = strVideoMode[1]; break;
    case DC1394_VIDEO_MODE_640x480_YUV411: _str = strVideoMode[2]; break;
    case DC1394_VIDEO_MODE_640x480_YUV422: _str = strVideoMode[3]; break;
    case DC1394_VIDEO_MODE_640x480_RGB8: _str = strVideoMode[4]; break;
    case DC1394_VIDEO_MODE_640x480_MONO8: _str = strVideoMode[5]; break;
    case DC1394_VIDEO_MODE_640x480_MONO16: _str = strVideoMode[6]; break;
    case DC1394_VIDEO_MODE_800x600_YUV422: _str = strVideoMode[7]; break;
    case DC1394_VIDEO_MODE_800x600_RGB8: _str = strVideoMode[8]; break;
    case DC1394_VIDEO_MODE_800x600_MONO8: _str = strVideoMode[9]; break;
    case DC1394_VIDEO_MODE_1024x768_YUV422: _str = strVideoMode[10]; break;
    case DC1394_VIDEO_MODE_1024x768_RGB8: _str = strVideoMode[11]; break;
    case DC1394_VIDEO_MODE_1024x768_MONO8: _str = strVideoMode[12]; break;
    case DC1394_VIDEO_MODE_800x600_MONO16: _str = strVideoMode[13]; break;
    case DC1394_VIDEO_MODE_1024x768_MONO16: _str = strVideoMode[14]; break;
    case DC1394_VIDEO_MODE_1280x960_YUV422: _str = strVideoMode[15]; break;
    case DC1394_VIDEO_MODE_1280x960_RGB8: _str = strVideoMode[16]; break;
    case DC1394_VIDEO_MODE_1280x960_MONO8: _str = strVideoMode[17]; break;
    case DC1394_VIDEO_MODE_1600x1200_YUV422: _str = strVideoMode[17]; break;
    case DC1394_VIDEO_MODE_1600x1200_RGB8: _str = strVideoMode[19]; break;
    case DC1394_VIDEO_MODE_1600x1200_MONO8: _str = strVideoMode[20]; break;
    case DC1394_VIDEO_MODE_1280x960_MONO16: _str = strVideoMode[21]; break;
    case DC1394_VIDEO_MODE_1600x1200_MONO16: _str = strVideoMode[22]; break;
    case DC1394_VIDEO_MODE_EXIF: _str = strVideoMode[23]; break;
    case DC1394_VIDEO_MODE_FORMAT7_0: _str = strVideoMode[24]; break;
    case DC1394_VIDEO_MODE_FORMAT7_1: _str = strVideoMode[25]; break;
    case DC1394_VIDEO_MODE_FORMAT7_2: _str = strVideoMode[26]; break;
    case DC1394_VIDEO_MODE_FORMAT7_3: _str = strVideoMode[27]; break;
    case DC1394_VIDEO_MODE_FORMAT7_4: _str = strVideoMode[28]; break;
    case DC1394_VIDEO_MODE_FORMAT7_5: _str = strVideoMode[29]; break;
    case DC1394_VIDEO_MODE_FORMAT7_6: _str = strVideoMode[30]; break;
    case DC1394_VIDEO_MODE_FORMAT7_7: _str = strVideoMode[31]; break;

    default:
      vpCERROR << "The video mode " << videomode
	       << " is not supported by the camera" << endl;
      break;
    }
  }
  else {
    vpCERROR << "The video mode " << videomode
	 << " is not supported by the camera" << endl;
  }

  return _str;
}

/*!

  Converts the framerate identifier into a string containing the description
  of the framerate.

  \param fps : The camera capture framerate.

  \return A string describing the framerate, an empty string if the framerate
  is not supported.

  \sa string2framerate()
*/
string vp1394TwoGrabber::framerate2string(vp1394TwoFramerate fps)
{
  string _str = "";
  dc1394framerate_t _fps = (dc1394framerate_t) fps;

  if ((_fps >= DC1394_FRAMERATE_MIN)
      && (_fps <= DC1394_FRAMERATE_MAX)) {
    switch (_fps) {
    case DC1394_FRAMERATE_1_875: _str = strFramerate[0]; break;
    case DC1394_FRAMERATE_3_75: _str = strFramerate[1]; break;
    case DC1394_FRAMERATE_7_5: _str = strFramerate[2]; break;
    case DC1394_FRAMERATE_15: _str = strFramerate[3]; break;
    case DC1394_FRAMERATE_30: _str = strFramerate[4]; break;
    case DC1394_FRAMERATE_60: _str = strFramerate[5]; break;
    case DC1394_FRAMERATE_120: _str = strFramerate[6]; break;
    case DC1394_FRAMERATE_240: _str = strFramerate[7]; break;

    default:
      vpCERROR << "The framerate " << fps
	       << " is not supported by the camera" << endl;
      break;
    }
  }
  else {
    vpCERROR << "The framerate " << fps
	     << " is not supported by the camera" << endl;
  }

  return _str;
}

/*!

  Converts the string containing the description of the vide mode into the
 video mode identifier.

  \param videomode : The string describing the video mode.

  \return The camera capture video mode identifier.

  \exception vpFrameGrabberException::settingError : If the required videomode
  is not valid.

  This method returns 0 if the string does not match to a video mode string.

  \sa videoMode2string()

*/
vp1394TwoGrabber::vp1394TwoVideoMode
vp1394TwoGrabber::string2videoMode(string videomode)
{
  vp1394TwoVideoMode _id;

  for (int i = DC1394_VIDEO_MODE_MIN; i <= DC1394_VIDEO_MODE_MAX; i ++) {
    _id = (vp1394TwoVideoMode) i;
    if (videomode.compare(videoMode2string(_id)) == 0)
      return _id;
  };

  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				 "The required videomode is not valid") );

  return (vp1394TwoVideoMode) 0;
}


/*!

  Converts the string containing the description of the framerate into the
  framerate identifier.

  \param framerate : The string describing the framerate.

  \return The camera capture framerate identifier.

  \exception vpFrameGrabberException::settingError : If the required framerate
  is not valid.

  This method returns 0 if the string does not match to a framerate string.

  \sa framerate2string()

*/
vp1394TwoGrabber::vp1394TwoFramerate
vp1394TwoGrabber::string2framerate(string framerate)
{
  vp1394TwoFramerate _id;

  for (int i = DC1394_FRAMERATE_MIN; i <= DC1394_FRAMERATE_MAX; i ++) {
    _id = (vp1394TwoFramerate) i;
    if (framerate.compare(framerate2string(_id)) == 0)
      return _id;
  };

  throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				 "The required framerate is not valid") );

  return (vp1394TwoFramerate) 0;
}

#endif

