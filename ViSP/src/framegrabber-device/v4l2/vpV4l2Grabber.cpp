/****************************************************************************
 *
 * $Id: vpV4l2Grabber.cpp,v 1.8 2007-01-30 15:25:03 fspindle Exp $
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
 * Framegrabber based on itifg-8.x (Coreco Imaging Technology) driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpV4l2Grabber.cpp
  \brief class for the Video For Linux 2 video device framegrabbing.
  \ingroup libdevice
*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_V4L2

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <errno.h>
#include <iostream>

#include <visp/vpV4l2Grabber.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpImageIo.h>


const int vpV4l2Grabber::DEFAULT_INPUT = 2;
const int vpV4l2Grabber::DEFAULT_SCALE = 2;
const int vpV4l2Grabber::MAX_INPUTS    = 16;
const int vpV4l2Grabber::MAX_NORM      = 16;
const int vpV4l2Grabber::MAX_FORMAT    = 32;
const int vpV4l2Grabber::MAX_CTRL      = 32;
const int vpV4l2Grabber::MAX_BUFFERS   = 32;
const int vpV4l2Grabber::FRAME_SIZE    = 288;

/*!
  Default constructor.

  Setup the Video For Linux Two (V4L2) driver in streaming mode.

  Default settings are:

  - Device name: /dev/video0: To change it use setDevice()

  - Number of ring buffers: 3. To change this value use setNBuffers(). For non
    real-time applications the number of buffers should be set to 1. For
    real-time applications to reach 25 fps or 50 fps a good compromise is to
    set the number of buffers to 3.

  - Framerate acquisition: 25 fps. Use setFramerate() to set 25 fps or 50
    fps. These framerates are reachable only if enought buffers are set.

  - Input board: vpV4l2Grabber::DEFAULT_INPUT. Video input port. Use setInput()
    to change it.

  - Image size acquisition: vpV4l2Grabber::DEFAULT_SCALE. Use either setScale()
    or setWidth() and setHeight to change it.

    \code
    vpImage<unsigned char> I; // Grey level image

    vpV4l2Grabber g;
    g.setInput(2);    // Input 2 on the board
    g.setWidth(768);  // Acquired images are 768 width
    g.setHeight(576); // Acquired images are 576 height
    g.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
    g.open(I);        // Open the grabber

    g.acquire(I);     // Acquire a 768x576 grey image

    \endcode

*/
vpV4l2Grabber::vpV4l2Grabber()
{
  fd        = -1;
  streaming = false;
  verbose   = false;
  field     = 0;
  ncols     = 0;
  nrows     = 0;
  queue     = 0;
  waiton_cpt= 0;
  index_buffer = 0;

  inp       = NULL;
  std       = NULL;
  fmt       = NULL;
  ctl       = NULL;
  buf_v4l2  = NULL;
  buf_me    = NULL;

  setDevice("/dev/video0");
  setNBuffers(3);
  setFramerate(vpV4l2Grabber::framerate_25fps);
  setInput(vpV4l2Grabber::DEFAULT_INPUT);
  setScale(vpV4l2Grabber::DEFAULT_SCALE);

  init = false;
}


/*!
  Constructor.

  Setup the Video For Linux Two (V4L2) driver in streaming mode.

  \param input : Video input port.
  \param scale : Decimation factor.

  Default settings are:

  - Device name: /dev/video0: To change it use setDevice()

  - Number of ring buffers: 3. To change this value use setNBuffers(). For non
    real-time applications the number of buffers should be set to 1. For
    real-time applications to reach 25 fps or 50 fps a good compromise is to
    set the number of buffers to 3.

  - Framerate acquisition: 25 fps. Use setFramerate() to set 25 fps or 50
    fps. These framerates are reachable only if enought buffers are set.

    \code
    vpImage<unsigned char> I; // Grey level image

    vpV4l2Grabber g(1, 2); // Select input 1, and half full size resolution images.
    g.open(I);             // Open the grabber

    g.acquire(I);          // Acquire a 384x288 grey image

    \endcode
*/
vpV4l2Grabber::vpV4l2Grabber(unsigned input, unsigned scale)
{
  fd        = -1;
  streaming = false;
  verbose   = false;
  field     = 0;
  ncols     = 0;
  nrows     = 0;
  queue     = 0;
  waiton_cpt= 0;
  index_buffer = 0;

  inp       = NULL;
  std       = NULL;
  fmt       = NULL;
  ctl       = NULL;
  buf_v4l2  = NULL;
  buf_me    = NULL;

  setDevice("/dev/video0");
  setNBuffers(3);
  setFramerate(vpV4l2Grabber::framerate_25fps);
  setInput(input);
  setScale(scale);

  init = false;
}

/*!
  Constructor.

  Setup the Video For Linux Two (V4L2) driver in streaming mode.

  \param I : Image data structure (grey 8 bits image)
  \param input : Video input port.
  \param scale : Decimation factor.

  Default settings are:

  - Device name: /dev/video0: To change it use setDevice()

  - Number of ring buffers: 3. To change this value use setNBuffers(). For non
    real-time applications the number of buffers should be set to 1. For
    real-time applications to reach 25 fps or 50 fps a good compromise is to
    set the number of buffers to 3.

  - Framerate acquisition: 25 fps. Use setFramerate() to set 25 fps or 50
    fps. These framerates are reachable only if enought buffers are set.

    \code
    vpImage<unsigned char> I; // Grey level image

    vpV4l2Grabber g(I, 1, 2); // Select input 1, and half full size resolution
                              // images and open the grabber

    g.acquire(I);             // Acquire a 384x288 grey image

    \endcode
*/
vpV4l2Grabber::vpV4l2Grabber(vpImage<unsigned char> &I,
			     unsigned input, unsigned scale )
{
  fd        = -1;
  streaming = false;
  verbose   = false;
  field     = 0;
  ncols     = 0;
  nrows     = 0;
  queue     = 0;
  waiton_cpt= 0;
  index_buffer = 0;

  inp       = NULL;
  std       = NULL;
  fmt       = NULL;
  ctl       = NULL;
  buf_v4l2  = NULL;
  buf_me    = NULL;

  setDevice("/dev/video0");
  setNBuffers(3);
  setFramerate(vpV4l2Grabber::framerate_25fps);
  setInput(input);
  setScale(scale);

  init = false;

  open(I);
}

/*!
  Constructor.

  Setup the Video For Linux Two (V4L2) driver in streaming mode.

  \param I : Image data structure (Color RGB32 bits image)
  \param _input : Video input port.
  \param _scale : Decimation factor.

  Default settings are:

  - Device name: /dev/video0: To change it use setDevice()

  - Number of ring buffers: 3. To change this value use setNBuffers(). For non
    real-time applications the number of buffers should be set to 1. For
    real-time applications to reach 25 fps or 50 fps a good compromise is to
    set the number of buffers to 3.

  - Framerate acquisition: 25 fps. Use setFramerate() to set 25 fps or 50
    fps. These framerates are reachable only if enought buffers are set.

    \code
    vpImage<vpRGBa> I;        // Color image

    vpV4l2Grabber g(I, 1, 2); // Select input 1, and half full size resolution
                              // images and open the grabber

    g.acquire(I);             // Acquire a 384x288 color image

    \endcode

*/
vpV4l2Grabber::vpV4l2Grabber(vpImage<vpRGBa> &I, unsigned _input, unsigned _scale )
{
  fd        = -1;
  streaming = false;
  verbose   = false;
  field     = 0;
  ncols     = 0;
  nrows     = 0;
  queue     = 0;
  waiton_cpt= 0;
  index_buffer = 0;

  inp       = NULL;
  std       = NULL;
  fmt       = NULL;
  ctl       = NULL;
  buf_v4l2  = NULL;
  buf_me    = NULL;

  setDevice("/dev/video0");
  setNBuffers(3);
  setFramerate(vpV4l2Grabber::framerate_25fps);
  setInput(_input);
  setScale(_scale);

  init = false;

  open(I);
}

/*!
  Destructor.

  \sa close()
*/
vpV4l2Grabber::~vpV4l2Grabber()
{
  close() ;
}

/*!
  Set the video input port on the board.
*/
void
vpV4l2Grabber::setInput(unsigned input)
{
  this->input = input;
}

/*!
  Set the decimation factor applied to full resolution images (768x576).

  \exception vpFrameGrabberException::settingError : Wrong scale (shoud be
  between 1 and 16).

  \param scale : Decimation factor. If scale is set to 2, 384x288 images will
  be acquired.

  An other way to specify the image size is to use setWidth() and setHeight().

*/
void
vpV4l2Grabber::setScale(unsigned scale)
{
  if ((scale <1) || (scale >16))
  {
    close();

    vpERROR_TRACE("Wrong scale %d, scale shoud be between 1 and 16",scale) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong scale") );
  }

  setWidth(768/scale);
  setHeight(576/scale);
}

/*!
  Initialize image acquisition in grey format.
  Set the pixel format acquisition to vpV4l2Grabber::V4L2_GREY_FORMAT.

  \param I : Image data structure (8 bits image). Once the device is open,
  the image is resized to the current acquisition size.

  \exception vpFrameGrabberException::settingError : Wrong input channel.

*/
void
vpV4l2Grabber::open(vpImage<unsigned char> &I)
{
  open();

  if( -1 == ioctl (fd, VIDIOC_S_INPUT, &input) )
  {
    close();

    vpERROR_TRACE("Wrong v4l2 frame grabber input %d channel",input) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong input channel") );
  }

  setPixelFormat(V4L2_GREY_FORMAT);
  vpCTRACE << width << "  " << height << endl;
  setFormat();

  startStreaming();

  I.resize(nrows, ncols) ;

  init = true;
}

/*!
  Initialize image acquisition in color RGB32 format.
  Set the pixel format acquisition to vpV4l2Grabber::V4L2_RGB32_FORMAT.


  \param I : Image data structure (RGB32 bits image). Once the device is open,
  the image is resized to the current acquisition size.

  \exception vpFrameGrabberException::settingError : Wrong input channel.
*/
void
vpV4l2Grabber::open(vpImage<vpRGBa> &I)
{
  open();

  if( -1 == ioctl (fd, VIDIOC_S_INPUT, &input) )
  {
    close();

    vpERROR_TRACE("Wrong v4l2 frame grabber input %d channel",input) ;
    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Wrong input channel") );
  }

  setPixelFormat(V4L2_RGB32_FORMAT);
  setFormat();

  startStreaming();

  I.resize(nrows, ncols) ;

  init = true;
}



/*!
  Acquire a grey level image.

  \param I : Image data structure (8 bits image)

  \exception vpFrameGrabberException::initializationError : Frame grabber not
  initialized.

  \sa getField()
*/
void
vpV4l2Grabber::acquire(vpImage<unsigned char> &I)
{

  if (init==false)
  {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "V4l2 frame grabber not initialized") );
  }

  unsigned  char *bitmap ;
  bitmap = waiton(index_buffer);

  if ((I.getCols() != ncols)||(I.getRows() != nrows))
    I.resize(nrows,ncols) ;

  memcpy(I.bitmap, bitmap,
	 ncols*nrows*sizeof(unsigned char));

  queueAll();
}

/*!
  Acquire a color image.

  \param I : Image data structure (32 bits image)

  \exception vpFrameGrabberException::initializationError : Frame grabber not
  initialized.

  \sa getField()
*/
void
vpV4l2Grabber::acquire(vpImage<vpRGBa> &I)
{

  if (init==false)
  {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "V4l2 frame grabber not initialized") );
  }

  unsigned  char *bitmap ;
  bitmap = waiton(index_buffer);

  if ((I.getCols() != ncols)||(I.getRows() != nrows))
    I.resize(nrows,ncols) ;

  // The framegrabber acquire aRGB format. We just shift the data from 1 byte all the data and initialize the last byte

  memcpy(I.bitmap, bitmap + 1,
	 ncols*nrows*sizeof(vpRGBa) - 1);
  I[nrows-1][ncols-1].A = 0;

  queueAll();
}
/*!

  Return the field (odd or even) corresponding to the last acquired
  frame.

  This method is to call after acquire() and has only a mean if the acquisition
  framerate is set to 50 fps.

  \return Field of the acquired frame (0 if odd field, 1 if even field).

  \exception vpFrameGrabberException::otherError : Video device returns a bad
  frame field.

  \sa acquire(), setFramerate()

*/
bool
vpV4l2Grabber::getField()
{
  if(field == 2) return 0; //top field
  else if (field == 3) return 1; //bottom field;
  else {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "V4l2 returns a bad frame field") );
    return false;
  }
}
/*!

  Set the frame format depending on the framerate acquisition.

  \param framerate : The framerate for the acquisition.
  - If vpV4l2Grabber::framerate_25fps use vpV4l2Grabber::V4L2_IMAGE_FORMAT,
  - else if vpV4l2Grabber::framerate_50fps use vpV4l2Grabber::V4L2_FRAME_FORMAT.
  \warning If you want to acquire frames at 25 fps or 50 fps, you have to be
  aware of the number of buffers required for the streaming. A typical value
  could be 3 (see setNBuffers()).

  \sa getFramerate(), setNBuffers()

*/
void
vpV4l2Grabber::setFramerate(vpV4l2Grabber::framerateEnum framerate)
{
   this->framerate = framerate;

   if (framerate == vpV4l2Grabber::framerate_25fps)
     setFrameFormat(V4L2_IMAGE_FORMAT);
   else
     setFrameFormat(V4L2_FRAME_FORMAT);
}

/*!

  Return the framerate of the acquisition.

  \return The actual framerate of the framegrabber.

  \sa setFramerate()
*/


vpV4l2Grabber::framerateEnum
vpV4l2Grabber::getFramerate()
{
  return framerate;
}


/*!
  Close the video device.
*/
void
vpV4l2Grabber::close()
{
  stopStreaming();
  streaming = false;

  if (fd >= 0){
    ::close (fd);
    fd = -1;
  }

  if (inp != NULL) { delete [] inp; inp = NULL; }
  if (std != NULL) { delete [] std; std = NULL; }
  if (fmt != NULL) { delete [] fmt; fmt = NULL; }
  if (ctl != NULL) { delete [] ctl; ctl = NULL; }
  if (buf_v4l2 != NULL) { delete [] buf_v4l2; buf_v4l2 = NULL; }
  if (buf_me != NULL)   { delete [] buf_me; buf_me = NULL; }
}

/*!

  Open the Video For Linux Two device.

  \exception vpFrameGrabberException::initializationError : Can't access to
  video device.

  \exception vpFrameGrabberException::otherError : Can't query video
  capabilities.

*/
void
vpV4l2Grabber::open()
{
  int err;

  /* Open Video Device */
  fd = ::open (device, O_RDWR);
  if (fd < 0) {
    close();

    vpERROR_TRACE ("No video device \"%s\"\n", device);
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Can't access to video device") );

  }

  if (inp != NULL) { delete [] inp; inp = NULL; }
  if (std != NULL) { delete [] std; std = NULL; }
  if (fmt != NULL) { delete [] fmt; fmt = NULL; }
  if (ctl != NULL) { delete [] ctl; ctl = NULL; }
  if (buf_v4l2 != NULL) { delete [] buf_v4l2; buf_v4l2 = NULL; }
  if (buf_me != NULL)   { delete [] buf_me; buf_me = NULL; }

  inp      = new struct v4l2_input     [vpV4l2Grabber::MAX_INPUTS];
  std      = new struct v4l2_standard  [vpV4l2Grabber::MAX_NORM];
  fmt      = new struct v4l2_fmtdesc   [vpV4l2Grabber::MAX_FORMAT];
  ctl      = new struct v4l2_queryctrl [vpV4l2Grabber::MAX_CTRL*2];
  buf_v4l2 = new struct v4l2_buffer    [vpV4l2Grabber::MAX_BUFFERS];
  buf_me   = new struct ng_video_buf   [vpV4l2Grabber::MAX_BUFFERS];

  /* Querry Video Device Capabilities */
  err = ioctl (fd, VIDIOC_QUERYCAP, &cap);
  if (err < 0) {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't query video capabilities") );
  }
  if (verbose) {
    fprintf(stdout, "v4l2 info:\n"
	    "     device: %s\n"
	    "     %s %d.%d.%d / %s @ %s\n",
	    device,
	    cap.driver,
	    (cap.version >> 16) & 0xff,
	    (cap.version >>  8) & 0xff,
	    cap.version         & 0xff,
	    cap.card, cap.bus_info);
    if (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
      fprintf(stdout, "     Support overlay\n");
    else
      fprintf(stdout, "     Does not support overlay\n");
    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
      fprintf(stdout, "     Support capture\n");
    else
      fprintf(stdout, "     Does not support capture\n");
    if (cap.capabilities & V4L2_CAP_TUNER)
      fprintf(stdout, "     Support tuning\n");
    else
      fprintf(stdout, "     Does not support tuning\n");
    if (cap.capabilities & V4L2_CAP_STREAMING)
      fprintf(stdout, "     Support streaming capture.\n");
    else
      fprintf(stdout, "     Does not support streaming capture.\n");
    if(cap.capabilities & V4L2_CAP_ASYNCIO)
      fprintf(stdout, "     Support asynchronous I/O methods.\n");
    else
      fprintf(stdout, "     Does not support asynchronous I/O methods.\n");
  }

  getCapabilities();
}

/*!

  Get device capabilities.

  \exception vpFrameGrabberException::otherError : Can't get video parameters.
*/
void
vpV4l2Grabber::getCapabilities()
{
  for (int ninputs = 0; ninputs < MAX_INPUTS; ninputs++) {
    inp[ninputs].index = ninputs;

    if (ioctl(fd, VIDIOC_ENUMINPUT, &inp[ninputs]))
      break;

  }
  for (int nstds = 0; nstds < MAX_NORM; nstds++) {
    std[nstds].index = nstds;
    if (ioctl(fd, VIDIOC_ENUMSTD, &std[nstds]))
      break;

  }
  for (int nfmts = 0; nfmts < MAX_FORMAT; nfmts++) {
    fmt[nfmts].index = nfmts;
    fmt[nfmts].type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_ENUM_FMT, &fmt[nfmts]))
      break;
  }

  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_PARM, &streamparm))
  {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't get video parameters") );
  }
}

/*!

  Set the capture format using the settings:
  - image size : specified by setWidth(), setHeight() or setScale()
  - frame format : specified by setFrameFormat()
  - pixel format : specified by setPixelFormat().

  \exception vpFrameGrabberException::settingError : Bad format, probably do to
  a wrong scale.

  \exception vpFrameGrabberException::otherError : Can't get video format.
*/
void
vpV4l2Grabber::setFormat()
{
  fmt_me.width  = width;
  fmt_me.height = height;
  fmt_me.bytesperline = width; // bad (normally width * depth / 8), but works
  // because initialized later by an ioctl call to VIDIOC_S_FMT

  switch(pixelformat) {
  case V4L2_GREY_FORMAT : fmt_me.pixelformat = V4L2_PIX_FMT_GREY; break;
  case V4L2_RGB24_FORMAT: fmt_me.pixelformat = V4L2_PIX_FMT_RGB24; break;
  case V4L2_RGB32_FORMAT: fmt_me.pixelformat = V4L2_PIX_FMT_RGB32; break;
  case V4L2_BGR24_FORMAT: fmt_me.pixelformat = V4L2_PIX_FMT_BGR24; break;
  case V4L2_BGR32_FORMAT: fmt_me.pixelformat = V4L2_PIX_FMT_BGR32; break;

  default:
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::settingError,
				   "Bad format, probably do to a wrong scale"));
  }

  /* Get Video Format */
  fmt_v4l2.type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl (fd, VIDIOC_G_FMT, &fmt_v4l2)) {
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't get video format") );
  }

  fmt_v4l2.fmt.pix.pixelformat  = fmt_me.pixelformat;
  fmt_v4l2.fmt.pix.width        = fmt_me.width;
  fmt_v4l2.fmt.pix.height       = fmt_me.height;

  switch (frameformat) {
  case V4L2_FRAME_FORMAT: fmt_v4l2.fmt.pix.field = V4L2_FIELD_ALTERNATE;
    break;
  case V4L2_IMAGE_FORMAT: fmt_v4l2.fmt.pix.field = V4L2_FIELD_INTERLACED;
    break;
  default:
    close();

    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Unrecognized frame format") );
  }

  //height and width of the captured image or frame
  ncols = width;
  nrows = height;
  if( frameformat == V4L2_FRAME_FORMAT && height > FRAME_SIZE )
  {
    nrows = FRAME_SIZE;
  }


  if (ioctl(fd, VIDIOC_S_FMT, &fmt_v4l2)) {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't set video format") );
  }
  if (fmt_v4l2.fmt.pix.pixelformat != fmt_me.pixelformat) {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Bad pixel format") );
  }

  fmt_me.width        = fmt_v4l2.fmt.pix.width;
  fmt_me.height       = fmt_v4l2.fmt.pix.height;
  fmt_me.bytesperline = fmt_v4l2.fmt.pix.bytesperline;

  if (verbose)
    fprintf(stdout,"v4l2: new capture params (%dx%d, %c%c%c%c, %d byte)\n",
	    fmt_me.width, fmt_me.height,
	    fmt_v4l2.fmt.pix.pixelformat & 0xff,
	    (fmt_v4l2.fmt.pix.pixelformat >>  8) & 0xff,
	    (fmt_v4l2.fmt.pix.pixelformat >> 16) & 0xff,
	    (fmt_v4l2.fmt.pix.pixelformat >> 24) & 0xff,
	    fmt_v4l2.fmt.pix.sizeimage);

}
/*!

  Launch the streaming capture mode and map device memory into application
  address space.

  \exception vpFrameGrabberException::otherError : If a problem occurs.

*/
void
vpV4l2Grabber::startStreaming()
{
  if (streaming == true) { // Acquisition in process.
    stopStreaming();
    streaming = false;
  }

  /* setup buffers */
  reqbufs.count  = nbuffers;
  reqbufs.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbufs.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &reqbufs))
  {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't require video buffers") );
  }

  for (unsigned i = 0; i < reqbufs.count; i++) {
    buf_v4l2[i].index  = i;
    buf_v4l2[i].type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_v4l2[i].memory = V4L2_MEMORY_MMAP;
    buf_v4l2[i].length = 0;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf_v4l2[i]))
    {
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't query video buffers") );
    }
    memcpy(&buf_me[i].fmt, &fmt_me, sizeof(ng_video_fmt));
    buf_me[i].size = buf_me[i].fmt.bytesperline * buf_me[i].fmt.height;

//     if (verbose)
//       cout << "1: buf_v4l2[" << i << "].length: " << buf_v4l2[i].length
// 	   << " buf_v4l2[" << i << "].offset: " <<  buf_v4l2[i].m.offset
// 	   << endl;


    buf_me[i].data = (unsigned char *) mmap(NULL, buf_v4l2[i].length,
					    PROT_READ | PROT_WRITE, MAP_SHARED,
					    fd, buf_v4l2[i].m.offset);

    if(buf_me[i].data == MAP_FAILED)
    {
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't map memory") );
    }

    buf_me[i].refcount = 0;

//     if (verbose)
//     {
//       cout << "2: buf_v4l2[" << i << "].length: " << buf_v4l2[i].length
// 	   << " buf_v4l2[" << i << "].offset: " <<  buf_v4l2[i].m.offset
// 	   << endl;
//       cout << "2: buf_me[" << i << "].size: " << buf_me[i].size << endl;
//     }

    if (verbose)
      printBufInfo(buf_v4l2[i]);

    if (MAP_FAILED == buf_me[i].data) {
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't map memory") );
    }
  }

  /* queue up all buffers */
  queueAll();

  /* Set video stream capture on */
  if (ioctl(fd, VIDIOC_STREAMON, &fmt_v4l2.type)<0)
  {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't start streaming") );
  }

  streaming = true;
}

/*!

  Stops the streaming capture mode and ummap the device memory.

  \exception vpFrameGrabberException::otherError : if can't stop streaming.
*/
void
vpV4l2Grabber::stopStreaming()
{
  unsigned int i;

  //nothing to do if (fd < 0) or if  (streaming == false)
  if ((fd >= 0) && (streaming == true)) {

    vpTRACE(" Stop the streaming...");
    /* stop capture */
    if (ioctl(fd, VIDIOC_STREAMOFF,&fmt_v4l2.type)) {
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "Can't stop streaming") );
    }
    /* free buffers */
    for (i = 0; i < reqbufs.count; i++) {
      if (0 != buf_me[i].refcount) {
	// ng_waiton_video_buf(&buf_me[i]);
	cout << "Normalement call ng_waiton_video_buf(&buf_me[i]); --------\n";
      }
      if (verbose)
	printBufInfo(buf_v4l2[i]);
      if (-1 == munmap(buf_me[i].data, buf_me[i].size)) {
	throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				       "Can't unmap memory") );
      }
    }
    queue = 0;
    waiton_cpt = 0;
    streaming = false;
  }
}

/*!
  Fill the next buffer. If all the buffers are filled return NULL.

  Update the buffer index. If all the buffers are filled index is set to -1.

  \exception vpFrameGrabberException::otherError : If can't access to the
  frame.
*/
unsigned char *
vpV4l2Grabber::waiton(int &index)
{
  struct v4l2_buffer buf;
  struct timeval tv;
  fd_set rdset;

  /* wait for the next frame */
 again:
  tv.tv_sec  = 1;
  tv.tv_usec = 0;
  FD_ZERO(&rdset);
  FD_SET(fd, &rdset);
  switch (select(fd + 1, &rdset, NULL, NULL, &tv)) {
  case -1:
    if (EINTR == errno)
      goto again;
    index = -1;
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't access to the frame") );
    return NULL;
  case  0:
    index = -1;
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				   "Can't access to the frame: timeout") );
    return NULL;
  }


  /* get it */
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == ioctl(fd,VIDIOC_DQBUF, &buf)) {
    index = -1;
    switch(errno)
    {
    case EAGAIN:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_DQBUF: EAGAIN") );
      break;
    case EINVAL:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_DQBUF: EINVAL") );
      break;
    case ENOMEM:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_DQBUF: ENOMEM") );
      break;
    default:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_DQBUF") );
      break;
    }
    return NULL;
  }

  waiton_cpt++;
  buf_v4l2[buf.index] = buf;

  index = buf.index;

  field = buf_v4l2[index].field;

  if(verbose)
  {
    vpERROR_TRACE("field: %d\n", buf_v4l2[index].field);

    vpERROR_TRACE("data adress : 0x%p\n", buf_me[buf.index].data);
  }
  return buf_me[buf.index].data;
}

/*!

 Capture helpers.

*/
int
vpV4l2Grabber::queueBuffer()
{
  int frame = queue % reqbufs.count;
  int rc;


  if (0 != buf_me[frame].refcount) {
    if (0 != queue - waiton_cpt)
      return -1;
    fprintf(stderr,"v4l2: waiting for a free buffer..............\n");
    //ng_waiton_video_buf(h->buf_me+frame);
    cout << "Normalement call ng_waiton_video_buf(buf_me+frame); --------\n";
  }

  //    cout << "frame: " << frame << endl;
  rc = ioctl(fd, VIDIOC_QBUF, &buf_v4l2[frame]);
  if (0 == rc)
    queue++;
  else
  {
    switch(errno)
    {
    case EAGAIN:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_QBUF: EAGAIN") );
      break;
    case EINVAL:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_QBUF: EINVAL") );
      break;
    case ENOMEM:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_QBUF: ENOMEM") );
      break;
    default:
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				     "VIDIOC_QBUF") );
      break;
    }
  }
  return rc;
}

/*!

  Call the queue buffer private method if needed

*/
void
vpV4l2Grabber::queueAll()
{
  for (;;) {
    if (queue - waiton_cpt >= reqbufs.count) {
      return;
    }
    if (0 != queueBuffer()) {
      return;
    }
  }
}

/*!

  Get device capabilities.

*/
void
vpV4l2Grabber::printBufInfo(struct v4l2_buffer buf)
{
  char type[40];

  switch(buf.type) {
  case V4L2_BUF_TYPE_VIDEO_CAPTURE: sprintf(type, "video-cap"); break;
  case V4L2_BUF_TYPE_VIDEO_OVERLAY: sprintf(type, "video-over"); break;
  case V4L2_BUF_TYPE_VIDEO_OUTPUT:  sprintf(type, "video-out"); break;
  case V4L2_BUF_TYPE_VBI_CAPTURE:   sprintf(type, "vbi-cap"); break;
  case V4L2_BUF_TYPE_VBI_OUTPUT:    sprintf(type, "vbi-out"); break;
  default:                          sprintf(type, "unknown"); break;
  }

  fprintf(stdout,"v4l2: buf %d: %d ad: 0x%lx offset 0x%x+%d (=0x%x),used %d\n",
	  buf.index, buf.type, buf.m.userptr, buf.m.offset,
	  buf.length, buf.length, buf.bytesused);

}
#endif
