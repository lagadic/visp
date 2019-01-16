/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Cameras video capture using OpenCV library.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpOpenCVGrabber.cpp
  \brief class for cameras video capture using OpenCV library.
*/

#include <visp3/sensor/vpOpenCVGrabber.h>

#if (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408))

#include <visp3/core/vpFrameGrabberException.h>
#include <visp3/core/vpImageConvert.h>

#include <iostream>
#include <math.h>

/*!
        Basic Constructor.
*/
vpOpenCVGrabber::vpOpenCVGrabber() : capture(NULL), DeviceType(0), flip(false)
{
  // public memebers
  init = false;

  // protected members
  width = height = 0;
}

/*!
        Basic destructor that calls the close() method.

        \sa close()
*/
vpOpenCVGrabber::~vpOpenCVGrabber() { close(); }

/*!
        Generic initialization of the grabber.
*/
void vpOpenCVGrabber::open()
{

  capture = cvCreateCameraCapture(DeviceType);

  if (capture != NULL) {
    init = true;
  }

  else {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                  "Initialization not done : camera already used or no camera found"));
  }
}

/*!
        Initialization of the grabber.
        Call the generic initialization method.

        \param I : Gray level image. This parameter is not used.

        \sa open()
*/
void vpOpenCVGrabber::open(vpImage<unsigned char> & /*I*/) { open(); }

/*!
        Initialization of the grabber.
        Call the generic initialization method.

        \param I : Color image. This parameter is not used.

        \sa open()
*/
void vpOpenCVGrabber::open(vpImage<vpRGBa> & /*I*/) { open(); }

/*!
        Grab a gray level image.

        \param I : Acquired gray level image.

        \exception vpFrameGrabberException::initializationError If the
        initialization of the grabber was not done previously.
*/
void vpOpenCVGrabber::acquire(vpImage<unsigned char> &I)
{
  IplImage *im;

  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  cvGrabFrame(capture);
  im = cvRetrieveFrame(capture);
  vpImageConvert::convert(im, I, flip);
}

/*!
        Grab a color image.

        \param I : Acquired color image.

        \exception vpFrameGrabberException::initializationError If the
        initialization of the grabber was not done previously.
*/
void vpOpenCVGrabber::acquire(vpImage<vpRGBa> &I)
{
  IplImage *im;

  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  cvGrabFrame(capture);
  im = cvRetrieveFrame(capture);
  vpImageConvert::convert(im, I, flip);
}

/*!
  Grab an image direclty in the OpenCV format.

  \return Pointer to the image (must not be freed).

  \exception vpFrameGrabberException::initializationError If the
  initialization of the grabber was not done previously.
*/
IplImage *vpOpenCVGrabber::acquire()
{
  IplImage *im;

  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  cvGrabFrame(capture);
  im = cvRetrieveFrame(capture);
  return im;
}

/*!
        Stop the acquisition of images and free the camera.
*/
void vpOpenCVGrabber::close()
{
  init = false;
  cvReleaseCapture(&capture);
  capture = NULL;
}

/*!
        Gets the capture frame rate.

        \param framerate : The value of the framerate is returned here.

*/
void vpOpenCVGrabber::getFramerate(double &framerate) { framerate = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS); }

/*!
        Sets the capture frame rate

        \param framerate : The requested value of the capture framerate.

*/
void vpOpenCVGrabber::setFramerate(const double framerate)
{
  cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, framerate);
}

/*!
        Sets the captured image width.

  \warning This function must be called after open() method.

  \param w : The requested value of the captured image width.

        \exception vpFrameGrabberException::initializationError If no
        camera was found.
*/
void vpOpenCVGrabber::setWidth(const unsigned int w)
{
  if (cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, w)) {
    close();
    vpERROR_TRACE("Impossible to set the size of the grabber");
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                  "Impossible to set the size of the grabber"));
  }

  this->width = w;
}

/*!
        Sets the captured image height.

  \warning This function must be called after open() method.

  \param h : The requested value of the captured image height.

        \exception vpFrameGrabberException::initializationError If no
        camera was found.
*/
void vpOpenCVGrabber::setHeight(const unsigned int h)
{
  if (cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, h)) {
    close();
    vpERROR_TRACE("Impossible to set the size of the grabber");
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                  "Impossible to set the size of the grabber"));
  }

  this->height = h;
}

/*!
        Set the expected type of device.

        \param type : expected type of device
        - CV_CAP_ANY
        - CV_CAP_MIL
        - CV_CAP_VFW
        - CV_CAP_V4L
        - CV_CAP_V4L2
        - CV_CAP_FIREWIRE
        - CV_CAP_IEEE1394
        - CV_CAP_DC1394
        - CV_CAP_CMU_1394
*/
void vpOpenCVGrabber::setDeviceType(int type)
{
  DeviceType = type;

  if (DeviceType != 0 && DeviceType != 100 && DeviceType != 200 && DeviceType != 300) {
    vpTRACE("The expected type of device may be unknown.");
  }
}

/*!
        Set the boolean variable flip to the expected value.

        \param flipType : Expected value of the variable flip. True means that
   the image is flipped during each image acquisition.

        \warning This function is only useful under Windows.

        \note The aim of this function is to fix a problem which appears under
   Windows. Indeed with several cameras the aquired images are flipped.
*/
void vpOpenCVGrabber::setFlip(bool flipType) { flip = flipType; }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpOpenCVGrabber.cpp.o) has
// no symbols
void dummy_vpOpenCVGrabber(){};
#endif
