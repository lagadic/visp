/****************************************************************************
 *
 * $Id: vpOpenCVGrabber.cpp,v 1.4 2008-11-21 14:40:16 nmelchio Exp $
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

#include <iostream>
#include <math.h>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#include <visp/vpImageConvert.h>
#include <visp/vpFrameGrabberException.h>
#include <visp/vpOpenCVGrabber.h>

unsigned int vpOpenCVGrabber::nbDevices = 0;


/*!
	Basic Constructor.
*/
vpOpenCVGrabber::vpOpenCVGrabber()
{
	// public memebers
	init = false;

	// protected members
	width = height = 0;

	// private members
	capture = NULL;
	DeviceType = 0;

#if ( defined(UNIX) )
	nbDevices = 1;
#endif

#if ( defined(WIN32) )
	nbDevices = cvcamGetCamerasCount();	//Available only with windows
#endif
}


/*!
	Basic destructor that calls the close() method.

	\sa close()
*/
vpOpenCVGrabber::~vpOpenCVGrabber( )
{
	close();
}


/*!
	Generic initialization of the grabber.
*/
void vpOpenCVGrabber::open()
{
	if (nbDevices > 0)
	{
		capture = cvCreateCameraCapture(DeviceType);
	}
	
	if (capture != NULL)
	{
		init = true;
	}

	else
	{
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
			"Initialization not done : camera already used or no camera found") );
	}
}


/*!
	Initialization of the grabber.
	Call the generic initialization method.

	\param I : Gray level image. This parameter is not used.

	\sa open()
*/
void vpOpenCVGrabber::open(vpImage<unsigned char> &/*I*/)
{
	open();
}


/*!
	Initialization of the grabber.
	Call the generic initialization method.

	\param I : Color image. This parameter is not used.

	\sa open()
*/
void vpOpenCVGrabber::open(vpImage<vpRGBa> &/*I*/)
{
	open();
}


/*!
	Grab a gray level image.

	\param I : Acquired gray level image.

	\exception vpFrameGrabberException::initializationError If the
	initialization of the grabber was not done previously.
*/
void vpOpenCVGrabber::acquire(vpImage<unsigned char> &I)
{
	IplImage *im;

	if (init==false)
	{
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "Initialization not done") );
	}

	cvGrabFrame(capture);
	im = cvRetrieveFrame(capture);
	vpImageConvert::convert(im, I);
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

	if (init==false)
	{
		close();
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "Initialization not done") );
	}

	cvGrabFrame(capture);
	im = cvRetrieveFrame(capture);
	vpImageConvert::convert(im, I);
}


/*!
	Stop the acquisition of images and free the camera.
*/
void vpOpenCVGrabber::close()
{
	init = false;
	cvReleaseCapture( &capture );
	capture = NULL;
}


/*!
	Gets the capture frame rate.

	\param framerate : The value of the framerate is returned here.

	\exception vpFrameGrabberException::initializationError If no
	camera was found.
*/
void vpOpenCVGrabber::getFramerate(double & framerate)
{
	if (! nbDevices) 
	{
		close();
		vpERROR_TRACE("No camera found");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"No camera found") );
	}

	framerate = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
}


/*!
	Sets the capture frame rate

	\param framerate : The requested value of the capture framerate.

	\exception vpFrameGrabberException::initializationError If no
	camera was found.
*/
void vpOpenCVGrabber::setFramerate(const double framerate)
{
	if (! nbDevices) 
	{
		close();
		vpERROR_TRACE("No camera found");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"No camera found") );
	}

	cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, framerate);
}


/*!
	Sets the captured image width.

	\param width : The requested value of the captured image width.

	\exception vpFrameGrabberException::initializationError If no
	camera was found.
*/
void vpOpenCVGrabber::setWidth(const unsigned int width)
{
	if (! nbDevices)
	{
		close();
		vpERROR_TRACE("No camera found");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"No camera found") );
	}

	if ( cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, width))
	{
	  	close();
		vpERROR_TRACE("Impossible to set the size of the grabber");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"Impossible to set the size of the grabber") );
	}

	this->width = width;
}

/*!
	Sets the captured image height

	\param height : The requested value of the captured image height.

	\exception vpFrameGrabberException::initializationError If no
	camera was found.
*/
void vpOpenCVGrabber::setHeight(const unsigned int height)
{
	if (! nbDevices)
	{
		close();
		vpERROR_TRACE("No camera found");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"No camera found") );
	}

	if ( cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, height))
	{
	  	close();
		vpERROR_TRACE("Impossible to set the size of the grabber");
		throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
										"Impossible to set the size of the grabber") );
	}

	this->height = height;
}


/*!
	Gets the number of capture devices connected on your computer.
			
	\Warning This function is available only under Windows.

	\return 1 under Linux, the number of devices under Windows.
*/
unsigned int vpOpenCVGrabber::getDeviceNumber()
{
#if ( defined(UNIX) )
	vpTRACE("This function is not available under Unix.");
	return(1);
#endif

#if ( defined(WIN32) )
	return(nbDevices);	
#endif
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
	
	if ( DeviceType != 0 && DeviceType != 100  &&DeviceType != 200 && DeviceType != 300)
	{
		vpTRACE("The expected type of device may be unknown.");
	}
}

#endif
