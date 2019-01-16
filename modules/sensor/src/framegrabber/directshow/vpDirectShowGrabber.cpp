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
 * DirectShow framegrabber.
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <visp3/sensor/vpDirectShowGrabber.h>
#include <visp3/sensor/vpDirectShowGrabberImpl.h>

/*!
        Returns the number of rows in the grabbed image
*/
unsigned int vpDirectShowGrabber::getHeight() { return grabber->getHeight(); }

/*!
        Returns the number of colunms in the grabbed image
*/
unsigned int vpDirectShowGrabber::getWidth() { return grabber->getWidth(); }

/*!
        Constructor.
        Initializes COM.
*/
vpDirectShowGrabber::vpDirectShowGrabber() { grabber = new vpDirectShowGrabberImpl(); }

/*!
        Destructor
*/
vpDirectShowGrabber::~vpDirectShowGrabber() { delete grabber; }
/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabber::open() { grabber->open(); }

/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<unsigned char> &I) { grabber->open(); }

/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<vpRGBa> &I) { grabber->open(); }

/*!
        Grabs a grayscale image from the video stream.
        Uses a semaphore to synchronize with the framegrabber callback.

        \param I The image to fill with the grabbed frame

        \exception otherError Can't grab the frame
*/
void vpDirectShowGrabber::acquire(vpImage<unsigned char> &I) { grabber->acquire(I); }

/*!
        Grabs a rgba image from the video stream.
        Uses a semaphore to synchronize with the framegrabber callback.

        \param I The image to fill with the grabbed frame

        \exception otherError Can't grab the frame
*/
void vpDirectShowGrabber::acquire(vpImage<vpRGBa> &I) { grabber->acquire(I); }

/*!
        Stops the framegrabber
*/
void vpDirectShowGrabber::close() { grabber->close(); }

/*!
        Gets the number of capture devices
        \return the number of capture devices
*/
unsigned int vpDirectShowGrabber::getDeviceNumber() { return grabber->getDeviceNumber(); }

/*!
        Set the device (or camera) from which images will be grabbed.
        \param id : Identifier of the device to use.

        \return true if the device is set successfully, false otherwise.
*/
bool vpDirectShowGrabber::setDevice(unsigned int id) { return grabber->setDevice(id); }

/*!
        Displays the list of devices on the standard output
*/
void vpDirectShowGrabber::displayDevices() { grabber->displayDevices(); }
/*!
        Set image Size
        \param width : Image width
        \param height : Image height

        \return true if successful
*/
bool vpDirectShowGrabber::setImageSize(unsigned int width, unsigned int height)
{
  return grabber->setImageSize(width, height);
}
/*!
        Set capture framerate
        \param framerate : Framerate in fps

        \return true if successful
*/
bool vpDirectShowGrabber::setFramerate(double framerate) { return grabber->setFramerate(framerate); }
/*!
        Set capture format
        \param width : Image width in pixel
        \param height : Image height in pixel
        \param framerate : Framerate in fps

        \return true if successful
*/
bool vpDirectShowGrabber::setFormat(unsigned int width, unsigned int height, double framerate)
{
  return grabber->setFormat(width, height, framerate);
}
/*
        Get capture format
        \param width : Pointer to a variable that receives the width in pixel
        \param height : Pointer to a variable that receives the height in
   pixel \param framerate : Pointer to a variable that receives the framerate
   in fps
*/
void vpDirectShowGrabber::getFormat(unsigned int &width, unsigned int &height, double &framerate)
{
  grabber->getFormat(width, height, framerate);
}

/*!
        Get the available capture formats

        \return true if successful
*/
bool vpDirectShowGrabber::getStreamCapabilities() { return grabber->getStreamCapabilities(); }

/*!
        Set capture MediaType
        \param mediaTypeID : mediaTypeID (available in calling
   getStreamCapabilities)

        \return true if successful
*/
bool vpDirectShowGrabber::setMediaType(int mediaTypeID) { return grabber->setMediaType(mediaTypeID); }

/*
        Get current capture MediaType

        \return the current mediaTypeID
*/
int vpDirectShowGrabber::getMediaType() { return grabber->getMediaType(); }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpDirectShowGrabber.cpp.o)
// has no symbols
void dummy_vpDirectShowGrabber(){};
#endif
