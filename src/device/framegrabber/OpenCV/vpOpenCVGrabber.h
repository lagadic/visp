/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Cameras video capture using OpenCV library.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpOpenCVGrabber.h
  \brief class for cameras video capture using OpenCV library.
*/

#ifndef vpOpenCVGrabber_h
#define vpOpenCVGrabber_h

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#  include <opencv2/highgui/highgui.hpp>
#else
#  include <highgui.h>
#endif

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

/*!
  \class vpOpenCVGrabber

  \ingroup Framegrabber CameraDriver
  
  \brief Class for cameras video capture using OpenCV library.
  
  Needs OpenCV available on http://opencv.willowgarage.com/wiki/.
  
  The code below shows how to use this class.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpOpenCVGrabber.h>

int main()
{
#if defined(VISP_HAVE_OPENCV)
  vpImage<unsigned char> I; // Create a gray level image container
  vpOpenCVGrabber g;        // Create a grabber based on OpenCV third party lib

  g.open(I);                           // Open the framegrabber
  g.acquire(I);                        // Acquire an image
  vpImageIo::writePGM(I, "image.pgm"); // Write image on the disk
#endif
}
  \endcode

  Note that it is also possible to grab images using OpenCV library by using directly
  cv::VideoCapture OpenCV class. The following code corresponding to grabOpenCV-2.cpp example shows how to acquire an image
  with cv::VideoCapture, how to transform this image in ViSP format and how to display it.
\code
#include <iostream>

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpOpenCVGrabber.h>

// usage: binary <device name>
// device name: 0 is the default to dial with the first camera,
//              1 to dial with a second camera attached to the computer
int main(int argc, char** argv)
{
  int device = 0;
  if (argc > 1)
    device = atoi(argv[1]);

  std::cout << "Use device: " << device << std::endl;
  cv::VideoCapture cap(device); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
    return -1;
  cv::Mat frame;
  cap >> frame; // get a new frame from camera

  IplImage iplimage = frame;
  std::cout << "Image size: " << iplimage.width << " "
            << iplimage.height << std::endl;

  //vpImage<vpRGBa> I; // for color images
  vpImage<unsigned char> I; // for gray images
  vpImageConvert::convert(&iplimage, I);
  vpDisplayOpenCV d(I);

  for(;;) {
    cap >> frame; // get a new frame from camera
    iplimage = frame;

    // Convert the image in ViSP format and display it
    vpImageConvert::convert(&iplimage, I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) // a click to exit
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}

#else
int main()
{
  std::cout << "OpenCV is not available" << std::endl;
}
\endcode
 */
class VISP_EXPORT vpOpenCVGrabber : public vpFrameGrabber
{
	private:

		CvCapture *capture;
		int DeviceType;
		bool flip;

	public:

		vpOpenCVGrabber();
		~vpOpenCVGrabber();

		void open();
		void open(vpImage<unsigned char> &I);
		void open(vpImage<vpRGBa> &I);

		void acquire(vpImage<unsigned char> &I);
		void acquire(vpImage<vpRGBa> &I);
    IplImage* acquire();

		void close();

		void getFramerate(double & framerate);
		void setFramerate(const double framerate);

		void setWidth(const unsigned int width);
		void setHeight(const unsigned int height);
		
		void setDeviceType(int type);

		void setFlip(bool flipType);
};

#endif
#endif
