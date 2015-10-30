/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
  \file vpOpenCVGrabber.h
  \brief class for cameras video capture using OpenCV library.
*/

#ifndef vpOpenCVGrabber_h
#define vpOpenCVGrabber_h

#include <visp3/core/vpConfig.h>

#if ( defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408))

#if VISP_HAVE_OPENCV_VERSION >= 0x020101
#  include <opencv2/highgui/highgui.hpp>
#else
#  include <highgui.h>
#endif

#include <visp3/core/vpImage.h>
#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpOpenCVGrabber

  \ingroup Framegrabber CameraDriver
  
  \brief Class for cameras video capture using OpenCV library.
  
  Needs OpenCV available on http://opencv.willowgarage.com/wiki/.
  
  The code below available in tutorial-grabber-opencv.cpp shows how to grab and
  display images using OpenCV wrappers impremented in ViSP.
  \include tutorial-grabber-opencv.cpp

  Note that it is also possible to grab images using OpenCV library by using
  directly OpenCV cv::VideoCapture class. The following code corresponding to
  tutorial-grabber-opencv-bis.cpp shows how to grab images in a cv::Mat structure
  and then convert OpenCV images in ViSP images.
  \include tutorial-grabber-opencv-bis.cpp

  An other example very close to the previous one available in grabOpenCV-2.cpp
  shows how to grab images in OpenCV IplImage structure and then how to convert
  OpenCV images in ViSP images.

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
