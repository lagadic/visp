/****************************************************************************
 *
 * $Id: vpOpenCVGrabber.h,v 1.2 2008-11-19 21:07:18 fspindle Exp $
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
  \file vpOpenCVGrabber.h
  \brief class for cameras video capture using OpenCV library.
*/

#ifndef vpOpenCVGrabber_h
#define vpOpenCVGrabber_h

#include <visp/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)

#include <highgui.h>
#ifdef WIN32
#  include <cvcam.h>
#endif

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>
#include <visp/vpList.h>

/*!
  \class vpOpenCVGrabber

  \ingroup Framegrabber
  
  \brief Class for cameras video capture using OpenCV library.
  
  Needs OpenCV 1.0 library or more recent versions
  available on http://sourceforge.net.
  
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
  
 */
class VISP_EXPORT vpOpenCVGrabber : public vpFrameGrabber
{
	public:
		

	private:

                static unsigned int nbDevices;
		CvCapture *capture;

	public:

		vpOpenCVGrabber();
		~vpOpenCVGrabber();

		void open();
		void open(vpImage<unsigned char> &I);
		void open(vpImage<vpRGBa> &I);

		void acquire(vpImage<unsigned char> &I);
		void acquire(vpImage<vpRGBa> &I);

		void close();

		unsigned int getDeviceNumber();

		void getFramerate(double & framerate);
		void setFramerate(const double framerate);

		void setWidth(const unsigned int width);
		void setHeight(const unsigned int height);
};

#endif
#endif
