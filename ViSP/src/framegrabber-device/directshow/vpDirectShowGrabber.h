/****************************************************************************
 *
 * $Id: vpDirectShowGrabber.h,v 1.8 2007-05-31 13:04:41 asaunier Exp $
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
 * DirectShow framegrabber.
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/


#ifndef vpDirectShowGrabber_hh
#define vpDirectShowGrabber_hh

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) )

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpFrameGrabber.h>

class vpDirectShowGrabberImpl;

/*!
  \class vpDirectShowGrabber
  \brief class for windows direct show devices

  This class' goal is to totally hide the implementation of the framegrabber
  from the rest of the application. This is necessary to avoid conflicts
  between dshow's Qedit.h and Direct3D's includes.

  \ingroup libdevice
*/

class VISP_EXPORT vpDirectShowGrabber : public vpFrameGrabber
{
	vpDirectShowGrabberImpl * grabber;


	public:

		unsigned int getHeight();
		unsigned int getWidth();

		vpDirectShowGrabber();
		virtual ~vpDirectShowGrabber();

		void open();
		void open(vpImage<unsigned char> &I);
		void open(vpImage<vpRGBa> &I);

		void acquire(vpImage<unsigned char> &I);
		void acquire(vpImage<vpRGBa> &I);

		void close();

		//get the number of capture devices
		unsigned int getDeviceNumber();

		//change the capture device
		bool setDevice(unsigned int id);

		//displays a list of available devices
		void displayDevices();

		//set image size
		bool setImageSize(unsigned int width,unsigned int height);

		//set source framerate
		bool setFramerate(double framerate);


		//set capture format
		bool setFormat(unsigned int width,unsigned int height, double framerate);

		//get capture format
		void getFormat(unsigned int &width,unsigned int &height, double &framerate);

		//Get the available capture formats
		bool getStreamCapabilities();

		//Set capture MediaType
		bool setMediaType(int mediaTypeID);

		//Get current capture MediaType
		int getMediaType();

};
#endif
#endif
