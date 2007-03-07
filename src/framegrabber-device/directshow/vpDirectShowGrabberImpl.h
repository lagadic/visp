/****************************************************************************
 *
 * $Id: vpDirectShowGrabberImpl.h,v 1.5 2007-03-07 17:51:46 asaunier Exp $
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
 * DirectShow framegrabber Implementation.
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#ifndef vpDirectShowGrabberImpl_hh
#define vpDirectShowGrabberImpl_hh

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <atlbase.h>
#include <qedit.h>
#include <dshow.h>

#include <visp/vpFrameGrabber.h>
#include <visp/vpFrameGrabberException.h>

#include <visp/vpDirectShowSampleGrabberI.h>
#include <visp/vpDirectShowDevice.h>

/*!
	\class vpDirectShowGrabberImpl
	\brief class for windows direct show devices - implementation

	\ingroup libdevice
*/

#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT CComPtr<IGraphBuilder>;
template class VISP_EXPORT CComPtr<ICaptureGraphBuilder2>;
template class VISP_EXPORT CComPtr<IBaseFilter>;
template class VISP_EXPORT CComPtr<ISampleGrabber>;
template class VISP_EXPORT CComPtr<IBaseFilter>;
template class VISP_EXPORT CComPtr<IMediaControl>;
template class VISP_EXPORT CComPtr<IMediaEvent>;
#endif

/*
	This code uses CComPtr which is the best way to be sure that all the interfaces
	are released sooner or later.
	Since this class may be exported, it may be necessary to explicitely instantiate
	each instance of CComPtr.
	eg. template class VISP_EXPORT CComPtr<IBaseFilter>;
*/
class VISP_EXPORT vpDirectShowGrabberImpl : public vpFrameGrabber
{
	static const int MAX_DELAY = 10000;
	static const int MAX_DEVICES = 10;


 	public:

		/*!
    Enumeration of video subtypes.
*/
/*	
  typedef enum {
	  //Known RGB formats
	  vpMEDIASUBTYPE_ARGB32 = MEDIASUBTYPE_ARGB32,
	  vpMEDIASUBTYPE_RGB32 = MEDIASUBTYPE_RGB32,
	  vpMEDIASUBTYPE_RGB24 = MEDIASUBTYPE_RGB24,
	  vpMEDIASUBTYPE_RGB555 = MEDIASUBTYPE_RGB555,
	  vpMEDIASUBTYPE_RGB565 = MEDIASUBTYPE_RGB565,
	  vpMEDIASUBTYPE_RGB8 = MEDIASUBTYPE_RGB8,
	  vpMEDIASUBTYPE_RGB4 = MEDIASUBTYPE_RGB4,
	  vpMEDIASUBTYPE_RGB1 = MEDIASUBTYPE_RGB1,
	  //Known YUV formats
	  vpMEDIASUBTYPE_AYUV = MEDIASUBTYPE_AYUV,
	  vpMEDIASUBTYPE_UYVY = MEDIASUBTYPE_UYVY,
	  vpMEDIASUBTYPE_Y411 = MEDIASUBTYPE_Y411,
	  vpMEDIASUBTYPE_Y41P = MEDIASUBTYPE_Y41P,
	  vpMEDIASUBTYPE_Y211 = MEDIASUBTYPE_Y211,
	  vpMEDIASUBTYPE_YUY2 = MEDIASUBTYPE_YUY2,
	  vpMEDIASUBTYPE_YVYU = MEDIASUBTYPE_YVYU,
	  vpMEDIASUBTYPE_YUYV = MEDIASUBTYPE_YUYV,
	  vpMEDIASUBTYPE_IF09 = MEDIASUBTYPE_IF09,
	  vpMEDIASUBTYPE_IYUV = MEDIASUBTYPE_IYUV,
	  vpMEDIASUBTYPE_YV12 = MEDIASUBTYPE_YV12,	
	  vpMEDIASUBTYPE_YVU9 = MEDIASUBTYPE_YVU9
  } vpDirectShowMediaSubtype;
*/

		vpDirectShowGrabberImpl();
		~vpDirectShowGrabberImpl();

		void open();
		void open(vpImage<unsigned char> &I);
		void open(vpImage<vpRGBa> &I);

		void acquire(vpImage<unsigned char> &I);
		void acquire(vpImage<vpRGBa> &I);

		void close();
			
		/*!
			Gets the number of capture devices
		*/
		unsigned int getDeviceNumber() {return nbDevices;}

		//change the capture device
		bool setDevice(unsigned int n);

		//displays a list of available devices
		void displayDevices();

		//set image size
		bool setImageSize(unsigned int _width,unsigned int _height);

		//set capture framerate
		bool setFramerate(double _framerate);

		//set capture format
		bool setFormat(unsigned int _width,unsigned int _height, double _framerate);

		//get capture format
		void getFormat(unsigned int* pWidth,unsigned int* pHeight, double* pFramerate);

		//set capture MediaType
		bool setMediaType(int mediaTypeID);	

		//get current capture MediaType
		int getMediaType();

		//Get the available capture formats
		bool getStreamCapabilities();


	private:
		
		CComPtr<IGraphBuilder> pGraph;			//our DS filter graph
		
		CComPtr<ICaptureGraphBuilder2> pBuild;	//the interface to the capture graph builder 
												//used to build the filter graph

		CComPtr<IBaseFilter> pCapSource;		//the capture source filter
		
		CComPtr<ISampleGrabber> pGrabberI;		//the sample grabber's interface and filter
		CComPtr<IBaseFilter> pGrabberFilter;
		
		CComPtr<IMediaControl> pControl; 		//The DS filter graph control interface
		CComPtr<IMediaEvent> pEvent;			//The DS filter graph event interface

		vpDirectShowSampleGrabberI sgCB;		//Interface used to implement the frame grabber callback

		HRESULT hr;								//contains the result of the last operation
	
		static vpDirectShowDevice * deviceList;	//This contains the list of the available capture devices
												//it is shared by all the DirectShow Grabbers
		
		static unsigned int nbDevices;			//the number of available devices
		int currentDevice;						//the number of the current device
		
		// flag to manage CoInitialize() and CoUnInitialze()
		bool initCo ; 
		//setup the directshow filtergraph with the first available device
		bool initDirectShow();

		//enumerates the different video inputs
		bool enumerate(CComPtr<IEnumMoniker>& ppVideoInputEnum);

		//selects a random video input from the enumeration and returns the associated filter
		bool selectRandomSource(CComPtr<IEnumMoniker>& ppVideoInputEnum, CComPtr<IBaseFilter>& pCapSource);

		//creates the filter graph
		bool createGraph();

		//creates the sample grabber
		bool createSampleGrabber(CComPtr<IBaseFilter>& ppGrabberFilter);

		//checks the capture filter's media type and sets flags as needed
		bool checkSourceType(CComPtr<IPin>& pCapSourcePin);

		//connects the filters as needed
		bool connectSourceToGrabber(CComPtr<IBaseFilter>& pCapSource, CComPtr<IBaseFilter>& pGrabberFilter);

		//used to convert HRESULT-associated error message to a string
		void HRtoStr(string str);
		
		//create the list of the available devices
		bool createDeviceList(CComPtr<IEnumMoniker>& ppVideoInputEnum);

		//get the n-th device if it is available
		bool getDevice(unsigned int n, CComPtr<IBaseFilter>& ppCapSource);

		//get the first available device if any
		unsigned int getFirstUnusedDevice(CComPtr<IBaseFilter>& ppDevice);

		//removes all the filters in the graph
		bool removeAll();

		//Deletes an allocated AM_MEDIA_TYPE structure, including the format block
		void MyDeleteMediaType(AM_MEDIA_TYPE *pmt);
		
		//Frees the format block in an AM_MEDIA_TYPE structure
		void MyFreeMediaType(AM_MEDIA_TYPE& mt);

};

#endif
#endif
#endif
