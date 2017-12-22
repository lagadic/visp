/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * DirectShow framegrabber Implementation.
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#ifndef vpDirectShowGrabberImpl_hh
#define vpDirectShowGrabberImpl_hh

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <atlbase.h>
#include <dshow.h>
#include <qedit.h>

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpFrameGrabberException.h>

#include <visp3/core/vpDebug.h>
#include <visp3/sensor/vpDirectShowDevice.h>
#include <visp3/sensor/vpDirectShowSampleGrabberI.h>
/*!
        \class vpDirectShowGrabberImpl
        \brief class for windows direct show devices - implementation

        This class uses CComPtr which is the best way to be sure that all the
   interfaces are released sooner or later.

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
  virtual ~vpDirectShowGrabberImpl();

  void open();
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);

  void close();

  /*!
          Gets the number of capture devices
  */
  unsigned int getDeviceNumber() { return nbDevices; }

  // change the capture device
  bool setDevice(unsigned int id);

  // displays a list of available devices
  void displayDevices();

  // set image size
  bool setImageSize(unsigned int width, unsigned int height);

  // set capture framerate
  bool setFramerate(double framerate);

  // set capture format
  bool setFormat(unsigned int width, unsigned int height, double framerate);

  // get capture format
  void getFormat(unsigned int &width, unsigned int &height, double &framerate);

  // set capture MediaType
  bool setMediaType(int mediaTypeID);

  // get current capture MediaType
  int getMediaType();

  // Get the available capture formats
  bool getStreamCapabilities();

private:
  CComPtr<IGraphBuilder> pGraph; // our DS filter graph

  CComPtr<ICaptureGraphBuilder2> pBuild; // the interface to the capture graph builder
                                         // used to build the filter graph

  CComPtr<IBaseFilter> pCapSource; // the capture source filter

  CComPtr<ISampleGrabber> pGrabberI; // the sample grabber's interface and filter
  CComPtr<IBaseFilter> pGrabberFilter;

  CComPtr<IMediaControl> pControl; // The DS filter graph control interface
  CComPtr<IMediaEvent> pEvent;     // The DS filter graph event interface

  vpDirectShowSampleGrabberI sgCB; // Interface used to implement the frame grabber callback

  HRESULT hr; // contains the result of the last operation

  static vpDirectShowDevice *deviceList; // This contains the list of the available capture devices
                                         // it is shared by all the DirectShow Grabbers

  static unsigned int nbDevices; // the number of available devices
  int currentDevice;             // the number of the current device

  // flag to manage CoInitialize() and CoUnInitialze()
  bool initCo;
  // setup the directshow filtergraph with the first available device
  bool initDirectShow();

  // enumerates the different video inputs
  bool enumerate(CComPtr<IEnumMoniker> &ppVideoInputEnum);

  // selects a random video input from the enumeration and returns the
  // associated filter
  bool selectRandomSource(CComPtr<IEnumMoniker> &ppVideoInputEnum, CComPtr<IBaseFilter> &pCapSource);

  // creates the filter graph
  bool createGraph();

  // creates the sample grabber
  bool createSampleGrabber(CComPtr<IBaseFilter> &ppGrabberFilter);

  // checks the capture filter's media type and sets flags as needed
  bool checkSourceType(CComPtr<IPin> &pCapSourcePin);

  // connects the filters as needed
  bool connectSourceToGrabber(CComPtr<IBaseFilter> &pCapSource, CComPtr<IBaseFilter> &pGrabberFilter);

  // used to convert HRESULT-associated error message to a string
  void HRtoStr(std::string str);

  // create the list of the available devices
  bool createDeviceList(CComPtr<IEnumMoniker> &ppVideoInputEnum);

  // get the n-th device if it is available
  bool getDevice(unsigned int n, CComPtr<IBaseFilter> &ppCapSource);

  // get the first available device if any
  unsigned int getFirstUnusedDevice(CComPtr<IBaseFilter> &ppDevice);

  // removes all the filters in the graph
  bool removeAll();

  // Deletes an allocated AM_MEDIA_TYPE structure, including the format block
  void MyDeleteMediaType(AM_MEDIA_TYPE *pmt);

  // Frees the format block in an AM_MEDIA_TYPE structure
  void MyFreeMediaType(AM_MEDIA_TYPE &mt);
};

#endif
#endif
#endif
