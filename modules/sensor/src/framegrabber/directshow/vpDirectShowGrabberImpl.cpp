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
 * DirectShow framegrabber implementation.
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <visp3/sensor/vpDirectShowGrabberImpl.h>

vpDirectShowDevice *vpDirectShowGrabberImpl::deviceList = NULL;
unsigned int vpDirectShowGrabberImpl::nbDevices;

/*!
        Converts a HRESULT into the corresponding error message
*/
void vpDirectShowGrabberImpl::HRtoStr(std::string str)
{
  TCHAR szErr[MAX_ERROR_TEXT_LEN];
  DWORD res = AMGetErrorText(hr, szErr, MAX_ERROR_TEXT_LEN);

  if (res == 0)
    str = "Unknown Error: 0x%2x";

  char msg[MAX_ERROR_TEXT_LEN];
  sprintf(msg, "%s", szErr);
  str = msg;
}

/*!
        Constructor.
        Initializes COM.
*/
vpDirectShowGrabberImpl::vpDirectShowGrabberImpl()
{
  init = false;
  initCo = false;
  // COM initialization
  if (FAILED(hr = CoInitializeEx(NULL, COINIT_MULTITHREADED))) {
    std::string err;
    HRtoStr(err);
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Can't initialize COM\n" + err));
  }
  initCo = true;

  // create the device list
  if (deviceList == NULL) {
    CComPtr<IEnumMoniker> pVideoInputEnum = NULL;

    if (enumerate(pVideoInputEnum)) {
      createDeviceList(pVideoInputEnum);
    }
    // not used anymore, so we release it
    pVideoInputEnum.Release();
  }
}

/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabberImpl::open()
{
  // create the device list
  if (deviceList == NULL) {
    CComPtr<IEnumMoniker> pVideoInputEnum = NULL;

    if (enumerate(pVideoInputEnum)) {
      createDeviceList(pVideoInputEnum);
    }
    // not used anymore, so we release it
    pVideoInputEnum.Release();
  }

  init = initDirectShow();
  if (!init) {
    std::string err;
    HRtoStr(err);
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, err));
  }
}
/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabberImpl::open(vpImage<unsigned char> &I) { open(); }

/*!
        Creates the filter graph and select the first available device.
        \exception initializationError
*/
void vpDirectShowGrabberImpl::open(vpImage<vpRGBa> &I) { open(); }

/*!
        Initialization method
        Creates the capture filter graph
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::initDirectShow()
{

  // get the first working device's filter (unused and getdevice works on it)
  currentDevice = getFirstUnusedDevice(pCapSource);

  if (currentDevice == nbDevices)
    return false;

  // create the filter graph
  if (!createGraph())
    return false;

  // we add the capture source to the filter graph
  if (FAILED(hr = pGraph->AddFilter(pCapSource, L"Capture Filter")))
    return false;

  // we create a sample grabber
  if (!createSampleGrabber(pGrabberFilter))
    return false;

  // we add the grabber to the filter graph
  if (FAILED(hr = pGraph->AddFilter(pGrabberFilter, L"SampleGrabber")))
    return false;

  // we connect the pins
  if (!connectSourceToGrabber(pCapSource, pGrabberFilter))
    return false;

  // get the current connected media type (needed by the callback)
  if (FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
    return false;

  // Gets the various graph's interfaces
  CComPtr<IMediaFilter> pMediaFilter;

  pGraph->QueryInterface(IID_IMediaFilter, (void **)&pMediaFilter);
  pGraph->QueryInterface(IID_IMediaControl, reinterpret_cast<void **>(&pControl));
  pGraph->QueryInterface(IID_IMediaEvent, (void **)&pEvent);

  pMediaFilter->SetSyncSource(NULL);
  pMediaFilter.Release();

  return true;
}

/*!
        Destructor
*/
vpDirectShowGrabberImpl::~vpDirectShowGrabberImpl() { close(); }

/*!
        Create a video device enumerator.
        \param ppVideoInputEnum The video device enumerator
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::enumerate(CComPtr<IEnumMoniker> &ppVideoInputEnum)
{
  CComPtr<ICreateDevEnum> pDevEnum = NULL;
  bool res = false;

  // Enumerate system devices
  hr = pDevEnum.CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER);

  // if it is a success
  if (SUCCEEDED(hr)) {
    // Create a video input device enumerator
    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &ppVideoInputEnum, 0);

    if (hr == S_OK)
      res = true;
  }

  pDevEnum.Release();
  return res;
}

/*!
        Create the device list by enumerating the video input devices
        \param ppVideoInputEnum A video device enumerator
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::createDeviceList(CComPtr<IEnumMoniker> &ppVideoInputEnum)
{
  CComPtr<IMoniker> pMoniker[10]; // const max devices
  unsigned long nbMoniker;

  ppVideoInputEnum->Reset();

  // Enumerates the different inputs
  ppVideoInputEnum->Next(10, reinterpret_cast<IMoniker **>(&pMoniker), &nbMoniker);

  // if no input device
  if (nbMoniker == 0)
    return false;

  deviceList = new vpDirectShowDevice[nbMoniker];

  nbDevices = (unsigned int)nbMoniker;

  // we try to get the properties of each moniker, if it fails, we skip to the
  // next one and  decrement the number of valid devices
  unsigned int i = 0;
  unsigned int j = 0;
  while (i < nbDevices) {
    if (!deviceList[i].init(pMoniker[j])) {
      // if we can't get the device properties, skip to the next device
      j++;
      nbDevices--;
    } else {
      i++;
      j++;
    }
  }

  // if no working input device
  if (nbDevices == 0)
    return false;

  // we release the monikers
  for (unsigned int i = 0; i < nbMoniker; i++) {
    pMoniker[i].Release();
  }

  return true;
}

/*!
        Gets the filter associated with device n if it exists.
        \param n Number of the device
        \param ppDevice The n-th device
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::getDevice(unsigned int n, CComPtr<IBaseFilter> &ppDevice)
{
  // if n is invalid, quit
  if (n >= nbDevices)
    return false;

  // if the device is already in use, quit
  if (deviceList[n].getState() == true)
    return false;

  // if we can't enumerate the devices, quit
  CComPtr<IEnumMoniker> pVideoInputEnum = NULL;
  if (!enumerate(pVideoInputEnum))
    return false;

  CComPtr<IMoniker> pMoniker = NULL;
  bool deviceFound = false;

  // Enumerates the different inputs
  while (pVideoInputEnum->Next(1, &pMoniker, NULL) == S_OK && !deviceFound) {
    // implicit conversion should work ...
    if (deviceList[n] == vpDirectShowDevice(pMoniker)) {
      // we get the filter
      if (SUCCEEDED(pMoniker->BindToObject(0, 0, IID_IBaseFilter, (void **)&ppDevice))) {
        // now the device is in use
        deviceList[n].setInUse();
        deviceFound = true;
      } else {
        break;
      } // we can't get the device's filter, quit
    }
    pMoniker.Release();
  }

  pVideoInputEnum.Release();

  return deviceFound;
}

/*!
        Searches for the first unused device.
        \param ppDevice The first device filter's interface
        \return number of the found device. If the operation wasn't
   successfull, n=nbDevices.
*/
unsigned int vpDirectShowGrabberImpl::getFirstUnusedDevice(CComPtr<IBaseFilter> &ppDevice)
{
  unsigned int n = 0;
  bool found = false;

  for (n = 0; n < nbDevices && !found; n++) {
    // if the device is not being used
    if (!deviceList[n].getState()) {
      if (getDevice(n, ppDevice)) {
        found = true;
        deviceList[n].setInUse();
        return n;
      }
    }
  }

  return n;
}

/*!
        Create the capture graph
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::createGraph()
{

  // Create the Capture Graph Builder.
  hr = pBuild.CoCreateInstance(CLSID_CaptureGraphBuilder2, 0, CLSCTX_INPROC_SERVER);

  if (SUCCEEDED(hr)) {
    // Create the Filter Graph Manager.
    hr = pGraph.CoCreateInstance(CLSID_FilterGraph, 0, CLSCTX_INPROC_SERVER);

    if (SUCCEEDED(hr)) {
      // Initialize the Capture Graph Builder.
      pBuild->SetFiltergraph(pGraph);

      return true;
    }
  }

  return false;
}

/*!
        Creates the grabber.
        \param ppGrabberFilter The created grabber filter's interface
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::createSampleGrabber(CComPtr<IBaseFilter> &ppGrabberFilter)
{
  // Creates the sample grabber
  hr = ppGrabberFilter.CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER);

  if (FAILED(hr))
    return false;

  // gets the SampleGrabber interface in order to configure it later
  hr = ppGrabberFilter->QueryInterface(IID_ISampleGrabber, reinterpret_cast<void **>(&pGrabberI));

  if (FAILED(hr))
    return false;

  // configure the grabber
  AM_MEDIA_TYPE mt;
  ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));

  mt.majortype = MEDIATYPE_Video;

  // ask for a connection
  mt.subtype = MEDIATYPE_NULL;

  if (FAILED(hr = pGrabberI->SetMediaType(&mt)))
    return false;

  // configure the callback of the grabber
  pGrabberI->SetCallback(&sgCB, 1);

  // grab only one frame at a time
  pGrabberI->SetOneShot(TRUE);

  // no need to bufferize the sample in the grabber
  pGrabberI->SetBufferSamples(false);

  return true;
}

/*!
  Checks the capture filter's ouput pin media type.
  Some formats' orientation is automatically handled by directshow but this
  isn't the case for all the existing formats. If the grabbed image is
  inverted, the source format's orientation may not be known by directshow so
  consider adding an additional handler for this FourCC format.

  \return True if the connection media type was a video media type.
*/
bool vpDirectShowGrabberImpl::checkSourceType(CComPtr<IPin> &pCapSourcePin)
{
  // retrieves the connected media type
  AM_MEDIA_TYPE mt;
  if (FAILED(pCapSourcePin->ConnectionMediaType(&mt)))
    return false;

  if (mt.majortype != MEDIATYPE_Video)
    return false;

  // Known RGB formats
  if (mt.subtype == MEDIASUBTYPE_ARGB32 || mt.subtype == MEDIASUBTYPE_RGB32 || mt.subtype == MEDIASUBTYPE_RGB24 ||
      mt.subtype == MEDIASUBTYPE_RGB555 || mt.subtype == MEDIASUBTYPE_RGB565 || mt.subtype == MEDIASUBTYPE_RGB8 ||
      mt.subtype == MEDIASUBTYPE_RGB4 || mt.subtype == MEDIASUBTYPE_RGB1) {
    // image orientation will be handled "automatically"
    sgCB.specialMediaType = false;
  }
  // Known YUV formats
  else if (mt.subtype == MEDIASUBTYPE_AYUV || mt.subtype == MEDIASUBTYPE_UYVY || mt.subtype == MEDIASUBTYPE_Y411 ||
           mt.subtype == MEDIASUBTYPE_Y41P || mt.subtype == MEDIASUBTYPE_Y211 || mt.subtype == MEDIASUBTYPE_YUY2 ||
           mt.subtype == MEDIASUBTYPE_YVYU || mt.subtype == MEDIASUBTYPE_YUYV || mt.subtype == MEDIASUBTYPE_IF09 ||
           mt.subtype == MEDIASUBTYPE_IYUV || mt.subtype == MEDIASUBTYPE_YV12 || mt.subtype == MEDIASUBTYPE_YVU9) {
    // image orientation will be handled "automatically"
    sgCB.specialMediaType = false;
  }
  // FOURCC formats
  else {
    // invertedSource boolean will decide the bitmap orientation
    sgCB.specialMediaType = true;

    DWORD format;
    VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER *>(mt.pbFormat);
    BITMAPINFOHEADER bmpInfo = pVih->bmiHeader;

    // get the fourcc code
    format = ((bmpInfo.biCompression & 0xFF000000) >> 24) | ((bmpInfo.biCompression & 0x00FF0000) >> 8) |
             ((bmpInfo.biCompression & 0x0000FF00) << 8) | (bmpInfo.biCompression & 0x000000FF) << 24;

    std::cout << "This format is not one of the standard YUV or RGB format "
                 "supported by DirectShow.\n"
              << "FourCC : " << (char)(bmpInfo.biCompression & 0x000000FF)
              << (char)((bmpInfo.biCompression & 0x0000FF00) >> 8) << (char)((bmpInfo.biCompression & 0x00FF0000) >> 16)
              << (char)((bmpInfo.biCompression & 0xFF000000) >> 24) << std::endl;

    // Y800 is top-down oriented so the image doesn't have to be flipped
    // vertically
    if (format == 'Y800') {
      sgCB.invertedSource = false;
    }
    // cyuv seems to be the only yuv bottom-up oriented format (image has to
    // be flipped)
    else if (format == 'cyuv') {
      sgCB.invertedSource = true;
    }
    // insert code for other fourcc formats here
    // see fourcc.org to know which format is bottom-up oriented and thus
    // needs invertedSource sets to true
    else {
      std::cout << "Unknown FourCC compression type, assuming top-down "
                   "orientation. Image may be inverted."
                << std::endl;
      sgCB.invertedSource = false; // consider that the image is topdown oriented by default
    }
  }

  return true;
}

/*!
        Connects the capture device's output pin to the grabber's input pin
        \param pCapSource The capture device
        \param pGrabber The grabber
        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::connectSourceToGrabber(CComPtr<IBaseFilter> &_pCapSource,
                                                     CComPtr<IBaseFilter> &_pGrabberFilter)
{
  /*
  //get the capture source's output pin
  CComPtr<IPin> pCapSourcePin;
  if(FAILED(pBuild->FindPin(_pCapSource, PINDIR_OUTPUT, NULL, NULL, false, 0,
  &pCapSourcePin))) return false;

  //get the grabber's input pin
  CComPtr<IPin> pGrabberInputPin;
  if(FAILED(pBuild->FindPin(_pGrabberFilter, PINDIR_INPUT, NULL, NULL, false,
  0, &pGrabberInputPin))) return false;

  //connect the two of them
  if(FAILED(pGraph->Connect(pCapSourcePin, pGrabberInputPin)))
          return false;

  //not used anymore, we can release it
  pGrabberInputPin.Release();
  */
  if (FAILED(hr = pBuild->RenderStream(NULL, NULL, _pCapSource, NULL, _pGrabberFilter)))
    return false;

  /*
  //get the grabber's output pin
  CComPtr<IPin> pGrabberOutputPin;
  if(FAILED(pBuild->FindPin(_pGrabberFilter, PINDIR_OUTPUT, NULL, NULL, false,
  0, &pGrabberOutputPin))) return false;
  */
  // get the Null renderer
  CComPtr<IBaseFilter> pNull = NULL;
  if (FAILED(pNull.CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER)))
    return false;
  /*
          //get the null renderer's input pin
          CComPtr<IPin> pNullInputPin;
          if(FAILED(pBuild->FindPin(pNull, PINDIR_INPUT, NULL, NULL, false, 0,
     &pNullInputPin))) return false;

          //connect the grabber's output to the null renderer
          if(	FAILED(pGraph->AddFilter(pNull, L"NullRenderer")) ||
                  FAILED(pGraph->Connect(pGrabberOutputPin, pNullInputPin)))
                  return false;
  */

  if (FAILED(pGraph->AddFilter(pNull, L"NullRenderer")) ||
      FAILED(pBuild->RenderStream(NULL, NULL, _pGrabberFilter, NULL, pNull)))
    return false;

  // get the capture source's output pin
  CComPtr<IPin> pCapSourcePin;
  if (FAILED(pBuild->FindPin(_pCapSource, PINDIR_OUTPUT, NULL, NULL, false, 0, &pCapSourcePin)))
    return false;
  // checks the media type of the capture filter
  // and if the image needs to be inverted
  if (!checkSourceType(pCapSourcePin))
    return false;

  // release the remaining interfaces
  pCapSourcePin.Release();
  pNull.Release();
  //	pGrabberOutputPin.Release();
  //	pNullInputPin.Release();

  return true;
}

/*!
        Removes all the filters from the filter graph

        \return Was the operation successful
*/
bool vpDirectShowGrabberImpl::removeAll()
{
  CComPtr<IEnumFilters> pEnum = NULL;
  CComPtr<IBaseFilter> pFilter;
  ULONG cFetched;

  if (FAILED(hr = pGraph->EnumFilters(&pEnum)))
    return false;

  while (pEnum->Next(1, &pFilter, &cFetched) == S_OK) {
    if (FAILED(hr = pGraph->RemoveFilter(pFilter)))
      return false;
    pFilter.Release();
    pEnum->Reset();
  }

  pEnum.Release();
  return true;
}

/*!
        Grabs a rgba image from the video stream.
        Uses a semaphore to synchronize with the framegrabber callback.

        \param I The image to fill with the grabbed frame

        \exception otherError Can't grab the frame
*/
void vpDirectShowGrabberImpl::acquire(vpImage<vpRGBa> &I)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  // set the rgbaIm pointer on I (will be filled on the next framegrabber
  // callback)
  sgCB.rgbaIm = &I;
  // there is an acquire demand (execute copy during callback)
  sgCB.acqRGBaDemand = true;

  // Run the graph to grab a frame
  pControl->Run();

  // Wait untill it's done
  long ev;
  hr = pEvent->WaitForCompletion(MAX_DELAY, &ev);

  width = I.getWidth();
  height = I.getHeight();

  // wait for the end of the next callback (copy)
  if (WaitForSingleObject(sgCB.copySem, MAX_DELAY) != WAIT_OBJECT_0)
    throw(vpFrameGrabberException(vpFrameGrabberException::otherError, "Can't grab the frame, callback timeout"));
}

/*!
        Grabs a grayscale image from the video stream.
        Uses a semaphore to synchronize with the framegrabber callback.

        \param I The image to fill with the grabbed frame

        \exception otherError Can't grab the frame
*/
void vpDirectShowGrabberImpl::acquire(vpImage<unsigned char> &I)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  // set the grayIm pointer on I (will be filled on the next framegrabber
  // callback)
  sgCB.grayIm = &I;
  // there is an acquire demand (execute copy during callback)
  sgCB.acqGrayDemand = true;

  // Run the graph to grab a frame
  pControl->Run();

  // Wait untill it's done
  long ev;
  hr = pEvent->WaitForCompletion(MAX_DELAY, &ev);

  width = I.getWidth();
  height = I.getHeight();

  // wait for the end of the next callback (copy)
  if (WaitForSingleObject(sgCB.copySem, MAX_DELAY) != WAIT_OBJECT_0)
    throw(vpFrameGrabberException(vpFrameGrabberException::otherError, "Can't grab the frame, callback timeout"));
}

/*!
        Set the device (or camera) used by the grabber
        \param id : Identifier of the device to use.

        \return true id the device is set successfully, false otherwise.
*/
bool vpDirectShowGrabberImpl::setDevice(unsigned int id)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  // if n is invalid, or the device is already in use, quit
  if (id >= nbDevices || deviceList[id].getState() == true)
    return false;

  // we stop the graph
  pControl->Stop();

  // then we can safely remove all the filters
  if (!removeAll())
    return false;

  // we release the previous source's interface
  pCapSource.Release();

  // here reset inUse in the old DSDevice
  deviceList[currentDevice].resetInUse();

  // we add the grabber back in the graph
  pGraph->AddFilter(pGrabberFilter, L"SampleGrabber");

  // get the n-th device's filter
  if (!getDevice(id, pCapSource))
    return false;

  // we add the capture source to the filter graph
  if (FAILED(hr = pGraph->AddFilter(pCapSource, L"Capture Filter")))
    return false;

  // we connect the pins
  if (!connectSourceToGrabber(pCapSource, pGrabberFilter))
    return false;

  // get the current connected media type (needed by the callback)
  if (FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType)))) {
    return false;
  }

  // the device is now in use
  deviceList[id].setInUse();
  currentDevice = id;

  return true;
}

/*!
        Displays the list of devices on the standard output
*/
void vpDirectShowGrabberImpl::displayDevices()
{
  if (deviceList == NULL) {
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  for (unsigned int i = 0; i < nbDevices; i++)
    std::cout << i << " : " << deviceList[i].getName() << std::endl;

  std::cout << "Current device : " << currentDevice << std::endl << std::endl;
}

/*!
        Stops the framegrabber
*/

void vpDirectShowGrabberImpl::close()
{
  // the current device isn't being used anymore
  if (init) {
    deviceList[currentDevice].resetInUse();
    init = false;
  }
  if (initCo) {
    // uninstalls COM
    CoUninitialize();
    initCo = false;
  }
}
/*!
        Set image size
*/
bool vpDirectShowGrabberImpl::setImageSize(unsigned int width, unsigned int height)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  return setFormat(width, height, NULL);
}

/*!
        Set capture framerate
*/
bool vpDirectShowGrabberImpl::setFramerate(double framerate)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)sgCB.connectedMediaType.pbFormat;
  return setFormat(pVih->bmiHeader.biWidth, pVih->bmiHeader.biHeight, framerate);
}
/*!
        Set the capture format
*/
bool vpDirectShowGrabberImpl::setFormat(unsigned int width, unsigned int height, double framerate)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  bool found = false;

  // gets the stream config interface
  IAMStreamConfig *pConfig = NULL;

  if (FAILED(hr = pBuild->FindInterface(&LOOK_UPSTREAM_ONLY, // Capture pin. / Preview pin
                                        0,                   // Any media type.
                                        pGrabberFilter,      // Pointer to the grabber filter.
                                        IID_IAMStreamConfig, (void **)&pConfig)))
    return false;

  // gets the video control interface
  IAMVideoControl *pVideoControl = NULL;

  if (FAILED(hr = pBuild->FindInterface(&LOOK_UPSTREAM_ONLY, // Capture pin. / Preview pin
                                        0,                   // Any media type.
                                        pGrabberFilter,      // Pointer to the grabber filter.
                                        IID_IAMVideoControl, (void **)&pVideoControl)))
    return false;

  // get the grabber's input pin
  CComPtr<IPin> pCapSourcePin;
  if (FAILED(pBuild->FindPin(pCapSource, PINDIR_OUTPUT, NULL, NULL, false, 0, &pCapSourcePin)))
    return false;

  int iCount = 0, iSize = 0;
  if (FAILED(hr = pConfig->GetNumberOfCapabilities(&iCount, &iSize)))
    return false;

  // Check the size to make sure we pass in the correct structure.
  if (iSize == sizeof(VIDEO_STREAM_CONFIG_CAPS)) {
    // Use the video capabilities structure.

    for (int iFormat = 0; iFormat < iCount; iFormat++) {
      VIDEO_STREAM_CONFIG_CAPS scc;
      AM_MEDIA_TYPE *pmtConfig;
      hr = pConfig->GetStreamCaps(iFormat, &pmtConfig, (BYTE *)&scc);
      //			VIDEOINFOHEADER *pVih =
      //(VIDEOINFOHEADER*)pmtConfig->pbFormat;

      //			pVih->bmiHeader.biWidth;
      //			pVih->bmiHeader.biHeight;
      //			10000000 /pVih->AvgTimePerFrame;
      //			std::cout<<"available image size :
      //"<<pVih->bmiHeader.biWidth<<" x "<<pVih->bmiHeader.biHeight<<" at
      //"<<10000000 /pVih->AvgTimePerFrame<<std::endl;
      //			std::cout<<"compression :
      //"<<pVih->bmiHeader.biCompression<<std::endl;
      if (SUCCEEDED(hr) && found == false) {
        /* Examine the format, and possibly use it. */
        if ((pmtConfig->majortype == sgCB.connectedMediaType.majortype) &&
            (pmtConfig->subtype == sgCB.connectedMediaType.subtype) &&
            (pmtConfig->formattype == sgCB.connectedMediaType.formattype) &&
            (pmtConfig->cbFormat >= sizeof(VIDEOINFOHEADER)) && (pmtConfig->pbFormat != NULL)) {
          VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)pmtConfig->pbFormat;

          LONG lWidth = pVih->bmiHeader.biWidth;
          LONG lHeight = pVih->bmiHeader.biHeight;
          if (framerate != NULL) {
            if ((unsigned int)lWidth == width && (unsigned int)lHeight == height) {

              pVih->AvgTimePerFrame = (LONGLONG)(10000000 / framerate);
              // set the capture media type and the grabber media type
              if (FAILED(hr = pConfig->SetFormat(pmtConfig)) || FAILED(hr = pGrabberI->SetMediaType(pmtConfig)))
                return false;
              // Run the graph to grab a frame
              pControl->Run();

              // get the current connected media type (needed by the callback)
              if (FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
                return false;
              pVih = (VIDEOINFOHEADER *)sgCB.connectedMediaType.pbFormat;
              LONGLONG ActualFrameDuration;
              if (FAILED(hr = pVideoControl->GetCurrentActualFrameRate(pCapSourcePin, &ActualFrameDuration)))
                std::cout << "Current format (not sure): " << width << " x " << height << " at "
                          << 10000000 / pVih->AvgTimePerFrame << " fps" << std::endl
                          << std::endl;
              else {
                std::cout << "Current format : " << width << " x " << height << " at " << 10000000 / ActualFrameDuration
                          << " fps" << std::endl
                          << std::endl;
                pVih->AvgTimePerFrame = ActualFrameDuration;
              }
              found = true;
            }
          } else {
            if ((unsigned int)lWidth == width && (unsigned int)lHeight == height) {
              pVih->AvgTimePerFrame = scc.MinFrameInterval;
              // set the capture media type and the grabber media type
              if (FAILED(hr = pConfig->SetFormat(pmtConfig)) || FAILED(hr = pGrabberI->SetMediaType(pmtConfig)))
                return false;
              // get the current connected media type (needed by the callback)
              if (FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
                return false;
              pVih = (VIDEOINFOHEADER *)sgCB.connectedMediaType.pbFormat;
              found = true;
              std::cout << "Current format : " << width << " x " << height << " at "
                        << (10000000 / pVih->AvgTimePerFrame) << " fps" << std::endl
                        << std::endl;
            }
          }
        }
      }
      // Delete the media type when you are done.
      MyDeleteMediaType(pmtConfig);
    }
  }
  if (!found)
    if (framerate != NULL)
      std::cout << "The " << width << " x " << height << " at " << framerate
                << " fps source image format is not available. " << std::endl
                << std::endl;
    else
      std::cout << "The " << width << " x " << height << "source image size is not available. " << std::endl
                << std::endl;

  return found;
}
/*!
        Get the current capture format and framerate.
        \param width : Image width.
        \param height : Image height.
        \param framerate : Framerate acquisition.
*/
void vpDirectShowGrabberImpl::getFormat(unsigned int &width, unsigned int &height, double &framerate)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }
  VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)sgCB.connectedMediaType.pbFormat;
  width = (unsigned int)pVih->bmiHeader.biWidth;
  height = (unsigned int)pVih->bmiHeader.biHeight;
  framerate = (double)(10000000 / pVih->AvgTimePerFrame);
}
/*!
        Get the available capture formats
*/
bool vpDirectShowGrabberImpl::getStreamCapabilities()
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  // gets the stream config interface
  IAMStreamConfig *pConfig = NULL;

  if (FAILED(hr = pBuild->FindInterface(&LOOK_UPSTREAM_ONLY, // Capture pin. / Preview pin
                                        0,                   // Any media type.
                                        pGrabberFilter,      // Pointer to the grabber filter.
                                        IID_IAMStreamConfig, (void **)&pConfig)))
    return false;

  int iCount = 0, iSize = 0;
  if (FAILED(hr = pConfig->GetNumberOfCapabilities(&iCount, &iSize)))
    return false;

  // Check the size to make sure we pass in the correct structure.
  if (iSize == sizeof(VIDEO_STREAM_CONFIG_CAPS)) {
    std::cout << "Available MediaTypes : " << std::endl << std::endl;
    // Use the video capabilities structure.
    for (int iFormat = 0; iFormat < iCount; iFormat++) {
      VIDEO_STREAM_CONFIG_CAPS scc;
      AM_MEDIA_TYPE *pmtConfig;
      hr = pConfig->GetStreamCaps(iFormat, &pmtConfig, (BYTE *)&scc);

      if (SUCCEEDED(hr)) {
        /* Examine the format, and possibly use it. */
        VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)pmtConfig->pbFormat;

        //				LONG lWidth = pVih->bmiHeader.biWidth;
        //				LONG lHeight =
        // pVih->bmiHeader.biHeight; 				SIZE
        // dimensions={lWidth,lHeight};
        //				LONGLONG lAvgTimePerFrame =
        // pVih->AvgTimePerFrame;
        std::cout << "MediaType : " << iFormat << std::endl;

        if (pmtConfig->subtype == MEDIASUBTYPE_ARGB32)
          std::cout << "subtype (not supported): MEDIASUBTYPE_ARGB32" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB32)
          std::cout << "subtype : MEDIASUBTYPE_RGB32" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB24)
          std::cout << "subtype : MEDIASUBTYPE_RGB24" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB555)
          std::cout << "subtype (not supported): MEDIASUBTYPE_RGB555" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB565)
          std::cout << "subtype (not supported): MEDIASUBTYPE_RGB565" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB8)
          std::cout << "subtype (not supported): MEDIASUBTYPE_RGB8" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB4)
          std::cout << "subtype (not supported): MEDIASUBTYPE_RGB4" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_RGB1)
          std::cout << "subtype (not supported): MEDIASUBTYPE_RGB1" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_YV12)
          std::cout << "subtype : MEDIASUBTYPE_YV12" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_YVU9)
          std::cout << "subtype : MEDIASUBTYPE_YVU9" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_YUY2)
          std::cout << "subtype : MEDIASUBTYPE_YUY2" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_YUYV)
          std::cout << "subtype : MEDIASUBTYPE_YUYV" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_YVYU)
          std::cout << "subtype : MEDIASUBTYPE_YVYU" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_IYUV)
          std::cout << "subtype : MEDIASUBTYPE_IYUV" << std::endl;
        else if (pmtConfig->subtype == MEDIASUBTYPE_UYVY)
          std::cout << "subtype : MEDIASUBTYPE_UYVY" << std::endl;
        else if ((((pVih->bmiHeader.biCompression & 0xFF000000) >> 24) |
                  ((pVih->bmiHeader.biCompression & 0x00FF0000) >> 8) |
                  ((pVih->bmiHeader.biCompression & 0x0000FF00) << 8) |
                  ((pVih->bmiHeader.biCompression & 0x000000FF) << 24)) == 'I420')
          std::cout << "subtype : I420" << std::endl;
        else
          std::cout << "subtype (not supported) :" << (char)(pVih->bmiHeader.biCompression & 0x000000FF)
                    << (char)((pVih->bmiHeader.biCompression & 0x0000FF00) >> 8)
                    << (char)((pVih->bmiHeader.biCompression & 0x00FF0000) >> 16)
                    << (char)((pVih->bmiHeader.biCompression & 0xFF000000) >> 24) << std::endl;

        std::cout << "image size : " << pVih->bmiHeader.biWidth << " x " << pVih->bmiHeader.biHeight << std::endl;
        std::cout << "framerate range: [" << 10000000 / scc.MaxFrameInterval << "," << 10000000 / scc.MinFrameInterval
                  << "]" << std::endl
                  << std::endl;

        /*
                                        long frameRateNum;
                                        LONGLONG *frameRateList;
                                        if(FAILED(hr =
           pVideoControl->GetFrameRateList(pCapSourcePin,iFormat,dimensions,
           //inputs &frameRateNum, &frameRateList))) //outputs return false;
                                        for(int i=0; i<(int)frameRateNum ;
           i++)
                                        {
                                                std::cout<<(float)(10000000/frameRateList[i])<<"
           fps"<<std::endl;
                                        }
                                        std::cout<<std::endl;
        */
      }
      // Delete the media type when you are done.
      MyDeleteMediaType(pmtConfig);
    }
  }
  return true;
}
/*!
        Set capture Mediatype
*/
bool vpDirectShowGrabberImpl::setMediaType(int mediaTypeID)
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  // gets the stream config interface
  IAMStreamConfig *pConfig = NULL;

  if (FAILED(hr = pBuild->FindInterface(&LOOK_UPSTREAM_ONLY, // Capture pin. / Preview pin
                                        0,                   // Any media type.
                                        pGrabberFilter,      // Pointer to the grabber filter.
                                        IID_IAMStreamConfig, (void **)&pConfig)))
    return false;

  VIDEO_STREAM_CONFIG_CAPS scc;
  AM_MEDIA_TYPE *pmtConfig;
  hr = pConfig->GetStreamCaps(mediaTypeID, &pmtConfig, (BYTE *)&scc);

  if (SUCCEEDED(hr)) {
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)pmtConfig->pbFormat;
    pVih->AvgTimePerFrame = scc.MinFrameInterval;
    // set the capture media type and the grabber media type
    if (FAILED(hr = pGrabberI->SetMediaType(pmtConfig)) || FAILED(hr = pConfig->SetFormat(pmtConfig)))
      return false;
    // get the current connected media type (needed by the callback)
    if (FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
      return false;
  }
  // Delete the media type when you are done.
  MyDeleteMediaType(pmtConfig);
  return true;
}

/*
        Get current capture MediaType
        \return mediaTypeID (-1 if failed)
*/
int vpDirectShowGrabberImpl::getMediaType()
{
  if (init == false) {
    close();
    throw(vpFrameGrabberException(vpFrameGrabberException::initializationError, "Initialization not done"));
  }

  int mediaTypeID = -1;
  VIDEOINFOHEADER *pVihConnected = (VIDEOINFOHEADER *)sgCB.connectedMediaType.pbFormat;

  // gets the stream config interface
  IAMStreamConfig *pConfig = NULL;

  if (FAILED(hr = pBuild->FindInterface(&LOOK_UPSTREAM_ONLY, // Capture pin. / Preview pin
                                        0,                   // Any media type.
                                        pGrabberFilter,      // Pointer to the grabber filter.
                                        IID_IAMStreamConfig, (void **)&pConfig)))
    return -1;

  int iCount = 0, iSize = 0;
  if (FAILED(hr = pConfig->GetNumberOfCapabilities(&iCount, &iSize)))
    return -1;

  // Check the size to make sure we pass in the correct structure.
  if (iSize == sizeof(VIDEO_STREAM_CONFIG_CAPS)) {
    // Use the video capabilities structure.
    for (int iFormat = 0; iFormat < iCount; iFormat++) {
      VIDEO_STREAM_CONFIG_CAPS scc;
      AM_MEDIA_TYPE *pmtConfig;
      hr = pConfig->GetStreamCaps(iFormat, &pmtConfig, (BYTE *)&scc);

      if (SUCCEEDED(hr)) {
        /* Examine the format, and possibly use it. */
        if ((pmtConfig->majortype == sgCB.connectedMediaType.majortype) &&
            (pmtConfig->subtype == sgCB.connectedMediaType.subtype) &&
            (pmtConfig->formattype == sgCB.connectedMediaType.formattype) &&
            (pmtConfig->cbFormat >= sizeof(VIDEOINFOHEADER)) && (pmtConfig->pbFormat != NULL)) {
          VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER *)pmtConfig->pbFormat;
          if (pVih->bmiHeader.biWidth == pVihConnected->bmiHeader.biWidth &&
              pVih->bmiHeader.biHeight == pVihConnected->bmiHeader.biHeight)
            mediaTypeID = iFormat;
        }
      }
      // Delete the media type when you are done.
      MyDeleteMediaType(pmtConfig);
    }
  }
  return mediaTypeID;
}

/*!
        Deletes an allocated AM_MEDIA_TYPE structure, including the format
   block
*/
void vpDirectShowGrabberImpl::MyDeleteMediaType(AM_MEDIA_TYPE *pmt)
{
  if (pmt != NULL) {
    MyFreeMediaType(*pmt); // See FreeMediaType for the implementation.
    CoTaskMemFree(pmt);
  }
}

/*!
        Frees the format block in an AM_MEDIA_TYPE structure.
*/
void vpDirectShowGrabberImpl::MyFreeMediaType(AM_MEDIA_TYPE &mt)
{
  if (mt.cbFormat != 0) {
    CoTaskMemFree((PVOID)mt.pbFormat);
    mt.cbFormat = 0;
    mt.pbFormat = NULL;
  }
  if (mt.pUnk != NULL) {
    // Unecessary because pUnk should not be used, but safest.
    mt.pUnk->Release();
    mt.pUnk = NULL;
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_sensor.a(vpDirectShowGrabberImpl.cpp.o) has no symbols
void dummy_vpDirectShowGrabberImpl(){};
#endif
#endif
