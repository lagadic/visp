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
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpImageConvert.h>
#include <visp3/sensor/vpDirectShowSampleGrabberI.h>

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

/*!
        Constructor - creates the semaphore
*/
vpDirectShowSampleGrabberI::vpDirectShowSampleGrabberI()
  : acqGrayDemand(false), acqRGBaDemand(false), specialMediaType(false), invertedSource(false)
{
  // semaphore(0), max value = 1
  copySem = CreateSemaphore(NULL, 0, 1, NULL);
}

/*!
        Destructor - destroys the semaphore
*/
vpDirectShowSampleGrabberI::~vpDirectShowSampleGrabberI()
{
  // destroys the semaphore
  CloseHandle(copySem);
}

STDMETHODIMP vpDirectShowSampleGrabberI::QueryInterface(REFIID riid, void **ppvObject)
{
  if (NULL == ppvObject)
    return E_POINTER;
  if (riid == __uuidof(IUnknown)) {
    *ppvObject = static_cast<IUnknown *>(this);
    return S_OK;
  }
  if (riid == __uuidof(ISampleGrabberCB)) {
    *ppvObject = static_cast<ISampleGrabberCB *>(this);
    return S_OK;
  }
  return E_NOTIMPL;
}

/*!
        The frame grabber callback -
        Called when the input buffer is full.
        Rq : BufferLen == bmpInfo.biWidth*bmpInfo.biHeight*sizeof(vpRGBa)
*/
STDMETHODIMP vpDirectShowSampleGrabberI::BufferCB(double Time, BYTE *pBuffer, long BufferLen)
{
  // if there has been a frame demand
  if (acqGrayDemand || acqRGBaDemand) {
    // check if the connected media is compatible
    if (connectedMediaType.formattype == FORMAT_VideoInfo) {
      // retrieve the image information
      VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER *>(connectedMediaType.pbFormat);
      BITMAPINFOHEADER bmpInfo = pVih->bmiHeader;

      // if biHeight > 0 and the source is not special
      // then  the image needs to be verticaly flipped
      bool flip;
      if (!specialMediaType)
        flip = bmpInfo.biHeight >= 0;
      // the source is fourcc and the image is inverted with this compression
      else if (invertedSource)
        flip = true;
      // fourcc and the image doesn't need to be flipped
      else
        flip = false;

      // if the buffer contains a RGB24 image (DS RGB24 <=> BGR)
      if (connectedMediaType.subtype == MEDIASUBTYPE_RGB24) {
        // if it was an RGBa image demand
        if (acqRGBaDemand) {
          // first, resizes the image as needed
          rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
          // copy and convert the image
          vpImageConvert::BGRToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap, rgbaIm->getWidth(), rgbaIm->getHeight(),
                                    flip);
          // reset the demand boolean
          acqRGBaDemand = false;
        } else // if it was a grayscale image demand
        {
          // first, resizes the image as needed
          grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
          // copy and convert the image
          vpImageConvert::BGRToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth(), grayIm->getHeight(), flip);
          // reset the demand boolean
          acqGrayDemand = false;
        }
      } else {
        unsigned long FourCC;
        FourCC = ((bmpInfo.biCompression & 0xFF000000) >> 24) | ((bmpInfo.biCompression & 0x00FF0000) >> 8) |
                 ((bmpInfo.biCompression & 0x0000FF00) << 8) | (bmpInfo.biCompression & 0x000000FF) << 24;
        // if the buffer contains a like YUV420 image
        if (connectedMediaType.subtype == MEDIASUBTYPE_IYUV || FourCC == 'I420') {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV420ToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap, rgbaIm->getWidth(),
                                         rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV420ToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }

        } else if (connectedMediaType.subtype == MEDIASUBTYPE_YV12) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YV12ToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap, rgbaIm->getWidth(),
                                       rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV420ToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        } else if (connectedMediaType.subtype == MEDIASUBTYPE_YVU9) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YVU9ToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap, rgbaIm->getWidth(),
                                       rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV420ToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        } else if (connectedMediaType.subtype == MEDIASUBTYPE_YUY2 || connectedMediaType.subtype == MEDIASUBTYPE_YUYV) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YCbCrToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap,
                                        rgbaIm->getWidth() * rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YCbCrToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        } else if (connectedMediaType.subtype == MEDIASUBTYPE_YVYU) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YCrCbToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap,
                                        rgbaIm->getWidth() * rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YCbCrToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        } else if (connectedMediaType.subtype == MEDIASUBTYPE_UYVY) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV422ToRGBa(pBuffer, (unsigned char *)rgbaIm->bitmap,
                                         rgbaIm->getWidth() * rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::YUV422ToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        } else if (connectedMediaType.subtype == MEDIASUBTYPE_RGB32) {
          // if it was an RGBa image demand
          if (acqRGBaDemand) {
            // first, resizes the image as needed
            rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            // copy(pBuffer ,pBuffer +
            // 4*rgbaIm->getWidth()*rgbaIm->getHeight(),rgbaIm->bitmap);
            memcpy(rgbaIm->bitmap, pBuffer, 4 * rgbaIm->getWidth() * rgbaIm->getHeight());
            // reset the demand boolean
            acqRGBaDemand = false;
          } else // if it was a grayscale image demand
          {
            // first, resizes the image as needed
            grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
            // copy and convert the image
            vpImageConvert::RGBaToGrey(pBuffer, grayIm->bitmap, grayIm->getWidth() * grayIm->getHeight());
            // reset the demand boolean
            acqGrayDemand = false;
          }
        }
      }
    }

    // increment the semaphore - allows acquire to continue execution
    ReleaseSemaphore(copySem, 1, NULL);
  }
  return S_OK;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_sensor.a(vpDirectShowSampleGrabberI.cpp.o) has no symbols
void dummy_vpDirectShowSampleGrabberI(){};
#endif

#endif
