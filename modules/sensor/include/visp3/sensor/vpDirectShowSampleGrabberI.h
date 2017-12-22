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
 * DirectShow framegrabber callback.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef vpDirectShowSampleGrabberI_hh
#define vpDirectShowSampleGrabberI_hh

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <dshow.h>
#include <qedit.h>
#include <stdio.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

/*!
        This class is needed in order to implement a callback function
        associated with the grabber
*/
class VISP_EXPORT vpDirectShowSampleGrabberI : public ISampleGrabberCB
{
  vpDirectShowSampleGrabberI();
  virtual ~vpDirectShowSampleGrabberI();

  // needed by the interface
  STDMETHODIMP_(ULONG) AddRef() { return 1; }
  STDMETHODIMP_(ULONG) Release() { return 2; }

  STDMETHODIMP QueryInterface(REFIID riid, void **ppvObject);

  // not implemented
  STDMETHODIMP SampleCB(double Time, IMediaSample *pSample) { return E_NOTIMPL; }

  // our callback function
  STDMETHODIMP BufferCB(double Time, BYTE *pBuffer, long BufferLen);

private:
  // the currently connected media type
  AM_MEDIA_TYPE connectedMediaType;

  // true if the source media type is not a standard one
  bool specialMediaType;
  // true if the image needs to be flipped (only for special media types)
  bool invertedSource;

  // booleans used to signal a demand from acquire
  bool acqGrayDemand;
  bool acqRGBaDemand;

  // pointer on the image to fill during the next callback if there has been a
  // demand
  vpImage<vpRGBa> *rgbaIm;
  vpImage<unsigned char> *grayIm;

  // semaphore used to synchronize the productor (callback) and the consumer
  // (acquire)
  HANDLE copySem;

  friend class vpDirectShowGrabberImpl;
};

#endif
#endif
#endif
