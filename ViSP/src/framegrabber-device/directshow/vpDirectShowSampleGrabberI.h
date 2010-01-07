/****************************************************************************
 *
 * $Id$
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
 * DirectShow framegrabber callback.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef vpDirectShowSampleGrabberI_hh
#define vpDirectShowSampleGrabberI_hh

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 

#include <qedit.h>
#include <stdio.h>
#include <dshow.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

/*!
	This class is needed in order to implement a callback function
	associated with the grabber
*/
class VISP_EXPORT vpDirectShowSampleGrabberI : public ISampleGrabberCB
{
	vpDirectShowSampleGrabberI();
	virtual ~vpDirectShowSampleGrabberI();

	//needed by the interface
	STDMETHODIMP_(ULONG) AddRef() { return 1; }
    STDMETHODIMP_(ULONG) Release() { return 2; }
 
    STDMETHODIMP QueryInterface(REFIID riid, void **ppvObject);
 
	//not implemented
    STDMETHODIMP SampleCB(double Time, IMediaSample *pSample)
    {
		return E_NOTIMPL;
    }
 
	//our callback function
    STDMETHODIMP BufferCB(double Time, BYTE *pBuffer, long BufferLen);

private:
	//the currently connected media type
	AM_MEDIA_TYPE connectedMediaType;

	//true if the source media type is not a standard one
	bool specialMediaType;
	//true if the image needs to be flipped (only for special media types)
	bool invertedSource;

	//booleans used to signal a demand from acquire
	bool acqGrayDemand;
	bool acqRGBaDemand;

	//pointer on the image to fill during the next callback if there has been a demand
	vpImage<vpRGBa> * rgbaIm;
	vpImage<unsigned char> * grayIm;

	//semaphore used to synchronize the productor (callback) and the consumer (acquire)
	HANDLE copySem;

	friend class vpDirectShowGrabberImpl;
};

#endif
#endif
#endif
