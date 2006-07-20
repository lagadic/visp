#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpDirectShowSampleGrabberI.h>
#include <visp/vpImageConvert.h>

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 



/*!
	Constructor - creates the semaphore
*/
vpDirectShowSampleGrabberI::vpDirectShowSampleGrabberI()
  : acqGrayDemand(false), acqRGBaDemand(false), specialMediaType(false), invertedSource(false)
{
	//semaphore(0), max value = 1
	copySem = CreateSemaphore (NULL,0,1,NULL);
}

/*!
	Destructor - destroys the semaphore
*/
vpDirectShowSampleGrabberI::~vpDirectShowSampleGrabberI()
{
	//destroys the semaphore
	CloseHandle(copySem);
}


STDMETHODIMP vpDirectShowSampleGrabberI::QueryInterface(REFIID riid, void **ppvObject)
{
	if (NULL == ppvObject) return E_POINTER;
	if (riid == __uuidof(IUnknown))
	{
		*ppvObject = static_cast<IUnknown*>(this);
		return S_OK;
	}
	if (riid == __uuidof(ISampleGrabberCB))
	{
		*ppvObject = static_cast<ISampleGrabberCB*>(this);
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
	//if there has been a frame demand
	if(acqGrayDemand || acqRGBaDemand)
	{
		//check if the connected media is compatible 
		if(connectedMediaType.formattype==FORMAT_VideoInfo)
		{
			//if the buffer contains a RGB24 image (DS RGB24 <=> BGR)
			if(connectedMediaType.subtype==MEDIASUBTYPE_RGB24)
			{
				//retrieve the image information
				VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER*>(connectedMediaType.pbFormat);
				BITMAPINFOHEADER bmpInfo = pVih->bmiHeader;

				//if biHeight > 0 and the source is not special
				//then  the image needs to be verticaly flipped
				bool flip;
				if(!specialMediaType)
				  flip = bmpInfo.biHeight>=0;
				//the source is fourcc and the image is inverted with this compression
				else if(invertedSource)
				  flip = true; 
				//fourcc and the image doesn't need to be flipped
				else
				  flip = false;


				//if it was an RGBa image demand
				if(acqRGBaDemand)
				{
					//first, resizes the image as needed
					rgbaIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
					//copy and convert the image
					vpImageConvert::BGRToRGBa(pBuffer,(unsigned char*) rgbaIm->bitmap,
								  rgbaIm->getCols() , rgbaIm->getRows(), flip);
					//reset the demand boolean
					acqRGBaDemand = false;
				}
				else//if it was a grayscale image demand
				{
					//first, resizes the image as needed
					grayIm->resize(abs(bmpInfo.biHeight), bmpInfo.biWidth);
					//copy and convert the image
					vpImageConvert::BGRToGrey(pBuffer, grayIm->bitmap,
								  grayIm->getCols(), grayIm->getRows(), flip);
					//reset the demand boolean
					acqGrayDemand = false;
				}
			}
		}
		
		//increment the semaphore - allows acquire to continue execution
		ReleaseSemaphore(copySem, 1, NULL);
	}
	return S_OK;
}
#endif

#endif
