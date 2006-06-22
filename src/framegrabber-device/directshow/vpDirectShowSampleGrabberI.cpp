#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpDirectShowSampleGrabberI.h>

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 


/*!
	Converts a BGRa image (pBuffer) to RGBa
	assumes that rgbaIm is already resized
*/
void BGRaToRGBa(vpImage<vpRGBa> * rgbaIm, const BYTE * pBuffer, long BufferLen)
{
	for(long i=0 ; i<BufferLen ; i+=4)
	{
		rgbaIm->bitmap[i>>2].R = pBuffer[i+2];
		rgbaIm->bitmap[i>>2].G = pBuffer[i+1];
		rgbaIm->bitmap[i>>2].B = pBuffer[i+0];
		rgbaIm->bitmap[i>>2].A = pBuffer[i+3];
	}
}

/*!
	Converts a BGRa image (pBuffer) to grayscale
	assumes that rgbaIm is already resized
*/
void BGRaToGray(vpImage<unsigned char> * grayIm, const BYTE * pBuffer, long BufferLen)
{
  for (int i =0 ; i<BufferLen ; i+=4)
  {
	  grayIm->bitmap[i>>2] = (unsigned char)( 0.299 * pBuffer[i+2]
										 + 0.597 * pBuffer[i+1] 
										 + 0.114 * pBuffer[i+0]) ;
  }
}

/*!
	Constructor - creates the semaphore
*/
vpDirectShowSampleGrabberI::vpDirectShowSampleGrabberI()
: acqGrayDemand(false), acqRGBaDemand(false)
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
		//check if the connected media is compatible (TODO maybe not necessary)
		if(connectedMediaType.formattype==FORMAT_VideoInfo)
		{
			//if the buffer contains an ARGB32 image (DS ARGB32 <=> BGRA)
			if(connectedMediaType.subtype==MEDIASUBTYPE_ARGB32)
			{
				//retrieve the image information
				VIDEOINFOHEADER *pVih = reinterpret_cast<VIDEOINFOHEADER*>(connectedMediaType.pbFormat);
				BITMAPINFOHEADER bmpInfo = pVih->bmiHeader;

				//if it was an RGBa image demand
				if(acqRGBaDemand)
				{
					//first, resizes the image as needed
					rgbaIm->resize(bmpInfo.biHeight, bmpInfo.biWidth);
					//copy and convert the image
					BGRaToRGBa(rgbaIm, pBuffer, BufferLen);
					//reset the demand boolean
					acqRGBaDemand = false;
				}
				else//if it was a grayscale image demand
				{
					//first, resizes the image as needed
					grayIm->resize(bmpInfo.biHeight, bmpInfo.biWidth);
					//copy and convert the image
					BGRaToGray(grayIm, pBuffer, BufferLen);
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
