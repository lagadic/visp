
#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 

#include <visp/vpDirectShowGrabberImpl.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

vpDirectShowDevice * vpDirectShowGrabberImpl::deviceList = NULL;

/*!
	Converts a HRESULT into the corresponding error message
*/
void vpDirectShowGrabberImpl::HRtoStr(string& str)
{
	TCHAR szErr[MAX_ERROR_TEXT_LEN];
	DWORD res = AMGetErrorText(hr, szErr, MAX_ERROR_TEXT_LEN);
    
	if (res == 0) str = "Unknown Error: 0x%2x";
	
	char msg[MAX_ERROR_TEXT_LEN];
	sprintf(msg,"%s",szErr);
	str = msg;
}


/*!
	Constructor.
	Initializes COM.
*/
vpDirectShowGrabberImpl::vpDirectShowGrabberImpl()
{
	//COM initialization
	if (FAILED(hr = CoInitialize(NULL)))
	{
		  string err;
		  HRtoStr(err);
		  throw(vpFrameGrabberException(
			  vpFrameGrabberException::initializationError,
			  "Can't initialize COM\n"+err));
    }
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabberImpl::open(vpImage<unsigned char> &I)
{
	if(! (init = initDirectShow()) )
	{
		string err;
		HRtoStr(err);
		throw (vpFrameGrabberException(
			  vpFrameGrabberException::initializationError, err));
	}
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabberImpl::open(vpImage<vpRGBa> &I)
{
	if(! (init = initDirectShow()) )
	{
		string err;
		HRtoStr(err);
		throw (vpFrameGrabberException(
			  vpFrameGrabberException::initializationError, err));
	}
}

/*!
	Initialization method
	Creates the capture filter graph
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::initDirectShow()
{
	CComPtr<IEnumMoniker> pVideoInputEnum = NULL;

	if(!enumerate(pVideoInputEnum))
		return false;

	//create the device list
	if(deviceList == NULL)
	{
		createDeviceList(pVideoInputEnum);
	}

	//not used anymore, so we release it
	pVideoInputEnum.Release();

	//get the first working device's filter (unused and getdevice works on it)
	if(!getFirstUnusedDevice(pCapSource))
		return false;
	
	
	currentDevice = 0;

	//create the filter graph
	if(!createGraph())
		return false;

	//we add the capture source to the filter graph
	if(FAILED(hr = pGraph->AddFilter(pCapSource, L"Capture Filter")))
		return false;

	//we create a sample grabber
	if(!createSampleGrabber(pGrabberFilter))
		return false;

	//we add the grabber to the filter graph
	if(FAILED(hr = pGraph->AddFilter(pGrabberFilter, L"SampleGrabber")))
		return false;
	
	//we connect the pins
	if(!connectSourceToGrabber(pCapSource, pGrabberFilter))
		return false;


	//get the current connected media type (needed by the callback)
	if(FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
		return false;


	
	//Gets the various graph's interfaces
	CComPtr<IMediaFilter> pMediaFilter;

	pGraph->QueryInterface(IID_IMediaFilter, (void **)&pMediaFilter);
	pGraph->QueryInterface(IID_IMediaControl, reinterpret_cast<void**>(&pControl));
	pGraph->QueryInterface(IID_IMediaEvent, (void **)&pEvent);

	// Turn off the reference clock.
	pMediaFilter->SetSyncSource(NULL); 
	pMediaFilter.Release();

	return true;
}

/*!
	Destructor
*/
vpDirectShowGrabberImpl::~vpDirectShowGrabberImpl()
{
  close();
}

/*!
	Create a video device enumerator.
	\param ppVideoInputEnum The video device enumerator
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::enumerate(CComPtr<IEnumMoniker>& ppVideoInputEnum)
{
	CComPtr<ICreateDevEnum> pDevEnum = NULL;
	bool res = false;
	
	//Enumerate system devices 
	hr = pDevEnum.CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER);

	//if it is a success
	if (SUCCEEDED(hr))
	{
		//Create a video input device enumerator
		hr = pDevEnum->CreateClassEnumerator(
			CLSID_VideoInputDeviceCategory,
			&ppVideoInputEnum, 0);
		
		if(hr == S_OK) res = true;
	}
	
	
	pDevEnum.Release();
	return res;
}


/*!
	Create the device list by enumerating the video input devices
	\param ppVideoInputEnum A video device enumerator 
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::createDeviceList(CComPtr<IEnumMoniker>& ppVideoInputEnum)
{
	CComPtr<IMoniker> pMoniker[10];	//const max devices
	unsigned long nbMoniker;

	ppVideoInputEnum->Reset();

	//Enumerates the different inputs
	ppVideoInputEnum->Next(10, reinterpret_cast<IMoniker **>(&pMoniker), &nbMoniker);

	//if no input device
	if(nbMoniker == 0) return false;

	deviceList = new vpDirectShowDevice[nbMoniker];

	nbDevices = nbMoniker;

	//we try tp get the properties of each moniker, if it fails, we skip to the next one and 
	//decrement the number of valid devices
	unsigned int i=0;
	unsigned int j=0;
	while(i<nbDevices)
	{
		if(!deviceList[i].init(pMoniker[j]))
		{
			//if we can't get the device properties, skip to the next device		
			j++;
			nbDevices--;
		}
		else
		{
			i++;
			j++;
		}
	}
	
	//if no working input device
	if(nbDevices == 0) return false;

	//we release the monikers
	for(unsigned int i=0 ; i<nbMoniker ;i++)
	{
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
bool vpDirectShowGrabberImpl::getDevice(unsigned int n, CComPtr<IBaseFilter>& ppDevice)
{
	//if n is invalid, quit
	if(n>=nbDevices)
		return false;

	//if the device is already in use, quit
	if(deviceList[n].getState() == true)
		return false;

	//if we can't enumerate the devices, quit
	CComPtr<IEnumMoniker> pVideoInputEnum = NULL;
	if(!enumerate(pVideoInputEnum))
		return false;

	CComPtr<IMoniker> pMoniker = NULL;
	bool deviceFound = false;

	//Enumerates the different inputs
	while (pVideoInputEnum->Next(1, &pMoniker, NULL) == S_OK 
		&& !deviceFound)
	{
		//implicit conversion should work ...
		if(deviceList[n]==vpDirectShowDevice(pMoniker))
		{
			//we get the filter
			if(SUCCEEDED(pMoniker->BindToObject(0, 0, IID_IBaseFilter, (void**)&ppDevice)))
			{
				//now the device is in use
				deviceList[n].setInUse();
				deviceFound = true;
			}
			else{ break; } //we can't get the device's filter, quit
		}
		pMoniker.Release();
	}

	pVideoInputEnum.Release();

	return deviceFound;
}

/*!
	Searches for the first unused device.
	\param ppDevice The first device filter's interface
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::getFirstUnusedDevice(CComPtr<IBaseFilter>& ppDevice)
{
	unsigned int n=0;
	bool found=false;

	for(n=0; n<nbDevices && !found; n++)
	{
		//if the device is not being used
		if(!deviceList[n].getState())
		{
			if(getDevice(n,ppDevice))
			{
				found = true;
				deviceList[n].setInUse();
			}
		}
	}

	return found;
}

/*!
	Create the capture graph
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::createGraph()
{

	// Create the Capture Graph Builder.
	hr = pBuild.CoCreateInstance(CLSID_CaptureGraphBuilder2, 0, CLSCTX_INPROC_SERVER);

	if (SUCCEEDED(hr))
	{
		// Create the Filter Graph Manager.
		hr = pGraph.CoCreateInstance(CLSID_FilterGraph, 0, CLSCTX_INPROC_SERVER);

		if (SUCCEEDED(hr))
		{
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
bool vpDirectShowGrabberImpl::createSampleGrabber(CComPtr<IBaseFilter>& ppGrabberFilter)
{
	//Creates the sample grabber
	hr = ppGrabberFilter.CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER);

	if (FAILED(hr))
		return false;

	//gets the SampleGrabber interface in order to configure it later
	hr = ppGrabberFilter->QueryInterface(IID_ISampleGrabber,
		reinterpret_cast<void**>(&pGrabberI));

	if (FAILED(hr))
		return false;

	//configure the grabber
	AM_MEDIA_TYPE mt;
	ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
	
	mt.majortype = MEDIATYPE_Video;

	//ask for a ARGB32 connection
	mt.subtype = MEDIASUBTYPE_ARGB32;


	if(FAILED(hr = pGrabberI->SetMediaType(&mt)))
		return false;

	//configure the callback of the grabber
	pGrabberI->SetCallback(&sgCB,1);

	//grab only one frame at a time
	pGrabberI->SetOneShot(TRUE);

	//no need to bufferize the sample in the grabber
	pGrabberI->SetBufferSamples(false);

	return true;
}

/*!
	Connects the capture device's output pin to the grabber's input pin
	\param pCapSource The capture device
	\param pGrabber The grabber
	\return Was the operation successful
*/
bool vpDirectShowGrabberImpl::connectSourceToGrabber(CComPtr<IBaseFilter>& pCapSource, CComPtr<IBaseFilter>& pGrabberFilter)
{
	//get the capture source's output pin
	CComPtr<IPin> pCapSourcePin;
	if(FAILED(pBuild->FindPin(pCapSource, PINDIR_OUTPUT, NULL, NULL, false, 0, &pCapSourcePin)))
		return false;
	
	//get the grabber's input pin
	CComPtr<IPin> pGrabberInputPin;
	if(FAILED(pBuild->FindPin(pGrabberFilter, PINDIR_INPUT, NULL, NULL, false, 0, &pGrabberInputPin)))
		return false;

	//connect the two of them
	if(FAILED(pGraph->Connect(pCapSourcePin, pGrabberInputPin)))
		return false;

	//not used anymore, we can release them
	pCapSourcePin.Release();
	pGrabberInputPin.Release();

	//get the grabber's output pin
	CComPtr<IPin> pGrabberOutputPin;
	if(FAILED(pBuild->FindPin(pGrabberFilter, PINDIR_OUTPUT, NULL, NULL, false, 0, &pGrabberOutputPin)))
		return false;

	//get the Null renderer
	CComPtr<IBaseFilter> pNull = NULL;
	if (FAILED(pNull.CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER)))
		return false;

	//get the null renderer's input pin
	CComPtr<IPin> pNullInputPin;
	if(FAILED(pBuild->FindPin(pNull, PINDIR_INPUT, NULL, NULL, false, 0, &pNullInputPin)))
		return false;

	//connect the grabber's output to the null renderer
	if(	FAILED(pGraph->AddFilter(pNull, L"NullRenderer")) ||
		FAILED(pGraph->Connect(pGrabberOutputPin, pNullInputPin)))
		return false;

	//release the remaining interfaces
	pNull.Release();	
	pGrabberOutputPin.Release();
	pNullInputPin.Release();

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

    while(pEnum->Next(1, &pFilter, &cFetched) == S_OK)
    {
		if(FAILED(hr = pGraph->RemoveFilter(pFilter))) return false;
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
  if (init==false)
    {
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				     "Initialization not done") );
    }  

	//set the rgbaIm pointer on I (will be filled on the next framegrabber callback)
	sgCB.rgbaIm = &I;
	//there is an acquire demand (execute copy during callback)
	sgCB.acqRGBaDemand = true;

	//Run the graph to grab a frame
	pControl->Run(); 
	
	// Wait untill it's done
	long ev;
	hr = pEvent->WaitForCompletion(MAX_DELAY, &ev);

	ncols = I.getCols();
	nrows = I.getRows();

	//wait for the end of the next callback (copy)
	if( WaitForSingleObject(sgCB.copySem,MAX_DELAY) != WAIT_OBJECT_0)
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				"Can't grab the frame, callback timeout") );
  
  
}

/*!
	Grabs a grayscale image from the video stream.
	Uses a semaphore to synchronize with the framegrabber callback.

	\param I The image to fill with the grabbed frame

	\exception otherError Can't grab the frame
*/
void vpDirectShowGrabberImpl::acquire(vpImage<unsigned char> &I)
{
  if (init==false)
 {
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Initialization not done") );
  }

	//set the grayIm pointer on I (will be filled on the next framegrabber callback)
	sgCB.grayIm = &I;
	//there is an acquire demand (execute copy during callback)
	sgCB.acqGrayDemand = true;

	//Run the graph to grab a frame
	pControl->Run(); 

	// Wait untill it's done
	long ev;
	hr = pEvent->WaitForCompletion(MAX_DELAY, &ev);

	ncols = I.getCols();
	nrows = I.getRows();

	//wait for the end of the next callback (copy)
	if( WaitForSingleObject(sgCB.copySem,MAX_DELAY) != WAIT_OBJECT_0)
		throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
				"Can't grab the frame, callback timeout") );
}

/*!
	Change the input device of the grabber
	\param n number of the device to use

	\return true was the change successful
*/
bool vpDirectShowGrabberImpl::setInput(unsigned int n)
{
  if (init==false)
    {
      close();
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Initialization not done") );
    }

   
	//if n is invalid, or the device is already in use, quit
        if(n>=nbDevices || deviceList[n].getState()==true)
	  return false;
	  
	//we stop the graph
	pControl->Stop();
	
	//then we can safely remove all the filters
	if(!removeAll()) return false;


	//we release the previous source's interface
	pCapSource.Release();

	//here reset inUse in the old DSDevice
	deviceList[currentDevice].resetInUse();


	//we add the grabber back in the graph
	pGraph->AddFilter(pGrabberFilter,NULL);

	//get the n-th device's filter
	if(!getDevice(n,pCapSource))
		return false;

	//we add the capture source to the filter graph
	if(FAILED(hr = pGraph->AddFilter(pCapSource, L"Capture Filter")))
		return false;

	//we connect the pins
	if(!connectSourceToGrabber(pCapSource, pGrabberFilter))
		return false;


	//get the current connected media type (needed by the callback)
	if(FAILED(hr = pGrabberI->GetConnectedMediaType(&(sgCB.connectedMediaType))))
	{	
		return false;
	}

	//the device is now in use
	deviceList[n].setInUse();
	currentDevice=n;

	return true;
}

/*!
	Displays the list of devices on the standard output
*/
void vpDirectShowGrabberImpl::displayDevices()
{
  if(deviceList == NULL)
    {
      throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
				   "Initialization not done") );
    }

	for(unsigned int i=0 ; i<nbDevices ; i++)
		cout<<i<<" : "<<deviceList[i]<<endl<<endl;
	
	cout<<"Current device : "<<currentDevice<<endl;
}

/*!
	Stops the framegrabber
*/
void vpDirectShowGrabberImpl::close()
{	
  //the current device isn't being used anymore
  deviceList[currentDevice].resetInUse();

  //uninstalls COM
  CoUninitialize();
}


#endif
#endif
