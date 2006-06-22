#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 

#include <visp/vpDirectShowDevice.h>


/*!
	Initialize the vpDirectShowDevice with the moniker's information
	\param pMoniker The moniker that contains the device's information
	\return Was the operation successfull
*/
bool vpDirectShowDevice::init(const CComPtr<IMoniker>& pMoniker)
{
	HRESULT hr;

	//Get the properties
	CComPtr<IPropertyBag> pPropBag;
	hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, 
		(void**)(&pPropBag));
	
	//get the name of the input
	VARIANT varName;
	VARIANT varDesc;
	VARIANT varDevPath;
	VariantInit(&varName);
	VariantInit(&varDesc);
	VariantInit(&varDevPath);
	char tmp[300];

	hr = pPropBag->Read(L"FriendlyName", &varName, 0);
		
	//successfully got the name
	if (SUCCEEDED(hr))
	{
		sprintf(tmp,"%S",varName.bstrVal);
		name = tmp;	
	}

	VariantClear(&varName);

	hr = pPropBag->Read(L"Description", &varDesc, 0);
		
	//successfully got the description
	if (SUCCEEDED(hr))
	{
		sprintf(tmp,"%S",varDesc.bstrVal);
		desc = tmp;	
	}

	VariantClear(&varDesc);

	hr = pPropBag->Read(L"DevicePath", &varDevPath, 0);
		
	//successfully got the device path
	if (SUCCEEDED(hr))
	{
		sprintf(tmp,"%S",varDevPath.bstrVal);
		devPath = tmp;	
	}

	VariantClear(&varDevPath);

	inUse=false;

	return true;
}

/*!
	Compares the two vpDirectShowDevice.
	\return true if they are equal
*/
bool vpDirectShowDevice::operator==(vpDirectShowDevice& dev)
{
	return name==dev.name
		&& desc==dev.desc
		&& devPath==dev.devPath;
}

#endif
#endif
