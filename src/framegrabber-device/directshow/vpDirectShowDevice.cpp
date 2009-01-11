/****************************************************************************
 *
 * $Id: vpDirectShowDevice.cpp,v 1.7 2009-01-11 16:56:42 fspindle Exp $
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
 * DirectShow device description.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <stdio.h>
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
	char tmp[FILENAME_MAX];

	hr = pPropBag->Read(L"FriendlyName", &varName, 0);

	//successfully got the name
	if (SUCCEEDED(hr))
	{
		sprintf(tmp, "%S", varName.bstrVal);
		name = tmp;
	}

	VariantClear(&varName);

	hr = pPropBag->Read(L"Description", &varDesc, 0);

	//successfully got the description
	if (SUCCEEDED(hr))
	{
		sprintf(tmp, "%S", varDesc.bstrVal);
		desc = tmp;
	}

	VariantClear(&varDesc);

	hr = pPropBag->Read(L"DevicePath", &varDevPath, 0);

	//successfully got the device path
	if (SUCCEEDED(hr))
	{
		sprintf(tmp, "%S",varDevPath.bstrVal);
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
