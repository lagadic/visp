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
 * DirectShow device description.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <stdio.h>
#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <visp3/sensor/vpDirectShowDevice.h>

/*!
        Initialize the vpDirectShowDevice with the moniker's information
        \param pMoniker The moniker that contains the device's information
        \return Was the operation successfull
*/
bool vpDirectShowDevice::init(const CComPtr<IMoniker> &pMoniker)
{
  HRESULT hr;

  // Get the properties
  CComPtr<IPropertyBag> pPropBag;
  pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)(&pPropBag));

  // get the name of the input
  VARIANT varName;
  VARIANT varDesc;
  VARIANT varDevPath;
  VariantInit(&varName);
  VariantInit(&varDesc);
  VariantInit(&varDevPath);
  char tmp[FILENAME_MAX];

  hr = pPropBag->Read(L"FriendlyName", &varName, 0);

  // successfully got the name
  if (SUCCEEDED(hr)) {
    sprintf(tmp, "%S", varName.bstrVal);
    name = tmp;
  }

  VariantClear(&varName);

  hr = pPropBag->Read(L"Description", &varDesc, 0);

  // successfully got the description
  if (SUCCEEDED(hr)) {
    sprintf(tmp, "%S", varDesc.bstrVal);
    desc = tmp;
  }

  VariantClear(&varDesc);

  hr = pPropBag->Read(L"DevicePath", &varDevPath, 0);

  // successfully got the device path
  if (SUCCEEDED(hr)) {
    sprintf(tmp, "%S", varDevPath.bstrVal);
    devPath = tmp;
  }

  VariantClear(&varDevPath);

  inUse = false;

  return true;
}

/*!
        Compares the two vpDirectShowDevice.
        \return true if they are equal
*/
bool vpDirectShowDevice::operator==(vpDirectShowDevice &dev)
{
  return name == dev.name && desc == dev.desc && devPath == dev.devPath;
}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDirectShowDevice &dev)
{
  return os << dev.name << std::endl << dev.desc << std::endl << dev.devPath;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpDirectShowDevice.cpp.o)
// has no symbols
void dummy_vpDirectShowDevice(){};
#endif
#endif
