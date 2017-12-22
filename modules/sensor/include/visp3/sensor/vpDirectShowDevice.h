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
 * DirectShow device description.
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#ifndef vpDirectShowDevice_hh
#define vpDirectShowDevice_hh

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_DIRECTSHOW))

#include <atlbase.h>
#include <dshow.h>
#include <iostream>
#include <string>

class VISP_EXPORT vpDirectShowDevice
{

  std::string name;    // the device's name
  std::string desc;    // the device's description
  std::string devPath; // the device's device path (unique)

  bool inUse; // true if the device is already used by a grabber

public:
  vpDirectShowDevice() : inUse(false) {}
  explicit vpDirectShowDevice(const CComPtr<IMoniker> &moniker) : inUse(false) { init(moniker); }

  bool init(const CComPtr<IMoniker> &moniker);

  bool getState() { return inUse; }
  void setInUse() { inUse = true; }
  void resetInUse() { inUse = false; }

  std::string &getName() { return name; }
  std::string &getDesc() { return desc; }
  std::string &getDevPath() { return devPath; }

  bool operator==(vpDirectShowDevice &dev);

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpDirectShowDevice &dev);
};
#endif
#endif
#endif
