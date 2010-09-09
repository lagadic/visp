/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) )

#include <atlbase.h>
#include <iostream>
#include <string>
#include <dshow.h>



class VISP_EXPORT vpDirectShowDevice
{

	std::string name;		//the device's name
	std::string desc;		//the device's description
	std::string devPath;		//the device's device path (unique)

	bool inUse;			//true if the device is already used by a grabber

public:
	vpDirectShowDevice() : inUse(false){}
	vpDirectShowDevice(const CComPtr<IMoniker>& moniker) : inUse(false){ init(moniker); }

	bool init(const CComPtr<IMoniker>& moniker);

	bool getState(){ return inUse; }
	void setInUse(){ inUse=true; }
	void resetInUse() { inUse=false; }

	std::string& getName(){ return name; }
	std::string& getDesc(){ return desc; }
	std::string& getDevPath(){ return devPath; }

	bool operator==(vpDirectShowDevice& dev);

	friend std::ostream& operator<<(std::ostream& os, vpDirectShowDevice& dev)
	{
		return os<<dev.name<<std::endl<<dev.desc<<std::endl<<dev.devPath;
	}
};
#endif
#endif
#endif
