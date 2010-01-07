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
