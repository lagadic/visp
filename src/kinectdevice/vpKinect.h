/****************************************************************************
 *
 * $Id:  vpKinect.h  2011-09-14 13:17:26  cteulier $
 *
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
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
 * API for using a Microsoft Kinect device
 * Requires libfreenect as a third party library
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/


#ifndef __VP_KINECT__
#define __VP_KINECT__

#include <visp/vpConfig.h>
#ifdef VISP_HAVE_LIBFREENECT

#include "libfreenect.hpp"
#include <pthread.h>
#include <iostream>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>

class Mutex {
public:
	Mutex() {
		pthread_mutex_init( &m_mutex, NULL );
	}
	void lock() {
		pthread_mutex_lock( &m_mutex );
	}
	void unlock() {
		pthread_mutex_unlock( &m_mutex );
	}

	class ScopedLock
	{
		Mutex & _mutex;
	public:
		ScopedLock(Mutex & mutex)
			: _mutex(mutex)
		{
			_mutex.lock();
		}
		~ScopedLock()
		{
			_mutex.unlock();
		}
	};
private:
	pthread_mutex_t m_mutex;
};


class vpKinect : public Freenect::FreenectDevice
{
public:

	vpKinect(freenect_context *_ctx, int _index);
	virtual ~vpKinect();

	void start();
	void stop();

	void setTiltAngle(float angle);

	bool getDepthMap(vpImage<float>& map_,vpImage<unsigned char>& Imap);
	bool getRGB(vpImage<vpRGBa>& IRGB);


	inline void getIRCamParameters(vpCameraParameters &cam)const{cam = IRcam;}
	inline void getRGBCamParameters(vpCameraParameters &cam)const{cam = RGBcam;}
	inline void setIRCamParameters(const vpCameraParameters &cam){IRcam = cam;}
	inline void setRGBCamParameters(const vpCameraParameters &cam){RGBcam = cam;}

private:
	//!Instantiation of Freenect virtual functions
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp);

	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp);

private:
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;

	bool m_new_rgb_frame;
	bool m_new_depth_frame;

	unsigned height,width;
	vpCameraParameters RGBcam, IRcam;//intrinsic parameters of the two cameras

	//Acces protected by a mutex
	vpImage<float> dmap;
	vpImage<unsigned char> Idmap;
	vpImage<vpRGBa> IRGB;
};



#endif//ViSP has libfreenect

#endif
