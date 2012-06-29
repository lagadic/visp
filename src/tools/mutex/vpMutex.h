/****************************************************************************
 *
 * $Id$
 *
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Mutex protection.
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/


#ifndef __VP_MUTEX__
#define __VP_MUTEX__

#include <visp/vpConfig.h>
#include <visp/vpException.h>
#ifdef VISP_HAVE_PTHREAD

#include <pthread.h>

/*!

  \class vpMutex

  \ingroup Mutex

  \brief Class that allows protection by mutex.

  \warning This class needs the pthread third-party library.
*/
class VISP_EXPORT vpMutex {
public:
	vpMutex() {
		pthread_mutex_init( &m_mutex, NULL );
	}
	void lock() {
		pthread_mutex_lock( &m_mutex );
	}
	void unlock() {
		pthread_mutex_unlock( &m_mutex );
	}

	/*!
	  
	  \class vpScopedLock
	  
	  \ingroup Mutex
	  
	  \brief Class that allows protection by mutex.
	  
	  \warning This class needs the pthread third-party library.
	*/
	class VISP_EXPORT vpScopedLock
	{
	private:
		vpMutex & _mutex;
	public:

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    void operator=(const vpScopedLock &){
      throw vpException(vpException::functionNotImplementedError,"Not implemented!");
    }
#endif

		vpScopedLock(vpMutex & mutex)
			: _mutex(mutex)
		{
			_mutex.lock();
		}
		~vpScopedLock()
		{
			_mutex.unlock();
		}
	};
private:
	pthread_mutex_t m_mutex;
};

#endif

#endif
