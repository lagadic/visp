/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Mutex protection.
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/


#ifndef __vpMutex_h_
#define __vpMutex_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PTHREAD) || defined(_WIN32)

#if defined(VISP_HAVE_PTHREAD)
#  include <pthread.h>
#elif defined(_WIN32)
#  include <windows.h>
#endif

/*!

   \class vpMutex

   \ingroup group_core_threading

   Class that allows protection by mutex.

   This class implements native pthread functionalities if available, of native Windows threading
   capabilities if pthread is not available under Windows.
*/
class vpMutex {
public:
  vpMutex() : m_mutex() {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_init( &m_mutex, NULL );
#elif defined(_WIN32)
    m_mutex = CreateMutex(
      NULL,              // default security attributes
      FALSE,             // initially not owned
      NULL);             // unnamed mutex
    if (m_mutex == NULL) {
      printf("CreateMutex error: %d\n", GetLastError());
      return;
    }
#endif
  }
	void lock() {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_lock( &m_mutex );
#elif defined(_WIN32)
    DWORD dwWaitResult;
    dwWaitResult = WaitForSingleObject(
          m_mutex,    // handle to mutex
          INFINITE);  // no time-out interval
#endif
  }
	void unlock() {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_unlock( &m_mutex );
#elif defined(_WIN32)
  // Release ownership of the mutex object
  if (!ReleaseMutex(m_mutex))
  {
    // Handle error.
    printf("unlock() error: %d\n", GetLastError());
  }
#endif
  }

	/*!
	  
	  \class vpScopedLock
	  
    \ingroup group_core_mutex
	  
	  \brief Class that allows protection by mutex.
	  
	  \warning This class needs the pthread third-party library.
	*/
	class VISP_EXPORT vpScopedLock
	{
	private:
		vpMutex & _mutex;

//  private:
//#ifndef DOXYGEN_SHOULD_SKIP_THIS
//    vpScopedLock &operator=(const vpScopedLock &){
//      throw vpException(vpException::functionNotImplementedError,"Not implemented!");
//      return *this;
//    }
//#endif

	public:
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
#if defined(VISP_HAVE_PTHREAD)
  pthread_mutex_t m_mutex;
#elif defined(_WIN32)
  HANDLE m_mutex;
#endif
};

#endif
#endif
