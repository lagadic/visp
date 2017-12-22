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
 * Mutex protection.
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/

#ifndef __vpMutex_h_
#define __vpMutex_h_

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))

#if defined(VISP_HAVE_PTHREAD)
#include <pthread.h>
#elif defined(_WIN32)
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>
#endif

/*!

   \class vpMutex

   \ingroup group_core_threading

   Class that allows protection by mutex.

   This class implements native pthread functionalities if available, of
   native Windows threading capabilities if pthread is not available under
   Windows.

   An example of vpMutex usage is given in testMutex.cpp.

   More examples are provided in \ref tutorial-multi-threading.

   \sa vpScopedLock
*/
class vpMutex
{
public:
  vpMutex() : m_mutex()
  {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_init(&m_mutex, NULL);
#elif defined(_WIN32)
#ifdef WINRT_8_1
    m_mutex = CreateMutexEx(NULL, NULL, 0, NULL);
#else
    m_mutex = CreateMutex(NULL,                   // default security attributes
                          FALSE,                  // initially not owned
                          NULL);                  // unnamed mutex
#endif
    if (m_mutex == NULL) {
      std::cout << "CreateMutex error: " << GetLastError() << std::endl;
      return;
    }
#endif
  }
  void lock()
  {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_lock(&m_mutex);
#elif defined(_WIN32)
    DWORD dwWaitResult;
#ifdef WINRT_8_1
    dwWaitResult = WaitForSingleObjectEx(m_mutex, INFINITE, FALSE);
#else
    dwWaitResult = WaitForSingleObject(m_mutex,   // handle to mutex
                                       INFINITE); // no time-out interval
#endif
    if (dwWaitResult == WAIT_FAILED)
      std::cout << "lock() error: " << GetLastError() << std::endl;
#endif
  }
  void unlock()
  {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_unlock(&m_mutex);
#elif defined(_WIN32)
    // Release ownership of the mutex object
    if (!ReleaseMutex(m_mutex)) {
      // Handle error.
      std::cout << "unlock() error: " << GetLastError() << std::endl;
    }
#endif
  }

  /*!

    \class vpScopedLock

    \ingroup group_core_mutex

    \brief Class that allows protection by mutex.

    The following example shows how to use this class to protect a portion of
    code from concurrent access. The scope of the mutex lock/unlock is determined
    by the constructor/destructor.

\code
 #include <visp3/core/vpMutex.h>

int main()
{
  vpMutex mutex;

  {
    vpMutex::vpScopedLock lock(mutex);
    // shared var to protect
  }
}
    \endcode

    Without using vpScopedLock, the previous example would become:
    \code
#include <visp3/core/vpMutex.h>

int main()
{
  vpMutex mutex;

  {
    mutex.lock();
    // shared var to protect
    mutex.unlock()
  }
}
    \endcode

    More examples are provided in \ref tutorial-multi-threading.

    \sa vpMutex
  */
  class vpScopedLock
  {
  private:
    vpMutex &_mutex;

    //  private:
    //#ifndef DOXYGEN_SHOULD_SKIP_THIS
    //    vpScopedLock &operator=(const vpScopedLock &){
    //      throw vpException(vpException::functionNotImplementedError,"Not
    //      implemented!"); return *this;
    //    }
    //#endif

  public:
    //! Constructor that locks the mutex.
    vpScopedLock(vpMutex &mutex) : _mutex(mutex) { _mutex.lock(); }
    //! Destructor that unlocks the mutex.
    ~vpScopedLock() { _mutex.unlock(); }
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
