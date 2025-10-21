/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

#ifndef _vpMutex_h_
#define _vpMutex_h_

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS) && (defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0)))

#if defined(VISP_HAVE_PTHREAD)
#include <pthread.h>
#elif defined(_WIN32)

// Mute warning with clang-cl
// warning : non-portable path to file '<WinSock2.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
// warning : non-portable path to file '<Windows.h>'; specified path differs in case from file name on disk [-Wnonportable-system-include-path]
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wnonportable-system-include-path"
#endif

// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>

#if defined(__clang__)
#  pragma clang diagnostic pop
#endif
#endif

#ifdef ENABLE_VISP_NAMESPACE
namespace VISP_NAMESPACE_NAME
{
#endif
/*!

   \class vpMutex

   \ingroup group_core_threading
   \deprecated Use rather std::mutex.

   Class that allows protection by mutex.

   This class implements native pthread functionalities if available, of
   native Windows threading capabilities if pthread is not available under
   Windows.

   \sa vpScopedLock
*/
class VP_DEPRECATED vpMutex
{
public:
  vpMutex() : m_mutex()
  {
#if defined(VISP_HAVE_PTHREAD)
    pthread_mutex_init(&m_mutex, nullptr);
#elif defined(_WIN32)
#ifdef WINRT_8_1
    m_mutex = CreateMutexEx(nullptr, nullptr, 0, nullptr);
#else
    m_mutex = CreateMutex(nullptr,                   // default security attributes
                          FALSE,                  // initially not owned
                          nullptr);                  // unnamed mutex
#endif
    if (m_mutex == nullptr) {
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

    \ingroup group_core_threading

    \brief Class that allows protection by mutex.

    The following example shows how to use this class to protect a portion of
    code from concurrent access. The scope of the mutex lock/unlock is determined
    by the constructor/destructor.

    \code
    #include <visp3/core/vpMutex.h>

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

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

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpScopedLock &operator=(const vpScopedLock &) = delete; // non copyable
#endif

  public:
    //! Constructor that locks the mutex.
    vpScopedLock(vpMutex &mutex) : _mutex(mutex) { _mutex.lock(); }
    //! Destructor that unlocks the mutex.
    virtual ~vpScopedLock() { _mutex.unlock(); }
  };

private:
#if defined(VISP_HAVE_PTHREAD)
  pthread_mutex_t m_mutex;
#elif defined(_WIN32)
  HANDLE m_mutex;
#endif
};
#ifdef ENABLE_VISP_NAMESPACE
}
#endif
#endif
#endif
