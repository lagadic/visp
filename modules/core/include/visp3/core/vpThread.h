#ifndef __vpPthread_h_
#define __vpPthread_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))

#if defined(VISP_HAVE_PTHREAD)
#include <pthread.h>
#include <string.h>
#elif defined(_WIN32)
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>
#endif

/*!
   \class vpThread

   \ingroup group_core_threading

   Class to represent individual threads of execution.
   This class implements native pthread functionalities if available, or
   native Windows threading capabilities if pthread is not available under
   Windows.

   There are two examples implemented in testMutex.cpp and testThread.cpp to
   show how to use this class. The content of test-thread.cpp that hightlights
   the main functionalities of this class is given hereafter: \snippet
   testThread.cpp Code

   More examples are provided in \ref tutorial-multi-threading.
 */
class vpThread
{
public:
#if defined(VISP_HAVE_PTHREAD)
  typedef void *Args;
  typedef void *Return;
  typedef void *(*Fn)(Args);
  typedef pthread_t Handle;
#elif defined(_WIN32)
  typedef LPVOID Args;
  typedef DWORD Return;
  typedef LPTHREAD_START_ROUTINE Fn;
  // typedef DWORD (*Fn)(Args);
  typedef HANDLE Handle;
#endif
  /*!
     Default constructor that does nothing. To attach a function to this
     thread of execution you need to call create().
   */
  vpThread() : m_handle(), m_isCreated(false), m_isJoinable(false) {}

  /*!
     Construct a thread object that represents a new joinable thread of
     execution. The new thread of execution calls \e fn passing \e args as
     arguments. \param fn : A pointer to a function. \param args : Arguments
     passed to the call to \e fn (if any).
   */
  vpThread(vpThread::Fn fn, vpThread::Args args = NULL) : m_handle(), m_isCreated(false), m_isJoinable(false)
  {
    create(fn, args);
  }

  /*!
     Creates a thread object that represents a new joinable thread of
     execution. \param fn : A pointer to a function. \param args : Arguments
     passed to the call to \e fn (if any).
   */
  void create(vpThread::Fn fn, vpThread::Args args = NULL)
  {
    if (m_isCreated)
      throw vpException(vpException::fatalError, "The thread is already created");
#if defined(VISP_HAVE_PTHREAD)
    int err = pthread_create(&m_handle, NULL, fn, args);
    if (err != 0) {
      throw vpException(vpException::cannotUseConstructorError, "Can't create thread : %s", strerror(err));
    }
#elif defined(_WIN32)
    DWORD dwThreadIdArray;
    m_handle = CreateThread(NULL,              // default security attributes
                            0,                 // use default stack size
                            fn,                // thread function name
                            args,              // argument to thread function
                            0,                 // use default creation flags
                            &dwThreadIdArray); // returns the thread identifier
#endif

    m_isJoinable = true;
  }

  /*!
     Destroy the thread.
   */
  virtual ~vpThread()
  {
    join();
#if defined(VISP_HAVE_PTHREAD)
#elif defined(_WIN32)
    CloseHandle(m_handle);
#endif
  }

  /*!
     This function return when the thread execution has completed.
     This blocks the execution of the thread that calls this function until
     the function called on construction returns (if it hasn't yet).

     After a call to this function, the thread object becomes non-joinable and
     can be destroyed safely.

     \sa joinable()
   */
  void join()
  {
    if (m_isJoinable) {
#if defined(VISP_HAVE_PTHREAD)
      pthread_join(m_handle, NULL);
#elif defined(_WIN32)
#if defined(WINRT_8_1)
      WaitForSingleObjectEx(m_handle, INFINITE, FALSE);
#else
      WaitForSingleObject(m_handle, INFINITE);
#endif
#endif
      m_isJoinable = false;
    }
  }

  /*!
     Returns a value used to access implementation-specific information
     associated to the thread.
   */
  Handle getHandle() { return m_handle; }

  /*!
     Returns whether the thread object is joinable.

     A thread object is not joinable in any of these cases:
     - if it was default-constructed and create() was not called.
     - if join() has been called.

     \sa join()
   */
  bool joinable() { return m_isJoinable; }

protected:
  Handle m_handle;   //!< Thread handle
  bool m_isCreated;  //!< Indicates if the thread is created
  bool m_isJoinable; //!< Indicates if the thread is joinable
};

#endif
#endif
