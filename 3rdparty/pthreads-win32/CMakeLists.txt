project(${PTHREADS_LIBRARY})

set(PTHREADS_MAJOR_VERSION 2 PARENT_SCOPE)
set(PTHREADS_MINOR_VERSION 9 PARENT_SCOPE)
set(PTHREADS_PATCH_VERSION 1 PARENT_SCOPE)

set(PTHREADS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(PTHREAD_API
  ${PTHREADS_SOURCE_DIR}/pthread.h
  ${PTHREADS_SOURCE_DIR}/sched.h
  ${PTHREADS_SOURCE_DIR}/semaphore.h)

source_group(PTHREAD_API FILES ${PTHREAD_API})

set(PTHREAD_SOURCES
  ${PTHREADS_SOURCE_DIR}/attr.c
  ${PTHREADS_SOURCE_DIR}/barrier.c
  ${PTHREADS_SOURCE_DIR}/cancel.c
  ${PTHREADS_SOURCE_DIR}/cleanup.c
  ${PTHREADS_SOURCE_DIR}/condvar.c
  ${PTHREADS_SOURCE_DIR}/create.c
  ${PTHREADS_SOURCE_DIR}/dll.c
  ${PTHREADS_SOURCE_DIR}/autostatic.c
  ${PTHREADS_SOURCE_DIR}/errno.c
  ${PTHREADS_SOURCE_DIR}/exit.c
  ${PTHREADS_SOURCE_DIR}/fork.c
  ${PTHREADS_SOURCE_DIR}/global.c
  ${PTHREADS_SOURCE_DIR}/misc.c
  ${PTHREADS_SOURCE_DIR}/mutex.c
  ${PTHREADS_SOURCE_DIR}/nonportable.c
  ${PTHREADS_SOURCE_DIR}/private.c
  ${PTHREADS_SOURCE_DIR}/rwlock.c
  ${PTHREADS_SOURCE_DIR}/sched.c
  ${PTHREADS_SOURCE_DIR}/semaphore.c
  ${PTHREADS_SOURCE_DIR}/signal.c
  ${PTHREADS_SOURCE_DIR}/spin.c
  ${PTHREADS_SOURCE_DIR}/sync.c
  ${PTHREADS_SOURCE_DIR}/tsd.c)
set(PTHREAD_RESOURCES
  ${PTHREADS_SOURCE_DIR}/version.rc)
set(PTHREAD_HEADERS
  ${PTHREADS_SOURCE_DIR}/config.h
  ${PTHREADS_SOURCE_DIR}/implement.h
  ${PTHREADS_SOURCE_DIR}/need_errno.h)

set(PTHREAD_STATIC_SOURCES
  ${PTHREADS_SOURCE_DIR}/pthread_attr_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getdetachstate.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setdetachstate.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getstackaddr.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setstackaddr.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getstacksize.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setstacksize.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getscope.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setscope.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setschedpolicy.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getschedpolicy.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setschedparam.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getschedparam.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_setinheritsched.c
  ${PTHREADS_SOURCE_DIR}/pthread_attr_getinheritsched.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrier_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrier_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrier_wait.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrierattr_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrierattr_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrierattr_setpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_barrierattr_getpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_setcancelstate.c
  ${PTHREADS_SOURCE_DIR}/pthread_setcanceltype.c
  ${PTHREADS_SOURCE_DIR}/pthread_testcancel.c
  ${PTHREADS_SOURCE_DIR}/pthread_cancel.c
  ${PTHREADS_SOURCE_DIR}/cleanup.c
  ${PTHREADS_SOURCE_DIR}/pthread_condattr_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_condattr_getpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_condattr_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_condattr_setpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_cond_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_cond_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_cond_signal.c
  ${PTHREADS_SOURCE_DIR}/pthread_cond_wait.c
  ${PTHREADS_SOURCE_DIR}/create.c
  ${PTHREADS_SOURCE_DIR}/dll.c
  ${PTHREADS_SOURCE_DIR}/autostatic.c
  ${PTHREADS_SOURCE_DIR}/errno.c
  ${PTHREADS_SOURCE_DIR}/pthread_exit.c
  ${PTHREADS_SOURCE_DIR}/fork.c
  ${PTHREADS_SOURCE_DIR}/global.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_getpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_setpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_settype.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_gettype.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_setrobust.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_getrobust.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_lock.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_timedlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_unlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_trylock.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutex_consistent.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_setkind_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_mutexattr_getkind_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_getw32threadhandle_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_getunique_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_delay_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_num_processors_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_win32_attach_detach_np.c
  ${PTHREADS_SOURCE_DIR}/pthread_equal.c
  ${PTHREADS_SOURCE_DIR}/pthread_getconcurrency.c
  ${PTHREADS_SOURCE_DIR}/pthread_once.c
  ${PTHREADS_SOURCE_DIR}/pthread_self.c
  ${PTHREADS_SOURCE_DIR}/pthread_setconcurrency.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlockattr_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlockattr_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlockattr_getpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlockattr_setpshared.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_rdlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_wrlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_unlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_tryrdlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_rwlock_trywrlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_setschedparam.c
  ${PTHREADS_SOURCE_DIR}/pthread_getschedparam.c
  ${PTHREADS_SOURCE_DIR}/pthread_timechange_handler_np.c
  ${PTHREADS_SOURCE_DIR}/ptw32_is_attr.c
  ${PTHREADS_SOURCE_DIR}/ptw32_processInitialize.c
  ${PTHREADS_SOURCE_DIR}/ptw32_processTerminate.c
  ${PTHREADS_SOURCE_DIR}/ptw32_threadStart.c
  ${PTHREADS_SOURCE_DIR}/ptw32_threadDestroy.c
  ${PTHREADS_SOURCE_DIR}/ptw32_tkAssocCreate.c
  ${PTHREADS_SOURCE_DIR}/ptw32_tkAssocDestroy.c
  ${PTHREADS_SOURCE_DIR}/ptw32_callUserDestroyRoutines.c
  ${PTHREADS_SOURCE_DIR}/ptw32_timespec.c
  ${PTHREADS_SOURCE_DIR}/ptw32_throw.c
  ${PTHREADS_SOURCE_DIR}/ptw32_getprocessors.c
  ${PTHREADS_SOURCE_DIR}/ptw32_calloc.c
  ${PTHREADS_SOURCE_DIR}/ptw32_new.c
  ${PTHREADS_SOURCE_DIR}/ptw32_reuse.c
  ${PTHREADS_SOURCE_DIR}/ptw32_rwlock_check_need_init.c
  ${PTHREADS_SOURCE_DIR}/ptw32_cond_check_need_init.c
  ${PTHREADS_SOURCE_DIR}/ptw32_mutex_check_need_init.c
  ${PTHREADS_SOURCE_DIR}/ptw32_semwait.c
  ${PTHREADS_SOURCE_DIR}/ptw32_relmillisecs.c
  ${PTHREADS_SOURCE_DIR}/ptw32_MCS_lock.c
  ${PTHREADS_SOURCE_DIR}/sched_get_priority_max.c
  ${PTHREADS_SOURCE_DIR}/sched_get_priority_min.c
  ${PTHREADS_SOURCE_DIR}/sched_setscheduler.c
  ${PTHREADS_SOURCE_DIR}/sched_getscheduler.c
  ${PTHREADS_SOURCE_DIR}/sched_yield.c
  ${PTHREADS_SOURCE_DIR}/sem_init.c
  ${PTHREADS_SOURCE_DIR}/sem_destroy.c
  ${PTHREADS_SOURCE_DIR}/sem_trywait.c
  ${PTHREADS_SOURCE_DIR}/sem_timedwait.c
  ${PTHREADS_SOURCE_DIR}/sem_wait.c
  ${PTHREADS_SOURCE_DIR}/sem_post.c
  ${PTHREADS_SOURCE_DIR}/sem_post_multiple.c
  ${PTHREADS_SOURCE_DIR}/sem_getvalue.c
  ${PTHREADS_SOURCE_DIR}/sem_open.c
  ${PTHREADS_SOURCE_DIR}/sem_close.c
  ${PTHREADS_SOURCE_DIR}/sem_unlink.c
  ${PTHREADS_SOURCE_DIR}/signal.c
  ${PTHREADS_SOURCE_DIR}/pthread_kill.c
  ${PTHREADS_SOURCE_DIR}/ptw32_spinlock_check_need_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_spin_init.c
  ${PTHREADS_SOURCE_DIR}/pthread_spin_destroy.c
  ${PTHREADS_SOURCE_DIR}/pthread_spin_lock.c
  ${PTHREADS_SOURCE_DIR}/pthread_spin_unlock.c
  ${PTHREADS_SOURCE_DIR}/pthread_spin_trylock.c
  ${PTHREADS_SOURCE_DIR}/pthread_detach.c
  ${PTHREADS_SOURCE_DIR}/pthread_join.c
  ${PTHREADS_SOURCE_DIR}/pthread_key_create.c
  ${PTHREADS_SOURCE_DIR}/pthread_key_delete.c
  ${PTHREADS_SOURCE_DIR}/pthread_setspecific.c
  ${PTHREADS_SOURCE_DIR}/pthread_getspecific.c
  ${PTHREADS_SOURCE_DIR}/w32_CancelableWait.c)


if(PTHREADS_BUILD_STATIC)
  set(SOURCES ${PTHREAD_STATIC_SOURCES})
  set(PTHREADS_LIBTYPE STATIC)
else()
  set(PTHREADS_LIBTYPE SHARED)
endif()

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  add_definitions( -DPTW32_ARCH="x64" )
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  add_definitions( -DPTW32_ARCH="x86" )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

if(MSVC)
  add_definitions(/DPTW32_RC_MSC)
  set(PTHREADS_COMPILER V)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(PTHREADS_COMPILER G)
else()
  message("WARNING: pthreads-win32 doesn't recognize your compiler!")
endif()

if(PTHREADS_BUILD_CPP)
  set(PTHREADS_EXCEPTION_SCHEME CE)
  add_definitions(/__CLEANUP_CXX)
  message("Building pthreads-win32 with C++ exception handling.")
elseif(PTHREADS_BUILD_SEH)
  set(PTHREADS_EXCEPTION_SCHEME SE)
  add_definitions(/__CLEANUP_SEH)
  message("Building pthreads-win32 with Microsoft SEH exception handling.")
else()
  #defaults to C - setjmp/longjmp
  set(PTHREADS_EXCEPTION_SCHEME C)
  message("Building pthreads-win32 with C setjmp/longjmp (default/recommended exception scheme).")
endif()

add_definitions(/DHAVE_PTW32_CONFIG_H)

include_directories(${PTHREADS_SOURCE_DIR})
add_library(${PTHREADS_LIBRARY} ${PTHREADS_LIBTYPE} ${PTHREAD_SOURCES} ${PTHREAD_API} ${PTHREAD_RESOURCES})

if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

#set_target_properties(${PTHREADS_LIBRARY} PROPERTIES PUBLIC_HEADER "${PTHREAD_API}")
if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${PTHREADS_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${PTHREADS_LIBRARY}
  PROPERTIES OUTPUT_NAME ${PTHREADS_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${PTHREADS_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${PTHREADS_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${PTHREADS_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()
