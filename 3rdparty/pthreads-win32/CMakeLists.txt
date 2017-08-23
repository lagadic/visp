project(${PTHREADS_LIBRARY})

set(PTHREADS_MAJOR_VERSION 2 PARENT_SCOPE)
set(PTHREADS_MINOR_VERSION 9 PARENT_SCOPE)
set(PTHREADS_PATCH_VERSION 1 PARENT_SCOPE)

set(PTHREAD_API
  pthread.h
  sched.h
  semaphore.h)

source_group(PTHREAD_API FILES ${PTHREAD_API})

set(PTHREAD_HEADERS
  config.h
  implement.h
  need_errno.h)

set(PTHREAD_STATIC_SOURCES
  pthread_attr_init.c
  pthread_attr_destroy.c
  pthread_attr_getdetachstate.c
  pthread_attr_setdetachstate.c
  pthread_attr_getstackaddr.c
  pthread_attr_setstackaddr.c
  pthread_attr_getstacksize.c
  pthread_attr_setstacksize.c
  pthread_attr_getscope.c
  pthread_attr_setscope.c
  pthread_attr_setschedpolicy.c
  pthread_attr_getschedpolicy.c
  pthread_attr_setschedparam.c
  pthread_attr_getschedparam.c
  pthread_attr_setinheritsched.c
  pthread_attr_getinheritsched.c
  pthread_barrier_init.c
  pthread_barrier_destroy.c
  pthread_barrier_wait.c
  pthread_barrierattr_init.c
  pthread_barrierattr_destroy.c
  pthread_barrierattr_setpshared.c
  pthread_barrierattr_getpshared.c
  pthread_setcancelstate.c
  pthread_setcanceltype.c
  pthread_testcancel.c
  pthread_cancel.c
  cleanup.c
  pthread_condattr_destroy.c
  pthread_condattr_getpshared.c
  pthread_condattr_init.c
  pthread_condattr_setpshared.c
  pthread_cond_destroy.c
  pthread_cond_init.c
  pthread_cond_signal.c
  pthread_cond_wait.c
  create.c
  dll.c
  autostatic.c
  errno.c
  pthread_exit.c
  fork.c
  global.c
  pthread_mutex_init.c
  pthread_mutex_destroy.c
  pthread_mutexattr_init.c
  pthread_mutexattr_destroy.c
  pthread_mutexattr_getpshared.c
  pthread_mutexattr_setpshared.c
  pthread_mutexattr_settype.c
  pthread_mutexattr_gettype.c
  pthread_mutexattr_setrobust.c
  pthread_mutexattr_getrobust.c
  pthread_mutex_lock.c
  pthread_mutex_timedlock.c
  pthread_mutex_unlock.c
  pthread_mutex_trylock.c
  pthread_mutex_consistent.c
  pthread_mutexattr_setkind_np.c
  pthread_mutexattr_getkind_np.c
  pthread_getw32threadhandle_np.c
  pthread_getunique_np.c
  pthread_delay_np.c
  pthread_num_processors_np.c
  pthread_win32_attach_detach_np.c
  pthread_equal.c
  pthread_getconcurrency.c
  pthread_once.c
  pthread_self.c
  pthread_setconcurrency.c
  pthread_rwlock_init.c
  pthread_rwlock_destroy.c
  pthread_rwlockattr_init.c
  pthread_rwlockattr_destroy.c
  pthread_rwlockattr_getpshared.c
  pthread_rwlockattr_setpshared.c
  pthread_rwlock_rdlock.c
  pthread_rwlock_wrlock.c
  pthread_rwlock_unlock.c
  pthread_rwlock_tryrdlock.c
  pthread_rwlock_trywrlock.c
  pthread_setschedparam.c
  pthread_getschedparam.c
  pthread_timechange_handler_np.c
  ptw32_is_attr.c
  ptw32_processInitialize.c
  ptw32_processTerminate.c
  ptw32_threadStart.c
  ptw32_threadDestroy.c
  ptw32_tkAssocCreate.c
  ptw32_tkAssocDestroy.c
  ptw32_callUserDestroyRoutines.c
  ptw32_timespec.c
  ptw32_throw.c
  ptw32_getprocessors.c
  ptw32_calloc.c
  ptw32_new.c
  ptw32_reuse.c
  ptw32_rwlock_check_need_init.c
  ptw32_cond_check_need_init.c
  ptw32_mutex_check_need_init.c
  ptw32_semwait.c
  ptw32_relmillisecs.c
  ptw32_MCS_lock.c
  sched_get_priority_max.c
  sched_get_priority_min.c
  sched_setscheduler.c
  sched_getscheduler.c
  sched_yield.c
  sem_init.c
  sem_destroy.c
  sem_trywait.c
  sem_timedwait.c
  sem_wait.c
  sem_post.c
  sem_post_multiple.c
  sem_getvalue.c
  sem_open.c
  sem_close.c
  sem_unlink.c
  signal.c
  pthread_kill.c
  ptw32_spinlock_check_need_init.c
  pthread_spin_init.c
  pthread_spin_destroy.c
  pthread_spin_lock.c
  pthread_spin_unlock.c
  pthread_spin_trylock.c
  pthread_detach.c
  pthread_join.c
  pthread_key_create.c
  pthread_key_delete.c
  pthread_setspecific.c
  pthread_getspecific.c
  w32_CancelableWait.c)

add_definitions( -DPTW32_BUILD_INLINED )

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(${PTHREADS_LIBRARY} STATIC ${PTHREAD_STATIC_SOURCES} ${PTHREAD_API} ${PTHREAD_HEADERS})

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
target_compile_definitions(${PTHREADS_LIBRARY} PUBLIC PTW32_STATIC_LIB HAVE_PTW32_CONFIG_H)
target_include_directories(${PTHREADS_LIBRARY} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
#set_property(TARGET ${PTHREADS_LIBRARY} APPEND PROPERTY
#  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${PTHREADS_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

if(MSVC)
  vp_set_source_file_compile_flag(pthread_spin_lock.c /wd4013)
  vp_set_source_file_compile_flag(pthread_spin_trylock.c /wd4013)
  vp_set_source_file_compile_flag(pthread_win32_attach_detach_np.c /wd4101)
  vp_set_source_file_compile_flag(ptw32_getprocessors.c /wd4013)
endif()