project(${PTHREADS_LIBRARY})

set(PTHREADS_MAJOR_VERSION 3 PARENT_SCOPE)
set(PTHREADS_MINOR_VERSION 0 PARENT_SCOPE)
set(PTHREADS_PATCH_VERSION 1 PARENT_SCOPE)

set(PTHREAD_API
  _ptw32.h
  pthread.h
  sched.h
  semaphore.h)

source_group(PTHREAD_API FILES ${PTHREAD_API})

set(PTHREAD_HEADERS
  config.h
  context.h
  implement.h
  need_errno.h)

set(PTHREAD_STATIC_SOURCES
  pthread.c
  )

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
target_compile_definitions(${PTHREADS_LIBRARY} PUBLIC __PTW32_STATIC_LIB HAVE_CONFIG_H)
target_include_directories(${PTHREADS_LIBRARY} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${PTHREADS_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

# From pthreads-win32 CMakeLists.txt
# if(MSVC)
  # if(BUILD_SHARED_LIBS)
    # vp_set_source_file_compile_flag(pthread_spin_lock.c /wd4013)
    # vp_set_source_file_compile_flag(pthread_spin_trylock.c /wd4013)
    # vp_set_source_file_compile_flag(pthread_win32_attach_detach_np.c /wd4101)
    # vp_set_source_file_compile_flag(ptw32_getprocessors.c /wd4013)
  # else()
    # vp_warnings_disable(CMAKE_C_FLAGS /wd4101 /wd4013)
  # endif()
# endif()