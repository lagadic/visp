#----------------------------------------------------------------------
# Add 3rd-party libraries build as static libs
#----------------------------------------------------------------------

if(WITH_ATIDAQ)
  set(ATIDAQ_LIBRARY atidaq_c)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/atidaq")
  set(ATIDAQ_INCLUDE_DIRS "${${ATIDAQ_LIBRARY}_SOURCE_DIR}" "${${ATIDAQ_LIBRARY}_BINARY_DIR}")
  set(ATIDAQ_LIBRARIES ${ATIDAQ_LIBRARY})
  set(ATIDAQ_VERSION ${ATIDAQ_MAJOR_VERSION}.${ATIDAQ_MINOR_VERSION}.${ATIDAQ_PATCH_VERSION})
endif()

if(WITH_CLIPPER)
  set(CLIPPER_LIBRARY clipper)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clipper")
  set(CLIPPER_INCLUDE_DIRS "${${CLIPPER_LIBRARY}_SOURCE_DIR}" "${${CLIPPER_LIBRARY}_BINARY_DIR}")
  set(CLIPPER_LIBRARIES ${CLIPPER_LIBRARY})
  set(CLIPPER_VERSION ${CLIPPER_MAJOR_VERSION}.${CLIPPER_MINOR_VERSION}.${CLIPPER_PATCH_VERSION})
endif()

if(WITH_LAPACK)
  set(LAPACK_LIBRARY visp_lapack)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clapack")
  set(LAPACK_LIBRARIES ${LAPACK_LIBRARY})
  set(LAPACK_VERSION ${LAPACK_MAJOR_VERSION}.${LAPACK_MINOR_VERSION}.${LAPACK_PATCH_VERSION})
endif()

if(WITH_PTHREAD)
  set(PTHREADS_LIBRARY visp_pthread)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/pthreads-win32")
  set(PTHREADS_INCLUDE_DIRS "${${PTHREADS_LIBRARY}_SOURCE_DIR}" "${${PTHREADS_LIBRARY}_BINARY_DIR}")
  set(PTHREADS_LIBRARIES ${PTHREADS_LIBRARY})
  set(PTHREADS_VERSION ${PTHREADS_MAJOR_VERSION}.${PTHREADS_MINOR_VERSION}.${PTHREADS_PATCH_VERSION})
endif()

if(WITH_APRILTAG)
  set(APRILTAG_LIBRARY apriltag)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/apriltag")
  set(APRILTAG_INCLUDE_DIRS "${${APRILTAG_LIBRARY}_SOURCE_DIR}" "${${APRILTAG_LIBRARY}_BINARY_DIR}")
  set(APRILTAG_LIBRARIES ${APRILTAG_LIBRARY})
endif()
