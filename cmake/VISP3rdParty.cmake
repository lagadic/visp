#----------------------------------------------------------------------
# Add 3rd-party libraries build as static libs
#----------------------------------------------------------------------

if(BUILD_ATIDAQ)
  set(ATIDAQ_LIBRARY atidaq_c)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/atidaq")
  set(ATIDAQ_INCLUDE_DIRS "${${ATIDAQ_LIBRARY}_SOURCE_DIR}" "${${ATIDAQ_LIBRARY}_BINARY_DIR}")
  set(ATIDAQ_LIBRARIES ${ATIDAQ_LIBRARY})
endif()

if(BUILD_CLIPPER)
  set(CLIPPER_LIBRARY clipper)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clipper")
  set(CLIPPER_INCLUDE_DIRS "${${CLIPPER_LIBRARY}_SOURCE_DIR}" "${${CLIPPER_LIBRARY}_BINARY_DIR}")
  set(CLIPPER_LIBRARIES ${CLIPPER_LIBRARY})
endif()

if(BUILD_LAPACK)
  set(LAPACK_LIBRARY visp_lapack)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/lapackblas")
  if(LAPACK_IS_BUILD)
    set(LAPACK_LIBRARIES ${LAPACK_LIBRARY})
    set(LAPACK_VERSION ${LAPACK_MAJOR_VERSION}.${LAPACK_MINOR_VERSION}.${LAPACK_PATCH_VERSION})
  else()
    unset(BUILD_LAPACK)
    set(BUILD_LAPACK OFF CACHE BOOL "Build lapack as private library" FORCE)
    message(WARNING "BUILD_LAPACK turned OFF since no Fortran compiler available.")
  endif()
endif()
