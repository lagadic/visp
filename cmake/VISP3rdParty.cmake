#----------------------------------------------------------------------
# Add 3rd-party libraries build as static libs
#----------------------------------------------------------------------

if(BUILD_ATIDAQ)
  set(ATIDAQ_LIBRARY atidaq_c)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/libatidaq-c")
  set(ATIDAQ_INCLUDE_DIRS "${${ATIDAQ_LIBRARY}_SOURCE_DIR}" "${${ATIDAQ_LIBRARY}_BINARY_DIR}")
  set(ATIDAQ_LIBRARIES ${ATIDAQ_LIBRARY})
endif()
