project(${ATIDAQ_LIBRARY})

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.c)
file(GLOB lib_hdrs *.h)

vp_list_filterout(lib_srcs xmltok_impl.c)
vp_list_filterout(lib_hdrs xmltok_impl.h)
vp_list_filterout(lib_srcs xmltok_ns.c)
vp_list_filterout(lib_srcs events.c)

add_library(${ATIDAQ_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${ATIDAQ_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

vp_warnings_disable(CMAKE_C_FLAGS -Wunused-parameter)

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${ATIDAQ_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${ATIDAQ_LIBRARY}
  PROPERTIES OUTPUT_NAME ${ATIDAQ_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${ATIDAQ_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${ATIDAQ_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${ATIDAQ_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()
