project(${APRILTAG_LIBRARY})

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.c common/*.c)
file(GLOB lib_hdrs *.h common/*.h common/sys/*.h)

vp_list_filterout(lib_srcs common/unionfind.c) # has no symbols

add_library(${APRILTAG_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})
if(WIN32 AND MSVC)
  target_link_libraries (${APRILTAG_LIBRARY} ${PTHREADS_LIBRARY})
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C99_C_FLAGS}")
if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${APRILTAG_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${APRILTAG_LIBRARY}
  PROPERTIES OUTPUT_NAME ${APRILTAG_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${APRILTAG_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${APRILTAG_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${APRILTAG_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()
