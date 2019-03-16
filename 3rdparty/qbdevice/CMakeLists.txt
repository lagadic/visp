project(${QBDEVICE_LIBRARY})

# Version updated manually from command.h (#define API_VERSION "v6.2.0")
set(QBDEVICE_MAJOR_VERSION 2 PARENT_SCOPE)
set(QBDEVICE_MINOR_VERSION 6 PARENT_SCOPE)
set(QBDEVICE_PATCH_VERSION 0 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.cpp)
file(GLOB lib_hdrs *.h)

add_library(${QBDEVICE_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  endif()
endif()

if(USE_CXX11 AND CXX11_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_CXX_FLAGS}")
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${QBDEVICE_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${QBDEVICE_LIBRARY}
  PROPERTIES OUTPUT_NAME ${QBDEVICE_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${QBDEVICE_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${QBDEVICE_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${QBDEVICE_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

if(MSVC)
  vp_warnings_disable(CMAKE_CXX_FLAGS /wd4305 /wd4309)
else()
  vp_set_source_file_compile_flag(qbmove_communications.cpp -Wno-unused-result)
endif()
