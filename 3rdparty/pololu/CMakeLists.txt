project(${POLOLU_LIBRARY})

set(POLOLU_MAJOR_VERSION 1 PARENT_SCOPE)
set(POLOLU_MINOR_VERSION 0 PARENT_SCOPE)
set(POLOLU_PATCH_VERSION 0 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(lib_srcs src/RPMSerialInterface.cpp)
set(lib_hdrs include/RPMSerialInterface.h)

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
	list(APPEND lib_hdrs include/RPMSerialInterfaceWindows.h)
	list(APPEND lib_srcs src/RPMSerialInterfaceWindows.cpp)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux" OR CMAKE_SYSTEM_NAME MATCHES "Darwin" OR CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
  # Could also be used on Windows with MinGW
  list(APPEND lib_hdrs include/RPMSerialInterfacePOSIX.h)
  list(APPEND lib_srcs src/RPMSerialInterfacePOSIX.cpp)
else()
	message("${PROJECT_NAME} is only available for Windows, Linux and Darwin. Disabled for ${CMAKE_SYSTEM_NAME}")
  unset(WITH_POLOLU)
  set(WITH_POLOLU OFF CACHE BOOL "Build rapa pololu as built-in library" FORCE)
  return()
endif()

add_library(${POLOLU_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  endif()
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${POLOLU_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${POLOLU_LIBRARY}
  PROPERTIES OUTPUT_NAME ${POLOLU_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${POLOLU_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${POLOLU_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${POLOLU_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()
