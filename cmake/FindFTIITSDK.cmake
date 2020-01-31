# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find IIT force-torque sensor SDK
#
# FTIIT_FOUND
# FTIIT_INCLUDE_DIRS
# FTIIT_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

# platform detection
if(${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set(OS "win")
  set(OS_PATH "win")
  if(${CMAKE_CL_64})
    set(ARCH "x64")
  else()
    set(ARCH "Win32")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  set(OS "lin" )
  if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(ARCH "x86_64" )
  else()
    set(ARCH "i686" )
  endif()

  find_program(LSB_RELEASE_EXEC lsb_release)
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(LSB_RELEASE_ID_SHORT MATCHES Ubuntu)
    set(OS_PATH "linux/ubuntu16.04")
  elseif(LSB_RELEASE_ID_SHORT MATCHES Arch)
    set(OS_PATH "linux/archLinux")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(OS "mac")
  set(ARCH "x86_64")
  return() # platform not supported
endif()

find_path(FTIITSDK_INCLUDE_DIR ftSensorLib/ftSensorLib.h
  PATHS
    "$ENV{FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/include"
    "${FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/include"
)

find_library(FTIITSDK_FT_LIBRARY
  NAMES libftSensorLib.so.0.0.1 ftSensorLib
  PATHS
    "$ENV{FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/bin/${OS}-${ARCH}/release"
    "${FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/bin/${OS}-${ARCH}/release"
    "$ENV{FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/lib/${OS}-${ARCH}/release"
    "${FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/lib/${OS}-${ARCH}/release"
)

find_library(FTIITSDK_COMMUNICATION_LIBRARY
  NAMES libcommunicationLib.so.0.0.1 communicationLib
  PATHS
    "$ENV{FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/bin/${OS}-${ARCH}/release"
    "${FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/bin/${OS}-${ARCH}/release"
    "$ENV{FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/lib/${OS}-${ARCH}/release"
    "${FTIITSDK_HOME}/${OS_PATH}/ftSensorLibReleaseExamples/library/lib/${OS}-${ARCH}/release"
)

include(FindPackageHandleStandardArgs)
# Handle the QUIETLY and REQUIRED arguments and set the FTIIT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(FTIITSDK DEFAULT_MSG
                                  FTIITSDK_FT_LIBRARY FTIITSDK_COMMUNICATION_LIBRARY FTIITSDK_INCLUDE_DIR)

if(FTIITSDK_FOUND)
  set(FTIITSDK_INCLUDE_DIRS ${FTIITSDK_INCLUDE_DIR})
  set(FTIITSDK_LIBRARIES ${FTIITSDK_FT_LIBRARY} ${FTIITSDK_COMMUNICATION_LIBRARY})
endif()

mark_as_advanced(
  FTIITSDK_INCLUDE_DIR
  FTIITSDK_INCLUDE_DIRS
  FTIITSDK_FT_LIBRARY
  FTIITSDK_COMMUNICATION_LIBRARY
  FTIITSDK_LIBRARIES
  LSB_RELEASE_EXEC
)

