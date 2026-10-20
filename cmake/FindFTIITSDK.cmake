#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
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
# FTIITSDK_FOUND
# FTIITSDK_INCLUDE_DIRS
# FTIITSDK_LIBRARIES
#
#############################################################################

# platform detection
if(CMAKE_SYSTEM_NAME MATCHES Windows)
  set(OS "win")
  set(OS_PATH "win")
  if(CMAKE_CL_64)
    set(ARCH "x64")
  else()
    set(ARCH "Win32")
  endif()
elseif(CMAKE_SYSTEM_NAME MATCHES Linux)
  set(OS "lin")
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH "x86_64")
  else()
    set(ARCH "i686")
  endif()

  find_program(LSB_RELEASE_EXEC lsb_release)
  if(LSB_RELEASE_EXEC)
    execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
      OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()

  if(LSB_RELEASE_ID_SHORT MATCHES Ubuntu)
    set(OS_PATH "linux/ubuntu16.04")
  elseif(LSB_RELEASE_ID_SHORT MATCHES Arch)
    set(OS_PATH "linux/archLinux")
  endif()
elseif(CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(OS "mac")
  set(ARCH "x86_64")
  return() # platform not supported
endif()


set(FTIITSDK_SEARCH_HINTS
  ${FTIITSDK_HOME}
  $ENV{FTIITSDK_HOME}
)
set(FTIITSDK_LIBRARY_DIR "${OS_PATH}/ftSensorLibReleaseExamples/library")
set(FTIITSDK_LIB_SUFFIXES
  ${FTIITSDK_LIBRARY_DIR}/bin/${OS}-${ARCH}/release
  ${FTIITSDK_LIBRARY_DIR}/lib/${OS}-${ARCH}/release
)

find_path(FTIITSDK_INCLUDE_DIR ftSensorLib/ftSensorLib.h
  HINTS ${FTIITSDK_SEARCH_HINTS}
  PATH_SUFFIXES ${FTIITSDK_LIBRARY_DIR}/include
)

find_library(FTIITSDK_FT_LIBRARY
  NAMES libftSensorLib.so.0.0.1 ftSensorLib
  HINTS ${FTIITSDK_SEARCH_HINTS}
  PATH_SUFFIXES ${FTIITSDK_LIB_SUFFIXES}
)

find_library(FTIITSDK_COMMUNICATION_LIBRARY
  NAMES libcommunicationLib.so.0.0.1 communicationLib
  HINTS ${FTIITSDK_SEARCH_HINTS}
  PATH_SUFFIXES ${FTIITSDK_LIB_SUFFIXES}
)

include(FindPackageHandleStandardArgs)
# Handle the QUIETLY and REQUIRED arguments and set the FTIITSDK_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(FTIITSDK DEFAULT_MSG
                                  FTIITSDK_FT_LIBRARY FTIITSDK_COMMUNICATION_LIBRARY FTIITSDK_INCLUDE_DIR)

if(FTIITSDK_FOUND)
  set(FTIITSDK_INCLUDE_DIRS ${FTIITSDK_INCLUDE_DIR})
  set(FTIITSDK_LIBRARIES ${FTIITSDK_FT_LIBRARY} ${FTIITSDK_COMMUNICATION_LIBRARY})
endif()

mark_as_advanced(
  FTIITSDK_INCLUDE_DIR
  FTIITSDK_FT_LIBRARY
  FTIITSDK_COMMUNICATION_LIBRARY
  LSB_RELEASE_EXEC
)
