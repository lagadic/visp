#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# CMake package config file for ViSP.
#
# ** File generated automatically, do not modify **
#
# This file will define the following CMake variables:
#   - VISP_INCLUDE_DIRS   : ViSP and third-party include directories
#   - VISP_LIBRARIES      : ViSP library to link against. Third-party libraries are
#                           linked automatically thanks to cmake export file VISPTargets.cmake
#   - VISP_VERSION_STRING : Full ViSP version that is build. Example: "2.10.0"
#   - VISP_VERSION_MAJOR  : Major version part of VISP_VERSION. Example: "2"
#   - VISP_VERSION_MINOR  : Minor version part of VISP_VERSION. Example: "10"
#   - VISP_VERSION_PATCH  : Patch version part of VISP_VERSION. Example: "0"
#
# Advanced variables:
#   - VISP_SHARED        : Use ViSP as shared library
#   - VISP_CONFIG_PATH   : Path to this VISPConfig.cmake
#   - VISP_FIND_QUIETLY  : If set to TRUE turn off messages during configuration
#   - VISP_USE_FILE      : File to include to use ViSP without specific cmake code
#
# Windows specific variables:
#   - VISP_STATIC        : If set to TRUE uses ViSP static library (.lib) rather then dynamic (.dll)
#
# Typical usage in user project:
#
#   find_package(VISP)
#   include_directories(${VISP_INCLUDE_DIRS})
#   target_link_libraries(MY_TARGET_NAME ${VISP_LIBRARIES})
#
# It is also possible to build your project using VISP_USE_FILE.
#
#   find_package(VISP)
#   if(VISP_FOUND)
#     include(${VISP_USE_FILE})
#   endif()
#
# Authors:
# Fabien Spindler
#
#############################################################################

# similar code exist in VISPDetectPlatform.cmake
if(DEFINED OpenCV_ARCH AND DEFINED OpenCV_RUNTIME)
  # custom overridden values
elseif(MSVC)
  if(CMAKE_CL_64)
    set(VISP_ARCH x64)
  elseif((CMAKE_GENERATOR MATCHES "ARM") OR ("${arch_hint}" STREQUAL "ARM") OR (CMAKE_VS_EFFECTIVE_PLATFORMS MATCHES "ARM|arm"))
    set(VISP_ARCH ARM)
  else()
    set(VISP_ARCH x86)
  endif()
  if(MSVC_VERSION EQUAL 1400)
    set(VISP_RUNTIME vc8)
  elseif(MSVC_VERSION EQUAL 1500)
    set(VISP_RUNTIME vc9)
  elseif(MSVC_VERSION EQUAL 1600)
    set(VISP_RUNTIME vc10)
  elseif(MSVC_VERSION EQUAL 1700)
    set(VISP_RUNTIME vc11)
  elseif(MSVC_VERSION EQUAL 1800)
    set(VISP_RUNTIME vc12)
  elseif(MSVC_VERSION EQUAL 1900)
    set(VISP_RUNTIME vc14)
  elseif(MSVC_VERSION MATCHES "^191[0-9]$")
    set(VISP_RUNTIME vc15)
  else()
    message(WARNING "ViSP does not recognize MSVC_VERSION \"${MSVC_VERSION}\". Cannot set VISP_RUNTIME")
  endif()
elseif(MINGW)
  set(VISP_RUNTIME mingw)

  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine
                  OUTPUT_VARIABLE VISP_GCC_TARGET_MACHINE
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(VISP_GCC_TARGET_MACHINE MATCHES "64")
    set(MINGW64 1)
    set(VISP_ARCH x64)
  else()
    set(VISP_ARCH x86)
  endif()
endif()

if(CMAKE_VERSION VERSION_GREATER 2.6.2)
  unset(VISP_CONFIG_PATH CACHE)
endif()

if(NOT VISP_FIND_QUIETLY)
  message(STATUS "ViSP ARCH: ${VISP_ARCH}")
  message(STATUS "ViSP RUNTIME: ${VISP_RUNTIME}")
endif()

get_filename_component(VISP_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)
if(VISP_RUNTIME AND VISP_ARCH)
  if(NOT DEFINED VISP_STATIC AND EXISTS "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/lib/VISPConfig.cmake")
    set(VISP_LIB_PATH "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/lib")
  elseif(NOT DEFINED VISP_STATIC AND EXISTS "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/staticlib/VISPConfig.cmake")
    set(VISP_LIB_PATH "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/staticlib")
  elseif(VISP_STATIC AND EXISTS "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/staticlib/VISPConfig.cmake")
    set(VISP_LIB_PATH "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/staticlib")
  elseif(VISP_STATIC AND EXISTS "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/lib/VISPConfig.cmake")
    set(VISP_LIB_PATH "${VISP_CONFIG_PATH}/${VISP_ARCH}/${VISP_RUNTIME}/lib")
  endif()
endif()

if(VISP_LIB_PATH AND EXISTS "${VISP_LIB_PATH}/VISPConfig.cmake")
  include("${VISP_LIB_PATH}/VISPConfig.cmake")

  set(VISP_FOUND TRUE CACHE BOOL "" FORCE)

  if(NOT VISP_FIND_QUIETLY)
    message(STATUS "Found VISP ${VISP_VERSION} in ${VISP_LIB_PATH}")
    if(NOT VISP_LIB_PATH MATCHES "/staticlib")
      get_filename_component(_VISP_LIB_PATH "${VISP_LIB_PATH}/../bin" ABSOLUTE)
      file(TO_NATIVE_PATH "${_VISP_LIB_PATH}" _VISP_LIB_PATH)
      message(STATUS "You might need to add ${_VISP_LIB_PATH} to your PATH to be able to run your applications.")
    endif()
  endif()
else()
  if(NOT VISP_FIND_QUIETLY)
    message(WARNING
"Found ViSP for Windows but it has no binaries compatible with your configuration.
You should manually point CMake variable VISP_DIR to your build of ViSP library."
    )
  endif()
  set(VISP_FOUND FALSE CACHE BOOL "" FORCE)
endif()
