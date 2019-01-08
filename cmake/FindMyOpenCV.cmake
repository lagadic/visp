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
# Try to find OpenCV framework. 
#
# OpenCV_FOUND
# OpenCV_INCLUDE_DIRS (only for naoqi)
# OpenCV_LIBRARIES
# OpenCV_VERSION_MAJOR
# OpenCV_VERSION_MINOR
# OpenCV_VERSION_PATCH
# OpenCV_VERSION
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Parse FILENAME in order to find the cxx macro DEFINE
# and get its value in VAR.
# Useful to get the value of CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION
macro(vp_get_opencv_version_from_hpp FILENAME DEFINE VAR)
  if(EXISTS "${FILENAME}")
    file(STRINGS ${FILENAME} __matches REGEX "#define ${DEFINE}")
    vp_create_list_from_string(${__matches} __list)
    list(LENGTH __list __length)
    if(__length EQUAL 3)
      list(REVERSE __list)
      list(GET __list 0 ${VAR})
    else()
      message("Warning: OpenCV macro ${DEFINE} not found in ${FILENAME}. The following var ${VAR} is set to 0. This may produce build issues.")
      set(${__var} 0)
    endif()
  else()
    message("Warning: Cannot retrive OpenCV version from non existing file ${FILENAME}. This may produce build issues.")
    set(${__var} 0)
  endif()
endmacro()

macro(find_package_naoqi module)
  set(${module}_FOUND FALSE)
  set(${module}_INCLUDE_DIRS "")
  set(${module}_LIBRARIES "")
  set(${module}_DEPS "")
  set(${module}_DEPS_PROCESSED "")

  _find_package_naoqi_internal(${module})

  # remove allready treated deps
  foreach(__d ${${module}_DEPS_PROCESSED})
    list(FIND ${module}_DEPS ${__d} __index)
    if(NOT __index LESS 0)
      list(REMOVE_AT ${module}_DEPS ${__index})
    endif()
  endforeach()

  # hack for ctc-linux??-atom-2.3.1.23 where alsa module required by ffmpeg is not a dependency
  string(FIND "${TOOLCHAIN_ROOT_DIR}" "-atom-2.3" __apply_hack)
  if(__apply_hack GREATER 0)
    list(APPEND ${module}_DEPS ALSA)
  endif()

  while(${module}_DEPS)
    vp_list_pop_front(${module}_DEPS __d)
    string(TOUPPER "${__d}" __upper_d)
    _find_package_naoqi_internal(${__upper_d})
    if(${__upper_d}_FOUND)
      if(${__upper_d}_INCLUDE_DIRS)
        list(APPEND ${module}_INCLUDE_DIRS "${${__upper_d}_INCLUDE_DIRS}")
      endif()
      if(${__upper_d}_LIBRARIES)
        list(APPEND ${module}_LIBRARIES "${${__upper_d}_LIBRARIES}")
      endif()
      list(APPEND ${module}_DEPS_PROCESSED ${__upper_d})
      if(${__upper_d}_DEPS)
        vp_list_unique(${__upper_d}_DEPS)
        # add only deps if not already processed
        foreach(__new_d ${${__upper_d}_DEPS})
          list(FIND ${module}_DEPS_PROCESSED ${__new_d} __index)
          if(__index LESS 0)
            list(APPEND ${module}_DEPS ${__new_d})
          endif()
        endforeach()
      endif()
      vp_list_unique(${module}_DEPS)
    endif()
  endwhile()

  vp_list_unique(${module}_INCLUDE_DIRS)
  vp_list_unique(${module}_LIBRARIES)
endmacro()

macro(_find_package_naoqi_internal module)
  string(TOLOWER "${module}" __lower_module)
  file(GLOB __submodule RELATIVE "${TOOLCHAIN_ROOT_DIR}/${__lower_module}/share/cmake" "${TOOLCHAIN_ROOT_DIR}/${__lower_module}/share/cmake/*")

  foreach(__s ${__submodule})
    set(${__s}_DIR "${TOOLCHAIN_ROOT_DIR}/${__lower_module}/share/cmake/${__s}")
    find_package(${__s})
    if(${__s}_FOUND)
      set(${module}_FOUND TRUE)
      string(TOUPPER "${__s}" __upper_s)
      if(${__upper_s}_INCLUDE_DIRS)
        list(APPEND ${module}_INCLUDE_DIRS ${${__upper_s}_INCLUDE_DIRS})
      endif()
      if(${__upper_s}_LIBRARIES)
        list(APPEND ${module}_LIBRARIES ${${__upper_s}_LIBRARIES})
      endif()
      if(${__upper_s}_DEPENDS)
        list(APPEND ${module}_DEPS ${${__upper_s}_DEPENDS})
      endif()
      list(APPEND ${module}_DEPS_PROCESSED ${__upper_s})
    endif()
    unset(${__s}_DIR)
  endforeach()

  if(${module}_FOUND)
    # check if there are not other libs that are not submodule and append
    file(GLOB __libs "${TOOLCHAIN_ROOT_DIR}/${__lower_module}/lib/lib*.so")

    foreach(__l ${__libs})
      list(FIND ${module}_LIBRARIES ${__l} __index)
      if(__index LESS 0)
        list(APPEND ${module}_LIBRARIES ${__l})
      endif()
    endforeach()
    vp_list_unique(${lower_submodule}_LIBRARIES)
  endif()
endmacro()

if(APPLE_FRAMEWORK)
  find_library(OpenCV_FRAMEWORK opencv2)

  if(OpenCV_FRAMEWORK)
    set(OpenCV_FOUND TRUE)
    set(OpenCV_LIBRARIES ${OpenCV_FRAMEWORK})

    file(GLOB_RECURSE opencv_version_file "${OpenCV_FRAMEWORK}/Headers/core/version.hpp")

    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MAJOR" OpenCV_VERSION_MAJOR)
    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MINOR" OpenCV_VERSION_MINOR)
    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_REVISION" OpenCV_VERSION_PATCH)
    set(OpenCV_VERSION "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")
  else()
    set(OpenCV_FOUND FALSE)
  endif()

  mark_as_advanced(OpenCV_FRAMEWORK)
endif()

# Detect if the toolchain is for Aldebaran naoqi
if(CMAKE_TOOLCHAIN_FILE AND I_AM_A_ROBOT)
  find_package_naoqi(OpenCV)
  if(OpenCV_FOUND)
    file(GLOB_RECURSE opencv_version_file "${TOOLCHAIN_ROOT_DIR}/opencv/include/opencv2/core/version.hpp")

    # check if old version like 2.4.5 has CV_VERSION_EPOCH
    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_EPOCH" OpenCV_VERSION_MAJOR)
    if(OpenCV_VERSION_MAJOR EQUAL 0)
      vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MAJOR" OpenCV_VERSION_MAJOR)
      vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MINOR" OpenCV_VERSION_MINOR)
      vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_REVISION" OpenCV_VERSION_PATCH)
    else()
      vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MAJOR" OpenCV_VERSION_MINOR)
      vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MINOR" OpenCV_VERSION_PATCH)
    endif()
    set(OpenCV_VERSION "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")

    # check if nonfree module exists
    foreach(__l ${OpenCV_LIBRARIES})
      string(REGEX MATCH "^.*opencv_nonfree" has_nonfree ${__l})
      if(has_nonfree)
        set(OPENCV_NONFREE_FOUND TRUE)
        break()
      endif()
    endforeach()
  endif()
endif()

