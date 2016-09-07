#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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

if(APPLE_FRAMEWORK)
  find_library(OpenCV_FRAMEWORK opencv2)

  if(OpenCV_FRAMEWORK)
    set(OpenCV_FOUND TRUE)
    set(OpenCV_LIBRARIES ${OpenCV_FRAMEWORK})

    file(GLOB_RECURSE opencv_version_file "${OpenCV_FRAMEWORK}/Headers/version.hpp")

    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MAJOR" OpenCV_VERSION_MAJOR)
    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_MINOR" OpenCV_VERSION_MINOR)
    vp_get_opencv_version_from_hpp(${opencv_version_file} "CV_VERSION_REVISION" OpenCV_VERSION_PATCH)
    set(OpenCV_VERSION "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")
  else()
    set(OpenCV_FOUND FALSE)
  endif()

  mark_as_advanced(OpenCV_FRAMEWORK)
endif()