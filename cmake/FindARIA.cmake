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
# Try to find Aria libraries and headers to control Pioneer mobile robots.
# Once run this will define: 
#
# ARIA_FOUND
# ARIA_INCLUDE_DIRS
# ARIA_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

find_path(ARIA_INCLUDE_DIR Aria.h
  $ENV{ARIA_HOME}/include
  /usr/local/Aria/include
  /usr/Aria/include
  /usr/include
  /usr/include/Aria
  /usr/local/include
  /usr/local/include/Aria
  "C:/Program Files/MobileRobots/Aria/include"
)
#MESSAGE("DBG ARIA_INCLUDE_DIR=${ARIA_INCLUDE_DIR}")  

if(UNIX)  
  find_library(ARIA_LIBRARY
    NAMES Aria
    PATHS 
    $ENV{ARIA_HOME}/lib
    /usr/local/Aria/lib
    /usr/Aria/lib
    /usr/lib
    /usr/local/lib
  )
  #MESSAGE("DBG ARIA_LIBRARY=${ARIA_LIBRARY}")
else()
  if(MSVC12)
    set(ARIA_SUFFIX_NAME VC12)
  elseif(MSVC11)
    set(ARIA_SUFFIX_NAME VC11)
  elseif(MSVC10)
    set(ARIA_SUFFIX_NAME VC10)
  elseif(MSVC90)
    set(ARIA_SUFFIX_NAME VC9)
  elseif(MSVC80)
    set(ARIA_SUFFIX_NAME VC8)
  elseif(MSVC71)
    set(ARIA_SUFFIX_NAME VC71)
  else()
    set(ARIA_SUFFIX_NAME "")
  endif()
  
  set(ARIA_LIB_SEARCH_PATH "")
  if(CMAKE_CL_64)
    list(APPEND ARIA_LIB_SEARCH_PATH "C:/Program Files/MobileRobots/Aria/lib64")
    list(APPEND ARIA_LIB_SEARCH_PATH $ENV{ARIA_HOME}/lib64)
  else()
    list(APPEND ARIA_LIB_SEARCH_PATH "C:/Program Files/MobileRobots/Aria/lib")
    list(APPEND ARIA_LIB_SEARCH_PATH $ENV{ARIA_HOME}/lib)
  endif()

  find_library(ARIA_LIBRARY_DEBUG
    NAMES AriaDebug${ARIA_SUFFIX_NAME}
    PATHS 
      ${ARIA_LIB_SEARCH_PATH}
  )
  find_library(ARIA_LIBRARY_RELEASE
    NAMES Aria${ARIA_SUFFIX_NAME}
    PATHS 
      ${ARIA_LIB_SEARCH_PATH}
  )
endif()
    
if(UNIX)  
  if(ARIA_LIBRARY AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    set(ARIA_LIBRARIES ${ARIA_LIBRARY})
    set(ARIA_FOUND TRUE)
  else()
    set(ARIA_FOUND FALSE)
  endif()
else()
  set(ARIA_LIBRARIES "")
  if(ARIA_LIBRARY_RELEASE AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    list(APPEND ARIA_LIBRARIES optimized ${ARIA_LIBRARY_RELEASE})
    set(ARIA_FOUND TRUE)
  endif()
  IF(ARIA_LIBRARY_DEBUG AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    list(APPEND ARIA_LIBRARIES debug ${ARIA_LIBRARY_DEBUG})
    set(ARIA_FOUND TRUE)
  endif()

endif()
  
mark_as_advanced(
  ARIA_INCLUDE_DIR
  ARIA_LIBRARY
  ARIA_LIBRARY_DEBUG
  ARIA_LIBRARY_RELEASE
  ARIA_LIB_SEARCH_PATH
)

