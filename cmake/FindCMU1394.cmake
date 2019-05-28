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
# Try to find CMU 1394 Digital Camera SDK to acquire images from firewire 
# cameras on Windows.
# Once run this will define: 
#
# CMU1394_FOUND
# CMU1394_INCLUDE_DIRS
# CMU1394_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(WIN32)
  FIND_PATH(CMU1394_INCLUDE_DIR 1394Camera.h
    $ENV{CMU1394_HOME}/include
	"C:/Program Files (x86)/CMU/1394Camera/include"
	"C:/Program Files/CMU/1394Camera/include"
    )
  #MESSAGE("DBG CMU1394_INCLUDE_DIR=${CMU1394_INCLUDE_DIR}")  
  
  if(CMAKE_CL_64)
    FIND_LIBRARY(CMU1394_LIBRARY_DEBUG
      NAMES 1394camerad
      PATHS 
      $ENV{CMU1394_HOME}/lib64/x64
	  "C:/Program Files (x86)/CMU/1394Camera/lib64/x64"
	  "C:/Program Files/CMU/1394Camera/lib64/x64"
      )
  
    FIND_LIBRARY(CMU1394_LIBRARY_RELEASE
      NAMES 1394camera
      PATHS 
      $ENV{CMU1394_HOME}/lib64/x64
	  "C:/Program Files (x86)/CMU/1394Camera/lib64/x64"
	  "C:/Program Files/CMU/1394Camera/lib64/x64"
      )
  else()
    FIND_LIBRARY(CMU1394_LIBRARY_DEBUG
      NAMES 1394camerad
      PATHS 
      $ENV{CMU1394_HOME}/lib
	  "C:/Program Files (x86)/CMU/1394Camera/lib"
	  "C:/Program Files/CMU/1394Camera/lib"
      )
  
    FIND_LIBRARY(CMU1394_LIBRARY_RELEASE
      NAMES 1394camera
      PATHS 
      $ENV{CMU1394_HOME}/lib
	  "C:/Program Files (x86)/CMU/1394Camera/lib"
	  "C:/Program Files/CMU/1394Camera/lib"
      )
  endif()
  #MESSAGE("DBG CMU1394_LIBRARY_DEBUG=${CMU1394_LIBRARY_DEBUG}")
  #MESSAGE("DBG CMU1394_LIBRARY_RELEASE=${CMU1394_LIBRARY_RELEASE}")

  set(CMU1394_LIBRARIES "")
  if(CMU1394_LIBRARY_RELEASE AND CMU1394_INCLUDE_DIR)
    set(CMU1394_INCLUDE_DIRS ${CMU1394_INCLUDE_DIR})
    list(APPEND CMU1394_LIBRARIES optimized ${CMU1394_LIBRARY_RELEASE})
    set(CMU1394_FOUND TRUE)
  endif()
  if(CMU1394_LIBRARY_DEBUG AND CMU1394_INCLUDE_DIR)
    set(CMU1394_INCLUDE_DIRS ${CMU1394_INCLUDE_DIR})
    list(APPEND CMU1394_LIBRARIES debug ${CMU1394_LIBRARY_DEBUG})
    set(CMU1394_FOUND TRUE)
  ENDIF()

  MARK_AS_ADVANCED(
    CMU1394_INCLUDE_DIR
    CMU1394_LIBRARY
    CMU1394_LIBRARY_DEBUG
    CMU1394_LIBRARY_RELEASE
    )
endif()
