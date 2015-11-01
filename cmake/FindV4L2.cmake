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
# Try to find linux/videodev.h for Video For Linux Two framegrabbing 
# capabilities.
# Once run this will define: 
#
# V4L2_FOUND
# V4L2_INCLUDE_DIRS
# V4L2_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindV4L2.cmake: only available for Unix.")
  SET(V4L2_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(V4L2_INCLUDE_VIDEODEV2 linux/videodev2.h
    $ENV{V4L2_HOME}/include
    $ENV{V4L2_DIR}/include
    /usr/include 
    /usr/local/include 
    /usr/src/linux/include
  )
  #MESSAGE("DBG V4L2_INCLUDE_VIDEODEV2=${V4L2_INCLUDE_VIDEODEV2}")  

  FIND_PATH(V4L2_INCLUDE_LIBV4L2 libv4l2.h
    $ENV{V4L2_HOME}/include
    $ENV{V4L2_DIR}/include
    /usr/include 
    /usr/local/include
  )
  #MESSAGE("DBG V4L2_INCLUDE_LIBV4L2=${V4L2_INCLUDE_LIBV4L2}")  
  
  FIND_LIBRARY(V4L2_LIBRARY_LIBV4L2
    NAMES v4l2
    PATHS 
    $ENV{V4L2_HOME}/lib
    $ENV{V4L2_DIR}/lib  
    /usr/lib
    /usr/local/lib
  )

  FIND_LIBRARY(V4L2_LIBRARY_LIBV4LCONVERT
    NAMES v4lconvert
    PATHS 
    $ENV{V4L2_HOME}/lib
    $ENV{V4L2_DIR}/lib  
    /usr/lib
    /usr/local/lib
  )

    
  ## --------------------------------
    
 
  IF(V4L2_INCLUDE_VIDEODEV2 AND V4L2_INCLUDE_LIBV4L2 AND V4L2_LIBRARY_LIBV4L2 AND V4L2_LIBRARY_LIBV4LCONVERT)
    SET(V4L2_INCLUDE_DIRS ${V4L2_INCLUDE_VIDEODEV2} ${V4L2_INCLUDE_LIBV4L2})
    SET(V4L2_LIBRARIES ${V4L2_LIBRARY_LIBV4L2} ${V4L2_LIBRARY_LIBV4LCONVERT})
    SET(V4L2_FOUND TRUE)
  ELSE()
    SET(V4L2_FOUND FALSE)
  ENDIF()
  
  MARK_AS_ADVANCED(
    V4L2_INCLUDE_DIRS
    V4L2_INCLUDE_VIDEODEV2
    V4L2_INCLUDE_LIBV4L2
    V4L2_LIBRARY_LIBV4L2
    V4L2_LIBRARY_LIBV4LCONVERT
    )
ENDIF(NOT UNIX)
