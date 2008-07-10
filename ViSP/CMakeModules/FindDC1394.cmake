#############################################################################
#
# $Id: FindDC1394.cmake,v 1.6 2008-07-10 08:36:58 asaunier Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Try to find libDC1394 for IEEE1394 camera. First search for libdc1394-2.x
# and if not found, search for libdc1394-1.x
# Once run this will define: 
#
# DC1394_FOUND
# DC1394_1_FOUND
# DC1394_2_FOUND
# DC1394_VERSION
# DC1394_INCLUDE_DIR
# DC1394_1_INCLUDE_DIR
# DC1394_2_INCLUDE_DIR
# DC1394_LIBRARIES
# DC1394_1_LIBRARY
# DC1394_2_LIBRARY
#
# The two defines below are only usefull to compile with libdc1394-2.x. In
# that case DC1394_VERSION=2. Since the libdc1394-2.x API is not stable, we 
# need to determine if dc1394_find_cameras() or dc1394_enumerate_cameras() 
# functions are available. dc1394_enumerate_cameras() was introduced after 
# libdc1394-2.0.0-rc7. DC1394_CAMERA_ENUMERATE_FOUND is TRUE when 
# dc1394_camera_enumerate() function is found. DC1394_FIND_CAMERAS_FOUND is 
# TRUE when dc1394_find_cameras() is found.
# DC1394_CAMERA_ENUMERATE_FOUND
# DC1394_FIND_CAMERAS_FOUND
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindDC1394.cmake: libdc1394 only available for Unix.")
  SET(DC1394_FOUND FALSE)
ELSE(NOT UNIX)
# Search for libdc1394-1.x
  
  FIND_PATH(DC1394_1_INCLUDE_DIR libdc1394/dc1394_control.h
    $ENV{DC1394_HOME}/include
    /usr/include )
#MESSAGE("DBG DC1394_1_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")

  FIND_LIBRARY(DC1394_1_LIBRARY
    NAMES dc1394_control
    PATHS
    $ENV{DC1394_HOME}/lib
    /usr/lib
    )
#MESSAGE("DBG DC1394_1_LIBRARY=${DC1394_LIBRARY}")

  IF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
    SET(DC1394_FOUND TRUE)
    SET(DC1394_1_FOUND TRUE)
    SET(DC1394_VERSION 1)
    SET(DC1394_LIBRARIES ${DC1394_1_LIBRARY})
    SET(DC1394_INCLUDE_DIR ${DC1394_1_INCLUDE_DIR})
  ELSE(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
    SET(DC1394_FOUND FALSE)
    SET(DC1394_1_FOUND FALSE)
  ENDIF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
  
  # Search for libdc1394-2.x
  FIND_PATH(DC1394_2_INCLUDE_DIR dc1394/control.h
    $ENV{DC1394_HOME}/include
    /usr/include )
  #MESSAGE("DBG DC1394_2_INCLUDE_DIR=${DC1394_2_INCLUDE_DIR}")  

  FIND_LIBRARY(DC1394_2_LIBRARY
    NAMES dc1394
    PATHS 
    $ENV{DC1394_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG DC1394_2_LIBRARY=${DC1394_2_LIBRARY}")

  IF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)

    # Since the libdc1394-2.x API is not stable, try to compile a
    # sample code to determine if we have to use dc1394_find_cameras() or
    # dc1394_enumerate_cameras() functions. dc1394_enumerate_cameras() was 
    # introduced after libdc1394-2.0.0-rc7

    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${DC1394_2_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${DC1394_2_INCLUDE_DIR})
	
    CHECK_CXX_SOURCE_COMPILES("
      #include <dc1394/control.h>
      #include <dc1394/utils.h>

      int main(){
        dc1394_t * d;
        dc1394camera_list_t * list;
        d = dc1394_new ();
        dc1394_camera_enumerate (d, &list);
        return 0;
      }
      " DC1394_CAMERA_ENUMERATE_FOUND) 
    #MESSAGE("DC1394_CAMERA_ENUMERATE_FOUND: ${DC1394_CAMERA_ENUMERATE_FOUND}")

    IF(NOT DC1394_CAMERA_ENUMERATE_FOUND)
      # Try to test the compatibility to libdc1394-2.0.0-rc7
      CHECK_CXX_SOURCE_COMPILES("
        #include <dc1394/control.h>
        #include <dc1394/utils.h>

        int main(){
          dc1394camera_t **cameras;
          unsigned int num_cameras;
          dc1394_find_cameras(&cameras, &num_cameras);
          return 0;
        }
        " DC1394_FIND_CAMERAS_FOUND) 
      #MESSAGE("DC1394_FIND_CAMERAS_FOUND: ${DC1394_FIND_CAMERAS_FOUND}")
    ENDIF(NOT DC1394_CAMERA_ENUMERATE_FOUND)

    IF(NOT DC1394_CAMERA_ENUMERATE_FOUND AND NOT DC1394_FIND_CAMERAS_FOUND)
       SET(DC1394_2_FOUND FALSE)
       MESSAGE(SEND_ERROR "libdc1394-2.x found but not compatible with ViSP. Please contact ViSP authors...")
    ELSE(NOT DC1394_CAMERA_ENUMERATE_FOUND AND NOT DC1394_FIND_CAMERAS_FOUND)
       SET(DC1394_FOUND TRUE)
       SET(DC1394_2_FOUND TRUE)
    ENDIF(NOT DC1394_CAMERA_ENUMERATE_FOUND AND NOT DC1394_FIND_CAMERAS_FOUND)

    SET(DC1394_VERSION 2)
    SET(DC1394_LIBRARIES ${DC1394_2_LIBRARY})
    SET(DC1394_INCLUDE_DIR ${DC1394_2_INCLUDE_DIR})
  ENDIF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)
  


  ## --------------------------------

  MARK_AS_ADVANCED(
    DC1394_1_LIBRARY
    DC1394_1_INCLUDE_DIR
    DC1394_2_LIBRARY
    DC1394_2_INCLUDE_DIR
    DC1394_INCLUDE_DIR
    DC1394_LIBRARIES
    DC1394_LIBRARY
    )
ENDIF(NOT UNIX)
