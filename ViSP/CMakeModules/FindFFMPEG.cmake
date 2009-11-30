#############################################################################
#
# $Id: FindLIBJPEG.cmake 2179 2009-06-09 16:33:50Z fspindle $
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
# This file is part of the ViSP toolkit.
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
# Try to find FFMPEG.
# Once run this will define: 
#
# FFMPEG_FOUND - system has FFMPEG
# FFMPEG_INCLUDE_DIRS - the FFMPEG include directory
# FFMPEG_LIBRARIES - Link these to use FFMPEG
#
# Authors:
# Fabien Spindler
#
#############################################################################

# detection of the FFMPEG headers location
  FIND_PATH(FFMPEG_INCLUDE_DIR_AVCODEC
    NAMES
      avcodec.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      ffmpeg
      libavcodec
      ffmpeg/libavcodec
  )

  FIND_PATH(FFMPEG_INCLUDE_DIR_AVFORMAT
    NAMES
      avformat.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      ffmpeg
      libavformat
      ffmpeg/libavformat
      )

  FIND_PATH(FFMPEG_INCLUDE_DIR_AVUTIL
    NAMES
      avutil.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      libavutil
      ffmpeg
      ffmpeg/libavutil
  )

  FIND_PATH(FFMPEG_INCLUDE_DIR_SWSCALE
    NAMES
      swscale.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      libswscale
      ffmpeg
      ffmpeg/libswscale
  )

# Detection of the FFMPEG library on Unix
  FIND_LIBRARY(AVUTIL_LIBRARY
    NAMES
      avutil
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{FFMPEG_DIR}/lib
    $ENV{FFMPEG_DIR}/Release
    $ENV{FFMPEG_DIR}
  )
  FIND_LIBRARY(AVCODEC_LIBRARY
    NAMES
      avcodec
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{FFMPEG_DIR}/lib
    $ENV{FFMPEG_DIR}/Release
    $ENV{FFMPEG_DIR}
  )

  FIND_LIBRARY(AVFORMAT_LIBRARY
    NAMES
      avformat
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{FFMPEG_DIR}/lib
    $ENV{FFMPEG_DIR}/Release
    $ENV{FFMPEG_DIR}
  )

  FIND_LIBRARY(SWSCALE_LIBRARY
    NAMES
      swscale
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{FFMPEG_DIR}/lib
    $ENV{FFMPEG_DIR}/Release
    $ENV{FFMPEG_DIR}
  )

  set(FFMPEG_INCLUDE_DIRS
    ${FFMPEG_INCLUDE_DIR_AVCODEC}
    ${FFMPEG_INCLUDE_DIR_AVFORMAT}
    ${FFMPEG_INCLUDE_DIR_AVUTIL}
    ${FFMPEG_INCLUDE_DIR_SWSCALE}
    ${FFMPEG_INCLUDE}
  )
  set(FFMPEG_LIBRARIES
    ${SWSCALE_LIBRARY}
    ${AVFORMAT_LIBRARY}
    ${AVCODEC_LIBRARY}
    ${AVUTIL_LIBRARY}
#   ${VORBISENC}
#   ${VORBIS}
#    ${OGG}
#    ${THEORA}
)

MARK_AS_ADVANCED(
  FFMPEG_INCLUDE_DIR_AVCODEC
  FFMPEG_INCLUDE_DIR_AVFORMAT
  FFMPEG_INCLUDE_DIR_AVUTIL
  FFMPEG_INCLUDE_DIR_SWSCALE
  FFMPEG_INCLUDE
  AVUTIL_LIBRARY
  AVFORMAT_LIBRARY
  AVCODEC_LIBRARY
  SWSCALE_LIBRARY
  AVUTIL_LIBRARY
  FFMPEG_INCLUDE_DIRS
  FFMPEG_LIBRARIES
)
## --------------------------------

  IF(FFMPEG_INCLUDE_DIRS AND FFMPEG_LIBRARIES)
     SET(FFMPEG_FOUND TRUE)

  ELSE(FFMPEG_INCLUDE_DIRS AND FFMPEG_LIBRARIES)
    SET(FFMPEG_FOUND FALSE)
  ENDIF (FFMPEG_INCLUDE_DIRS AND FFMPEG_LIBRARIES)

#   IF(FFMPEG_FOUND)
#     IF(NOT FFMPEG_FIND_QUIETLY)
#       MESSAGE(STATUS "Found FFMPEG: ${FFMPEG_LIBRARIES}")
#     ENDIF(NOT FFMPEG_FIND_QUIETLY)
#   ELSE(FFMPEG_FOUND)
#     IF(FFMPEG_FIND_REQUIRED)
#       MESSAGE(FATAL_ERROR "Could not find FFMPEG")
#     endif (FFMPEG_FIND_REQUIRED)
#   endif (FFMPEG_FOUND)

  # show the FFMPEG_INCLUDE_DIRS and FFMPEG_LIBRARIES variables only in the advanced view
#   mark_as_advanced(FFMPEG_INCLUDE_DIRS FFMPEG_LIBRARIES)

# endif (FFMPEG_LIBRARIES AND FFMPEG_INCLUDE_DIRS)
