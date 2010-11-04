#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find FFMPEG. FFMpeg depend son Zlib.
# Once run this will define: 
#
# FFMPEG_FOUND - system has FFMPEG
# FFMPEG_INCLUDE_DIRS - the FFMPEG include directory
# FFMPEG_LIBRARIES - Link these to use FFMPEG
# FFMPEG_WITH_DECODE_VIDEO2_FOUND - true if avcodec_decode_video2() is available
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

  FIND_PATH(FFMPEG_INCLUDE_DIR
    NAMES
      libavcodec/avcodec.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      ffmpeg
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
  FIND_LIBRARY(FFMPEG_AVUTIL_LIBRARY
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
  FIND_LIBRARY(FFMPEG_AVCODEC_LIBRARY
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

  FIND_LIBRARY(FFMPEG_AVFORMAT_LIBRARY
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

  FIND_LIBRARY(FFMPEG_SWSCALE_LIBRARY
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

  # FFMpeg depend son Zlib
  FIND_PACKAGE(ZLIB)
  # FFMpeg depend son BZip2
  FIND_PACKAGE(BZip2)

  # Try to find if avcodec_decode_video2() is avalaible since 
  # avcodec_decode_video() is deprecated. To do that we try to compile 
  # a sample code
  IF(FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_FOUND)
  INCLUDE (CheckCXXSourceCompiles)

    #MESSAGE("zlib ${ZLIB_LIBRARY}")
    SET(CMAKE_REQUIRED_LIBRARIES ${FFMPEG_AVCODEC_LIBRARY} ${FFMPEG_AVUTIL_LIBRARY} ${ZLIB_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${FFMPEG_INCLUDE_DIR} ${FFMPEG_INCLUDE_DIR_AVCODEC})
    SET(CMAKE_REQUIRED_DEFINITIONS "-D__STDC_CONSTANT_MACROS")
    CHECK_CXX_SOURCE_COMPILES("
  #define __STDC_CONSTANT_MACROS
  extern \"C\"
  {
  #include <avcodec.h>
  }

  int main(){
  AVCodecContext *avctx;
  AVFrame *picture;
  int *got_picture_ptr;
  AVPacket *avpkt;
  avcodec_decode_video2(avctx, picture, got_picture_ptr, avpkt);
  return 0;
  }" FFMPEG_WITH_DECODE_VIDEO2_FOUND) 

  #MESSAGE("FFMPEG_WITH_DECODE_VIDEO2_FOUND: ${FFMPEG_WITH_DECODE_VIDEO2_FOUND}")

  ELSE(FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_FOUND)
    SET(FFMPEG_WITH_DECODE_VIDEO2_FOUND FALSE)
  ENDIF(FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_FOUND)


IF(FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_INCLUDE_DIR_AVFORMAT AND FFMPEG_INCLUDE_DIR_AVUTIL AND FFMPEG_INCLUDE_DIR_SWSCALE AND FFMPEG_SWSCALE_LIBRARY AND FFMPEG_AVFORMAT_LIBRARY AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_LIBRARY AND BZIP2_LIBRARIES)
  SET(FFMPEG_FOUND TRUE)
  SET(FFMPEG_INCLUDE_DIRS
    ${FFMPEG_INCLUDE_DIR}
    ${FFMPEG_INCLUDE_DIR_AVCODEC}
    ${FFMPEG_INCLUDE_DIR_AVFORMAT}
    ${FFMPEG_INCLUDE_DIR_AVUTIL}
    ${FFMPEG_INCLUDE_DIR_SWSCALE}
  )
  SET(FFMPEG_LIBRARIES
    ${FFMPEG_SWSCALE_LIBRARY}
    ${FFMPEG_AVFORMAT_LIBRARY}
    ${FFMPEG_AVCODEC_LIBRARY}
    ${FFMPEG_AVUTIL_LIBRARY}
    ${ZLIB_LIBRARY}
    ${BZIP2_LIBRARIES}
  )
ELSE(FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_INCLUDE_DIR_AVFORMAT AND FFMPEG_INCLUDE_DIR_AVUTIL AND FFMPEG_INCLUDE_DIR_SWSCALE AND FFMPEG_SWSCALE_LIBRARY AND FFMPEG_AVFORMAT_LIBRARY AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_LIBRARY AND BZIP2_LIBRARIES)
  SET(FFMPEG_FOUND FALSE)
ENDIF (FFMPEG_INCLUDE_DIR AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_INCLUDE_DIR_AVFORMAT AND FFMPEG_INCLUDE_DIR_AVUTIL AND FFMPEG_INCLUDE_DIR_SWSCALE AND FFMPEG_SWSCALE_LIBRARY AND FFMPEG_AVFORMAT_LIBRARY AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_LIBRARY AND BZIP2_LIBRARIES)

MARK_AS_ADVANCED(
  FFMPEG_INCLUDE_DIR
  FFMPEG_INCLUDE_DIR_AVCODEC
  FFMPEG_INCLUDE_DIR_AVFORMAT
  FFMPEG_INCLUDE_DIR_AVUTIL
  FFMPEG_INCLUDE_DIR_SWSCALE
  FFMPEG_AVUTIL_LIBRARY
  FFMPEG_AVFORMAT_LIBRARY
  FFMPEG_AVCODEC_LIBRARY
  FFMPEG_SWSCALE_LIBRARY
  FFMPEG_INCLUDE_DIRS
  FFMPEG_LIBRARIES
)

