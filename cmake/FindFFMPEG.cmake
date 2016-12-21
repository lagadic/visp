#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
# Try to find FFMPEG. FFMpeg depend son Zlib.
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
if(MINGW)
  find_path(FFMPEG_INCLUDE_DIR_AVCODEC
    NAMES
      libavcodec/avcodec.h
    PATHS
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    PATH_SUFFIXES
      ffmpeg
  )

  find_path(FFMPEG_INCLUDE_DIR_AVFORMAT
    NAMES
      libavformat/avformat.h
    PATHS
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    PATH_SUFFIXES
      ffmpeg
  )

  find_path(FFMPEG_INCLUDE_DIR_AVUTIL
    NAMES
      libavutil/avutil.h
    PATHS
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    PATH_SUFFIXES
      ffmpeg
  )

  find_path(FFMPEG_INCLUDE_DIR_SWSCALE
    NAMES
      libswscale/swscale.h
    PATHS
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    PATH_SUFFIXES
      libswscale
      ffmpeg
  )

  # Detection of the FFMPEG library on Unix
  find_library(FFMPEG_AVUTIL_LIBRARY
    NAMES
      avutil
    PATHS
   "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )
  find_library(FFMPEG_AVCODEC_LIBRARY
    NAMES
      avcodec
    PATHS
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )
  find_library(FFMPEG_AVFORMAT_LIBRARY
    NAMES
      avformat
    PATHS
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )

  find_library(FFMPEG_AVCORE_LIBRARY
    NAMES
      avcore
    PATHS
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )

  find_library(FFMPEG_SWSCALE_LIBRARY
    NAMES
      swscale
    PATHS
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )
else(MINGW)
  find_path(FFMPEG_INCLUDE_DIR_AVCODEC
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

  find_path(FFMPEG_INCLUDE_DIR_AVFORMAT
    NAMES
      libavformat/avformat.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      ffmpeg
  )

  find_path(FFMPEG_INCLUDE_DIR_AVUTIL
    NAMES
      libavutil/avutil.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      ffmpeg
  )

  find_path(FFMPEG_INCLUDE_DIR_SWSCALE
    NAMES
      libswscale/swscale.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{FFMPEG_DIR}/include
    $ENV{FFMPEG_DIR}
    PATH_SUFFIXES
      libswscale
      ffmpeg
  )

  # Detection of the FFMPEG library on Unix
  find_library(FFMPEG_AVUTIL_LIBRARY
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
  find_library(FFMPEG_AVCODEC_LIBRARY
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
  find_library(FFMPEG_AVFORMAT_LIBRARY
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

  find_library(FFMPEG_AVCORE_LIBRARY
    NAMES
      avcore
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{FFMPEG_DIR}/lib
    $ENV{FFMPEG_DIR}/Release
    $ENV{FFMPEG_DIR}
  )

  find_library(FFMPEG_SWSCALE_LIBRARY
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
endif(MINGW)

  # FFMpeg depend son Zlib
  find_package(ZLIB)
  if(NOT ZLIB_FOUND)
    find_package(MyZLIB)
  endif()

  # FFMpeg depend son BZip2
  find_package(BZip2 QUIET)

  # FFMpeg may depend also on iconv since probably version 1.1.3 where if detected,
  # iconv usage is enabled by default
  find_package(ICONV QUIET)
  #message("ICONV_FOUND: ${ICONV_FOUND}")

IF(FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_INCLUDE_DIR_AVFORMAT AND FFMPEG_INCLUDE_DIR_AVUTIL AND FFMPEG_INCLUDE_DIR_SWSCALE AND FFMPEG_SWSCALE_LIBRARY AND FFMPEG_AVFORMAT_LIBRARY AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_LIBRARIES AND BZIP2_LIBRARIES)
  SET(FFMPEG_FOUND TRUE)
  SET(FFMPEG_INCLUDE_DIRS
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
  )
  if(FFMPEG_AVCORE_LIBRARY)
    LIST(APPEND FFMPEG_LIBRARIES ${FFMPEG_AVCORE_LIBRARY})
  endif()
  list(APPEND FFMPEG_LIBRARIES ${ZLIB_LIBRARIES} ${BZIP2_LIBRARIES})
  if(ICONV_FOUND)
    list(APPEND FFMPEG_LIBRARIES ${ICONV_LIBRARIES})
  endif()

elseif(MINGW AND FFMPEG_INCLUDE_DIR_AVCODEC AND FFMPEG_INCLUDE_DIR_AVFORMAT AND FFMPEG_INCLUDE_DIR_AVUTIL AND FFMPEG_INCLUDE_DIR_SWSCALE AND FFMPEG_SWSCALE_LIBRARY AND FFMPEG_AVFORMAT_LIBRARY AND FFMPEG_AVCODEC_LIBRARY AND FFMPEG_AVUTIL_LIBRARY AND ZLIB_LIBRARIES)
  # Bzip2 is nor requested with mingw-w64
  SET(FFMPEG_FOUND TRUE)
  SET(FFMPEG_INCLUDE_DIRS
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
  )
  if(FFMPEG_AVCORE_LIBRARY)
    LIST(APPEND FFMPEG_LIBRARIES ${FFMPEG_AVCORE_LIBRARY})
  endif()
  list(APPEND FFMPEG_LIBRARIES ${ZLIB_LIBRARIES})
  if(ICONV_FOUND)
    list(APPEND FFMPEG_LIBRARIES ${ICONV_LIBRARIES})
  endif()

ELSE()
  SET(FFMPEG_FOUND FALSE)
ENDIF ()

MARK_AS_ADVANCED(
  BZIP2_DIR
  FFMPEG_INCLUDE_DIR_AVCODEC
  FFMPEG_INCLUDE_DIR_AVFORMAT
  FFMPEG_INCLUDE_DIR_AVUTIL
  FFMPEG_INCLUDE_DIR_SWSCALE
  FFMPEG_AVUTIL_LIBRARY
  FFMPEG_AVFORMAT_LIBRARY
  FFMPEG_AVCODEC_LIBRARY
  FFMPEG_SWSCALE_LIBRARY
  FFMPEG_AVCORE_LIBRARY
  FFMPEG_INCLUDE_DIRS
  FFMPEG_LIBRARIES
)

