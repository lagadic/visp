#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
# ViSP packaging configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################


# To install the generated debian package use the gdebi-gtk package 
# installer on Ubuntu:
# gdebi-gtk libvisp-2.6.1-dev.deb
#
# Since there is no opencv package compatible with ViSP for the moment
# we desactivate the opencv usage and we activate the soqt usage since 
# there exist a soqt package
# cmake -DUSE_OPENCV=OFF -DUSE_SOQT=ON

# $ dpkg --print-architecture
FIND_PROGRAM(DPKG_CMD dpkg)
MARK_AS_ADVANCED(DPKG_CMD)
IF(NOT DPKG_CMD)
  MESSAGE(STATUS "Can not find dpkg in your path, default to i386.")
  SET(CPACK_SYSTEM_NAME i386)
ELSE()
  EXECUTE_PROCESS(COMMAND "${DPKG_CMD}" --print-architecture
    OUTPUT_VARIABLE CPACK_SYSTEM_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
ENDIF()

set(CMAKE_INSTALL_PREFIX "/usr" CACHE String "Debian package install prefix" FORCE)

set(BUILD_SHARED_LIBS CACHE FORCE "Build ViSP with shared libraries." ON)
list(APPEND CPACK_GENERATOR DEB)

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${VISP_VERSION}-${VISP_REVISION}_${CPACK_SYSTEM_NAME}")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "cmake (>=2.6)")
IF(USE_X11)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libx11-dev (>=2:1.3.2)")
ENDIF()
IF(USE_LAPACK)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, liblapack-dev(>=3.2)")
ENDIF()
IF(USE_GSL)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libgsl0-dev (>=1.13)")
ENDIF()
IF(USE_V4L2)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libv4l-dev (>=0.6.4)")
ENDIF()
IF(USE_DC1394_2)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libdc1394-22-dev (>=2.1.2)")
ENDIF()
IF(USE_XML2)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libxml2-dev (>=2.7.2)")
ENDIF()
IF(USE_LIBPNG)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libpng12-dev (>=1.2.42)")
ENDIF()
IF(USE_LIBJPEG)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libjpeg62-dev (>=6b-15)")
ENDIF()
IF(USE_FFMPEG)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libswscale-dev (>=4:0.5.1), libavutil-dev (>=4:0.5.1), libavformat-dev (>=4:0.5.1), libavcodec-dev (>=4:0.5.1), libbz2-dev (>=1.0.5-4), libbz2-1.0 (>=1.0.5-4)")
ENDIF()
IF(USE_COIN)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libcoin60-dev (>=3.1.2-1)")
ENDIF()
IF(USE_SOQT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libsoqt4-dev (>=1.4.2~svn20090224-2)")
ENDIF()
IF(USE_OGRE)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libogre-dev (>=1.7.3), ogre-samples-media (>=1.7.3)")
ENDIF()
IF(USE_OIS)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libois-dev (>=1.3.0)")
ENDIF()

IF(USE_OPENCV)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libcv-dev (>=2.1), libcvaux-dev (>=2.1), libhighgui-dev (>=2.1)")
ENDIF()
#message("CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS}")

#SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "${CPACK_PACKAGE_DESCRIPTION_SUMMARY}\r\rViSP stands for Visual Servoing Platform. ViSP is a complete cross-platform\rlibrary that allows prototyping and developing applications in visual tracking\rand visual servoing. \r\rThis package contains headers and library necessary for developing software\rthat uses ViSP. \r\rViSP web site address is http://www.irisa.fr/lagadic/visp/visp.html")

SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Visual tracking and visual servoing library written in C++ (development files).\r\n ViSP stands for Visual Servoing Platform. ViSP is a complete cross-platform library that allows prototyping and developing applications in visual tracking and visual servoing. This package contains headers and library necessary for developing software that uses ViSP. ViSP web site address is http://www.irisa.fr/lagadic/visp/visp.html")