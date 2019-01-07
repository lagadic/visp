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
# ViSP packaging configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################


# To install the generated debian package use the gdebi-gtk package 
# installer on Ubuntu:
# gdebi-gtk libvisp-dev-2.6.1-1_i386.deb

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

# $ lsb_release -sr (ex: 12.04)
FIND_PROGRAM(LSB_RELEASE_CMD lsb_release)
MARK_AS_ADVANCED(LSB_RELEASE_CMD)
IF(NOT LSB_RELEASE_CMD)
  MESSAGE(STATUS "Can not find lsb_release in your path.")
  SET(DEBIAN_DISTRO_RELEASE "")
ELSE()
  EXECUTE_PROCESS(COMMAND "${LSB_RELEASE_CMD}" -sr
    OUTPUT_VARIABLE DEBIAN_DISTRO_RELEASE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  if(DEBIAN_DISTRO_RELEASE)
    #message("DEBIAN_DISTRO_RELEASE 1: ${DEBIAN_DISTRO_RELEASE}")
    STRING(REGEX REPLACE "[.]" "" DEBIAN_DISTRO_RELEASE ${DEBIAN_DISTRO_RELEASE})
    #message("DEBIAN_DISTRO_RELEASE 2: ${DEBIAN_DISTRO_RELEASE}")
  endif()
ENDIF()


list(APPEND CPACK_GENERATOR DEB)

SET(CPACK_PACKAGE_VERSION ${VISP_VERSION}-${VISP_REVISION})
SET(CPACK_PACKAGE_NAME "libvisp-dev")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${VISP_VERSION}-${VISP_REVISION}_${CPACK_SYSTEM_NAME}")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "cmake (>=2.6)")
IF(USE_X11)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libx11-dev (>=2:1.3.2)")
ENDIF()
IF(USE_GTK2)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libgtk2.0-dev (>=2.20.1), libglib2.0-dev (>=2.24.1), libpango1.0-dev (>=1.28.0), libatk1.0-dev (>=1.30.0), libcairo2-dev (>=1.8.10)")
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
  if(DEBIAN_DISTRO_RELEASE)
    if(DEBIAN_DISTRO_RELEASE VERSION_LESS "1204")
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libjpeg62-dev (>=6b-15)")
    else()
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libjpeg-dev (>=6b-15)")
    endif()
  else()
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libjpeg-dev (>=6b-15)")
  endif()
ENDIF()
IF(USE_COIN)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libcoin60-dev (>=3.1.2-1)")
ENDIF()
IF(USE_SOQT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libsoqt4-dev (>=1.4.2~svn20090224-2)")
ENDIF()
IF(USE_OGRE)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libogre-dev (>=1.7.3)")
  # since ogre-samples-media is not available on ubuntu oneiric 11.10 for example, we need an extra test to add this dependency
  IF(OGRE_MEDIA_DIR)
    if(DEBIAN_DISTRO_RELEASE)
      if(DEBIAN_DISTRO_RELEASE VERSION_LESS "1204")
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ogre-samples-media (>=1.7.3)")
      else()
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ogre-samples (>=1.7.3)")
      endif()
    else()
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ogre-samples-media (>=1.7.3)")
    endif()
  ENDIF()
ENDIF()
IF(USE_OIS)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libois-dev (>=1.3.0)")
ENDIF()
IF(USE_OPENCV)
  if(DEBIAN_DISTRO_RELEASE)
    if(DEBIAN_DISTRO_RELEASE VERSION_LESS "1204")
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libcv-dev (>=2.0), libcvaux-dev (>=2.0), libhighgui-dev (>=2.0)")
    else()
      set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libopencv-dev (>=2.3.1)")
    endif()
  else()
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libopencv-dev (>=2.3.1)")
  endif()
ENDIF()
IF(USE_LIBFREENECT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, freenect (>=1:0.0.1)")
ENDIF()
if(USE_LIBUSB_1)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libusb-1.0-0-dev (>=2:1.0)")
endif()

set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libc6 (>=2.4)")

SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Visual Servoing Platform development files
 ViSP, standing for Visual Servoing Platform, is unique. This software
 is a complete cross-platform solution that allows prototyping and
 developing applications in visual tracking and visual servoing.
 .
 ViSP can be useful in robotics, computer vision, augmented reality and
 computer animation.
 .
 This package contains development files (headers and shared library
 symbolic link).")

SET(CPACK_DEBIAN_PACKAGE_SECTION "libdevel")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.irisa.fr/lagadic/visp")

