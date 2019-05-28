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


# $ uname -i

FIND_PROGRAM(UNAME_CMD uname)
MARK_AS_ADVANCED(UNAME_CMD)
IF(NOT UNAME_CMD)
  MESSAGE(STATUS "Can not find uname in your path, default to i386.")
  SET(CPACK_SYSTEM_NAME i386)
ELSE()
  EXECUTE_PROCESS(COMMAND "${UNAME_CMD}" -m
    OUTPUT_VARIABLE CPACK_SYSTEM_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
ENDIF()

# create a fedora short name from cat /etc/fedora-release
# for example, cat .. return "Fedora release 13 (Goddard)" we build "fc13" 
FIND_PROGRAM(CAT_CMD cat)
MARK_AS_ADVANCED(CAT_CMD)
IF(NOT CAT_CMD)
  MESSAGE(STATUS "Can not find cat in your path, default to empty.")
  SET(DIST_SHORT_NAME "")
ELSE()
  SET(DIST_SHORT_NAME "")
  EXECUTE_PROCESS(COMMAND "${CAT_CMD}" /etc/fedora-release
    OUTPUT_VARIABLE FEDORA_FULL_RELEASE_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  #message("fedora release found 1: ${FEDORA_FULL_RELEASE_NAME}")
  IF(FEDORA_FULL_RELEASE_NAME)
    # replace " " space separator by ";" separator
    STRING(REGEX REPLACE "[ ]" ";" FEDORA_FULL_RELEASE_NAME ${FEDORA_FULL_RELEASE_NAME})
    # message("fedora release found 2: ${FEDORA_FULL_RELEASE_NAME}")
    LIST(GET FEDORA_FULL_RELEASE_NAME 0 ITEM1) # here ITEM1 should contain "Fedora"
    # message("item1: ${ITEM1}")
    STRING(COMPARE EQUAL ${ITEM1} "Fedora" IS_FEDORA)
    IF(IS_FEDORA)
      LIST(GET FEDORA_FULL_RELEASE_NAME 2 ITEM3) # here ITEM3 should contain the dist number
      # message("dist number: ${ITEM3}")
      IF(ITEM3)
        SET(DIST_SHORT_NAME "fc${ITEM3}")
      ENDIF()
    ENDIF()
  ENDIF()
ENDIF()


list(APPEND CPACK_GENERATOR RPM)

SET(CPACK_PACKAGE_NAME "libvisp-devel")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${VISP_VERSION}-${VISP_REVISION}.${DIST_SHORT_NAME}.${CPACK_SYSTEM_NAME}")
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
set(CPACK_RPM_PACKAGE_ARCHITECTURE "${CPACK_SYSTEM_NAME}")
set(CPACK_RPM_PACKAGE_LICENSE "GPL")
set(CPACK_RPM_PACKAGE_RELEASE ${VISP_REVISION})

set(CPACK_RPM_PACKAGE_DEPENDS "cmake >= 2.6")
IF(USE_X11)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libX11-devel >= 1.3")
ENDIF()
IF(USE_GTK2)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, gtk2-devel, glib2-devel, pango-devel, atk-devel, cairo-devel")
ENDIF()
IF(USE_LAPACK)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, lapack-devel >= 3.2")
ENDIF()
IF(USE_GSL)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, gsl-devel >= 1.13")
ENDIF()
IF(USE_V4L2)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libv4l-devel >= 0.6.2")
ENDIF()
IF(USE_DC1394_2)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libdc1394-devel >= 2.1.2")
ENDIF()
IF(USE_XML2)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libxml2-devel >= 2.7.6")
ENDIF()
IF(USE_LIBPNG)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libpng-devel >= 2:1.2.44")
ENDIF()
IF(USE_LIBJPEG)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, libjpeg-devel >= 6b-46")
ENDIF()
IF(USE_COIN)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, Coin2-devel >= 2.5.0")
ENDIF()
IF(USE_SOQT)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, SoQt-devel >= 1.5.0")
ENDIF()
IF(USE_OPENCV)
  set(CPACK_RPM_PACKAGE_DEPENDS "${CPACK_RPM_PACKAGE_DEPENDS}, opencv-devel >= 2.0")
ENDIF()
#message("CPACK_RPM_PACKAGE_DEPENDS ${CPACK_RPM_PACKAGE_DEPENDS}")

set(CPACK_RPM_PACKAGE_REQUIRES "${CPACK_RPM_PACKAGE_DEPENDS}")

set(CPACK_RPM_PACKAGE_DESCRIPTION "Visual Servoing Platform development files
 ViSP, standing for Visual Servoing Platform, is unique. This software
 is a complete cross-platform solution that allows prototyping and
 developing applications in visual tracking and visual servoing.
 .
 ViSP can be useful in robotics, computer vision, augmented reality and
 computer animation.
 .
 This package contains development files (headers and shared library
 symbolic link).")

set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/usr/share/man" "/usr/share/man/man1" "/usr/lib64/pkgconfig" "/usr/lib/pkgconfig" "/usr/lib64/cmake" "/usr/lib/cmake")
