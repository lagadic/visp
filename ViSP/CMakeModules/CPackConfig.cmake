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
# ViSP packaging configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################


message(STATUS "Configuring CPack")
mark_as_advanced(BUILD_PACKAGE)

  
SET(CPACK_PACKAGE_NAME "libvisp")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Visual tracking and visual servoing library in C++ (development files)")
SET(CPACK_PACKAGE_CONTACT "visp@inria.fr")
SET(CPACK_PACKAGE_VENDOR "Inria, French National Institute for Research in Computer Science and Control")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")

SET(CPACK_PACKAGE_VERSION ${VISP_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${VISP_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${VISP_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${VISP_VERSION_PATCH})


# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
SET(CPACK_SOURCE_IGNORE_FILES
  "/\\\\.svn/"
  "/autom4te.cache/"
  "/build/"
  "/lib/"
  "~$"
)

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME} ${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}")

if(UNIX AND NOT APPLE AND NOT WIN32) # =linux
  option(BUILD_PACKAGE_DEBIAN "Build debian package" ON)
  option(BUILD_PACKAGE_RPM "Build rpm package" ON)
endif()

if(WIN32 AND NOT UNIX)
  include(${VISP_SOURCE_DIR}/CMakeModules/CPackConfigNsis.cmake)
elseif(APPLE)
  set(CPACK_GENERATOR "PackageMaker;TBZ2")
elseif(UNIX)
  if(BUILD_PACKAGE_DEBIAN)
    include(${VISP_SOURCE_DIR}/CMakeModules/CPackConfigDeb.cmake)
  elseif(BUILD_PACKAGE_RPM)
    include(${VISP_SOURCE_DIR}/CMakeModules/CPackConfigRpm.cmake)
  endif()
endif()

INCLUDE(CPack)
