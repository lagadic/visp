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


message(STATUS "Configuring CPack")
#mark_as_advanced(BUILD_PACKAGE)


SET(CPACK_PACKAGE_NAME "libvisp")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Visual tracking and visual servoing library in C++ (development files)")
SET(CPACK_PACKAGE_CONTACT "Fabien Spindler <Fabien.Spindler@inria.fr>")
SET(CPACK_PACKAGE_VENDOR "Inria, French National Institute for Research in Computer Science and Control")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")

SET(CPACK_PACKAGE_VERSION ${VISP_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${VISP_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${VISP_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${VISP_VERSION_PATCH})

SET(CPACK_COMPONENTS_ALL libraries headers)
SET(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
SET(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers")
SET(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "ViSP ${VISP_VERSION} libraries")
SET(CPACK_COMPONENT_HEADERS_DESCRIPTION "C/C++ header files for use with ViSP ${VISP_VERSION} libraries")
SET(CPACK_COMPONENT_HEADERS_DEPENDS libraries)
SET(CPACK_COMPONENT_LIBRARIES_GROUP "Development")
SET(CPACK_COMPONENT_HEADERS_GROUP "Development")	
SET(CPACK_COMPONENT_GROUP_DEVELOPMENT_DESCRIPTION "All of the tools you'll ever need to develop software with ViSP")
SET(CPACK_ALL_INSTALL_TYPES Full Developer)
SET(CPACK_COMPONENT_LIBRARIES_INSTALL_TYPES Developer Full)
SET(CPACK_COMPONENT_HEADERS_INSTALL_TYPES Developer Full)

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

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME} ${VISP_VERSION}")

# In ViSP packages we only want to have the libraries and the headers (nor the demo, example, and test)
# That's why we turn off the demo, example and test building
SET(BUILD_DEMOS OFF CACHE BOOL "Build ViSP demos." FORCE )
SET(BUILD_EXAMPLES OFF CACHE BOOL "Build ViSP examples." FORCE)
SET(BUILD_TESTING OFF CACHE BOOL "Build ViSP tests." FORCE)
SET(BUILD_TUTORIAL OFF CACHE BOOL "Build ViSP tutorials." FORCE)


if(UNIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE String "Package install prefix" FORCE)
elseif(WIN32)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/ViSP ${VISP_VERSION}" CACHE String "Package install prefix" FORCE)
endif()

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build ViSP with shared libraries." FORCE)


if(WIN32 AND NOT UNIX)
  include(${VISP_CMAKE_MODULE_PATH}/CPackConfigNsis.cmake)
elseif(APPLE)
  set(CPACK_GENERATOR "PackageMaker;TBZ2")
elseif(UNIX)
  if(BUILD_PACKAGE_DEBIAN)
    include(${VISP_CMAKE_MODULE_PATH}/CPackConfigDeb.cmake)
  elseif(BUILD_PACKAGE_RPM)
    include(${VISP_CMAKE_MODULE_PATH}/CPackConfigRpm.cmake)
  endif()
endif()

INCLUDE(CPack)
