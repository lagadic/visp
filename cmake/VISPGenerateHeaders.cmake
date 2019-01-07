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
# Authors:
# Fabien Spindler
#
#############################################################################

# ----------------------------------------------------------------------------
# platform-specific config file vpConfig.h
# ----------------------------------------------------------------------------

# case 1: when ViSP is build with make; files are used in <binary dir>
set(data_location_ ${VISP_BINARY_DIR})
set(VISP_SCENES_DIR ${data_location_}/data/wireframe-simulator)
set(VISP_ROBOT_ARMS_DIR ${data_location_}/data/robot-simulator)
# Note that VISP_HAVE_OGRE_RESOURCES_PATH and VISP_HAVE_OGRE_PLUGINS_PATH are set in OgreTools.cmake
set(data_location_ "${CMAKE_INSTALL_PREFIX}/${VISP_INSTALL_DATAROOTDIR}")
list(APPEND VISP_SCENES_DIR ${data_location_}/data/wireframe-simulator)
list(APPEND VISP_ROBOT_ARMS_DIR ${data_location_}/data/robot-simulator)
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  list(APPEND VISP_HAVE_OGRE_RESOURCES_PATH "${data_location_}/data/ogre-simulator")
endif()
# append to VISP_HAVE_OGRE_PLUGINS_PATH the path of the installed plugins.cfg file
# to be able to use ViSP from install tree
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  list(APPEND VISP_HAVE_OGRE_PLUGINS_PATH "${CMAKE_INSTALL_PREFIX}/${VISP_LIB_INSTALL_PATH}/visp/data/ogre-simulator")
endif()

configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpConfig.h.in" "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h")

# case 2: when ViSP is build with make install; files are used in <install dir>
if(UNIX)
  set(data_location_ "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}")
else()
  set(data_location_ "${CMAKE_INSTALL_PREFIX}")
endif()
set(VISP_SCENES_DIR ${data_location_}/data/wireframe-simulator)
set(VISP_ROBOT_ARMS_DIR ${data_location_}/data/robot-simulator)
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  set(VISP_HAVE_OGRE_RESOURCES_PATH "${data_location_}/data/ogre-simulator")
endif()
# append to VISP_HAVE_OGRE_PLUGINS_PATH the path of the installed plugins.cfg file
# to be able to use ViSP from install tree
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  set(VISP_HAVE_OGRE_PLUGINS_PATH "${CMAKE_INSTALL_PREFIX}/${VISP_LIB_INSTALL_PATH}/visp/data/ogre-simulator")
endif()

if(UNIX)
  configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpConfig.h.in" "${VISP_BINARY_DIR}/unix-install/vpConfig.h")

  install(FILES "${VISP_BINARY_DIR}/unix-install/vpConfig.h"
    DESTINATION ${VISP_INC_INSTALL_PATH}/visp3/core
    COMPONENT dev
  )

else()
  configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpConfig.h.in" "${VISP_BINARY_DIR}/win-install/vpConfig.h")

  install(FILES "${VISP_BINARY_DIR}/win-install/vpConfig.h"
    DESTINATION ${VISP_INC_INSTALL_PATH}/visp3/core
    COMPONENT dev
  )
endif()

# ----------------------------------------------------------------------------
# visp_modules.h that contains all the build modules defines
# ----------------------------------------------------------------------------
set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "#ifndef _visp_modules_h_\n#define _visp_modules_h_\n\n")

set(VISP_MOD_LIST ${VISP_MODULES_PUBLIC})
vp_list_sort(VISP_MOD_LIST)
foreach(m ${VISP_MOD_LIST})
  if(m MATCHES "^visp_")
    string(REGEX REPLACE "^visp_" "" m "${m}")
  endif()
  string(TOUPPER "${m}" m)
  set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}#define VISP_HAVE_MODULE_${m}\n")
endforeach()

set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}\n#endif\n")

configure_file("${VISP_SOURCE_DIR}/cmake/templates/visp_modules.h.in" "${VISP_INCLUDE_DIR}/visp3/visp_modules.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/visp_modules.h"
  DESTINATION ${VISP_INC_INSTALL_PATH}/visp3
  COMPONENT dev
)

# ----------------------------------------------------------------------------
# visp.h that includes all build modules headers
# Note: for other meta headers visp3/visp_<module>.h see vp_create_global_module_header() macro
# ----------------------------------------------------------------------------
set(VISP_ALL_HEADERS_CONFIGMAKE "#ifndef _visp_h_\n#define _visp_h_\n\n")

set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}// File that defines which modules where included during ViSP build\n")
set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}// It contains the defines of the correct VISP_HAVE_MODULE_<modulename> values\n")
set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}#include <visp3/visp_modules.h>\n\n")
set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}// Then the list of defines is checked to include the correct headers\n")
set(VISP_MOD_LIST ${VISP_MODULES_BUILD} ${VISP_MODULES_DISABLED_USER} ${VISP_MODULES_DISABLED_AUTO} ${VISP_MODULES_DISABLED_FORCE})
vp_list_sort(VISP_MOD_LIST)
foreach(m ${VISP_MOD_LIST})
  if(m MATCHES "^visp_")
    string(REGEX REPLACE "^visp_" "" m "${m}")
  endif()
  string(TOUPPER "${m}" M)
  if(m MATCHES "core")
    set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}// Core library is always included; without no ViSP functionality available\n")
    set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}#include <visp3/visp_${m}.h>\n\n")
  else()
    set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}#ifdef VISP_HAVE_MODULE_${M}\n#  include <visp3/visp_${m}.h>\n#endif\n\n")
  endif()
endforeach()

set(VISP_ALL_HEADERS_CONFIGMAKE "${VISP_ALL_HEADERS_CONFIGMAKE}#endif\n")

configure_file("${VISP_SOURCE_DIR}/cmake/templates/visp.h.in" "${VISP_INCLUDE_DIR}/visp3/visp.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/visp.h"
  DESTINATION ${VISP_INC_INSTALL_PATH}/visp3
  COMPONENT dev
)

# ----------------------------------------------------------------------------
#  install old headers to keep compat with dev based on ViSP 2.x releases
# ----------------------------------------------------------------------------
file(GLOB old_hdrs "${VISP_INCLUDE_DIR}/visp/*.h")
install(FILES ${old_hdrs}
  DESTINATION ${VISP_INC_INSTALL_PATH}/visp
  COMPONENT dev
)
