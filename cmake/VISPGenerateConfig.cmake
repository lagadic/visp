#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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
# This file generates the ViSPConfig.cmake file: 
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use with "make install"
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Macro that returns the relative path to go from a child folder to the parent folder
# input: path_to_child
# output: path_to_parent, the relative path to go from path_to_child to parent
# example: if input =lib/x86_64-linux-gnu, then output=../..
macro(get_path_to_parent path_to_child path_to_parent)
  set(${path_to_parent} "")
  set(input_ "${path_to_child}")
  while(input_)
    if(input_)
      set(${path_to_parent} "${${path_to_parent}}../")
    endif()
    get_filename_component(input_ "${input_}" PATH)
  endwhile(input_)
endmacro()

# Here we determine the relative path from ./${CMAKE_INSTALL_LIBDIR} to its parent folder
# if CMAKE_INSTALL_LIBDIR=lib, then VISP_INSTALL_LIBDIR_TO_PARENT=../
# if CMAKE_INSTALL_LIBDIR=lib/x86_64-linux-gnu, then VISP_INSTALL_LIBDIR_TO_PARENT=../..
get_path_to_parent(${CMAKE_INSTALL_LIBDIR} VISP_INSTALL_LIBDIR_TO_PARENT)

#build list of modules available for the ViSP user
set(VISP_LIB_COMPONENTS "")
foreach(m ${VISP_MODULES_PUBLIC})
  list(INSERT VISP_LIB_COMPONENTS 0 ${${m}_MODULE_DEPS_OPT} ${m})
endforeach()
vp_list_unique(VISP_LIB_COMPONENTS)
#message("VISP_LIB_COMPONENTS: ${VISP_LIB_COMPONENTS}")
set(VISP_MODULES_CONFIGCMAKE ${VISP_LIB_COMPONENTS})
vp_list_filterout(VISP_LIB_COMPONENTS "^visp_")
if(VISP_LIB_COMPONENTS)
  list(REMOVE_ITEM VISP_MODULES_CONFIGCMAKE ${VISP_LIB_COMPONENTS})
endif()


# -------------------------------------------------------------------------------------------
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
# -------------------------------------------------------------------------------------------

# Export the library
export(TARGETS ${VISPModules_TARGETS} FILE "${PROJECT_BINARY_DIR}/VISPModules.cmake")

## Update include dirs
set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR}")
foreach(m ${VISP_MODULES_BUILD})
  if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
    list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_MODULE_${m}_LOCATION}/include")
  endif()
  list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE ${VISP_MODULE_${m}_INC_DEPS})
endforeach()
vp_list_unique(VISP_INCLUDE_DIRS_CONFIGCMAKE)

configure_file(
  cmake/templates/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/VISPConfig.cmake
  IMMEDIATE @ONLY
)

configure_file(
  cmake/templates/VISPConfigVersion.cmake.in
  ${VISP_BINARY_DIR}/VISPConfigVersion.cmake
  IMMEDIATE @ONLY
)

configure_file(
  cmake/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/VISPUse.cmake
  IMMEDIATE @ONLY
)

# --------------------------------------------------------------------------------------------
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use *with* "make install"
# -------------------------------------------------------------------------------------------

if(UNIX)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\${VISP_INSTALL_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE ${VISP_MODULE_${m}_INC_DEPS})
  endforeach()
  vp_list_unique(VISP_INCLUDE_DIRS_CONFIGCMAKE)

  configure_file(
    cmake/templates/VISPConfig.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPConfig.cmake
    IMMEDIATE @ONLY
  )

  configure_file(
    cmake/templates/VISPConfigVersion.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPConfigVersion.cmake
    IMMEDIATE @ONLY
  )

  configure_file(
    cmake/VISPUse.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPUse.cmake
    IMMEDIATE @ONLY
  )

  install(FILES
    ${VISP_BINARY_DIR}/unix-install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/unix-install/VISPConfigVersion.cmake
    ${VISP_BINARY_DIR}/unix-install/VISPUse.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/visp"
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT dev
  )

  # Install the export set for use with the install-tree
  install(EXPORT VISPModules
    FILE VISPModules.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/visp"
    COMPONENT dev
  )
endif()

# --------------------------------------------------------------------------------------------
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
# --------------------------------------------------------------------------------------------
if(WIN32)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\${VISP_CONFIG_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE ${VISP_MODULE_${m}_INC_DEPS})
  endforeach()
  vp_list_unique(VISP_INCLUDE_DIRS_CONFIGCMAKE)

  configure_file(
    cmake/templates/VISPConfig.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPConfig.cmake
    IMMEDIATE @ONLY
  )

  configure_file(
    cmake/templates/VISPConfigVersion.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPConfigVersion.cmake
    IMMEDIATE @ONLY
  )

  configure_file(
    cmake/VISPUse.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPUse.cmake
    IMMEDIATE @ONLY
  )

  if(BUILD_SHARED_LIBS)
    install(FILES
      "${CMAKE_BINARY_DIR}/win-install/ViSPConfig.cmake"
      "${CMAKE_BINARY_DIR}/win-install/ViSPUse.cmake"
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}"
      COMPONENT dev)
    install(EXPORT VISPModules 
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}"
      FILE VISPModules.cmake 
      COMPONENT dev)
  else()
    install(FILES
      "${CMAKE_BINARY_DIR}/win-install/ViSPConfig.cmake"
      "${CMAKE_BINARY_DIR}/win-install/ViSPUse.cmake"
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}"
      COMPONENT dev)
    install(EXPORT VISPModules 
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}"
      FILE VISPModules.cmake 
      COMPONENT dev)
  endif()

  install(FILES
    "cmake/VISPConfig.cmake"
    "${VISP_BINARY_DIR}/win-install/VISPConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT dev
  )
endif()
