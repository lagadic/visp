#############################################################################
#
# $Id: GenerateConfigt.cmake 4676 2014-02-17 22:08:37Z fspindle $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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

# -------------------------------------------------------------------------------------------
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
# -------------------------------------------------------------------------------------------

# Export the library
export(TARGETS ${VISP_INTERN_LIBRARY} FILE "${PROJECT_BINARY_DIR}/VISPTargets.cmake")

# Update include dirs
set(VISP_INCLUDE_DIRS_CONFIGCMAKE_ "${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR}")
list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE_ ${VISP_EXTERN_INCLUDE_DIRS})
set(VISP_INCLUDE_DIRS_CONFIGCMAKE "")
foreach(val ${VISP_INCLUDE_DIRS_CONFIGCMAKE_})
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_INCLUDE_DIRS_CONFIGCMAKE} \"${val}\"")
endforeach()
#message("VISP_INCLUDE_DIRS_CONFIGCMAKE: ${VISP_INCLUDE_DIRS_CONFIGCMAKE}")

configure_file(
  ${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/VISPConfig.cmake
  IMMEDIATE @ONLY)

configure_file(
  ${VISP_CMAKE_MODULE_PATH}/VISPConfigVersion.cmake.in
  ${VISP_BINARY_DIR}/VISPConfigVersion.cmake
  IMMEDIATE @ONLY)

configure_file(
  ${VISP_CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/VISPUse.cmake
  IMMEDIATE @ONLY)

# --------------------------------------------------------------------------------------------
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use *with* "make install"
# -------------------------------------------------------------------------------------------

if(UNIX)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE_ "\${VISP_INSTALL_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")
  list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE_ ${VISP_EXTERN_INCLUDE_DIRS})
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "")
  foreach(val ${VISP_INCLUDE_DIRS_CONFIGCMAKE_})
    set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_INCLUDE_DIRS_CONFIGCMAKE} \"${val}\"")
  endforeach()
  #message("VISP_INCLUDE_DIRS_CONFIGCMAKE: ${VISP_INCLUDE_DIRS_CONFIGCMAKE}")

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPConfig.cmake
    IMMEDIATE @ONLY)

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPConfigVersion.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPConfigVersion.cmake
    IMMEDIATE @ONLY)

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPUse.cmake.in
    ${VISP_BINARY_DIR}/unix-install/VISPUse.cmake
    IMMEDIATE @ONLY)

  configure_file(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake
    ${VISP_BINARY_DIR}/unix-install/vpConfig.h @ONLY)

  install(FILES
    ${VISP_BINARY_DIR}/unix-install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/unix-install/VISPConfigVersion.cmake
    ${VISP_BINARY_DIR}/unix-install/VISPUse.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/visp"
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT libraries)

  # Install the export set for use with the install-tree
  install(EXPORT VISPTargets
    FILE VISPTargets.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/visp"
    COMPONENT libraries)
endif()

# --------------------------------------------------------------------------------------------
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
# --------------------------------------------------------------------------------------------
if(WIN32)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE_ "\${VISP_CONFIG_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")
  list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE_ ${VISP_EXTERN_INCLUDE_DIRS})
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "")
  foreach(val ${VISP_INCLUDE_DIRS_CONFIGCMAKE_})
    set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_INCLUDE_DIRS_CONFIGCMAKE} \"${val}\"")
  endforeach()
  #message("VISP_INCLUDE_DIRS_CONFIGCMAKE: ${VISP_INCLUDE_DIRS_CONFIGCMAKE}")

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPConfig.cmake
    IMMEDIATE @ONLY)

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPConfigVersion.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPConfigVersion.cmake
    IMMEDIATE @ONLY)

  configure_file(
    ${VISP_CMAKE_MODULE_PATH}/VISPUse.cmake.in
    ${VISP_BINARY_DIR}/win-install/VISPUse.cmake
    IMMEDIATE @ONLY)

  if(BUILD_SHARED_LIBS)
    install(FILES
      "${CMAKE_BINARY_DIR}/win-install/ViSPConfig.cmake"
      "${CMAKE_BINARY_DIR}/win-install/ViSPUse.cmake"
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}"
      COMPONENT libraries)
    install(EXPORT VISPTargets 
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}"
      FILE VISPTargets.cmake 
      COMPONENT libraries)
  else()
    install(FILES
      "${CMAKE_BINARY_DIR}/win-install/ViSPConfig.cmake"
      "${CMAKE_BINARY_DIR}/win-install/ViSPUse.cmake"
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}"
      COMPONENT libraries)
    install(EXPORT VISPTargets 
      DESTINATION "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}"
      FILE VISPTargets.cmake 
      COMPONENT libraries)
  endif()

  install(FILES
    "${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake"
    "${VISP_BINARY_DIR}/win-install/VISPConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_PREFIX}"
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT libraries)
endif()
