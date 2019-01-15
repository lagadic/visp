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

# Here we determine the relative path from ./${VISP_LIB_INSTALL_PATH} to its parent folder
# if VISP_LIB_INSTALL_PATH=lib, then VISP_INSTALL_LIBDIR_TO_PARENT=../
# if VISP_LIB_INSTALL_PATH=lib/x86_64-linux-gnu, then VISP_INSTALL_LIBDIR_TO_PARENT=../..
get_path_to_parent(${VISP_LIB_INSTALL_PATH} VISP_INSTALL_LIBDIR_TO_PARENT)

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
#  Handle cmake extra vars (like USTK_HAVE_FFTW) that should be exported in VISPConfig.cmake thanks
#  to vp_cmake_configure()
#  All these vars are added in VISP_CONTRIB_MODULES_CONFIGCMAKE that is used in VISPConfig.cmake.in
#  See ustk_core/cmake/templates/VISPConfig-ustk_core.cmake.in example
# -------------------------------------------------------------------------------------------
set(VISP_CONTRIB_MODULES_CONFIGCMAKE "")
foreach(m ${VISP_MODULES_BUILD} ${VISP_MODULES_DISABLED_USER} ${VISP_MODULES_DISABLED_AUTO} ${VISP_MODULES_DISABLED_FORCE})
  set(__shortname ${m})
  vp_short_module_name(__shortname)
  if(EXISTS "${CMAKE_BINARY_DIR}/CMakeConfig-${__shortname}.cmake")
    file(READ "${CMAKE_BINARY_DIR}/CMakeConfig-${__shortname}.cmake" __var)
    set(VISP_CONTRIB_MODULES_CONFIGCMAKE "${VISP_CONTRIB_MODULES_CONFIGCMAKE}\n${__var}")
  endif()
endforeach()

if(ANDROID)
  if(NOT ANDROID_NATIVE_API_LEVEL)
    set(VISP_ANDROID_NATIVE_API_LEVEL_CONFIGCMAKE 0)
  else()
    set(VISP_ANDROID_NATIVE_API_LEVEL_CONFIGCMAKE "${ANDROID_NATIVE_API_LEVEL}")
  endif()
  vp_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-ANDROID.cmake.in" ANDROID_CONFIGCMAKE @ONLY)
endif()

# -------------------------------------------------------------------------------------------
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
# -------------------------------------------------------------------------------------------

# Export the library
export(TARGETS ${VISPModules_TARGETS} FILE "${PROJECT_BINARY_DIR}/VISPModules.cmake")

## Update include dirs
set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_INCLUDE_DIR}")
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
  cmake/templates/VISPConfig-version.cmake.in
  ${VISP_BINARY_DIR}/VISPConfig-version.cmake
  IMMEDIATE @ONLY
)

configure_file(
  cmake/templates/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/VISPUse.cmake
  IMMEDIATE @ONLY
)

# --------------------------------------------------------------------------------------------
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use *with* "make install"
# -------------------------------------------------------------------------------------------

if(UNIX)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\${VISP_INSTALL_PATH}/${VISP_INC_INSTALL_PATH}")
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE ${VISP_MODULE_${m}_INC_DEPS})
  endforeach()
  vp_list_unique(VISP_INCLUDE_DIRS_CONFIGCMAKE)
endif()

if((CMAKE_HOST_SYSTEM_NAME MATCHES "Linux" OR UNIX) AND NOT ANDROID)
  vp_gen_config("${CMAKE_BINARY_DIR}/unix-install" "" "")
endif()

if(ANDROID)
  vp_gen_config("${CMAKE_BINARY_DIR}/unix-install" "abi-${ANDROID_NDK_ABI_NAME}" "VISPConfig.root-ANDROID.cmake.in")
  install(FILES "${VISP_SOURCE_DIR}/platforms/android/android.toolchain.cmake" DESTINATION "${VISP_CONFIG_INSTALL_PATH}" COMPONENT dev)
endif()

# --------------------------------------------------------------------------------------------
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
# --------------------------------------------------------------------------------------------
if(WIN32)
  set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\${VISP_CONFIG_PATH}/${VISP_INC_INSTALL_PATH}")
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_INCLUDE_DIRS_CONFIGCMAKE ${VISP_MODULE_${m}_INC_DEPS})
  endforeach()
  vp_list_unique(VISP_INCLUDE_DIRS_CONFIGCMAKE)

  if(CMAKE_HOST_SYSTEM_NAME MATCHES Windows)
    if(BUILD_SHARED_LIBS)
      set(_lib_suffix "lib")
    else()
      set(_lib_suffix "staticlib")
    endif()
    vp_gen_config("${CMAKE_BINARY_DIR}/win-install" "${VISP_INSTALL_BINARIES_PREFIX}${_lib_suffix}" "VISPConfig.root-WIN32.cmake.in")
  else()
    vp_gen_config("${CMAKE_BINARY_DIR}/win-install" "" "")
  endif()
endif()
