#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# See https://visp.inria.fr for more information.
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
# ViSP Python bindings module
#
#############################################################################

set(json_config_file "{}")
set(json_config_file_path "${CMAKE_CURRENT_BINARY_DIR}/cmake_config.json")

# Paths to important directories
string(JSON json_config_file SET ${json_config_file} "xml_doc_path" "\"${VISP_DOC_DIR}/xml\"")
string(JSON json_config_file SET ${json_config_file} "build_dir" "\"${CMAKE_BINARY_DIR}\"")
string(JSON json_config_file SET ${json_config_file} "source_dir" "\"${CMAKE_SOURCE_DIR}\"")

# Add include directories to config file
set(json_include_dirs "[]")
set(include_dirs_count 0)
foreach(include_dir ${VISP_INCLUDE_DIRS})
  string(JSON json_include_dirs SET ${json_include_dirs} "${include_dirs_count}" "\"${include_dir}\"")
  MATH(EXPR include_dirs_count "${include_dirs_count}+1")
endforeach()
string(JSON json_config_file SET ${json_config_file} "include_dirs" "${json_include_dirs}")

# For each bound module, add its headers and dependencies to config file
set(json_modules "{}")
foreach(module ${python_bound_modules})
  string(REPLACE "visp_" "" clean_module_name ${module})
  string(JSON json_modules SET ${json_modules} ${clean_module_name} "{}")
  # Get module headers
  set(json_header_list "[]")
  set(header_count 0)
  foreach(module_header ${VISP_MODULE_${module}_HEADERS})
    string(JSON json_header_list SET ${json_header_list} "${header_count}" "\"${module_header}\"")
    MATH(EXPR header_count "${header_count}+1")
  endforeach()
  string(JSON json_modules SET ${json_modules} ${clean_module_name} "headers" "${json_header_list}")
  # Get module dependencies
  set(json_deps_list "[]")
  set(dep_count 0)
  foreach(dep ${VISP_MODULE_${module}_DEPS})
    string(REPLACE "visp_" "" clean_dep ${dep})
    string(JSON json_deps_list SET ${json_deps_list} "${dep_count}" "\"${clean_dep}\"")
    MATH(EXPR dep_count "${dep_count}+1")
  endforeach()
  string(JSON json_modules SET ${json_modules} ${clean_module_name} "dependencies" "${json_deps_list}")
endforeach()
string(JSON json_config_file SET ${json_config_file} "modules" ${json_modules})

# Define platform specific macros
# These should be the same as those defined when compiling the visp libraries
# The impact will only be visible if the macros defined (or not) below appear in ViSP's headers
# See https://github.com/cpredef/predef/tree/master for compiler/OS specific #defines
set(json_defines "{}")
string(JSON json_defines SET ${json_defines} "__cplusplus" "${VISP_CXX_STANDARD}")
# Compiler
if(CMAKE_COMPILER_IS_GNUCXX AND NOT (MSVC AND CMAKE_COMPILER_IS_CLANGCXX))
  string(REPLACE "." ";" GCC_VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
  list(GET GCC_VERSION_LIST 0 GCC_MAJOR)
  list(GET GCC_VERSION_LIST 1 GCC_MINOR)
  list(GET GCC_VERSION_LIST 2 GCC_PATCH)

  string(JSON json_defines SET ${json_defines} "__GNUC__" "${GCC_MAJOR}")
  string(JSON json_defines SET ${json_defines} "__GNUC_MINOR__" "${GCC_MINOR}")
  string(JSON json_defines SET ${json_defines} "__GNUC_PATCHLEVEL__" "${GCC_PATCH}")
endif()


if(CMAKE_COMPILER_IS_CLANGCXX)
  string(REPLACE "." ";" CLANG_VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
  list(GET CLANG_VERSION_LIST 0 CLANG_MAJOR)
  list(GET CLANG_VERSION_LIST 1 CLANG_MINOR)
  list(GET CLANG_VERSION_LIST 2 CLANG_PATCH)

  string(JSON json_defines SET ${json_defines} "__clang__" "1")
  string(JSON json_defines SET ${json_defines} "__clang_major__" "${CLANG_MAJOR}")
  string(JSON json_defines SET ${json_defines} "__clang_minor__" "${CLANG_MINOR}")
  string(JSON json_defines SET ${json_defines} "__clang_patchlevel__" "${CLANG_PATCH}")
  string(JSON json_defines SET ${json_defines} "__clang_version__" "${CMAKE_CXX_COMPILER_VERSION}")
endif()

if(MSVC)
  string(JSON json_defines SET ${json_defines} "_MSC_VER" "${MSVC_VERSION}")
endif()

if(MINGW)
  string(JSON json_defines SET ${json_defines} "__MINGW32__" "null")
endif()
# OS
if(WIN32)
  string(JSON json_defines SET ${json_defines} "_WIN32" "null")
  string(JSON json_defines SET ${json_defines} "DWORD" "\"uint64_t\"")
  string(JSON json_defines SET ${json_defines} "WINAPI" "\"__stdcall\"")
  string(JSON json_defines SET ${json_defines} "LPVOID" "\"void*\"")
endif()
if(UNIX)
  string(JSON json_defines SET ${json_defines} "__linux__" "null")
  string(JSON json_defines SET ${json_defines} "__unix__" "null")
  string(JSON json_defines SET ${json_defines} "_unix" "null")
endif()
if(APPLE)
  string(JSON json_defines SET ${json_defines} "__APPLE__" "null")
  string(JSON json_defines SET ${json_defines} "__MACH__" "null")
endif()

string(JSON json_config_file SET ${json_config_file} "defines" ${json_defines})

message(STATUS "-------------JSON CONFIG FILE --------------")
message(STATUS "${json_config_file}")
message(STATUS "-------------EOF  --------------")
file(WRITE ${json_config_file_path} "${json_config_file}")
