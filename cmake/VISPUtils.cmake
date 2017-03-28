#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
# Authors:
# Fabien Spindler
#
#############################################################################

if(NOT COMMAND find_host_program)
  # macro to find programs on the host OS
  macro( find_host_program )
    set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER )
    set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER )
    set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER )
    find_program( ${ARGN} )
    set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY )
    set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
    set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )
    mark_as_advanced(${ARGV0})
  endmacro()
endif()

if(NOT COMMAND find_host_package)
  macro(find_host_package)
    find_package(${ARGN})
  endmacro()
endif()

# adds include directories in such way that directories from the ViSP source tree go first
function(vp_include_directories)
  vp_debug_message("vp_include_directories( ${ARGN} )")
  set(__add_before "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
#   if("${__abs_dir}" MATCHES "^${VISP_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${VISP_BINARY_DIR}") # not compatible with cmake 2.8.12.2
    if("${__abs_dir}" MATCHES "^${VISP_SOURCE_DIR}")
      list(APPEND __add_before "${dir}")
    elseif("${__abs_dir}" MATCHES "^${VISP_BINARY_DIR}")
      list(APPEND __add_before "${dir}")
    else()
      include_directories(AFTER SYSTEM "${dir}")
    endif()
  endforeach()
  include_directories(BEFORE ${__add_before})
endfunction()

# adds include directories in such way that directories from the ViSP source tree go first
function(vp_target_include_directories target)
  set(__params "")
  foreach(dir ${ARGN})
    get_filename_component(__abs_dir "${dir}" ABSOLUTE)
    string(REPLACE "+" "\\+" __VISP_BINARY_DIR_filtered ${VISP_BINARY_DIR})
#   if("${__abs_dir}" MATCHES "^${VISP_SOURCE_DIR}" OR "${__abs_dir}" MATCHES "^${__VISP_BINARY_DIR_filtered}")  # not compatible with cmake 2.8.12.2
    if("${__abs_dir}" MATCHES "^${VISP_SOURCE_DIR}")
      list(APPEND __params "${__abs_dir}")
    elseif("${__abs_dir}" MATCHES "^${__VISP_BINARY_DIR_filtered}")
      list(APPEND __params "${__abs_dir}")
    else()
      list(APPEND __params "${dir}")
    endif()
  endforeach()
  if(__params)
    if(CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories(${__params})
    else()
      if(TARGET ${target})
        target_include_directories(${target} PRIVATE ${__params})
      else()
        set(__new_inc "${VP_TARGET_INCLUDE_DIRS_${target}};${__params}")
        set(VP_TARGET_INCLUDE_DIRS_${target} "${__new_inc}" CACHE INTERNAL "")
      endif()
    endif()
  endif()
endfunction()

# clears all passed variables
macro(vp_clear_vars)
  foreach(_var ${ARGN})
    unset(${_var} CACHE)
  endforeach()
endmacro()

# print message
macro(vp_debug_message)
  #string(REPLACE ";" " " __msg "${ARGN}")
  #message(STATUS "${__msg}")
endmacro()

# remove all matching elements from the list
macro(vp_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()

# stable & safe duplicates removal macro
macro(vp_list_unique __lst)
  if(${__lst})
    list(REMOVE_DUPLICATES ${__lst})
  endif()
endmacro()

# list empty elements removal macro
macro(vp_list_remove_empty __lst)
  if(${__lst})
    list(REMOVE_ITEM ${__lst} "")
  endif()
endmacro()

# list elements removal macro
macro(vp_list_remove_item __lst __item)
  if(${__lst})
    list(REMOVE_ITEM ${__lst} ${__item})
  endif()
endmacro()

# macro that creates a list from a string. Spaces or tab are considered as list separator
# In other words split a string into list elements
macro(vp_create_list_from_string STR LST)
  if(NOT ${STR} STREQUAL "")
    set(__lst ${STR})
    string(REPLACE " " ";" __lst ${__lst})
    vp_list_remove_empty(${__lst})
    set(${LST} ${__lst})
  endif()
endmacro()

# safe list sorting macro
macro(vp_list_sort __lst)
  if(${__lst})
    list(SORT ${__lst})
  endif()
endmacro()

# gets and removes the first element from list
macro(vp_list_pop_front LST VAR)
  if(${LST})
    list(GET ${LST} 0 ${VAR})
    list(REMOVE_AT ${LST} 0)
  else()
    set(${VAR} "")
  endif()
endmacro()

# remove cmake ; list separator
macro(vp_list_remove_separator __lst)
  if(${__lst})
    set(__lst_reformated "")
    foreach(element ${${__lst}})
      set(__lst_reformated "${__lst_reformated} ${element}")
    endforeach()
    set(${__lst} ${__lst_reformated})
  endif()
endmacro()

# Provides an option that the user can optionally select.
# Can accept condition to control when option is available for user.
# Usage:
#   vp_option(<option_variable>
#             <package to find to activate the option>
#             <QUIET or "">
#             "help string describing the option"
#             "advanced list of vars separated by ;"
#             <initial value or boolean expression> [IF <condition>])
#
# Example:
#   VP_OPTION(USE_VTK "VTK" "QUIET" "Include vtk support" "" ON)
#   VP_OPTION(USE_VTK "VTK;COMPONENTS;vtkCommonCore;vtkFiltersSources" "" "Include vtk support" "" ON)

macro(VP_OPTION variable package quiet description advanced value)
  set(__option TRUE)
  set(__value ${value})
  set(__condition "")
  set(__varname "__value")

  set(__components "")
  set(__eltIsComponent FALSE)
  set(__package "") # without components
  foreach(p ${package})
    if(${p} MATCHES "COMPONENTS")
      set(__eltIsComponent TRUE)
    elseif(__eltIsComponent)
      list(APPEND __components ${p})
    else()
      list(APPEND __package ${p})
    endif()
  endforeach()

  # get the first package considered as the main package from a list: ie "Zlib;MyZlib"
  set(__first_package "")
  foreach(p ${__package})
    if(${p} MATCHES "^My")
      string(REGEX REPLACE "^My" "" p "${p}")
    endif()
    set(__first_package ${p})
    break()
  endforeach()

  if(NOT ${__first_package} STREQUAL "")
    string(TOLOWER "${__first_package}" __package_lower)
    string(TOUPPER "${__first_package}" __package_upper) # useful for Qt -> QT_FOUND

    # make var <package>_DIR advanced
    mark_as_advanced(${__first_package}_DIR)
  endif()

  foreach(arg ${ARGN})
    if(arg STREQUAL "IF" OR arg STREQUAL "if")
      set(__varname "__condition")
    else()
      list(APPEND ${__varname} ${arg})
    endif()
  endforeach()
  unset(__varname)
  if(__condition STREQUAL "")
    set(__condition 2 GREATER 1)
  endif()
  if(${__condition})

    if(NOT ${__first_package} STREQUAL "")
      foreach(p ${__package})
        if("${quiet}" STREQUAL "")
          if(__components)
            find_package(${p} COMPONENTS ${__components})
          else()
            find_package(${p})
          endif()
        else()
          if(__components)
            find_package(${p} ${quiet} COMPONENTS ${__components})
          else()
            find_package(${p} ${quiet})
          endif()
        endif()
        if(${__package_upper}_FOUND OR ${__first_package}_FOUND)
          set(__option TRUE)
          break()
        else()
          set(__option FALSE)
        endif()
      endforeach()
    endif()
    if(${__option})
      if(__value MATCHES ";")
        if(${__value})
          option(${variable} "${description}" ON)
        else()
          option(${variable} "${description}" OFF)
        endif()
      elseif(DEFINED ${__value})
        if(${__value})
          option(${variable} "${description}" ON)
        else()
          option(${variable} "${description}" OFF)
        endif()
      else()
        option(${variable} "${description}" ${__value})
      endif()
    else()
      unset(${variable} CACHE)
    endif()
    unset(__condition)
    unset(__value)

  else()
    set(${variable} OFF)
  endif()
  foreach(a ${advanced})
    mark_as_advanced(${a})
  endforeach()
  if(${variable} AND NOT ${__first_package} STREQUAL "")
    # set VISP_HAVE_<package>=TRUE and VISP_HAVE_<package>_FOUND="yes"
    message(STATUS "${__package_lower} found")
  endif()
  unset(__option)
  unset(__alias_have)
endmacro()

# Provides a macro to set a var.
# Can accept condition to set var.
# Usage:
#   vp_set(<option_variable>
#          <initial value or boolean expression> [IF <condition>])
macro(VP_SET variable value)
  set(__value ${value})
  set(__condition "")
  set(__varname "__value")

  foreach(arg ${ARGN})
    if(arg STREQUAL "IF" OR arg STREQUAL "if")
      set(__varname "__condition")
    else()
      list(APPEND ${__varname} ${arg})
    endif()
  endforeach()
  unset(__varname)
  if(__condition STREQUAL "")
    set(__condition 2 GREATER 1)
  endif()
  if(${__condition})
    if(__value MATCHES ";")
      if(${__value})
        set(${variable} TRUE)
      else()
        set(${variable} FALSE)
      endif()
    elseif(DEFINED ${__value})
      if(${__value})
        set(${variable} TRUE)
      else()
        set(${variable} FALSE)
      endif()
    else()
      set(${variable} ${__value})
    endif()
  endif()
  unset(__condition)
  unset(__value)
endmacro()

# short command to setup source group
function(vp_source_group group)
  cmake_parse_arguments(SG "" "DIRBASE" "GLOB;GLOB_RECURSE;FILES" ${ARGN})
  set(files "")
  if(SG_FILES)
    list(APPEND files ${SG_FILES})
  endif()
  if(SG_GLOB)
    file(GLOB srcs ${SG_GLOB})
    list(APPEND files ${srcs})
  endif()
  if(SG_GLOB_RECURSE)
    file(GLOB_RECURSE srcs ${SG_GLOB_RECURSE})
    list(APPEND files ${srcs})
  endif()
  if(SG_DIRBASE)
    foreach(f ${files})
      file(RELATIVE_PATH fpart "${SG_DIRBASE}" "${f}")
      if(fpart MATCHES "^\\.\\.")
        message(AUTHOR_WARNING "Can't detect subpath for vp_source_group command: Group=${group} FILE=${f} DIRBASE=${SG_DIRBASE}")
        set(fpart "")
      else()
        get_filename_component(fpart "${fpart}" PATH)
        if(fpart)
          set(fpart "/${fpart}") # add '/'
          string(REPLACE "/" "\\" fpart "${fpart}")
        endif()
      endif()
      source_group("${group}${fpart}" FILES ${f})
    endforeach()
  else()
    source_group(${group} FILES ${files})
  endif()
endfunction()

# convert list of paths to full paths
macro(vp_convert_to_full_paths VAR)
  if(${VAR})
    set(__tmp "")
    foreach(path ${${VAR}})
      get_filename_component(${VAR} "${path}" ABSOLUTE)
      list(APPEND __tmp "${${VAR}}")
    endforeach()
    set(${VAR} ${__tmp})
    unset(__tmp)
  endif()
endmacro()

# add install command
function(vp_install_target)
  install(TARGETS ${ARGN})

  set(isPackage 0)
  unset(__package)
  unset(__target)
  foreach(e ${ARGN})
    if(NOT DEFINED __target)
      set(__target "${e}")
    endif()
    if(isPackage EQUAL 1)
      set(__package "${e}")
      break()
    endif()
    if(e STREQUAL "EXPORT")
      set(isPackage 1)
    endif()
  endforeach()

  if(DEFINED __package)
    list(APPEND ${__package}_TARGETS ${__target})
    set(${__package}_TARGETS "${${__package}_TARGETS}" CACHE INTERNAL "List of ${__package} targets")
  endif()

  if(INSTALL_CREATE_DISTRIB)
    if(MSVC AND NOT BUILD_SHARED_LIBS)
      set(__target "${ARGV0}")

      set(isArchive 0)
      set(isDst 0)
      unset(__dst)
      foreach(e ${ARGN})
        if(isDst EQUAL 1)
          set(__dst "${e}")
          break()
        endif()
        if(isArchive EQUAL 1 AND e STREQUAL "DESTINATION")
          set(isDst 1)
        endif()
        if(e STREQUAL "ARCHIVE")
          set(isArchive 1)
        else()
          set(isArchive 0)
        endif()
      endforeach()

#      message(STATUS "Process ${__target} dst=${__dst}...")
      if(DEFINED __dst)
        if(CMAKE_VERSION VERSION_LESS 2.8.12)
          get_target_property(fname ${__target} LOCATION_DEBUG)
          if(fname MATCHES "\\.lib$")
            string(REGEX REPLACE "\\.lib$" ".pdb" fname "${fname}")
            install(FILES ${fname} DESTINATION ${__dst} CONFIGURATIONS Debug)
          endif()

          get_target_property(fname ${__target} LOCATION_RELEASE)
          if(fname MATCHES "\\.lib$")
            string(REGEX REPLACE "\\.lib$" ".pdb" fname "${fname}")
            install(FILES ${fname} DESTINATION ${__dst} CONFIGURATIONS Release)
          endif()
        else()
          # CMake 2.8.12 brokes PDB support in STATIC libraries for MSVS
        endif()
      endif()
    endif()
  endif()
endfunction()

function(vp_target_link_libraries target)
  set(LINK_DEPS ${ARGN})
  target_link_libraries(${target} ${LINK_DEPS})
endfunction()

function(_vp_append_target_includes target)
  if(DEFINED VP_TARGET_INCLUDE_DIRS_${target})
    target_include_directories(${target} PRIVATE ${VP_TARGET_INCLUDE_DIRS_${target}})
    unset(VP_TARGET_INCLUDE_DIRS_${target} CACHE)
  endif()
endfunction()

function(vp_add_executable target)
  add_executable(${target} ${ARGN})
  _vp_append_target_includes(${target})
endfunction()

function(vp_add_library target)
  add_library(${target} ${ARGN})

  _vp_append_target_includes(${target})
endfunction()

# Macros that checks if package have been installed.
# After it set vars:
#   <package>_FOUND
#   <package>_INCLUDE_DIRS
#   <package>_LIBRARIES
#   VISP_HAVE_<package>
macro(VP_CHECK_PACKAGE package)
  set(ALIAS                 ${package})
  string(TOUPPER "${ALIAS}" ALIAS_UPPER) # useful for OpenGL
  set(ALIAS_FOUND           ${ALIAS}_FOUND)
  set(ALIAS_UPPER_FOUND     ${ALIAS_UPPER}_FOUND)
  set(ALIAS_INCLUDE_DIRS    ${ALIAS}_INCLUDE_DIRS)
  set(ALIAS_LIBRARIES       ${ALIAS}_LIBRARIES)
  set(ALIAS_VISP_HAVE       VISP_HAVE_${ALIAS})
  set(ALIAS_UPPER_VISP_HAVE VISP_HAVE_${ALIAS_UPPER})

  find_package(${ALIAS})

  if(${ALIAS_FOUND} OR ${ALIAS_UPPER_FOUND})
    set(${ALIAS_VISP_HAVE} 1)
    set(${ALIAS_UPPER_VISP_HAVE} 1)
  endif()
endmacro()

# Macro the get the list of subdirs from the path
# var: returned variable name
# path: path from witch relative subdirs are
macro(vp_get_relative_subdirs var path)
  set(ALIAS                 ${var})
  file(GLOB_RECURSE rel_path_lst_ RELATIVE ${path} ${path}/*)
    set(${ALIAS} "")
    foreach(f ${rel_path_lst_})
      get_filename_component(d ${f} PATH)
      list(APPEND ${ALIAS} ${d})
    endforeach()
    list(REMOVE_DUPLICATES ${ALIAS})
endmacro()

set(VP_COMPILER_FAIL_REGEX
    "command line option .* is valid for .* but not for C\\+\\+" # GNU
    "command line option .* is valid for .* but not for C" # GNU
    "unrecognized .*option"                     # GNU
    "unknown .*option"                          # Clang
    "ignoring unknown option"                   # MSVC
    "warning D9002"                             # MSVC, any lang
    "option .*not supported"                    # Intel
    "[Uu]nknown option"                         # HP
    "[Ww]arning: [Oo]ption"                     # SunPro
    "command option .* is not recognized"       # XL
    "not supported in this configuration; ignored"       # AIX
    "File with unknown suffix passed to linker" # PGI
    "WARNING: unknown flag:"                    # Open64
  )

# test if a compiler flag is supported
macro(vp_check_compiler_flag LANG FLAG RESULT)
  if(NOT DEFINED ${RESULT})

    if("_${LANG}_" MATCHES "_CXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.cxx")
      #if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_C_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c")
      #if("${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main(void) { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main(void) { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_OBJCXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.mm")
      #if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      #else()
      #  file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      #endif()
    elseif("_${LANG}_" MATCHES "_Fortran_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.f")
      file(WRITE "${_fname}" "      PROGRAM TEST\n      RETURN\n      END\n")
    else()
      unset(_fname)
    endif()
    if(_fname)
      message(STATUS "Performing Test ${RESULT}")
      try_compile(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        COMPILE_DEFINITIONS "${FLAG}"
        OUTPUT_VARIABLE OUTPUT)

      foreach(_regex ${VP_COMPILER_FAIL_REGEX})
        if("${OUTPUT}" MATCHES "${_regex}")
          set(${RESULT} 0)
          break()
        endif()
      endforeach()

      if(${RESULT})
        set(${RESULT} 1 CACHE INTERNAL "Test ${RESULT}")
        message(STATUS "Performing Test ${RESULT} - Success")
      else()
        message(STATUS "Performing Test ${RESULT} - Failed")
        set(${RESULT} "" CACHE INTERNAL "Test ${RESULT}")
        file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Compilation failed:\n"
            "    source file: '${_fname}'\n"
            "    check option: '${FLAG}'\n"
            "===== BUILD LOG =====\n"
            "${OUTPUT}\n"
            "===== END =====\n\n")
      endif()
    else()
      set(${RESULT} 0)
    endif()
  endif()
endmacro()

# check if a compiler flag is supported
macro(vp_check_flag_support lang flag varname)
  if("_${lang}_" MATCHES "_CXX_")
    set(_lang CXX)
  elseif("_${lang}_" MATCHES "_C_")
    set(_lang C)
  elseif("_${lang}_" MATCHES "_Fortran_")
    set(_lang Fortran)
  else()
    set(_lang ${lang})
  endif()

  string(TOUPPER "${flag}" ${varname})
  string(REGEX REPLACE "^(/|-)" "HAVE_${_lang}_" ${varname} "${${varname}}")
  string(REGEX REPLACE " -|-|=| |\\." "_" ${varname} "${${varname}}")
  vp_check_compiler_flag("${_lang}" "${ARGN} ${flag}" ${${varname}})
endmacro()

# turns off warnings
macro(vp_warnings_disable)
    set(_flag_vars "")
    set(_msvc_warnings "")
    set(_gxx_warnings "")
    foreach(arg ${ARGN})
      if(arg MATCHES "^CMAKE_")
        list(APPEND _flag_vars ${arg})
      elseif(arg MATCHES "^/wd")
        list(APPEND _msvc_warnings ${arg})
      elseif(arg MATCHES "^-W")
        list(APPEND _gxx_warnings ${arg})
      endif()
    endforeach()
    if(MSVC AND _msvc_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_msvc_warnings})
          set(${var} "${${var}} ${warning}")
        endforeach()
      endforeach()
    elseif((CMAKE_COMPILER_IS_GNUCXX OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")) AND _gxx_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_gxx_warnings})
          if(NOT warning MATCHES "^-Wno-")
            string(REPLACE "${warning}" "" ${var} "${${var}}")
            string(REPLACE "-W" "-Wno-" warning "${warning}")
          endif()
          vp_check_flag_support(${var} "${warning}" _varname)
          if(${_varname})
            set(${var} "${${var}} ${warning}")
          endif()
        endforeach()
      endforeach()
    endif()
    unset(_flag_vars)
    unset(_msvc_warnings)
    unset(_gxx_warnings)
endmacro()

macro(vp_set_source_file_compile_flag file)
  if(ACTIVATE_WARNING_3PARTY_MUTE)
    set(__cxxflags "")
    set(__lang "CXX")
    get_filename_component(__ext ${file} EXT)
    if(__ext MATCHES "$.c")
      set(__lang "C")
    elseif(__ext MATCHES "$.f")
      set(__lang "Fortran")
    elseif(__ext MATCHES "$.F")
      set(__lang "Fortran")
    endif()
    foreach(cxxflag ${ARGN})
      vp_check_flag_support(${__lang} ${cxxflag} __support_flag)
      if(${__support_flag})
        set(__cxxflags "${__cxxflags} ${cxxflag}")
      endif()
    endforeach()
    if(NOT ${__cxxflags} STREQUAL "")
      if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/${file})
        set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/${file} PROPERTIES COMPILE_FLAGS ${__cxxflags})
      elseif(EXISTS ${file}) # for files that are in the build tree (like those produced by qt moc)
        set_source_files_properties(${file} PROPERTIES COMPILE_FLAGS ${__cxxflags})
      endif()
    endif()
  endif()
endmacro()

macro(vp_add_subdirectories lst subdir)
  if(${lst})
    foreach(__path ${${lst}})
      if(EXISTS ${__path}/${subdir})
        file(GLOB __subdirs RELATIVE "${__path}/${subdir}" "${__path}/${subdir}/*")
        foreach(__s ${__subdirs})
          if(EXISTS "${__path}/${subdir}/${__s}/CMakeLists.txt")
            add_subdirectory("${__path}/${subdir}/${__s}" "${CMAKE_BINARY_DIR}/${subdir}/${__s}")
          endif()
        endforeach()
      endif()
    endforeach()
  endif()
endmacro()

set(VISP_BUILD_INFO_FILE "${CMAKE_BINARY_DIR}/visp-build-info.tmp")
set(VISP_TXT_INFO_FILE "${CMAKE_BINARY_DIR}/ViSP-third-party.txt")
file(REMOVE "${VISP_BUILD_INFO_FILE}")
file(REMOVE "${VISP_TXT_INFO_FILE}")
function(vp_output_status msg)
  message(STATUS "${msg}")
  string(REPLACE "\\" "\\\\" msg "${msg}")
  string(REPLACE "\"" "\\\"" msg "${msg}")
  string(REPLACE "\\\\" "\\" msg_txt "${msg}")
  file(APPEND "${VISP_BUILD_INFO_FILE}" "\"${msg}\\n\"\n")
  file(APPEND "${VISP_TXT_INFO_FILE}" "${msg_txt}\n")
endfunction()

macro(vp_finalize_status)
  if(DEFINED VISP_MODULE_visp_core_BINARY_DIR)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${VISP_BUILD_INFO_FILE}" "${VISP_MODULE_visp_core_BINARY_DIR}/version_string.inc" OUTPUT_QUIET)
  endif()
endmacro()

# Status report function.
# Automatically align right column and selects text based on condition.
# Usage:
#   status(<text>)
#   status(<heading> <value1> [<value2> ...])
#   status(<heading> <condition> THEN <text for TRUE> ELSE <text for FALSE> )
function(status text)
  set(status_cond)
  set(status_then)
  set(status_else)

  set(status_current_name "cond")
  foreach(arg ${ARGN})
    if(arg STREQUAL "THEN")
      set(status_current_name "then")
    elseif(arg STREQUAL "ELSE")
      set(status_current_name "else")
    else()
      list(APPEND status_${status_current_name} ${arg})
    endif()
  endforeach()

  if(DEFINED status_cond)
    set(status_placeholder_length 32)
    string(RANDOM LENGTH ${status_placeholder_length} ALPHABET " " status_placeholder)
    string(LENGTH "${text}" status_text_length)
    if(status_text_length LESS status_placeholder_length)
      string(SUBSTRING "${text}${status_placeholder}" 0 ${status_placeholder_length} status_text)
    elseif(DEFINED status_then OR DEFINED status_else)
      vp_output_status("${text}")
      set(status_text "${status_placeholder}")
    else()
      set(status_text "${text}")
    endif()

    if(DEFINED status_then OR DEFINED status_else)
      if(${status_cond})
        string(REPLACE ";" " " status_then "${status_then}")
        string(REGEX REPLACE "^[ \t]+" "" status_then "${status_then}")
        vp_output_status("${status_text} ${status_then}")
      else()
        string(REPLACE ";" " " status_else "${status_else}")
        string(REGEX REPLACE "^[ \t]+" "" status_else "${status_else}")
        vp_output_status("${status_text} ${status_else}")
      endif()
    else()
      string(REPLACE ";" " " status_cond "${status_cond}")
      string(REGEX REPLACE "^[ \t]+" "" status_cond "${status_cond}")
      vp_output_status("${status_text} ${status_cond}")
    endif()
  else()
    vp_output_status("${text}")
  endif()
endfunction()

# read set of version defines from the header file
macro(vp_parse_header FILENAME FILE_VAR)
  set(vars_regex "")
  set(__parnet_scope OFF)
  set(__add_cache OFF)
  foreach(name ${ARGN})
    if(${name} STREQUAL "PARENT_SCOPE")
      set(__parnet_scope ON)
    elseif(${name} STREQUAL "CACHE")
      set(__add_cache ON)
    elseif(vars_regex)
      set(vars_regex "${vars_regex}|${name}")
    else()
      set(vars_regex "${name}")
    endif()
  endforeach()
  if(EXISTS "${FILENAME}")
    file(STRINGS "${FILENAME}" ${FILE_VAR} REGEX "#define[ \t]+(${vars_regex})[ \t]+[0-9]+" )
  else()
    unset(${FILE_VAR})
  endif()
  foreach(name ${ARGN})
    if(NOT ${name} STREQUAL "PARENT_SCOPE" AND NOT ${name} STREQUAL "CACHE")
      if(${FILE_VAR})
        if(${FILE_VAR} MATCHES ".+[ \t]${name}[ \t]+([0-9]+).*")
          string(REGEX REPLACE ".+[ \t]${name}[ \t]+([0-9]+).*" "\\1" ${name} "${${FILE_VAR}}")
        else()
          set(${name} "")
        endif()
        if(__add_cache)
          set(${name} ${${name}} CACHE INTERNAL "${name} parsed from ${FILENAME}" FORCE)
        elseif(__parnet_scope)
          set(${name} "${${name}}" PARENT_SCOPE)
        endif()
      else()
        unset(${name} CACHE)
      endif()
    endif()
  endforeach()
endmacro()

# read single version define from the header file
macro(vp_parse_header2 LIBNAME HDR_PATH VARNAME)
  vp_clear_vars(${LIBNAME}_VERSION_MAJOR
                ${LIBNAME}_VERSION_MAJOR
                ${LIBNAME}_VERSION_MINOR
                ${LIBNAME}_VERSION_PATCH
                ${LIBNAME}_VERSION_TWEAK
                ${LIBNAME}_VERSION_STRING)
  set(${LIBNAME}_H "")
  if(EXISTS "${HDR_PATH}")
    file(STRINGS "${HDR_PATH}" ${LIBNAME}_H REGEX "^#define[ \t]+${VARNAME}[ \t]+\"[^\"]*\".*$" LIMIT_COUNT 1)
  endif()

  if(${LIBNAME}_H)
    string(REGEX REPLACE "^.*[ \t]${VARNAME}[ \t]+\"([0-9]+).*$" "\\1" ${LIBNAME}_VERSION_MAJOR "${${LIBNAME}_H}")
    string(REGEX REPLACE "^.*[ \t]${VARNAME}[ \t]+\"[0-9]+\\.([0-9]+).*$" "\\1" ${LIBNAME}_VERSION_MINOR  "${${LIBNAME}_H}")
    string(REGEX REPLACE "^.*[ \t]${VARNAME}[ \t]+\"[0-9]+\\.[0-9]+\\.([0-9]+).*$" "\\1" ${LIBNAME}_VERSION_PATCH "${${LIBNAME}_H}")
    set(${LIBNAME}_VERSION_MAJOR ${${LIBNAME}_VERSION_MAJOR} ${ARGN})
    set(${LIBNAME}_VERSION_MINOR ${${LIBNAME}_VERSION_MINOR} ${ARGN})
    set(${LIBNAME}_VERSION_PATCH ${${LIBNAME}_VERSION_PATCH} ${ARGN})
    set(${LIBNAME}_VERSION_STRING "${${LIBNAME}_VERSION_MAJOR}.${${LIBNAME}_VERSION_MINOR}.${${LIBNAME}_VERSION_PATCH}")

    # append a TWEAK version if it exists:
    set(${LIBNAME}_VERSION_TWEAK "")
    if("${${LIBNAME}_H}" MATCHES "^.*[ \t]${VARNAME}[ \t]+\"[0-9]+\\.[0-9]+\\.[0-9]+\\.([0-9]+).*$")
      set(${LIBNAME}_VERSION_TWEAK "${CMAKE_MATCH_1}" ${ARGN})
    endif()
    if(${LIBNAME}_VERSION_TWEAK)
      set(${LIBNAME}_VERSION_STRING "${${LIBNAME}_VERSION_STRING}.${${LIBNAME}_VERSION_TWEAK}" ${ARGN})
    else()
      set(${LIBNAME}_VERSION_STRING "${${LIBNAME}_VERSION_STRING}" ${ARGN})
    endif()
  endif()
endmacro()

# read single version info from the pkg file
macro(vp_get_version_from_pkg LIBNAME PKG_PATH OUTPUT_VAR)
  if(EXISTS "${PKG_PATH}/${LIBNAME}.pc")
    file(STRINGS "${PKG_PATH}/${LIBNAME}.pc" line_to_parse REGEX "^Version:[ \t]+[0-9.]*.*$" LIMIT_COUNT 1)
    string(REGEX REPLACE ".*Version: ([^ ]+).*" "\\1" ${OUTPUT_VAR} "${line_to_parse}" )
  endif()
endmacro()
