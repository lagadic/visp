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

# TODO: adding macros for android build
# -------------------------------------------

macro(vp_path_join result_var P1 P2_)
  string(REGEX REPLACE "^[/]+" "" P2 "${P2_}")
  if("${P1}" STREQUAL "" OR "${P1}" STREQUAL ".")
    set(${result_var} "${P2}")
  elseif("${P1}" STREQUAL "/")
    set(${result_var} "/${P2}")
  elseif("${P2}" STREQUAL "")
    set(${result_var} "${P1}")
  else()
    set(${result_var} "${P1}/${P2}")
  endif()
  string(REGEX REPLACE "([/\\]?)[\\.][/\\]" "\\1" ${result_var} "${${result_var}}")
  if("${${result_var}}" STREQUAL "")
    set(${result_var} ".")
  endif()
  #message(STATUS "'${P1}' '${P2_}' => '${${result_var}}'")
endmacro()

# Used to parse Android SDK 'source.properties' files
# File lines format:
# - '<var_name>=<value>' (with possible 'space' symbols around '=')
# - '#<any comment>'
# Parsed values are saved into CMake variables:
# - '${var_prefix}_${var_name}'
# Flags:
# - 'CACHE_VAR <var1> <var2>' - put these properties into CMake internal cache
# - 'MSG_PREFIX <msg>' - prefix string for emitted messages
# - flag 'VALIDATE' - emit messages about missing values from required cached variables
# - flag 'WARNING' - emit CMake WARNING instead of STATUS messages
function(vp_parse_properties_file file var_prefix)
  cmake_parse_arguments(PARSE_PROPERTIES_PARAM "VALIDATE;WARNING" "" "CACHE_VAR;MSG_PREFIX" ${ARGN})

  set(__msg_type STATUS)
  if(PARSE_PROPERTIES_PARAM_WARNING)
    set(__msg_type WARNING)
  endif()

  if(EXISTS "${file}")
    set(SOURCE_PROPERTIES_REGEX "^[ ]*([^=:\n\"' ]+)[ ]*=[ ]*(.*)$")
    file(STRINGS "${file}" SOURCE_PROPERTIES_LINES REGEX "^[ ]*[^#].*$")
    foreach(line ${SOURCE_PROPERTIES_LINES})
      if(line MATCHES "${SOURCE_PROPERTIES_REGEX}")
        set(__name "${CMAKE_MATCH_1}")
        set(__value "${CMAKE_MATCH_2}")
        string(REGEX REPLACE "[^a-zA-Z0-9_]" "_" __name ${__name})
        if(";${PARSE_PROPERTIES_PARAM_CACHE_VAR};" MATCHES ";${__name};")
          set(${var_prefix}_${__name} "${__value}" CACHE INTERNAL "from ${file}")
        else()
          set(${var_prefix}_${__name} "${__value}" PARENT_SCOPE)
        endif()
      else()
        message(${__msg_type} "${PARSE_PROPERTIES_PARAM_MSG_PREFIX}Can't parse source property: '${line}' (from ${file})")
      endif()
    endforeach()
    if(PARSE_PROPERTIES_PARAM_VALIDATE)
      set(__missing "")
      foreach(__name ${PARSE_PROPERTIES_PARAM_CACHE_VAR})
        if(NOT DEFINED ${var_prefix}_${__name})
          list(APPEND __missing ${__name})
        endif()
      endforeach()
      if(__missing)
        message(${__msg_type} "${PARSE_PROPERTIES_PARAM_MSG_PREFIX}Can't read properties '${__missing}' from '${file}'")
      endif()
    endif()
  else()
    message(${__msg_type} "${PARSE_PROPERTIES_PARAM_MSG_PREFIX}Can't find file: ${file}")
  endif()
endfunction()

macro(vp_update VAR)
  if(NOT DEFINED ${VAR})
    if("x${ARGN}" STREQUAL "x")
      set(${VAR} "")
    else()
      set(${VAR} ${ARGN})
    endif()
  endif()
endmacro()

function(vp_gen_config TMP_DIR NESTED_PATH ROOT_NAME)
  vp_path_join(__install_nested "${VISP_CONFIG_INSTALL_PATH}" "${NESTED_PATH}")
  vp_path_join(__tmp_nested "${TMP_DIR}" "${NESTED_PATH}")

  configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISPConfig-version.cmake.in" "${TMP_DIR}/VISPConfig-version.cmake" @ONLY)

  configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISPConfig.cmake.in" "${__tmp_nested}/VISPConfig.cmake" @ONLY)
  configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISPUse.cmake.in" "${__tmp_nested}/VISPUse.cmake" @ONLY)
  install(EXPORT VISPModules DESTINATION "${__install_nested}" FILE VISPModules.cmake COMPONENT dev)
  install(FILES
      "${TMP_DIR}/VISPConfig-version.cmake"
      "${__tmp_nested}/VISPConfig.cmake"
      "${__tmp_nested}/VISPUse.cmake"
      DESTINATION "${__install_nested}" COMPONENT dev)

  if(ROOT_NAME)
    # Root config file
    configure_file("${VISP_SOURCE_DIR}/cmake/templates/${ROOT_NAME}" "${TMP_DIR}/VISPConfig.cmake" @ONLY)
    install(FILES
        "${TMP_DIR}/VISPConfig-version.cmake"
        "${TMP_DIR}/VISPConfig.cmake"
        DESTINATION "${VISP_CONFIG_INSTALL_PATH}" COMPONENT dev)
  endif()
endfunction()

# Finished adding android macros for visp
# -------------------------------------------

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
    if(TARGET ${target})
      target_include_directories(${target} PRIVATE ${__params})
    else()
      set(__new_inc "${VP_TARGET_INCLUDE_DIRS_${target}};${__params}")
      set(VP_TARGET_INCLUDE_DIRS_${target} "${__new_inc}" CACHE INTERNAL "")
    endif()
  endif()
endfunction()

# clears all passed variables
macro(vp_clear_vars)
  foreach(_var ${ARGN})
    unset(${_var} CACHE)
  endforeach()
endmacro()

# assert macro
# Note: it doesn't support lists in arguments
# Usage samples:
#   vp_assert(MyLib_FOUND)
#   vp_assert(DEFINED MyLib_INCLUDE_DIRS)
macro(vp_assert)
  if(NOT (${ARGN}))
    string(REPLACE ";" " " __assert_msg "${ARGN}")
    message(AUTHOR_WARNING "Assertion failed: ${__assert_msg}")
  endif()
endmacro()

# add prefix to each item in the list
macro(vp_list_add_prefix LST PREFIX)
  set(__tmp "")
  foreach(item ${${LST}})
    list(APPEND __tmp "${PREFIX}${item}")
  endforeach()
  set(${LST} ${__tmp})
  unset(__tmp)
endmacro()


# add suffix to each item in the list
macro(vp_list_add_suffix LST SUFFIX)
  set(__tmp "")
  foreach(item ${${LST}})
    list(APPEND __tmp "${item}${SUFFIX}")
  endforeach()
  set(${LST} ${__tmp})
  unset(__tmp)
endmacro()

macro(vp_copyfiles_append_dir list_var src dst)
  set(__glob ${ARGN})
  list(LENGTH ${list_var} __id)
  list(APPEND ${list_var} ${__id})
  set(${list_var}_SRC_${__id} "${src}")
  set(${list_var}_DST_${__id} "${dst}")
  set(${list_var}_MODE_${__id} "COPYDIR")
  if(__glob)
    set(${list_var}_GLOB_${__id} ${__glob})
  endif()
endmacro()

macro(vp_copyfiles_make_config_string content_var list_var)
  set(var_name "${list_var}")
  set(${content_var} "${${content_var}}
set(${var_name} \"${${var_name}}\")
")
  foreach(__id ${${list_var}})
    set(${content_var} "${${content_var}}
set(${list_var}_SRC_${__id} \"${${list_var}_SRC_${__id}}\")
set(${list_var}_DST_${__id} \"${${list_var}_DST_${__id}}\")
")
    if(DEFINED ${list_var}_MODE_${__id})
      set(${content_var} "${${content_var}}set(${list_var}_MODE_${__id} \"${${list_var}_MODE_${__id}}\")\n")
    endif()
    if(DEFINED ${list_var}_GLOB_${__id})
      set(${content_var} "${${content_var}}set(${list_var}_GLOB_${__id} \"${${list_var}_GLOB_${__id}}\")\n")
    endif()
  endforeach()
endmacro()

macro(vp_copyfiles_make_config_file filename_var list_var)
  vp_copyfiles_make_config_string(${list_var}_CONFIG ${list_var})
  set(${filename_var} "${CMAKE_CURRENT_BINARY_DIR}/copyfiles-${list_var}.cmake")
  file(WRITE "${${filename_var}}" "${${list_var}_CONFIG}")
endmacro()

macro(vp_copyfiles_add_forced_target target list_var comment_str)
  vp_copyfiles_make_config_file(CONFIG_FILE ${list_var})
  vp_cmake_byproducts(__byproducts BYPRODUCTS "${VISP_DEPHELPER}/${target}")
  add_custom_target(${target}
      ${__byproducts}  # required for add_custom_target() by ninja
      COMMAND ${CMAKE_COMMAND}
        "-DCONFIG_FILE:PATH=${CONFIG_FILE}"
        "-DCOPYLIST_VAR:STRING=${list_var}"
        "-DDEPHELPER=${VISP_DEPHELPER}/${target}"
        -P "${VISP_SOURCE_DIR}/cmake/copy_files.cmake"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "${comment_str}"
      DEPENDS "${VISP_SOURCE_DIR}/cmake/copy_files.cmake"
              # ninja warn about file(WRITE): "${SRC_COPY_CONFIG_FILE}"
  )
endmacro()

macro(vp_copyfiles_add_target target list_var comment_str)
  set(deps ${ARGN})
  vp_copyfiles_make_config_file(CONFIG_FILE ${list_var})
  add_custom_command(OUTPUT "${VISP_DEPHELPER}/${target}"
      COMMAND ${CMAKE_COMMAND}
        "-DCONFIG_FILE:PATH=${CONFIG_FILE}"
        "-DCOPYLIST_VAR:STRING=${list_var}"
        "-DDEPHELPER=${VISP_DEPHELPER}/${target}"
        -P "${VISP_SOURCE_DIR}/cmake/copy_files.cmake"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "${comment_str}"
      DEPENDS "${VISP_SOURCE_DIR}/cmake/copy_files.cmake" ${deps}
              # ninja warn about file(WRITE): "${SRC_COPY_CONFIG_FILE}"
  )
  add_custom_target(${target} DEPENDS "${VISP_DEPHELPER}/${target}")
endmacro()

# Used in detecting PATH variables
macro(vp_check_environment_variables)
  foreach(_var ${ARGN})
    if(" ${${_var}}" STREQUAL " " AND DEFINED ENV{${_var}})
      set(__value "$ENV{${_var}}")
      file(TO_CMAKE_PATH "${__value}" __value) # Assume that we receive paths
      set(${_var} "${__value}")
      message(STATUS "Update variable ${_var} from environment: ${${_var}}")
    endif()
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

# safe list reversal macro
macro(vp_list_reverse __lst)
  if(${__lst})
    list(REVERSE ${__lst})
  endif()
endmacro()

# list empty elements removal macro
macro(vp_list_remove_empty __lst)
  if(${__lst})
    list(REMOVE_ITEM ${__lst} "")
  endif()
endmacro()

# needed by visp-java
if(CMAKE_VERSION VERSION_LESS "3.2")
  macro(vp_cmake_byproducts var_name)
    set(${var_name}) # nothing
  endmacro()
else()
  macro(vp_cmake_byproducts var_name)
    set(${var_name} BYPRODUCTS ${ARGN})
  endmacro()
endif()

set(VISP_DEPHELPER "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/dephelper" CACHE INTERNAL "")
file(MAKE_DIRECTORY ${VISP_DEPHELPER})

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

# replace cmake ; list separator
macro(vp_list_replace_separator __lst __separator)
  if(${__lst})
    list(GET ${__lst} 0 __lst_reformated)
    list(LENGTH ${__lst} __length)
    if(__length GREATER 1)
      MATH(EXPR __length "${__length} - 1")
      foreach(_i RANGE 1 ${__length}-1)
        list(GET ${__lst} ${_i} element)
       set(__lst_reformated "${__lst_reformated}${__separator}${element}")
      endforeach()
    endif()
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
  if(APPLE_FRAMEWORK AND BUILD_SHARED_LIBS)
    install(TARGETS ${ARGN} FRAMEWORK DESTINATION ${VISP_3P_LIB_INSTALL_PATH})
  else()
    install(TARGETS ${ARGN})
  endif()

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

  if(MSVC)
    set(__target "${ARGV0}")

    # don't move this into global scope of this file: compiler settings (like MSVC variable) are not available during processing
    if(BUILD_SHARED_LIBS)  # no defaults for static libs (modern CMake is required)
      if(NOT CMAKE_VERSION VERSION_LESS 3.6.0)
        option(INSTALL_PDB_COMPONENT_EXCLUDE_FROM_ALL "Don't install PDB files by default" ON)
        option(INSTALL_PDB "Add install PDB rules" ON)
      elseif(NOT CMAKE_VERSION VERSION_LESS 3.1.0)
        option(INSTALL_PDB_COMPONENT_EXCLUDE_FROM_ALL "Don't install PDB files by default (not supported)" OFF)
        option(INSTALL_PDB "Add install PDB rules" OFF)
      endif()
    endif()

    if(INSTALL_PDB AND NOT INSTALL_IGNORE_PDB
        AND NOT OPENCV_${__target}_PDB_SKIP
    )
      set(__location_key "ARCHIVE")  # static libs
      get_target_property(__target_type ${__target} TYPE)
      if("${__target_type}" STREQUAL "SHARED_LIBRARY")
        set(__location_key "RUNTIME")  # shared libs (.DLL)
      endif()

      set(processDst 0)
      set(isDst 0)
      unset(__dst)
      foreach(e ${ARGN})
        if(isDst EQUAL 1)
          set(__dst "${e}")
          break()
        endif()
        if(processDst EQUAL 1 AND e STREQUAL "DESTINATION")
          set(isDst 1)
        endif()
        if(e STREQUAL "${__location_key}")
          set(processDst 1)
        else()
          set(processDst 0)
        endif()
      endforeach()

#      message(STATUS "Process ${__target} dst=${__dst}...")
      if(DEFINED __dst)
        if(NOT CMAKE_VERSION VERSION_LESS 3.1.0)
          set(__pdb_install_component "pdb")
          if(DEFINED INSTALL_PDB_COMPONENT AND INSTALL_PDB_COMPONENT)
            set(__pdb_install_component "${INSTALL_PDB_COMPONENT}")
          endif()
          set(__pdb_exclude_from_all "")
          if(INSTALL_PDB_COMPONENT_EXCLUDE_FROM_ALL)
            if(NOT CMAKE_VERSION VERSION_LESS 3.6.0)
              set(__pdb_exclude_from_all EXCLUDE_FROM_ALL)
            else()
              message(WARNING "INSTALL_PDB_COMPONENT_EXCLUDE_FROM_ALL requires CMake 3.6+")
            endif()
          endif()

#          message(STATUS "Adding PDB file installation rule: target=${__target} dst=${__dst} component=${__pdb_install_component}")
          if("${__target_type}" STREQUAL "SHARED_LIBRARY" OR "${__target_type}" STREQUAL "MODULE_LIBRARY")
            install(FILES "$<TARGET_PDB_FILE:${__target}>" DESTINATION "${__dst}"
                COMPONENT ${__pdb_install_component} OPTIONAL ${__pdb_exclude_from_all})
          else()
            # There is no generator expression similar to TARGET_PDB_FILE and TARGET_PDB_FILE can't be used: https://gitlab.kitware.com/cmake/cmake/issues/16932
            # However we still want .pdb files like: 'lib/Debug/visp_core331d.pdb' or '3rdparty/lib/visp_apriltag.pdb'
            install(FILES "$<TARGET_PROPERTY:${__target},ARCHIVE_OUTPUT_DIRECTORY>/$<CONFIG>/$<IF:$<BOOL:$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME_DEBUG>>,$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME_DEBUG>,$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME>>.pdb"
                DESTINATION "${__dst}" CONFIGURATIONS Debug
                COMPONENT ${__pdb_install_component} OPTIONAL ${__pdb_exclude_from_all})
            install(FILES "$<TARGET_PROPERTY:${__target},ARCHIVE_OUTPUT_DIRECTORY>/$<CONFIG>/$<IF:$<BOOL:$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME_RELEASE>>,$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME_RELEASE>,$<TARGET_PROPERTY:${__target},COMPILE_PDB_NAME>>.pdb"
                DESTINATION "${__dst}" CONFIGURATIONS Release
                COMPONENT ${__pdb_install_component} OPTIONAL ${__pdb_exclude_from_all})
          endif()
        else()
          message(WARNING "PDB files installation is not supported (need CMake >= 3.1.0)")
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

  if(APPLE_FRAMEWORK AND BUILD_SHARED_LIBS)
    message(STATUS "Setting Apple target properties for ${target}")

    set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG 1)

    if(IOS AND NOT MAC_CATALYST)
      set(VISP_APPLE_INFO_PLIST "${CMAKE_BINARY_DIR}/ios/Info.plist")
    else()
      set(VISP_APPLE_INFO_PLIST "${CMAKE_BINARY_DIR}/osx/Info.plist")
    endif()

    set_target_properties(${target} PROPERTIES
      FRAMEWORK TRUE
      MACOSX_FRAMEWORK_IDENTIFIER org.visp
      MACOSX_FRAMEWORK_INFO_PLIST ${VISP_APPLE_INFO_PLIST}
      # "current version" in semantic format in Mach-O binary file
      VERSION ${VISP_VERSION}
      # "compatibility version" in semantic format in Mach-O binary file
      SOVERSION ${VISP_VERSION}
      INSTALL_RPATH ""
      INSTALL_NAME_DIR "@rpath"
      BUILD_WITH_INSTALL_RPATH 1
      LIBRARY_OUTPUT_NAME "visp"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
    )
  endif()

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

# Macros that checks if a functions exists in libraries
# After it set vars:
#   HAVE_FUNC_<function>
include(CheckFunctionExists)
macro(VP_CHECK_FUNCTION_EXISTS function libraries)
  set(ALIAS                 ${function})
  string(TOUPPER "${ALIAS}" ALIAS_UPPER)
  set(ALIAS_HAVE            HAVE_FUNC_${ALIAS_UPPER})
  set(CMAKE_REQUIRED_LIBRARIES "${libraries}")
  check_function_exists(${ALIAS} ${ALIAS_HAVE})
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
  set(_fname "${ARGN}")
  if(NOT DEFINED ${RESULT})
    if(_fname)
      # nothing
    elseif("_${LANG}_" MATCHES "_CXX_")
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
      if(NOT "x${ARGN}" STREQUAL "x")
        file(RELATIVE_PATH __msg "${CMAKE_SOURCE_DIR}" "${ARGN}")
        set(__msg " (check file: ${__msg})")
      else()
        set(__msg "")
      endif()
      if(CMAKE_REQUIRED_LIBRARIES)
        set(__link_libs LINK_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
      else()
        set(__link_libs)
      endif()
      set(__cmake_flags "")
      if(CMAKE_EXE_LINKER_FLAGS)  # CMP0056 do this on new CMake
        list(APPEND __cmake_flags "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}")
      endif()

      # CMP0067 do this on new CMake
      if(DEFINED CMAKE_CXX_STANDARD)
        list(APPEND __cmake_flags "-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")
      endif()
      if(DEFINED CMAKE_CXX_STANDARD_REQUIRED)
        list(APPEND __cmake_flags "-DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}")
      endif()
      if(DEFINED CMAKE_CXX_EXTENSIONS)
        list(APPEND __cmake_flags "-DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}")
      endif()

      message(STATUS "Performing Test ${RESULT}${__msg}")
      try_compile(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        CMAKE_FLAGS ${__cmake_flags}
        COMPILE_DEFINITIONS "${FLAG}"
        ${__link_libs}
        OUTPUT_VARIABLE OUTPUT)

      if(${RESULT})
        string(REPLACE ";" "," OUTPUT_LINES "${OUTPUT}")
        string(REPLACE "\n" ";" OUTPUT_LINES "${OUTPUT_LINES}")
        foreach(_regex ${VP_COMPILER_FAIL_REGEX})
          if(NOT ${RESULT})
            break()
          endif()
          foreach(_line ${OUTPUT_LINES})
            if("${_line}" MATCHES "${_regex}")
              file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
                  "Build output check failed:\n"
                  "    Regex: '${_regex}'\n"
                  "    Output line: '${_line}'\n")
              set(${RESULT} 0)
              break()
            endif()
          endforeach()
        endforeach()
      endif()

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
macro(vp_check_flag_support lang flag varname base_options)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()

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
  string(REGEX REPLACE " -|-|=| |\\.|," "_" ${varname} "${${varname}}")
  if(DEFINED CMAKE_${_lang}_COMPILER)
    vp_check_compiler_flag("${_lang}" "${base_options} ${flag}" ${${varname}} ${ARGN})
  endif()
endmacro()

# turns off warnings
macro(vp_warnings_disable)
  if(ACTIVATE_WARNING_3PARTY_MUTE)
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
    elseif((CMAKE_COMPILER_IS_GNUCXX OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")) AND _gxx_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_gxx_warnings})
          if(NOT warning MATCHES "^-Wno-")
            string(REPLACE "${warning}" "" ${var} "${${var}}")
            string(REPLACE "-W" "-Wno-" warning "${warning}")
          endif()
          vp_check_flag_support(${var} "${warning}" _varname "")
          if(${_varname})
            set(${var} "${${var}} ${warning}")
          endif()
        endforeach()
      endforeach()
    endif()
    unset(_flag_vars)
    unset(_msvc_warnings)
    unset(_gxx_warnings)
  endif()
endmacro()

macro(vp_set_source_file_compile_flag file)
  if(ACTIVATE_WARNING_3PARTY_MUTE)
    set(__cxxflags "")
    # Set CXX as default language for cpp|cc|cxx|c file extension
    set(__lang "CXX")
    if("${file}" MATCHES "\\.(f|F)$")
      set(__lang "Fortran")
    endif()
    # Since cxx11 option makes try_compile() result wrong, we remove all the CXX_FLAGS
    # when we check if an option is available or not
    #set(CXX_FLAGS_BACKUP ${CMAKE_CXX_FLAGS})
    #set(CMAKE_CXX_FLAGS "")
    foreach(cxxflag ${ARGN})
      vp_check_flag_support(${__lang} ${cxxflag} __support_flag "")
      if(${__support_flag})
        set(__cxxflags "${__cxxflags} ${cxxflag}")
      endif()
    endforeach()
    #set(CMAKE_CXX_FLAGS ${CXX_FLAGS_BACKUP})
    #unset(CXX_FLAGS_BACKUP)
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
            # Add subdir only if ut doesn't exist
            if(NOT EXISTS "${CMAKE_BINARY_DIR}/${subdir}/${__s}")
              add_subdirectory("${__path}/${subdir}/${__s}" "${CMAKE_BINARY_DIR}/${subdir}/${__s}")
            endif()
          endif()
        endforeach()
      endif()
    endforeach()
  endif()
endmacro()

macro(vp_check_subdirectories lst subdir result)
  set(${result} FALSE)
  if(${lst})
    foreach(__path ${${lst}})
      if(EXISTS ${__path}/${subdir})
        file(GLOB __subdirs RELATIVE "${__path}/${subdir}" "${__path}/${subdir}/*")
        foreach(__s ${__subdirs})
          if(EXISTS "${__path}/${subdir}/${__s}/CMakeLists.txt")
            set(${result} TRUE)
          endif()
        endforeach()
      endif()
    endforeach()
  endif()
endmacro()

set(VISP_BUILD_INFO_STR "" CACHE INTERNAL "")
set(VISP_TXT_INFO_STR "" CACHE INTERNAL "")
function(vp_output_status msg)
  message(STATUS "${msg}")
  string(REPLACE "\\" "\\\\" msg "${msg}")
  string(REPLACE "\"" "\\\"" msg "${msg}")
  string(REPLACE "\\\\" "\\" msg_txt "${msg}")
  set(VISP_BUILD_INFO_STR "${VISP_BUILD_INFO_STR}\"${msg}\\n\"\n" CACHE INTERNAL "")
  set(VISP_TXT_INFO_STR "${VISP_TXT_INFO_STR}${msg_txt}\n" CACHE INTERNAL "")
endfunction()

macro(vp_finalize_status)
  set(VISP_BUILD_INFO_FILE "${CMAKE_BINARY_DIR}/visp-build-info.tmp")
  set(VISP_TXT_INFO_FILE "${CMAKE_BINARY_DIR}/ViSP-third-party.txt")

  if(EXISTS "${VISP_BUILD_INFO_FILE}")
    file(READ "${VISP_BUILD_INFO_FILE}" __content_build_info)
  else()
    set(__content_build_info "")
  endif()
  if("${__content_build_info}" STREQUAL "${VISP_BUILD_INFO_STR}")
    #message(STATUS "${VISP_BUILD_INFO_FILE} contains the same content")
  else()
    file(WRITE "${VISP_BUILD_INFO_FILE}" "${VISP_BUILD_INFO_STR}")
  endif()
  unset(__content_build_info)
  unset(VISP_BUILD_INFO_STR CACHE)

  if(EXISTS "${VISP_TXT_INFO_FILE}")
    file(READ "${VISP_TXT_INFO_FILE}" __content_txt_info)
  else()
    set(__content_txt_info "")
  endif()
  if("${__content_txt_info}" STREQUAL "${VISP_TXT_INFO_STR}")
    #message(STATUS "${VISP_TXT_INFO_FILE} contains the same content")
  else()
    file(WRITE "${VISP_TXT_INFO_FILE}" "${VISP_TXT_INFO_STR}")
  endif()
  unset(__content_txt_info)
  unset(VISP_TXT_INFO_STR CACHE)

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
  set(__parent_scope OFF)
  set(__add_cache OFF)
  foreach(name ${ARGN})
    if(${name} STREQUAL "PARENT_SCOPE")
      set(__parent_scope ON)
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
        elseif(__parent_scope)
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

# parse header file to get library version
# the REGEX matches a line that starts with #define + DEFINE_NAME + "(name) 0.0.0(.dev)"
# with optional (name) and (.dev)
# e.g. #define OPENBLAS_VERSION " OpenBLAS 0.3.12.dev "
# e.g. #define PNG_LIBPNG_VER_STRING "1.5.10"
macro(vp_parse_header3 LIBNAME HDR_PATH DEFINE_NAME OUTPUT_VAR)
  foreach(name ${ARGN})
    if(${name} STREQUAL "PARENT_SCOPE")
      set(__parent_scope ON)
    endif()
  endforeach()

  if(EXISTS "${HDR_PATH}")
    file(STRINGS "${HDR_PATH}" line_to_parse REGEX "^#define[ \t]+${DEFINE_NAME}[ \t]+\"[ \t]*[^0-9]*([0-9]+\\.[0-9]+\\.[0-9]+(\\.[^0-9 \t]+)?)[ \t]*\"$" LIMIT_COUNT 1)
    string(REGEX REPLACE "^#define[ \t]+${DEFINE_NAME}[ \t]+\"[ \t]*[^0-9]*([0-9]+\\.[0-9]+\\.[0-9]+(\\.[^0-9 \t]+)?)[ \t]*\"$" "\\1" lib_version "${line_to_parse}")

    if(__parent_scope)
      set(${OUTPUT_VAR} ${lib_version} PARENT_SCOPE)
    else()
      set(${OUTPUT_VAR} ${lib_version})
    endif()
  endif()
endmacro()

# parse header file to get library version
# the REGEX matches a line that starts with #define + DEFINE_NAME + name(0, 0, 0)"
# with optional (name) and (.dev)
# e.g. #   define UEYE_VERSION_CODE   UEYE_VERSION(4, 93, 0)
macro(vp_parse_header4 LIBNAME HDR_PATH DEFINE_NAME OUTPUT_VAR)
  if(EXISTS "${HDR_PATH}")
    file(STRINGS "${HDR_PATH}" line_to_parse REGEX "^#[ \t]*define[ \t]+${DEFINE_NAME}[ \t]+[^0-9]*[(]([0-9]+),[ \t]*([0-9]+),[ \t]*([0-9]+)[)]$" LIMIT_COUNT 1)
    string(REGEX REPLACE "^#[ \t]*define[ \t]+${DEFINE_NAME}[ \t]+[^0-9]*[(]([0-9]+),[ \t]*([0-9]+),[ \t]*([0-9]+)[)]$" "\\1.\\2.\\3" ${OUTPUT_VAR} "${line_to_parse}" )
  endif()
endmacro()

# Get package version from pkg-config
macro(vp_get_version_from_pkg LIBNAME PKG_PATH OUTPUT_VAR)
  if(EXISTS "${PKG_PATH}/${LIBNAME}.pc")
    # Consider the case where pkg-config is not installed
    file(STRINGS "${PKG_PATH}/${LIBNAME}.pc" line_to_parse REGEX "^Version:[ \t]+[0-9.]*.*$" LIMIT_COUNT 1)
    string(REGEX REPLACE ".*Version: ([^ ]+).*" "\\1" ${OUTPUT_VAR} "${line_to_parse}" )
  else()
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
      string(TOUPPER ${LIBNAME} LIBNAME_UPPER)
      pkg_get_variable(${LIBNAME_UPPER}_PCFILEDIR ${LIBNAME} pcfiledir)
      if(EXISTS "${${LIBNAME_UPPER}_PCFILEDIR}/${LIBNAME}.pc")
        file(STRINGS "${${LIBNAME_UPPER}_PCFILEDIR}/${LIBNAME}.pc" line_to_parse REGEX "^Version:[ \t]+[0-9.]*.*$" LIMIT_COUNT 1)
        string(REGEX REPLACE ".*Version: ([^ ]+).*" "\\1" ${OUTPUT_VAR} "${line_to_parse}" )
        unset(LIBNAME_UPPER)
        mark_as_advanced(${LIBNAME_UPPER}_PCFILEDIR)
      endif()
    endif()
  endif()
endmacro()

macro(vp_cmake_script_append_var content_var)
  foreach(var_name ${ARGN})
    set(${content_var} "${${content_var}}
set(${var_name} \"${${var_name}}\")
")
  endforeach()
endmacro()

# files: A list of input files like headers (in)
# paths: A list of corresponding paths (out)
# usage: vp_find_path(type.h paths PATHS /usr/local)
macro(vp_find_path files paths)
  set(__paths "")
  foreach(arg_ ${ARGN})
    if("${arg_}" STREQUAL "PATHS")
      set(__varname "__paths")
    else()
      list(APPEND ${__varname} ${arg_})
    endif()
  endforeach()
  unset(__varname)
  foreach(f_ ${${files}})
    find_path(path_ ${f_} PATHS ${__paths})
    if(path_)
      list(APPEND ${paths} ${path_})
    endif()
    unset(path_ CACHE)
  endforeach()
  vp_list_unique(${paths})
endmacro()

# A list of libs
macro(vp_get_interface_link_libraries libs link_libraries)
  set(__imported_libs ${${libs}})
  set(__libs ${${libs}})
  foreach(lib_ ${${libs}})
#    message("lib_: ${lib_}")
    if(TARGET ${lib_})
      get_target_property(imported_libs_ ${lib_} INTERFACE_LINK_LIBRARIES)
      if(imported_libs_)
        list(APPEND __imported_libs ${imported_libs_})
        list(APPEND __libs ${lib_})
      else()
        list(APPEND __libs ${lib_})
      endif()
    else()
      list(APPEND __libs ${lib_})
    endif()
  endforeach()
#  message("__imported_libs: ${__imported_libs}")
#  message("__libs: ${__libs}")
  vp_list_unique(__imported_libs)
  vp_list_unique(__libs)
#  message("fin __imported_libs: ${__imported_libs}")
#  message("fin __libs: ${__libs}")

  while(__imported_libs)
#    message("begin while __imported_libs: ${__imported_libs}")
    vp_list_pop_front(__imported_libs elt)
#    message("Process elt: ${elt}")
    if(TARGET ${elt} AND NOT elt MATCHES "^-framework") # to avoid precessing -framework ApplicationServices -framework CoreServices
#      message("elt is a target and not framework: ${elt}")
      get_target_property(imported_libs_ ${elt} INTERFACE_LINK_LIBRARIES)
      if(imported_libs_)
        list(APPEND __imported_libs ${imported_libs_})
      else()
        list(APPEND __libs ${elt})
      endif()
    else()
      list(APPEND __libs ${elt})
    endif()
    vp_list_unique(__imported_libs)
  endwhile()
  vp_list_unique(__libs)
#  message("fin2 __imported_libs: ${__imported_libs}")
#  message("fin2 __libs: ${__libs}")

  set(__config "RELEASE" "DEBUG")
  foreach(config_ ${__config})
    foreach(lib_ ${__libs})
#      message("lib_: ${lib_}")
      if(TARGET ${lib_})

        get_target_property(imported_libs_ ${lib_} IMPORTED_IMPLIB_${config_})
        if(NOT EXISTS ${imported_libs_})
          get_target_property(lib_location_ ${lib_} IMPORTED_LOCATION_${config_})
        endif()
  #      message("lib_location_: ${lib_location_}")
        if(WIN32 AND EXISTS "${lib_location_}" AND "${config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
          list(APPEND ${link_libraries} optimized "${lib_location_}")
        elseif(WIN32 AND EXISTS "${lib_location_}" AND "${config_}" MATCHES "DEBUG")
          list(APPEND ${link_libraries} debug     "${lib_location_}")
        elseif(EXISTS ${lib_location_})
          list(APPEND ${link_libraries} ${lib_location_})
        endif()

        get_target_property(lib_deps_ ${lib_} IMPORTED_LINK_INTERFACE_LIBRARIES_${config_})
  #      message("lib_deps_ ---------: ${lib_deps_}")
        if(lib_deps_)
          foreach(deps_ ${lib_deps_})
            get_target_property(deps_location_ ${deps_} IMPORTED_LOCATION_${config_})
            if(EXISTS "${deps_location_}")
              if(WIN32 AND "${config_}" MATCHES "RELEASE")
                list(APPEND ${link_libraries} optimized ${deps_location_})
              elseif(WIN32 AND "${config_}" MATCHES "DEBUG")
                list(APPEND ${link_libraries} debug ${deps_location_})
              else()
                list(APPEND ${link_libraries} ${deps_location_})
              endif()
            endif()
          endforeach()
        endif()
      else()
        # VTK_LIBRARIES does import /Library/Developer/CommandLineTools/SDKs/MacOSX13.sdk/usr/lib/libz.tbd
        # that doesn't exist and that does lied to a link error when building ustk_gui module:
        #   No rule to make target `/Library/Developer/CommandLineTools/SDKs/MacOSX13.sdk/usr/lib/libz.tbd',
        #   needed by `lib/libvisp_ustk_gui.3.6.1.dylib'.  Stop.
        # Here we introduce an additional check to ensure that libraries suffixed by .tbd exist
        if(${lib_} MATCHES "/([^/]+)\\.tbd$")
          if(EXISTS ${lib_})
            # Add only if .tbd file exists
            list(APPEND ${link_libraries} ${lib_})
          endif()
        else()
          list(APPEND ${link_libraries} ${lib_})
        endif()
      endif()

    endforeach()
  endforeach()
  vp_list_unique(${link_libraries})
#  message("link_libraries: ${link_libraries}")

endmacro()

# Concatenate in_file to out_file
function(vp_cat_file in_file out_file)
  file(READ ${in_file} CONTENTS)
  file(APPEND ${out_file} "${CONTENTS}")
endfunction()

# Extract library name without lib prefix and extension suffix
# UNIX  : libvisp_core.so.3.3 -> visp_core
# UNIX  : libz.so -> z
# UNIX  : libvtkRenderingOpenGL2-7.1.so.7.1p.1 -> vtkRenderingOpenGL2-7.1
# MacOSX: libvisp_core-3.3.1.dylib -> visp_core-3.3.1
# MacOSX: libz.dylib -> z
macro(vp_get_libname var_name)
  get_filename_component(__libname "${ARGN}" NAME)
  string(REGEX REPLACE "^lib(.*)\\.(a|so|dll)(\\.[.0-9a-z]+)?$" "\\1" __libname "${__libname}")
  string(REGEX REPLACE "^lib(.*[^.])(\\.[0-9]+\\.)?.dylib$" "\\1" __libname "${__libname}")
  set(${var_name} "${__libname}")
endmacro()

# Extract framework name to use with -framework name or path to use with -F path
# /usr/local/opt/python/Frameworks/Python.framework/Versions/3.7/Python
#   NAME: Python
#   PATH: /usr/local/opt/python/Frameworks
# /usr/local/opt/qt/lib/QtWidgets.framework/QtWidgets
#   NAME: QtWidgets
#   PATH: /usr/local/opt/qt/lib
# /Library/Frameworks/pylon.framework
#   NAME: pylon
#   PATH: /Library/Frameworks
# /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/System/Library/Frameworks/OpenGL.framework
#   NAME: OpenGL
#   PATH: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/System/Library/Frameworks
macro(vp_get_framework var_name)
  set(__option)
  foreach(d ${ARGN})
    if(d MATCHES "PATH")
      set(__option "PATH")
    elseif(d MATCHES "NAME")
      set(__option "NAME")
    else()
      set(__framework1 "${d}")
    endif()
  endforeach()

  if (__option MATCHES "PATH")
    if (__framework1 MATCHES ".framework/")
      string(REGEX REPLACE "(.+)?/(.+).framework/(.+)$" "\\1" __framework "${__framework1}")
    elseif(__framework1 MATCHES "/([^/]+)\\.framework$")
      string(REGEX REPLACE "(.+)?/(.+)\\.framework$" "\\1" __framework "${__framework1}")
    endif()
  elseif(__option MATCHES "NAME") # Default NAME
    if (__framework1 MATCHES ".framework/")
      string(REGEX REPLACE "(.+)?/(.+).framework/(.+)$" "\\2" __framework "${__framework1}")
    elseif(__framework1 MATCHES "/([^/]+)\\.framework$")
      string(REGEX REPLACE "(.+)?/(.+)\\.framework$" "\\2" __framework "${__framework1}")
    endif()
  else()
    set(__framework1 "${d}")
  endif()
  set(${var_name} "${__framework}")
endmacro()

# Extract tbd name to use with -lname or path to use with -Lpath
# /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib/libz.tbd
#   NAME: z
#   PATH: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib
# /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib/libpthread.tbd
#   NAME: pthread
#   PATH: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib
# /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib/libxml2.tbd
#   NAME: xml2
#   PATH: /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/usr/lib
macro(vp_get_tbd var_name)
  set(__option)
  foreach(d ${ARGN})
    if(d MATCHES "PATH")
      set(__option "PATH")
    elseif(d MATCHES "NAME")
      set(__option "NAME")
    else()
      set(__tbd1 "${d}")
    endif()
  endforeach()

  if (__option MATCHES "PATH")
    string(REGEX REPLACE "(.+)?/lib(.+)\\.tbd$" "\\1" __tbd "${__tbd1}")
  elseif(__option MATCHES "NAME") # Default NAME
    string(REGEX REPLACE "(.+)?/lib(.+)\\.tbd$" "\\2" __tbd "${__tbd1}")
  else()
    set(__tbd1 "${d}")
  endif()
  set(${var_name} "${__tbd}")
endmacro()

# build the list of visp cxx flags to propagate (OpenMP, CXX11) in scripts
#  _cxx_flags : variable to hold list of cxx flags we depend on
macro(vp_get_all_cflags _cxx_flags)
  set(${_cxx_flags} "")

  if(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    list(INSERT ${_cxx_flags} 0 "-ftest-coverage")
    list(INSERT ${_cxx_flags} 0 "-fprofile-arcs")
  endif()

  # Propagate c++ standard compiler option if enabled during ViSP build
  if((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_11) AND CXX11_CXX_FLAGS)
    list(INSERT ${_cxx_flags} 0  ${CXX11_CXX_FLAGS})
  elseif((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_14) AND CXX14_CXX_FLAGS)
    list(INSERT ${_cxx_flags} 0  ${CXX14_CXX_FLAGS})
  elseif((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_17) AND CXX17_CXX_FLAGS)
    list(INSERT ${_cxx_flags} 0  ${CXX17_CXX_FLAGS})
  endif()

  # Propagate openmp compiler option if enabled during ViSP build
  if(VISP_HAVE_OPENMP)
    list(INSERT ${_cxx_flags} 0  ${OpenMP_CXX_FLAGS})
  endif()
endmacro()

# build the list of visp includes for all modules and dependencies
#  _include_modules : variable to hold list of includes for all modules
#  _include_extra : variable to hold list of includes for extra dependencies
macro(vp_get_all_includes _includes_modules _includes_extra _system_include_dirs)
  set(${_includes_modules} "")
  set(${_includes_extra} "")
  set(${_system_include_dirs} "")

  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND ${_includes_extra} ${VISP_MODULE_${m}_INC_DEPS})
    if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
      list(INSERT ${_includes_modules} 0 "${VISP_MODULE_${m}_LOCATION}/include")
    endif()
  endforeach()

  #---------------------------------------------------------------------
  # Get system_include_dirs to propagate in scripts for SonarQube
  #----------------------------------------------------------------------
  list(APPEND _system_include_dirs ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})

  foreach(lst ${_includes_modules} ${_includes_extra} ${_system_include_dirs})
    vp_list_unique(${lst})
  endforeach()
endmacro()

# build the list of visp libs and dependencies for all modules
#  _modules : variable to hold list of all modules
#    When linking against static libraries, if libfoo depends on libbar, then
#    libfoo must come first in the linker flags. This is done
#    in _module where modules are reordered.
#  _extra_opt : variable to hold list of extra dependencies (default)
#  _extra_dbg : variable to hold list of extra dependencies (debug configuration)
#  _3rdparty : variable to hold list of prebuilt 3rdparty libraries
macro(vp_get_all_libs _modules _extra_opt _extra_dbg _3rdparty)
  set(${_modules} "")
  set(${_extra_dbg} "")
  set(${_extra_opt} "")
  set(${_3rdparty} "")
  foreach(m ${VISP_MODULES_PUBLIC})
    if(TARGET ${m})
      get_target_property(deps ${m} INTERFACE_LINK_LIBRARIES)
      if(NOT deps)
        set(deps "")
      endif()
    else()
      set(deps "")
    endif()
    set(_rev_deps "${deps};${m}")
    vp_list_reverse(_rev_deps)
    foreach (dep ${_rev_deps})
      if(DEFINED VISP_MODULE_${dep}_LOCATION)
        list(INSERT ${_modules} 0 ${dep})
      endif()
    endforeach()

    foreach (dep ${deps}) # Should be remove ? ${VISP_LINKER_LIBS})
      if (NOT DEFINED VISP_MODULE_${dep}_LOCATION)
        if(dep MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
          set(dep_dbg "${CMAKE_MATCH_1}")
          set(dep_opt "${CMAKE_MATCH_1}")
        elseif(dep MATCHES "^\\$<\\$<CONFIG:DEBUG>:([^>]+)>$")
          set(dep_dbg "${CMAKE_MATCH_1}")
        elseif(dep MATCHES "^\\$<\\$<NOT:\\$<CONFIG:DEBUG>>:([^>]+)>$")
          set(dep_opt "${CMAKE_MATCH_1}")
        elseif(dep MATCHES "^\\$<")
          #message(WARNING "Unexpected CMake generator expression: ${dep}")
        else()
          set(dep_dbg ${dep})
          set(dep_opt ${dep})
        endif()
        if (TARGET ${dep_opt})
          get_target_property(_type ${dep_opt} TYPE)
          if((_type STREQUAL "STATIC_LIBRARY" AND BUILD_SHARED_LIBS)
              OR _type STREQUAL "INTERFACE_LIBRARY"
              OR DEFINED VISP_MODULE_${dep_opt}_LOCATION  # ViSP modules
          )
            # nothing
          else()
            get_target_property(_output ${dep_opt} IMPORTED_LOCATION)
            if(NOT _output)
              get_target_property(_output ${dep_opt} ARCHIVE_OUTPUT_DIRECTORY)
              get_target_property(_output_name ${dep_opt} OUTPUT_NAME)
              if(NOT _output_name)
                set(_output_name "${dep_opt}")
              endif()
            else()
              get_filename_component(_output_name "${_output}" NAME)
            endif()
            string(FIND "${_output}" "${CMAKE_BINARY_DIR}" _POS)
            if (_POS EQUAL 0)
              vp_get_libname(_libname "${_output_name}")
              list(INSERT ${_3rdparty} 0 ${dep_opt})
            else()
              if(_output)
                list(INSERT ${_extra_opt} 0 ${_output})
              else()
                list(INSERT ${_extra_opt} 0 ${dep_opt})
              endif()
            endif()
          endif()
        elseif(NOT "${dep_opt}" STREQUAL "") # avoid empty string insertion
          list(INSERT ${_extra_opt} 0 ${dep_opt})
        endif()
        if (TARGET ${dep_dbg})
          get_target_property(_type ${dep_dbg} TYPE)
          if((_type STREQUAL "STATIC_LIBRARY" AND BUILD_SHARED_LIBS)
              OR _type STREQUAL "INTERFACE_LIBRARY"
              OR DEFINED VISP_MODULE_${dep_dbg}_LOCATION  # ViSP modules
          )
            # nothing
          else()
            get_target_property(_output ${dep_dbg} IMPORTED_LOCATION)
            if(NOT _output)
              get_target_property(_output ${dep_dbg} ARCHIVE_OUTPUT_DIRECTORY)
              get_target_property(_output_name ${dep_dbg} OUTPUT_NAME)
              if(NOT _output_name)
                set(_output_name "${dep_dbg}")
              endif()
            else()
              get_filename_component(_output_name "${_output}" NAME)
            endif()
            string(FIND "${_output}" "${CMAKE_BINARY_DIR}" _POS)
            if (_POS EQUAL 0)
              vp_get_libname(_libname "${_output_name}")
              list(INSERT ${_3rdparty} 0 ${dep_dbg})
            else()
              if(_output)
                list(INSERT ${_extra_dbg} 0 ${_output})
              else()
                list(INSERT ${_extra_dbg} 0 ${dep_dbg})
              endif()
            endif()
          endif()
        elseif(NOT "${dep_dbg}" STREQUAL "") # avoid empty string insertion
          list(INSERT ${_extra_dbg} 0 ${dep_dbg})
        endif()
      endif()
    endforeach()
  endforeach()

  vp_list_filterout(${_modules} "^[\$]<")
  vp_list_filterout(${_3rdparty} "^[\$]<")
  vp_list_filterout(${_extra_opt} "^[\$]<")
  vp_list_filterout(${_extra_dbg} "^[\$]<")

  # convert CMake lists to makefile literals
  foreach(lst ${_modules} ${_3rdparty} ${_extra_opt} ${_extra_dbg})
    vp_list_unique(${lst})
    vp_list_reverse(${lst})
  endforeach()
endmacro()

# Filter libraries.
# When a library is a target like Boost::thread or Boost::date_time
# replace the target by its imported location /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
macro(vp_filter_libraries_with_imported_location libs)
  set(__libs)
  foreach(lib_ ${${libs}})
    if(TARGET ${lib_})
      get_target_property(imported_libs_ ${lib_} INPORTED_LOCATION)
      if(imported_libs_)
        list(APPEND __libs ${imported_libs_})
      endif()
    else()
      list(APPEND __libs ${lib_})
    endif()
  endforeach()
  set(${libs} ${__libs})
endmacro()

function(vp_find_dataset found location version major minor patch)
  set(file_to_test "mbt/cube.cao")
  set(_found FALSE)
  set(_location "")
  set(_version "")
  set(_major "")
  set(_minor "")
  set(_patch "")

  if(DEFINED ENV{VISP_INPUT_IMAGE_PATH})
    if(EXISTS "$ENV{VISP_INPUT_IMAGE_PATH}/${file_to_test}")
      set(_location "$ENV{VISP_INPUT_IMAGE_PATH}")
      set(_found TRUE)
    elseif(EXISTS "$ENV{VISP_INPUT_IMAGE_PATH}/ViSP-images/${file_to_test}")
      set(_location "$ENV{VISP_INPUT_IMAGE_PATH}/ViSP-images")
      set(_found TRUE)
    elseif(EXISTS "$ENV{VISP_INPUT_IMAGE_PATH}/visp-images/${file_to_test}")
      set(_location "$ENV{VISP_INPUT_IMAGE_PATH}/visp-images")
      set(_found TRUE)
    endif()
  endif()

  if(NOT _found)
    if(EXISTS "/usr/share/visp-images-data/ViSP-images/${file_to_test}")
      set(_location "/usr/share/visp-images-data/ViSP-images")
      set(_found TRUE)
    elseif(EXISTS "/usr/share/visp-images-data/visp-images/${file_to_test}")
      set(_location "/usr/share/visp-images-data/visp-images")
      set(_found TRUE)
    endif()
  endif()

  # Check version
  if(_found)
    if(EXISTS "${_location}/circle/circle.png")
      set(_major "3")
      set(_minor "6")
      set(_patch "0")
    elseif(EXISTS "${_location}/Solvay/Solvay_conference_1927_Version2_1024x705.jpg")
      set(_major "3")
      set(_minor "5")
      set(_patch "0")
    elseif(EXISTS "${_location}/Gaussian-filter/Klimt_RGB_Gaussian_blur_sigma=0.5.png")
      set(_major "3")
      set(_minor "4")
      set(_patch "0")
    elseif(EXISTS "${_location}/faces/1280px-Solvay_conference_1927.png")
      set(_major "3")
      set(_minor "3")
      set(_patch "0")
    elseif(EXISTS "${_location}/endianness/test_endianness_little_endian.bin")
      set(_major "3")
      set(_minor "2")
      set(_patch "0")
    else()
      set(_major "3")
      set(_minor "1")
      set(_patch "0")
    endif()
    set(_version "${_major}.${_minor}.${_patch}")
  endif()

  # Export return values
  set(${found} "${_found}" CACHE INTERNAL "")
  set(${location} "${_location}" CACHE INTERNAL "")
  set(${version} "${_version}" CACHE INTERNAL "")
  set(${major} "${_major}" CACHE INTERNAL "")
  set(${minor} "${_minor}" CACHE INTERNAL "")
  set(${patch} "${_patch}" CACHE INTERNAL "")
endfunction(vp_find_dataset)

# Considering an interface like MAVSDK::mavsdk extract the interface include dirs and libraries
function(vp_get_external_target name inc_dirs libs)
  if(inc_dirs)
    get_target_property(_inc_dirs ${name} INTERFACE_INCLUDE_DIRECTORIES)
    if(_inc_dirs)
      vp_list_unique(_inc_dirs)
      set(${inc_dirs} "${_inc_dirs}" CACHE INTERNAL "")
    endif()
  endif()
  if(libs)
    get_target_property(_cfgs ${name} IMPORTED_CONFIGURATIONS)
    get_target_property(_link_libs ${name} INTERFACE_LINK_LIBRARIES)
    if(_cfgs)
      foreach(_cfg ${_cfgs})
        string(TOUPPER "${_cfg}" _cfg_up)
        get_target_property(_imp_lib ${name} IMPORTED_LOCATION_${_cfg_up})
        if(_imp_lib)
          vp_list_unique(_imp_lib)
          set(${libs} "${_imp_lib}" CACHE INTERNAL "")
        endif()
      endforeach()
    endif()
    if(_link_libs)
      vp_list_unique(_link_libs)
      set(${libs} "${${libs}};${_link_libs}" CACHE INTERNAL "")
    endif()
  endif()
endfunction()

# Check for available cpu optimization
macro(vp_check_cpu_optimization cpu_optim)
  set(__cpu_optim)
  vp_check_compiler_flag(CXX "" HAVE_SSE2    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
  vp_check_compiler_flag(CXX "" HAVE_SSE3    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse3.cpp")
  vp_check_compiler_flag(CXX "" HAVE_SSSE3   "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_ssse3.cpp")
  vp_check_compiler_flag(CXX "" HAVE_SSE4_1  "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse41.cpp")
  vp_check_compiler_flag(CXX "" HAVE_SSE4_2  "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse42.cpp")
  vp_check_compiler_flag(CXX "" HAVE_AVX     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
  vp_check_compiler_flag(CXX "" HAVE_AVX2    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")
  vp_check_compiler_flag(CXX "" HAVE_NEON    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_neon.cpp")

  if(HAVE_SSE2)
    list(APPEND __cpu_optim "SSE2")
  endif()
  if(HAVE_SSE3)
    list(APPEND __cpu_optim "SSE3")
  endif()
  if(HAVE_SSSE3)
    list(APPEND __cpu_optim "SSSE3")
  endif()
  if(HAVE_SSE4_1)
    list(APPEND __cpu_optim "SSE4_1")
  endif()
  if(HAVE_SSE4_2)
    list(APPEND __cpu_optim "SSE4_2")
  endif()
  if(HAVE_NEON)
    list(APPEND __cpu_optim "NEON")
  endif()
  vp_list_unique(__cpu_optim)
  vp_list_remove_separator(__cpu_optim)
  set(${cpu_optim} ${__cpu_optim})
endmacro()

macro(vp_system_information NUMBER_OF_LOGICAL_CORES NUMBER_OF_PHYSICAL_CORES TOTAL_PHYSICAL_MEMORY OS_NAME OS_RELEASE OS_VERSION OS_PLATFORM PROCESSOR_NAME IS_64BIT HAS_FPU CPU_OPTIM)
  set(__NUMBER_OF_LOGICAL_CORES)
  set(__NUMBER_OF_PHYSICAL_CORES)
  set(__TOTAL_PHYSICAL_MEMORY)
  set(__OS_NAME)
  set(__OS_RELEASE)
  set(__OS_VERSION)
  set(__OS_PLATFORM)
  set(__PROCESSOR_NAME)
  set(__IS_64BIT)
  set(__HAS_FPU)
  set(__CPU_OPTIM)

  if(CMAKE_VERSION VERSION_GREATER 3.10)
    cmake_host_system_information(RESULT SYS_INFO QUERY NUMBER_OF_LOGICAL_CORES NUMBER_OF_PHYSICAL_CORES TOTAL_PHYSICAL_MEMORY OS_NAME OS_RELEASE OS_VERSION OS_PLATFORM PROCESSOR_NAME IS_64BIT HAS_FPU)
  else()
    cmake_host_system_information(RESULT SYS_INFO QUERY NUMBER_OF_LOGICAL_CORES NUMBER_OF_PHYSICAL_CORES TOTAL_PHYSICAL_MEMORY)
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
    list(APPEND SYS_INFO "N/A")
  endif()

  list(GET SYS_INFO 0 __NUMBER_OF_LOGICAL_CORES)
  list(GET SYS_INFO 1 __NUMBER_OF_PHYSICAL_CORES)
  list(GET SYS_INFO 2 __TOTAL_PHYSICAL_MEMORY)
  list(GET SYS_INFO 3 __OS_NAME)
  list(GET SYS_INFO 4 __OS_RELEASE)
  list(GET SYS_INFO 5 __OS_VERSION)
  list(GET SYS_INFO 6 __OS_PLATFORM)
  list(GET SYS_INFO 7 __PROCESSOR_NAME)
  list(GET SYS_INFO 8 __IS_64BIT)
  list(GET SYS_INFO 9 __HAS_FPU)

  vp_check_cpu_optimization(__CPU_OPTIM)
  set(${NUMBER_OF_LOGICAL_CORES} ${__NUMBER_OF_LOGICAL_CORES})
  set(${NUMBER_OF_PHYSICAL_CORES} ${__NUMBER_OF_PHYSICAL_CORES})
  set(${TOTAL_PHYSICAL_MEMORY} ${__TOTAL_PHYSICAL_MEMORY})
  set(${OS_NAME} ${__OS_NAME})
  set(${OS_RELEASE} ${__OS_RELEASE})
  set(${OS_VERSION} ${__OS_VERSION})
  set(${OS_PLATFORM} ${__OS_PLATFORM})
  set(${PROCESSOR_NAME} ${__PROCESSOR_NAME})
  set(${IS_64BIT} ${__IS_64BIT})
  set(${HAS_FPU} ${__HAS_FPU})
  set(${CPU_OPTIM} ${__CPU_OPTIM})
endmacro()

# Replace regular expression in a var
macro(vp_replace_string var_in var_out regular_expression replacement_expression)
  set(__var_out ${var_out})
  if(${var_in} MATCHES "${regular_expression}")
    string(REGEX REPLACE "${regular_expression}" "${replacement_expression}" ${var_out} ${${var_in}})
  else()
    set(${__var_out} ${${var_in}})
  endif()
endmacro()

macro(vp_list_replace_string list_in list_out regular_expression replacement_expression)
  set(__list_out ${var_out})
  foreach(item ${${list_in}})
    if(item MATCHES "${regular_expression}")
      string(REGEX REPLACE "${regular_expression}" "${replacement_expression}" var_out ${item})
    else()
      set(var_out ${item})
    endif()
    list(APPEND __list_out ${var_out})
  endforeach()
  set(${list_out} ${__list_out})
endmacro()

macro(vp_git_describe var_name path)
  if(GIT_FOUND)
    execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --exact-match --dirty
      WORKING_DIRECTORY "${path}"
      OUTPUT_VARIABLE ${var_name}
      RESULT_VARIABLE GIT_RESULT
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT GIT_RESULT EQUAL 0)
      execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "v[0-9].[0-9].[0-9]*"
        WORKING_DIRECTORY "${path}"
        OUTPUT_VARIABLE ${var_name}
        RESULT_VARIABLE GIT_RESULT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    endif()
  else()
    set(${var_name} "unknown")
  endif()
endmacro()
