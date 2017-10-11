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

# Local variables (set for each module):
#
# name       - short name in lower case i.e. core
# the_module - full name in lower case i.e. visp_core

# Global variables:
#
# VISP_MODULE_${the_module}_LOCATION
# VISP_MODULE_${the_module}_BINARY_DIR
# VISP_MODULE_${the_module}_DESCRIPTION
# VISP_MODULE_${the_module}_CLASS - PUBLIC|INTERNAL|BINDINGS
# VISP_MODULE_${the_module}_HEADERS
# VISP_MODULE_${the_module}_SOURCES
# VISP_MODULE_${the_module}_DEPS - final flattened set of module dependencies
# VISP_MODULE_${the_module}_DEPS_TO_LINK - differs from above for world build only
# VISP_MODULE_${the_module}_DEPS_EXT - non-module dependencies
# VISP_MODULE_${the_module}_REQ_DEPS
# VISP_MODULE_${the_module}_OPT_DEPS
# VISP_MODULE_${the_module}_PRIVATE_REQ_DEPS - private module deps that are not exposed in interface
# VISP_MODULE_${the_module}_PRIVATE_OPT_DEPS - private module deps that are not exposed in interface
# VISP_MODULE_${the_module}_CHILDREN - list of submodules for compound modules (cmake >= 2.8.8)
# HAVE_${the_module} - for fast check of module availability

# To control the setup of the module you could also set:
# the_description - text to be used as current module description
# VISP_MODULE_TYPE - STATIC|SHARED - set to force override global settings for current module
# BUILD_${the_module}_INIT - ON|OFF (default ON) - initial value for BUILD_${the_module}
# VISP_MODULE_CHILDREN - list of submodules

# The verbose template for ViSP module:
#
#   vp_add_module(modname <dependencies>)
#   vp_glob_module_sources((<extra sources&headers>)
#                          or glob them manually and vp_set_module_sources(...)
#   vp_module_include_directories(<extra include directories>)
#   vp_create_module()
#
# If module have no "extra" then you can define it in one line:
#
#   vp_define_module(modname <dependencies>)

# clean flags for modules enabled on previous cmake run
# this is necessary to correctly handle modules removal
foreach(mod ${VISP_MODULES_BUILD} ${VISP_MODULES_DISABLED_USER} ${VISP_MODULES_DISABLED_AUTO} ${VISP_MODULES_DISABLED_FORCE})
  if(HAVE_${mod})
    unset(HAVE_${mod} CACHE)
  endif()
  unset(VISP_MODULE_${mod}_DEPS CACHE)
  unset(VISP_MODULE_${mod}_DEPS_EXT CACHE)
  unset(VISP_MODULE_${mod}_REQ_DEPS CACHE)
  unset(VISP_MODULE_${mod}_OPT_DEPS CACHE)
  unset(VISP_MODULE_${mod}_PRIVATE_REQ_DEPS CACHE)
  unset(VISP_MODULE_${mod}_PRIVATE_OPT_DEPS CACHE)
  unset(VISP_MODULE_${mod}_LINK_DEPS CACHE)
  unset(VISP_MODULE_${mod}_INC_DEPS CACHE)
endforeach()

# clean modules info which needs to be recalculated
set(VISP_MODULES_PUBLIC         "" CACHE INTERNAL "List of ViSP modules marked for export")
set(VISP_MODULES_BUILD          "" CACHE INTERNAL "List of ViSP modules included into the build")
set(VISP_MODULES_DISABLED_USER  "" CACHE INTERNAL "List of ViSP modules explicitly disabled by user")
set(VISP_MODULES_DISABLED_AUTO  "" CACHE INTERNAL "List of ViSP modules implicitly disabled due to dependencies")
set(VISP_MODULES_DISABLED_FORCE "" CACHE INTERNAL "List of ViSP modules which can not be build in current configuration")

# adds dependencies to ViSP module
# Usage:
#   add_dependencies(visp_<name> [REQUIRED] [<list of dependencies>] [OPTIONAL <list of modules>])
# Notes:
# * <list of dependencies> - can include full names of modules or full pathes to shared/static libraries or cmake targets
macro(vp_add_dependencies full_modname)
  vp_debug_message("vp_add_dependencies(" ${full_modname} ${ARGN} ")")
  #we don't clean the dependencies here to allow this macro several times for every module
  foreach(d "REQUIRED" ${ARGN})
    if(d STREQUAL "REQUIRED")
      set(__depsvar VISP_MODULE_${full_modname}_REQ_DEPS)
    elseif(d STREQUAL "OPTIONAL")
      set(__depsvar VISP_MODULE_${full_modname}_OPT_DEPS)
    elseif(d STREQUAL "PRIVATE_REQUIRED")
      set(__depsvar VISP_MODULE_${full_modname}_PRIVATE_REQ_DEPS)
    elseif(d STREQUAL "PRIVATE_OPTIONAL")
      set(__depsvar VISP_MODULE_${full_modname}_PRIVATE_OPT_DEPS)
    else()
      list(APPEND ${__depsvar} "${d}")
    endif()
  endforeach()
  unset(__depsvar)

  vp_list_unique(VISP_MODULE_${full_modname}_REQ_DEPS)
  vp_list_unique(VISP_MODULE_${full_modname}_OPT_DEPS)
  vp_list_unique(VISP_MODULE_${full_modname}_PRIVATE_REQ_DEPS)
  vp_list_unique(VISP_MODULE_${full_modname}_PRIVATE_OPT_DEPS)

  set(VISP_MODULE_${full_modname}_REQ_DEPS ${VISP_MODULE_${full_modname}_REQ_DEPS}
    CACHE INTERNAL "Required dependencies of ${full_modname} module")
  set(VISP_MODULE_${full_modname}_OPT_DEPS ${VISP_MODULE_${full_modname}_OPT_DEPS}
    CACHE INTERNAL "Optional dependencies of ${full_modname} module")
  set(VISP_MODULE_${full_modname}_PRIVATE_REQ_DEPS ${VISP_MODULE_${full_modname}_PRIVATE_REQ_DEPS}
    CACHE INTERNAL "Required private dependencies of ${full_modname} module")
  set(VISP_MODULE_${full_modname}_PRIVATE_OPT_DEPS ${VISP_MODULE_${full_modname}_PRIVATE_OPT_DEPS}
    CACHE INTERNAL "Optional private dependencies of ${full_modname} module")
endmacro()

# declare new ViSP module in current folder
# Usage:
#   vp_add_module(<name> [INTERNAL|BINDINGS] [REQUIRED] [<list of dependencies>]
#                        [OPTIONAL <list of optional dependencies>]
#                        [PRIVATE_OPTIONAL|PRIVATE_REQUIRED] [<list of private dependencies>])
# Example:
#   vp_add_module(mymodule INTERNAL visp_core OPTIONAL visp_ar)
macro(vp_add_module _name)
  vp_debug_message("vp_add_module(" ${_name} ${ARGN} ")")
  string(TOLOWER "${_name}" name)
  set(the_module visp_${name})
  #message("Found module: ${the_module}")

  # the first pass - collect modules info, the second pass - create targets
  if(VISP_INITIAL_PASS)
    #guard agains redefinition
    if(";${VISP_MODULES_BUILD};${VISP_MODULES_DISABLED_USER};" MATCHES ";${the_module};")
      message(FATAL_ERROR "Redefinition of the ${the_module} module.
  at:                    ${CMAKE_CURRENT_SOURCE_DIR}
  previously defined at: ${VISP_MODULE_${the_module}_LOCATION}
")
    endif()

    if(NOT DEFINED the_description)
      set(the_description "The ViSP ${name} module")
    endif()

    if(NOT DEFINED BUILD_${the_module}_INIT)
      set(BUILD_${the_module}_INIT ON)
    endif()

    # create option to enable/disable this module
    option(BUILD_MODULE_${the_module} "Include ${the_module} module into ViSP build" ${BUILD_${the_module}_INIT})

    # remember the module details
    set(VISP_MODULE_${the_module}_DESCRIPTION "${the_description}" CACHE INTERNAL "Brief description of ${the_module} module")
    set(VISP_MODULE_${the_module}_LOCATION    "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "Location of ${the_module} module sources")

    set(VISP_MODULE_${the_module}_LINK_DEPS "" CACHE INTERNAL "")
    set(VISP_MODULE_${the_module}_INC_DEPS "" CACHE INTERNAL "")

    # parse list of dependencies
    if("${ARGV1}" STREQUAL "INTERNAL" OR "${ARGV1}" STREQUAL "BINDINGS")
      set(VISP_MODULE_${the_module}_CLASS "${ARGV1}" CACHE INTERNAL "The category of the module")
      set(__vp_argn__ ${ARGN})
      list(REMOVE_AT __vp_argn__ 0)
      vp_add_dependencies(${the_module} ${__vp_argn__})
      unset(__vp_argn__)
    else()
      set(VISP_MODULE_${the_module}_CLASS "PUBLIC" CACHE INTERNAL "The category of the module")
      vp_add_dependencies(${the_module} ${ARGN})
      if(BUILD_MODULE_${the_module})
        set(VISP_MODULES_PUBLIC ${VISP_MODULES_PUBLIC} "${the_module}" CACHE INTERNAL "List of ViSP modules marked for export")
      endif()
    endif()

    if(BUILD_MODULE_${the_module})
      set(VISP_MODULES_BUILD ${VISP_MODULES_BUILD} "${the_module}" CACHE INTERNAL "List of ViSP modules included into the build")
    else()
      set(VISP_MODULES_DISABLED_USER ${VISP_MODULES_DISABLED_USER} "${the_module}" CACHE INTERNAL "List of ViSP modules explicitly disabled by user")
    endif()

    # add submodules if any
    set(VISP_MODULE_${the_module}_CHILDREN "${VISP_MODULE_CHILDREN}" CACHE INTERNAL "List of ${the_module} submodules")

    # stop processing of current file
    return()
  else()
    set(VISP_MODULE_${the_module}_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "")
    if(NOT BUILD_MODULE_${the_module})
      return() # extra protection from redefinition
    endif()
  endif()
endmacro()

# remove visp_ prefix from name
macro(vp_short_module_name name)
  if(${name} MATCHES "^visp_")
    string(REGEX REPLACE "^visp_" "" ${name} "${${name}}")
  endif()
endmacro()

# collect modules from specified directories
# NB: must be called only once!
macro(vp_glob_modules)
  if(DEFINED VISP_INITIAL_PASS)
    message(FATAL_ERROR "ViSP has already loaded its modules. Calling vp_glob_modules second time is not allowed.")
  endif()
  set(__directories_observed "")

  # collect modules
  set(VISP_INITIAL_PASS ON)
  set(VISP_PROCESSING_EXTRA_MODULES 0)
  foreach(__path ${ARGN})
    if("${__path}" STREQUAL "EXTRA")
      set(VISP_PROCESSING_EXTRA_MODULES 1)
    else()
      get_filename_component(__path "${__path}" ABSOLUTE)

      list(FIND __directories_observed "${__path}" __pathIdx)
      if(__pathIdx GREATER -1)
        message(FATAL_ERROR "The directory ${__path} is observed for ViSP modules second time.")
      endif()
      list(APPEND __directories_observed "${__path}")

      file(GLOB __vpmodules RELATIVE "${__path}" "${__path}/*")

      vp_list_remove_item(__vpmodules ".git")

      if(VISP_PROCESSING_EXTRA_MODULES)
        # Remove tutorial, example, demo from potential contrib module list
        # They will be processed in visp/CMakeLists.txt
        vp_list_remove_item(__vpmodules "tutorial")
        vp_list_remove_item(__vpmodules "example")
        vp_list_remove_item(__vpmodules "demo")
        vp_list_remove_item(__vpmodules "doc")
        vp_list_remove_item(__vpmodules ".gitignore")
        vp_list_remove_item(__vpmodules ".travis.yml")
        vp_list_remove_item(__vpmodules "LICENSE")
        vp_list_remove_item(__vpmodules "README.md")
      endif()
      # TODO: Improve the following if to put a macro instead of manually copying the code
      #       Here we have 3 internal loops. The depth of the loop (3) could be a var
      set(__count 0)
      if(__vpmodules)
        list(SORT __vpmodules)
        foreach(mod ${__vpmodules})
          get_filename_component(__modpath "${__path}/${mod}" ABSOLUTE)
          set(__propagate FALSE) # Indicate if we should check for subdirs that may contain a CMakeLists.txt file
          if(EXISTS "${__modpath}/CMakeLists.txt")
            if(${mod} STREQUAL "modules")
              # Process specific case where there is a <contrib>/modules/CMakeLists.txt file common to all <contrib> modules
              include(${__modpath}/CMakeLists.txt)
              set(__propagate TRUE)
            else()
              list(FIND __directories_observed "${__modpath}" __pathIdx)
              if(__pathIdx GREATER -1)
                message(FATAL_ERROR "The module from ${__modpath} is already loaded.")
              endif()
              list(APPEND __directories_observed "${__modpath}")
              add_subdirectory("${__modpath}" "${CMAKE_CURRENT_BINARY_DIR}/${mod}/.${mod}")
              if (DEFINED VISP_MODULE_visp_${mod}_LOCATION)
                math(EXPR __count "${__count} + 1")
              endif()
            endif()
          else()
            set(__propagate TRUE)
          endif()
          if(__propagate)
            # modules in visp/tracker
            get_filename_component(__subpath "${__path}/${mod}" ABSOLUTE)
            file(GLOB __vpsubmodules RELATIVE "${__subpath}" "${__subpath}/*")

            if(__vpsubmodules)
              list(SORT __vpsubmodules)
              foreach(submod ${__vpsubmodules})
                get_filename_component(__submodpath "${__subpath}/${submod}" ABSOLUTE)
                if(EXISTS "${__submodpath}/CMakeLists.txt")
                  list(FIND __directories_observed "${__submodpath}" __pathIdx)
                  if(__pathIdx GREATER -1)
                    message(FATAL_ERROR "The module from ${__submodpath} is already loaded.")
                  endif()
                  list(APPEND __directories_observed "${__submodpath}")
                  add_subdirectory("${__submodpath}" "${CMAKE_CURRENT_BINARY_DIR}/${submod}/.${submod}")
                  if (DEFINED VISP_MODULE_visp_${submod}_LOCATION)
                    math(EXPR __count "${__count} + 1")
                  endif()

                else()
                  # modules in ustk/image_processing
                  get_filename_component(__subsubpath "${__subpath}/${submod}" ABSOLUTE)
                  file(GLOB __vpsubsubmodules RELATIVE "${__subsubpath}" "${__subsubpath}/*")

                  if(__vpsubsubmodules)
                    list(SORT __vpsubsubmodules)
                    foreach(subsubmod ${__vpsubsubmodules})
                      get_filename_component(__subsubmodpath "${__subsubpath}/${subsubmod}" ABSOLUTE)
                      if(EXISTS "${__subsubmodpath}/CMakeLists.txt")
                        list(FIND __directories_observed "${__subsubmodpath}" __pathIdx)
                        if(__pathIdx GREATER -1)
                          message(FATAL_ERROR "The module from ${__subsubmodpath} is already loaded.")
                        endif()
                        list(APPEND __directories_observed "${__subsubmodpath}")
                        add_subdirectory("${__subsubmodpath}" "${CMAKE_CURRENT_BINARY_DIR}/${subsubmod}/.${subsubmod}")
                        if (DEFINED VISP_MODULE_visp_${subsubmod}_LOCATION)
                          math(EXPR __count "${__count} + 1")
                        endif()

                      else()
                        # modules in ustk/image_processing/tracking
                        get_filename_component(__subsubsubpath "${__subsubpath}/${subsubmod}" ABSOLUTE)
                        file(GLOB __vpsubsubsubmodules RELATIVE "${__subsubsubpath}" "${__subsubsubpath}/*")

                        if(__vpsubsubsubmodules)
                          list(SORT __vpsubsubsubmodules)
                          foreach(subsubsubmod ${__vpsubsubsubmodules})
                            get_filename_component(__subsubsubmodpath "${__subsubsubpath}/${subsubsubmod}" ABSOLUTE)
                            if(EXISTS "${__subsubsubmodpath}/CMakeLists.txt")
                              list(FIND __directories_observed "${__subsubsubmodpath}" __pathIdx)
                              if(__pathIdx GREATER -1)
                                message(FATAL_ERROR "The module from ${__subsubsubmodpath} is already loaded.")
                              endif()
                              list(APPEND __directories_observed "${__subsubsubmodpath}")
                              add_subdirectory("${__subsubsubmodpath}" "${CMAKE_CURRENT_BINARY_DIR}/${subsubsubmod}/.${subsubsubmod}")
                              if (DEFINED VISP_MODULE_visp_$subsubsubmod_LOCATION)
                                math(EXPR __count "${__count} + 1")
                              endif()
                            endif()
                          endforeach()
                        endif()
                      endif()
                    endforeach()
                  endif()
                endif()
              endforeach()
            endif()
          endif()
        endforeach()
      endif()
    endif()

    if (VISP_PROCESSING_EXTRA_MODULES AND ${__count} LESS 1)
      message(SEND_ERROR "No contrib modules found in folder: ${__path}\nPlease provide path to 'visp_contrib/modules' folder.")
    endif()
  endforeach()
  vp_clear_vars(__vpmodules __directories_observed __path __modpath __pathIdx __vpsubmodules __subpath __submodpath)

  # resolve dependencies
  __vp_resolve_dependencies()

  # create modules
  set(VISP_INITIAL_PASS OFF PARENT_SCOPE)
  set(VISP_INITIAL_PASS OFF)

  foreach(m ${VISP_MODULES_BUILD})
    if(m MATCHES "^visp_")
      string(REGEX REPLACE "^visp_" "" __shortname "${m}")
      add_subdirectory("${VISP_MODULE_${m}_LOCATION}" "${CMAKE_CURRENT_BINARY_DIR}/${__shortname}")
    else()
      message(WARNING "Check module name: ${m}")
      add_subdirectory("${VISP_MODULE_${m}_LOCATION}" "${CMAKE_CURRENT_BINARY_DIR}/${m}")
    endif()
  endforeach()

  unset(__shortname)
endmacro()

# disables ViSP module with missing dependencies
function(__vp_module_turn_off the_module)
  list(REMOVE_ITEM VISP_MODULES_DISABLED_AUTO "${the_module}")
  list(APPEND VISP_MODULES_DISABLED_AUTO "${the_module}")
  list(REMOVE_ITEM VISP_MODULES_BUILD "${the_module}")
  list(REMOVE_ITEM VISP_MODULES_PUBLIC "${the_module}")
  set(HAVE_${the_module} OFF CACHE INTERNAL "Module ${the_module} can not be built in current configuration")

  set(VISP_MODULES_DISABLED_AUTO "${VISP_MODULES_DISABLED_AUTO}" CACHE INTERNAL "")
  set(VISP_MODULES_BUILD "${VISP_MODULES_BUILD}" CACHE INTERNAL "")
  set(VISP_MODULES_PUBLIC "${VISP_MODULES_PUBLIC}" CACHE INTERNAL "")
endfunction()

# sort modules by dependencies
function(__vp_sort_modules_by_deps __lst)
  vp_list_sort(${__lst})
  set(input ${${__lst}})
  set(result "")
  while(input)
    list(LENGTH input length_before)
    foreach (m ${input})
      # check if module is in the result already
      if (NOT ";${result};" MATCHES ";${m};")
        # scan through module dependencies...
        set(unresolved_deps_found FALSE)
        foreach (d ${VISP_MODULE_${m}_CHILDREN} ${VISP_MODULE_${m}_DEPS})
          # ... which are not already in the result and are enabled
          if ((NOT ";${result};" MATCHES ";${d};") AND HAVE_${d})
            set(unresolved_deps_found TRUE)
            break()
          endif()
        endforeach()
        # chek if all dependencies for this module has been resolved
        if (NOT unresolved_deps_found)
          list(APPEND result ${m})
          list(REMOVE_ITEM input ${m})
        endif()
      endif()
    endforeach()
    list(LENGTH input length_after)
    # check for infinite loop or unresolved dependencies
    if (NOT length_after LESS length_before)
      message(WARNING "Unresolved dependencies or loop in dependency graph (${length_after})\n"
        "Processed ${__lst}: ${${__lst}}\n"
        "Good modules: ${result}\n"
        "Bad modules: ${input}"
      )
      list(APPEND result ${input})
      break()
    endif()
  endwhile()
  set(${__lst} "${result}" PARENT_SCOPE)
endfunction()

# resolve dependensies
function(__vp_resolve_dependencies)
  foreach(m ${VISP_MODULES_DISABLED_USER})
    set(HAVE_${m} OFF CACHE INTERNAL "Module ${m} will not be built in current configuration")
  endforeach()
  foreach(m ${VISP_MODULES_BUILD})
    set(HAVE_${m} ON CACHE INTERNAL "Module ${m} will be built in current configuration")
  endforeach()

  # disable MODULES with unresolved dependencies
  set(has_changes ON)
  while(has_changes)
    set(has_changes OFF)
    foreach(m ${VISP_MODULES_BUILD})
      set(__deps ${VISP_MODULE_${m}_REQ_DEPS} ${VISP_MODULE_${m}_PRIVATE_REQ_DEPS})
      while(__deps)
        vp_list_pop_front(__deps d)
        string(TOLOWER "${d}" upper_d)
        if(NOT (HAVE_${d} OR HAVE_${upper_d} OR TARGET ${d} OR EXISTS ${d}))
          if(d MATCHES "^visp_") # TODO Remove this condition in the future and use HAVE_ variables only
            message(STATUS "Module ${m} disabled because ${d} dependency can't be resolved!")
            __vp_module_turn_off(${m})
            set(has_changes ON)
            break()
          else()
            message(STATUS "Assume that non-module dependency is available: ${d} (for module ${m})")
          endif()
        endif()
      endwhile()
    endforeach()
  endwhile()

#  message(STATUS "List of active modules: ${VISP_MODULES_BUILD}")

  foreach(m ${VISP_MODULES_BUILD})
    set(deps_${m} ${VISP_MODULE_${m}_REQ_DEPS})
    foreach(d ${VISP_MODULE_${m}_OPT_DEPS})
      if(NOT (";${deps_${m}};" MATCHES ";${d};"))
        if(HAVE_${d} OR TARGET ${d})
          list(APPEND deps_${m} ${d})
        endif()
      endif()
    endforeach()
#    message(STATUS "Initial deps of ${m} (w/o private deps): ${deps_${m}}")
  endforeach()

  # propagate dependencies
  set(has_changes ON)
  while(has_changes)
    set(has_changes OFF)
    foreach(m2 ${VISP_MODULES_BUILD}) # transfer deps of m2 to m
      foreach(m ${VISP_MODULES_BUILD})
        if((NOT m STREQUAL m2) AND ";${deps_${m}};" MATCHES ";${m2};")
          foreach(d ${deps_${m2}})
            if(NOT (";${deps_${m}};" MATCHES ";${d};"))
#              message(STATUS "  Transfer dependency ${d} from ${m2} to ${m}")
              list(APPEND deps_${m} ${d})
              set(has_changes ON)
            endif()
          endforeach()
        endif()
      endforeach()
    endforeach()
  endwhile()

  # process private deps
  #foreach(m ${VISP_MODULES_BUILD})
  #  foreach(d ${VISP_MODULE_${m}_PRIVATE_REQ_DEPS})
  #    if(NOT (";${deps_${m}};" MATCHES ";${d};"))
  #      list(APPEND deps_${m} ${d})
  #    endif()
  #  endforeach()
  #  foreach(d ${VISP_MODULE_${m}_PRIVATE_OPT_DEPS})
  #    if(NOT (";${deps_${m}};" MATCHES ";${d};"))
  #      if(HAVE_${d} OR TARGET ${d})
  #        list(APPEND deps_${m} ${d})
  #      endif()
  #    endif()
  #  endforeach()
  #endforeach()

  vp_list_sort(VISP_MODULES_BUILD)

  foreach(m ${VISP_MODULES_BUILD})
    #message(STATUS "FULL deps of ${m}: ${deps_${m}}")
    set(VISP_MODULE_${m}_DEPS ${deps_${m}})
    set(VISP_MODULE_${m}_DEPS_EXT ${deps_${m}})
    vp_list_filterout(VISP_MODULE_${m}_DEPS_EXT "^visp_[^ ]+$")
    if(VISP_MODULE_${m}_DEPS_EXT AND VISP_MODULE_${m}_DEPS)
      list(REMOVE_ITEM VISP_MODULE_${m}_DEPS ${VISP_MODULE_${m}_DEPS_EXT})
    endif()
  endforeach()

  # reorder dependencies
  foreach(m ${VISP_MODULES_BUILD})
    __vp_sort_modules_by_deps(VISP_MODULE_${m}_DEPS)
    vp_list_sort(VISP_MODULE_${m}_DEPS_EXT)

    set(LINK_DEPS ${VISP_MODULE_${m}_DEPS})

    set(VISP_MODULE_${m}_DEPS ${VISP_MODULE_${m}_DEPS} CACHE INTERNAL "Flattened dependencies of ${m} module")
    set(VISP_MODULE_${m}_DEPS_EXT ${VISP_MODULE_${m}_DEPS_EXT} CACHE INTERNAL "Extra dependencies of ${m} module")
    set(VISP_MODULE_${m}_DEPS_TO_LINK ${LINK_DEPS} CACHE INTERNAL "Flattened dependencies of ${m} module (for linker)")

#    message(STATUS "  module deps of ${m}: ${VISP_MODULE_${m}_DEPS}")
#    message(STATUS "  module link deps of ${m}: ${VISP_MODULE_${m}_DEPS_TO_LINK}")
#    message(STATUS "  extra deps of ${m}: ${VISP_MODULE_${m}_DEPS_EXT}")
#    message(STATUS "")
  endforeach()

  __vp_sort_modules_by_deps(VISP_MODULES_BUILD)

  set(VISP_MODULES_PUBLIC        ${VISP_MODULES_PUBLIC}        CACHE INTERNAL "List of ViSP modules marked for export")
  set(VISP_MODULES_BUILD         ${VISP_MODULES_BUILD}         CACHE INTERNAL "List of ViSP modules included into the build")
  set(VISP_MODULES_DISABLED_AUTO ${VISP_MODULES_DISABLED_AUTO} CACHE INTERNAL "List of ViSP modules implicitly disabled due to dependencies")
endfunction()

# setup include paths for the list of passed modules
macro(vp_target_include_modules target)
  foreach(d ${ARGN})
    if(d MATCHES "^visp_" AND HAVE_${d})
      if (EXISTS "${VISP_MODULE_${d}_LOCATION}/include")
        vp_target_include_directories(${target} "${VISP_MODULE_${d}_LOCATION}/include")
        # Work arround to be able to build the modules without INTERFACE_INCLUDE_DIRECTORIES
        # that was only introduces since CMake 2.8.12
        if (CMAKE_VERSION VERSION_LESS 2.8.12)
          vp_target_include_directories(${target} "${VISP_MODULE_${d}_INC_DEPS}")
        endif()
      endif()
    elseif(EXISTS "${d}")
      # FS keep external deps inc
      set(VISP_MODULE_${the_module}_INC_DEPS "${VISP_MODULE_${the_module}_INC_DEPS};${d}" CACHE INTERNAL "")
      vp_target_include_directories(${target} "${d}")
    endif()
  endforeach()
  vp_list_unique(VISP_MODULE_${the_module}_INC_DEPS)
endmacro()

# setup include path for ViSP headers for specified module
# vp_module_include_directories(<extra include directories/extra include modules>)
macro(vp_module_include_directories)
  vp_target_include_directories(${the_module}
      "${VISP_MODULE_${the_module}_LOCATION}/include"
      "${VISP_MODULE_${the_module}_LOCATION}/src"
      "${CMAKE_CURRENT_BINARY_DIR}" # for precompiled headers
      )
  vp_target_include_modules(${the_module} ${VISP_MODULE_${the_module}_DEPS} ${ARGN})
endmacro()

# sets header and source files for the current module
# NB: all files specified as headers will be installed
# Usage:
# vp_set_module_sources([HEADERS] <list of files> [SOURCES] <list of files>)
macro(vp_set_module_sources)
  vp_debug_message("vp_set_module_sources(" ${ARGN} ")")

  set(VISP_MODULE_${the_module}_HEADERS "")
  set(VISP_MODULE_${the_module}_SOURCES "")

  foreach(f "HEADERS" ${ARGN})
    if(f STREQUAL "HEADERS" OR f STREQUAL "SOURCES")
      set(__filesvar "VISP_MODULE_${the_module}_${f}")
    else()
      list(APPEND ${__filesvar} "${f}")
    endif()
  endforeach()

  # use full paths for module to be independent from the module location
  vp_convert_to_full_paths(VISP_MODULE_${the_module}_HEADERS)

  if(${the_module} MATCHES visp_core)
    list(APPEND VISP_MODULE_${the_module}_HEADERS "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h")
    list(APPEND VISP_MODULE_${the_module}_HEADERS "${VISP_INCLUDE_DIR}/visp3/visp_modules.h")
  endif()

  set(VISP_MODULE_${the_module}_HEADERS ${VISP_MODULE_${the_module}_HEADERS} CACHE INTERNAL "List of header files for ${the_module}")
  set(VISP_MODULE_${the_module}_SOURCES ${VISP_MODULE_${the_module}_SOURCES} CACHE INTERNAL "List of source files for ${the_module}")
endmacro()

# finds and sets headers and sources for the standard ViSP module
# Usage:
# vp_glob_module_sources(<extra sources&headers in the same format as used in vp_set_module_sources>)
macro(vp_glob_module_sources)
  vp_debug_message("vp_glob_module_sources(" ${ARGN} ")")
  set(_argn ${ARGN})

  file(GLOB_RECURSE lib_srcs
       "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
  )
  file(GLOB_RECURSE lib_int_hdrs
       "${CMAKE_CURRENT_LIST_DIR}/src/*.hpp"
       "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
  )
  file(GLOB lib_hdrs
       "${CMAKE_CURRENT_LIST_DIR}/include/visp3/*.h"
       "${CMAKE_CURRENT_LIST_DIR}/include/visp3/${name}/*.h"
  )

  vp_source_group("Src" DIRBASE "${CMAKE_CURRENT_LIST_DIR}/src" FILES ${lib_srcs} ${lib_int_hdrs})
  vp_source_group("Include" DIRBASE "${CMAKE_CURRENT_LIST_DIR}/include" FILES ${lib_hdrs})

  vp_set_module_sources(${_argn} HEADERS ${lib_hdrs}
                        SOURCES ${lib_srcs} ${lib_int_hdrs})
endmacro()

# finds and copy data from a source to a destination
# Usage:
# vp_glob_module_data(<source> <destination>)
macro(vp_glob_module_copy_data src dst)
  set(__data "")
  file(GLOB_RECURSE __data
       "${CMAKE_CURRENT_LIST_DIR}/${src}"
  )

  foreach(__d ${__data})
    file(COPY ${__d}
       DESTINATION "${VISP_BINARY_DIR}/${dst}"
       FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
       OWNER_WRITE
    )

    # install
    if(UNIX)
      set(__install_dst "${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/${dst}")
    else()
      set(__install_dst "${dst}")
    endif()

    install(FILES ${__d}
       DESTINATION "${__install_dst}"
       PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
       OWNER_WRITE
    )
  endforeach()
endmacro()

# creates old headers for compat with previous releases in include/visp
# Usage:
#   vp_create_compat_headers(<list of new headers>)
macro(vp_create_compat_headers)
  vp_debug_message("vp_create_compat_headers(" ${ARGN} ")")

  set(VISP_HEADER_CONTENT_CONFIGMAKE "")

  foreach(h ${ARGN})
    get_filename_component(__h_name_we ${h} NAME_WE)
    get_filename_component(__h_name ${h} NAME)
    set(VISP_HEADER_CONTENT_CONFIGMAKE "#ifndef __${__h_name_we}_gen_h_\n#define __${__h_name_we}_gen_h_\n\n#include <visp3/${name}/${__h_name}>\n\n#endif\n")
    set(__compat_header_dst "${VISP_INCLUDE_DIR}/visp/${__h_name_we}.h")
    configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpHeader.h.in" ${__compat_header_dst})
  endforeach()

  unset(__h_name_we)
  unset(__h_name)
  unset(__compat_header_dst)
endmacro()

# creates headers for modules include/visp3/<module>/vp<module>.h
# Usage:
#   vp_create_global_module_header(<module>)
macro(vp_create_global_module_header module)
  vp_debug_message("vp_create_global_module_header(" ${module} ")")

  set(__name ${module})
  vp_short_module_name(__name)
  set(__module_header_dst "${VISP_INCLUDE_DIR}/visp3/${module}.h")
  set(VISP_HEADER_CONTENT_CONFIGMAKE "#ifndef __${module}_h_\n#define __${module}_h_\n")

  # when core, include also vpConfig.h
  if(__name MATCHES "core")
    set(VISP_HEADER_CONTENT_CONFIGMAKE "${VISP_HEADER_CONTENT_CONFIGMAKE}\n#include <visp3/${__name}/vpConfig.h>")
  endif()

  # include the modules we depend on
  if(VISP_MODULE_${module}_REQ_DEPS)
    foreach(dep ${VISP_MODULE_${module}_REQ_DEPS})
      vp_short_module_name(dep)
    set(VISP_HEADER_CONTENT_CONFIGMAKE "${VISP_HEADER_CONTENT_CONFIGMAKE}\n#include <visp3/visp_${dep}.h>")
    endforeach()
  endif()

  foreach(h ${VISP_MODULE_${module}_HEADERS})
    string(REGEX REPLACE "^.*/include/visp3" "visp3" h "${h}")
    set(VISP_HEADER_CONTENT_CONFIGMAKE "${VISP_HEADER_CONTENT_CONFIGMAKE}\n#include <${h}>")
  endforeach()

  set(VISP_HEADER_CONTENT_CONFIGMAKE "${VISP_HEADER_CONTENT_CONFIGMAKE}\n\n#endif\n")
  configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpHeader.h.in" ${__module_header_dst})

  install(FILES ${__module_header_dst}
    DESTINATION ${VISP_INC_INSTALL_PATH}/visp3
    COMPONENT dev
  )

  unset(__h_name_we)
  unset(__h_name)
  unset(__module_header_dst)
endmacro()

# creates ViSP module in current folder
# creates new target, configures standard dependencies, compilers flags, install rules
# Usage:
#   vp_create_module(<extra link dependencies> LINK_PRIVATE <private link dependencies>)
#   vp_create_module()
macro(vp_create_module)
  vp_debug_message("vp_create_module(" ${ARGN} ")")
  set(VISP_MODULE_${the_module}_LINK_DEPS "${VISP_MODULE_${the_module}_LINK_DEPS};${ARGN}" CACHE INTERNAL "")
  _vp_create_module()
  set(the_module_target ${the_module})
endmacro()

macro(_vp_create_module)
  vp_create_compat_headers(${VISP_MODULE_${the_module}_HEADERS})
  vp_create_global_module_header(${the_module})

  vp_add_library(${the_module} ${VISP_MODULE_TYPE} ${VISP_MODULE_${the_module}_HEADERS} ${VISP_MODULE_${the_module}_SOURCES})

  vp_target_link_libraries(${the_module}
    LINK_PUBLIC
      ${VISP_MODULE_${the_module}_DEPS_TO_LINK}
      ${VISP_MODULE_${the_module}_DEPS_EXT}
      ${VISP_MODULE_${the_module}_LINK_DEPS}
      ${VISP_LINKER_LIBS}
    LINK_PRIVATE
      ${VISP_MODULE_${the_module}_PRIVATE_REQ_DEPS}
      ${VISP_MODULE_${the_module}_PRIVATE_OPT_DEPS})
  add_dependencies(visp_modules ${the_module})

  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(${the_module} PROPERTIES FOLDER "modules")
  endif()

  set_target_properties(${the_module} PROPERTIES
    OUTPUT_NAME "${the_module}${VISP_DLLVERSION}"
    DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
    ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}
    LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}
    RUNTIME_OUTPUT_DIRECTORY ${BINARY_OUTPUT_PATH}
  )

  set_property(TARGET ${the_module} APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${VISP_MODULE_${the_module}_INC_DEPS}
  )

  # For dynamic link numbering convenions
  if(NOT ANDROID)
    # Android SDK build scripts can include only .so files into final .apk
    # As result we should not set version properties for Android
    set_target_properties(${the_module} PROPERTIES
      VERSION ${VISP_VERSION}
      SOVERSION ${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}
    )
  endif()

  if((NOT DEFINED VISP_MODULE_TYPE AND BUILD_SHARED_LIBS)
      OR (DEFINED VISP_MODULE_TYPE AND VISP_MODULE_TYPE STREQUAL SHARED))
    set_target_properties(${the_module} PROPERTIES COMPILE_DEFINITIONS visp_EXPORTS)
    set_target_properties(${the_module} PROPERTIES DEFINE_SYMBOL visp_EXPORTS)
  endif()

  if(MSVC)
    if(CMAKE_CROSSCOMPILING)
      set_target_properties(${the_module} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
    endif()
    set_target_properties(${the_module} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
  endif()

  vp_install_target(${the_module} EXPORT VISPModules OPTIONAL
    RUNTIME DESTINATION ${VISP_BIN_INSTALL_PATH} COMPONENT libs
    LIBRARY DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT libs
    ARCHIVE DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT dev
    )

  foreach(m ${VISP_MODULE_${the_module}_CHILDREN} ${the_module})
    # only "public" headers need to be installed
    if(VISP_MODULE_${m}_HEADERS AND ";${VISP_MODULES_PUBLIC};" MATCHES ";${m};")
      foreach(hdr ${VISP_MODULE_${m}_HEADERS})
        string(REGEX REPLACE "^.*visp3/" "visp3/" hdr2 "${hdr}")
        if(NOT hdr2 MATCHES "visp3/${m}/private.*" AND hdr2 MATCHES "^(visp3/?.*)/[^/]+.h(..)?$" )
          install(FILES ${hdr} OPTIONAL DESTINATION "${VISP_INC_INSTALL_PATH}/${CMAKE_MATCH_1}" COMPONENT dev)
        endif()
      endforeach()
    endif()
  endforeach()

endmacro()


# short command for adding simple ViSP module
# see vp_add_module for argument details
# Usage:
# vp_define_module(module_name  [INTERNAL] [REQUIRED] [<list of dependencies>] [OPTIONAL <list of optional dependencies>])
macro(vp_define_module module_name)
  vp_debug_message("vp_define_module(" ${module_name} ${ARGN} ")")
  set(_argn ${ARGN})

  vp_add_module(${module_name} ${_argn})
  vp_glob_module_sources()
  vp_module_include_directories()
  vp_create_module()
endmacro()

# ensures that all passed modules are available
# sets VP_DEPENDENCIES_FOUND variable to TRUE/FALSE
macro(vp_check_dependencies)
  set(VP_DEPENDENCIES_FOUND TRUE)
  foreach(d ${ARGN})
    if(d MATCHES "^visp_[^ ]+$" AND NOT HAVE_${d})
      set(VP_DEPENDENCIES_FOUND FALSE)
      break()
    endif()
  endforeach()
endmacro()

# auxiliary macro to parse arguments of vp_add_tests commands
macro(__vp_parse_test_sources tests_type)
  set(VISP_${tests_type}_${the_module}_SOURCES "")
  set(VISP_${tests_type}_${the_module}_SOURCES_EXCLUDE "")
  set(VISP_${tests_type}_${the_module}_DEPS "")
  set(VISP_${tests_type}_${the_module}_CTEST_EXCLUDE_FOLDER "")
  set(VISP_${tests_type}_${the_module}_CTEST_EXCLUDE_FILE "")
  set(__file_group_name "")
  set(__file_group_sources "")
  foreach(arg "DEPENDS_ON" ${ARGN} "FILES")
    if(arg STREQUAL "FILES")
      set(__currentvar "__file_group_sources")
      if(__file_group_name AND __file_group_sources)
        source_group("${__file_group_name}" FILES ${__file_group_sources})
        list(APPEND VISP_${tests_type}_${the_module}_SOURCES ${__file_group_sources})
      endif()
      set(__file_group_name "")
      set(__file_group_sources "")
    elseif(arg STREQUAL "DEPENDS_ON")
      set(__currentvar "VISP_${tests_type}_${the_module}_DEPS")
    elseif(" ${__currentvar}" STREQUAL " __file_group_sources" AND NOT __file_group_name) # spaces to avoid CMP0054
      set(__file_group_name "${arg}")
    elseif(arg STREQUAL "CTEST_EXCLUDE_PATH")
      set(__currentvar "VISP_${tests_type}_${the_module}_CTEST_EXCLUDE_FOLDER")
    elseif(arg STREQUAL "CTEST_EXCLUDE_FILE")
      set(__currentvar "VISP_${tests_type}_${the_module}_CTEST_EXCLUDE_FILE")
    elseif(arg STREQUAL "SOURCES_EXCLUDE")
      set(__currentvar "VISP_${tests_type}_${the_module}_SOURCES_EXCLUDE")
    else()
      list(APPEND ${__currentvar} "${arg}")
    endif()
  endforeach()
  unset(__file_group_name)
  unset(__file_group_sources)
  unset(__currentvar)
endmacro()

# this is a command for adding ViSP tests to the module
# vp_add_tests([FILES <source group name> <list of sources>]
#              [FILES_EXCLUDE <list of sources>]
#              [DEPENDS_ON] <list of extra dependencies>
#              [CTEST_EXCLUDE_PATH] <list of folders to exclude from ctest>)
#              [CTEST_EXCLUDE_FILE] <list of files to exclude from ctest>)
macro(vp_add_tests)
  vp_debug_message("vp_add_tests(" ${ARGN} ")")

  set(test_path "${CMAKE_CURRENT_LIST_DIR}/test")
  if(BUILD_TESTS AND EXISTS "${test_path}")
    __vp_parse_test_sources(TEST ${ARGN})

    set(__exclude_ctest "")
    foreach(__folder ${VISP_TEST_${the_module}_CTEST_EXCLUDE_FOLDER} )
      file(GLOB_RECURSE __files "${CMAKE_CURRENT_LIST_DIR}/test/${__folder}/*.cpp")
      list(APPEND __exclude_ctest ${__files})
    endforeach()
    foreach(__file ${VISP_TEST_${the_module}_CTEST_EXCLUDE_FILE} )
      if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/test/${__file}")
        list(APPEND __exclude_ctest "${CMAKE_CURRENT_LIST_DIR}/test/${__file}")
      endif()
    endforeach()
    set(__exclude_sources "")
    foreach(__source ${VISP_TEST_${the_module}_SOURCES_EXCLUDE} )
      file(GLOB __files "${CMAKE_CURRENT_LIST_DIR}/test/${__source}")
      list(APPEND __exclude_sources ${__files})
    endforeach()

    set(test_deps ${the_module} ${VISP_MODULE_${the_module}_DEPS})

    foreach(d ${VISP_TEST_${the_module}_DEPS})
      list(APPEND test_deps ${d})
      list(APPEND test_deps ${VISP_MODULE_${d}_DEPS})
      # Work arround to be able to build the modules without INTERFACE_INCLUDE_DIRECTORIES
      # that was only introduces since CMake 2.8.12
      if(CMAKE_VERSION VERSION_LESS 2.8.12)
        list(APPEND test_deps "${VISP_MODULE_${__m}_INC_DEPS}")
      endif()
    endforeach()

    vp_check_dependencies(${test_deps})
    if(VP_DEPENDENCIES_FOUND)
      if(NOT VISP_TEST_${the_module}_SOURCES)
        file(GLOB_RECURSE test_srcs "${test_path}/*.cpp")
        vp_source_group("Src" DIRBASE "${test_path}" FILES ${test_srcs})
        set(VISP_TEST_${the_module}_SOURCES ${test_srcs})
      endif()

      foreach(t ${VISP_TEST_${the_module}_SOURCES})
        # check if source is not in exclude list
        list(FIND __exclude_sources ${t} __to_exclude_from_sources)
        if(${__to_exclude_from_sources} EQUAL -1)
          # Compute the name of the binary to create
          get_filename_component(the_target ${t} NAME_WE)
          # From source compile the binary and add link rules
          vp_add_executable(${the_target} ${t})
          vp_target_include_modules(${the_target} ${test_deps})
          vp_target_link_libraries(${the_target} ${test_deps} ${VISP_MODULE_${the_module}_DEPS} ${VISP_LINKER_LIBS})

          # ctest only if not in the exclude list
          list(FIND __exclude_ctest ${t} __to_exclude_from_ctest)
          if(${__to_exclude_from_ctest} EQUAL -1)
            add_test(${the_target} ${the_target} -c ${OPTION_TO_DESACTIVE_DISPLAY})
          endif()
          # TODO FS add visp_test_${name} target to group all the tests
          add_dependencies(visp_tests ${the_target})
          if(ENABLE_SOLUTION_FOLDERS)
            set_target_properties(${the_target} PROPERTIES FOLDER "tests")
          endif()
        endif()
      endforeach()

    else(VP_DEPENDENCIES_FOUND)
      # TODO: warn about unsatisfied dependencies
    endif(VP_DEPENDENCIES_FOUND)

  endif()
endmacro()

# setup include paths for the list of passed modules
macro(vp_include_modules)
  foreach(d ${ARGN})
    if(d MATCHES "^visp_" AND HAVE_${d})
      if (EXISTS "${VISP_MODULE_${d}_LOCATION}/include")
        vp_include_directories("${VISP_MODULE_${d}_LOCATION}/include")
      endif()
    elseif(EXISTS "${d}")
      vp_include_directories("${d}")
    endif()
  endforeach()
endmacro()

# same as previous but with dependencies
macro(vp_include_modules_recurse)
  vp_include_modules(${ARGN})
  foreach(d ${ARGN})
    if(d MATCHES "^visp_" AND HAVE_${d} AND DEFINED VISP_MODULE_${d}_DEPS)
      foreach (sub ${VISP_MODULE_${d}_DEPS})
        vp_include_modules(${sub})
      endforeach()
    endif()
  endforeach()
endmacro()

# This is a command to configure files as include headers of the corresponding module.
# vp_add_config_file(<list of header config files>)
#
# If the input config filename is suffixed by .in or .cmake the suffix is removed
# in the configured file.
#
# Warning: Should be called after add_module()
#
# Example:
#   add_module(my_module visp_core)
#   vp_add_config_file(cmake/template/vpConfigMyModule.h.in)
#   creates include/visp3/my_module/vpConfigMyModule.h
macro(vp_add_config_file)
  foreach(d ${ARGN})
    # Removes first "/" if it exists
    string(FIND ${d} "/" FIRST_SEPARATOR_POS)
    if(${FIRST_SEPARATOR_POS} EQUAL 0)
      string(SUBSTRING ${d} 1 -1 d)
    endif()

    # Find start of file name
    string(FIND ${d} "/" LAST_SEPARATOR_POS REVERSE)
    if(${LAST_SEPARATOR_POS} EQUAL -1)
      set(START 0)
    else()
      math(EXPR START "${LAST_SEPARATOR_POS}+1")
    endif()

    # Save entire path
    set(FILENAME_CONFIG ${d})

    # Find file name
    string(FIND ${d} "." EXTENSION_POS REVERSE)

    if(${EXTENSION_POS} EQUAL -1)
      string(SUBSTRING ${d} ${START} -1 FILENAME_CONFIG_SHORT)
    else()
      string(SUBSTRING ${d} ${EXTENSION_POS} -1 EXT_CONFIG_FILE)
      if(EXT_CONFIG_FILE MATCHES ".cmake" OR EXT_CONFIG_FILE MATCHES ".in")
        math(EXPR LENGTH "${EXTENSION_POS} - ${START}")
        string(SUBSTRING ${d} ${START} ${LENGTH} FILENAME_CONFIG_SHORT)
      else()
        string(SUBSTRING ${d} ${START} -1 FILENAME_CONFIG_SHORT)
      endif()
    endif()

    set(MODULE_NAME ${the_module})
    if(MODULE_NAME MATCHES "^visp_")
      string(REGEX REPLACE "^visp_" "" MODULE_NAME "${MODULE_NAME}")
    endif()
    configure_file("${VISP_MODULE_${the_module}_LOCATION}/${FILENAME_CONFIG}" "${VISP_INCLUDE_DIR}/visp3/${MODULE_NAME}/${FILENAME_CONFIG_SHORT}")

    vp_create_compat_headers("${VISP_INCLUDE_DIR}/visp3/${MODULE_NAME}/${FILENAME_CONFIG_SHORT}")

    install(FILES "${VISP_INCLUDE_DIR}/visp3/${MODULE_NAME}/${FILENAME_CONFIG_SHORT}"
      DESTINATION ${VISP_INC_INSTALL_PATH}/visp3/${MODULE_NAME}
      COMPONENT dev
    )

  endforeach()
endmacro()

# This is a command to add a list of paths associated to the corresponding module
# to the CMAKE_MODULE_PATH global var to find specific cmake material
# vp_add_cmake_module_path(<list of cmake module paths>)
# Example:
#   vp_add_cmake_module_path(cmake)
#   Appends the cmake full path to CMAKE_MODULE_PATH var.
macro(vp_add_cmake_module_path)
  foreach(d ${ARGN})
    # Removes first "/" if it exists
    string(FIND ${d} "/" FIRST_SEPARATOR_POS)
    if(${FIRST_SEPARATOR_POS} EQUAL 0)
      string(SUBSTRING ${d} 1 -1 d)
    endif()
    if(EXISTS "${VISP_MODULE_${the_module}_LOCATION}/${d}")
      list(APPEND CMAKE_MODULE_PATH "${VISP_MODULE_${the_module}_LOCATION}/${d}")
    endif()
  endforeach()
endmacro()
