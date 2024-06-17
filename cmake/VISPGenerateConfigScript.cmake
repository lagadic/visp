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
# This file generates the ViSP library config shell scripts:
# - visp-config in <build dir>/bin from visp-config.in
# - visp-config in <build dir>/install from visp-config.install.in
#   When make install, this file is copied in <install dir>/bin
# - visp.pc in <build dir>/install from visp.pc.in
#   When make install, this file is copied in <install dir>/lib/pkgconfig
#
#############################################################################

if(MINGW OR IOS)
  # not implemented yet
  return()
endif()

macro(fix_include_prefix lst)
  set(_lst)
  foreach(item ${${lst}})
    list(APPEND _lst "-I${item}")
  endforeach()
  vp_list_unique(_lst)
  set(${lst} ${_lst})
  unset(_lst)
endmacro()

macro(fix_prefix lst isown)
  set(_lst)
  foreach(item ${${lst}})
    if(DEFINED TARGET_LOCATION_${item})
      set(item "${TARGET_LOCATION_${item}}")
      if(${isown})
        get_filename_component(item "${item}" NAME)
        vp_get_libname(item "${item}")
      endif()
    endif()
    if(item MATCHES "^-l")
      list(APPEND _lst "${item}")
    elseif(item MATCHES "^-framework") # MacOS framework (assume single entry "-framework OpenCL")
      list(APPEND _lst "${item}")
    elseif(item MATCHES "^-") #could be "-pthread" (occurred with Ubuntu 18.04)
      list(APPEND _lst "${item}")
    elseif(item MATCHES ".framework/" OR item MATCHES "/([^/]+)\\.framework$")
      vp_get_framework(_fmk_name "${item}" NAME)
      vp_get_framework(_fmk_path "${item}" PATH)
      list(APPEND _lst "-F${_fmk_path} -framework ${_fmk_name}")
    elseif(item MATCHES "/([^/]+)\\.tbd$")
      vp_get_tbd(_fmk_name "${item}" NAME)
      vp_get_tbd(_fmk_path "${item}" PATH)
      list(APPEND _lst "-L${_fmk_path} -l${_fmk_name}")
    elseif(item MATCHES "[\\/]")
      get_filename_component(_libdir "${item}" PATH)
      get_filename_component(_libname "${item}" NAME)
      vp_get_libname(libname "${_libname}")
      list(APPEND _lst "-L${_libdir}" "-l${libname}")
    else()
      list(APPEND _lst "-l${item}")
    endif()
  endforeach()
  vp_list_unique(_lst)
  set(${lst} ${_lst})
  unset(_lst)
endmacro()

macro(fix_suffix_win lst isown lst_dbg lst_opt)
  set(_lst_dbg)
  set(_lst_opt)
  foreach(item ${${lst}})
    if(DEFINED TARGET_LOCATION_${item})
      set(item "${TARGET_LOCATION_${item}}")
      if(${isown})
        get_filename_component(item "${item}" NAME_WE)
        list(APPEND _lst_dbg "${item}${VISP_DEBUG_POSTFIX}.lib")
        list(APPEND _lst_opt "${item}.lib")
      endif()
    endif()
  endforeach()
  vp_list_unique(_lst_dbg)
  vp_list_unique(_lst_opt)
  set(${lst_dbg} ${_lst_dbg})
  set(${lst_opt} ${_lst_opt})
  unset(_lst_dbg)
  unset(_lst_opt)
endmacro()

macro(get_libname_win lst lst_lib)
  set(_lst_lib)
  foreach(item ${${lst}})
    get_filename_component(item "${item}" NAME)
    list(APPEND _lst_lib "${item}")
  endforeach()
  vp_list_unique(_lst_lib)
  set(${lst_lib} ${_lst_lib})
  unset(_lst_lib)
endmacro()

macro(get_libdir_win lst lst_libdir)
  set(_lst_libdir)
  set(${lst_libdir} ${_lst_libdir})
  foreach(item ${${lst}})
    get_filename_component(item "${item}" PATH)
    list(APPEND _lst_libdir "${item}")
  endforeach()
  set(${lst_libdir} ${_lst_libdir})
  unset(_lst_libdir)
endmacro()

if(NOT DEFINED CMAKE_HELPER_SCRIPT)
  # build the list of cxxflags for all modules
  vp_get_all_cflags(_cxx_flags)
  if (APPLE)
    # Needed on macOS Big Sur 11.6.2 to avoid the following warning
    # ld: warning: dylib (/usr/local/lib/libvisp_core.3.4.1.dylib) was built for newer macOS version (11.6) than being linked (11.0)
    if(CMAKE_OSX_DEPLOYMENT_TARGET)
      list(APPEND _cxx_flags "-mmacosx-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}")
    endif()
  endif()
  # build the list of includes for all modules and dependencies
  vp_get_all_includes(_includes_modules _includes_extra _system_include_dirs)
  # build the list of libs and dependencies for all modules
  vp_get_all_libs(_modules _extra_opt _extra_dbg _3rdparty)

  if(UNIX)
    # for unix/osx platforms
    set(FILE_VISP_SCRIPT_CONFIG         "${BINARY_OUTPUT_PATH}/visp-config")
    set(FILE_VISP_SCRIPT_CONFIG_INSTALL "${VISP_BINARY_DIR}/unix-install/visp-config")
    set(FILE_VISP_SCRIPT_PC_INSTALL     "${VISP_BINARY_DIR}/unix-install/visp.pc")
  else()
    # for windows platforms
    set(FILE_VISP_SCRIPT_CONFIG         "${BINARY_OUTPUT_PATH}/visp-config.bat")
    set(FILE_VISP_SCRIPT_CONFIG_INSTALL "${VISP_BINARY_DIR}/win-install/visp-config-${VISP_ARCH}-${VISP_RUNTIME}.bat")
  endif()

  set(HELPER_SCRIPT "")

  vp_cmake_script_append_var(HELPER_SCRIPT
    BUILD_SHARED_LIBS
    CMAKE_BINARY_DIR
    CMAKE_INSTALL_PREFIX
    BINARY_OUTPUT_PATH

    VISP_SOURCE_DIR
    VISP_VERSION
    VISP_BINARY_DIR
    VISP_LIB_INSTALL_PATH
    VISP_BIN_INSTALL_PATH
    VISP_INC_INSTALL_PATH
    VISP_3P_LIB_INSTALL_PATH
    VISP_DEBUG_POSTFIX
    VISP_ARCH
    VISP_RUNTIME
    VISP_HAVE_OPENMP

    WITH_CATCH2
    WITH_PUGIXML
    WITH_SIMDLIB
    WITH_STBIMAGE
    WITH_TINYEXR

    FILE_VISP_SCRIPT_CONFIG
    FILE_VISP_SCRIPT_CONFIG_INSTALL
    FILE_VISP_SCRIPT_PC_INSTALL

    _cxx_flags
    _includes_modules
    _includes_extra
    _system_include_dirs

    _modules
    _extra_opt
    _extra_dbg
    _3rdparty
  )

  foreach(item ${_modules} ${_extra_opt} ${_extra_dbg} ${_3rdparty})
    if(TARGET ${item})
      if(UNIX)
        set(HELPER_SCRIPT "${HELPER_SCRIPT}
set(TARGET_LOCATION_${item} \"$<TARGET_FILE:${item}>\")
")
      else()
        set(HELPER_SCRIPT "${HELPER_SCRIPT}
set(TARGET_LOCATION_${item} \"${item}${VISP_VERSION_MAJOR}${VISP_VERSION_MINOR}${VISP_VERSION_PATCH}.dll\")
")
      endif()
    endif()
  endforeach()

  set(CMAKE_HELPER_SCRIPT "${CMAKE_BINARY_DIR}/VISPGenerateConfigScript.info.cmake")
  file(GENERATE OUTPUT "${CMAKE_HELPER_SCRIPT}" CONTENT "${HELPER_SCRIPT}")

  add_custom_command(
    OUTPUT "${FILE_VISP_SCRIPT_CONFIG}"
    COMMAND ${CMAKE_COMMAND} "-DCMAKE_HELPER_SCRIPT=${CMAKE_HELPER_SCRIPT}" -P "${VISP_SOURCE_DIR}/cmake/VISPGenerateConfigScript.cmake"
    DEPENDS "${CMAKE_BINARY_DIR}/VISPGenerateConfigScript.info.cmake"
            "${VISP_SOURCE_DIR}/cmake/VISPGenerateConfigScript.cmake"
    COMMENT "Generate visp-config"
  )
  add_custom_target(developer_scripts ALL SOURCES "${FILE_VISP_SCRIPT_CONFIG}")

  #----------------------------------------------------------------------
  # customize install target
  #----------------------------------------------------------------------
  if(NOT ANDROID)
  # install rule for visp-config shell script
  install(FILES ${FILE_VISP_SCRIPT_CONFIG_INSTALL}
    DESTINATION ${VISP_BIN_INSTALL_PATH}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
    OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    OWNER_WRITE
    COMPONENT dev
  )

  # install rule for visp.pc pkg-config file
  if(UNIX)
    install(FILES ${FILE_VISP_SCRIPT_PC_INSTALL}
      DESTINATION ${VISP_LIB_INSTALL_PATH}/pkgconfig
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_WRITE
      COMPONENT dev
    )
  else()
    # not implemented yet
  endif()
  endif()

# =============================================================================
else() # DEFINED CMAKE_HELPER_SCRIPT

  cmake_minimum_required(VERSION 3.5)
  include("${CMAKE_HELPER_SCRIPT}")
  include("${VISP_SOURCE_DIR}/cmake/VISPUtils.cmake")

  #---------------------------------------------------------------------
  # Updates VISP_SCRIPT_CONFIG_PREFIX
  #----------------------------------------------------------------------
  set(VISP_SCRIPT_CONFIG_PREFIX "${CMAKE_INSTALL_PREFIX}")

  if(UNIX)
    #######################################################################
    #
    # for Unix platforms: Linux, OSX
    # Two kind of vars:
    # - *_SCRIPT_CONFIG_* to manage visp-config files
    # - *_SCRIPT_PC_*     to manage visp.pc files
    #
    #######################################################################
    set(FILE_VISP_SCRIPT_CONFIG_IN "cmake/templates/visp-config.in")
    set(FILE_VISP_SCRIPT_CONFIG_INSTALL_IN "cmake/templates/visp-config.install.in")
    set(FILE_VISP_SCRIPT_PC_INSTALL_IN "cmake/templates/visp.pc.in")

    fix_prefix(_modules 1)
    fix_prefix(_extra_opt 0) # We don't consider _extra_dbg on UNIX
    fix_prefix(_3rdparty 1)

    fix_include_prefix(_includes_modules)
    fix_include_prefix(_includes_extra)

    #---------------------------------------------------------------------
    # Updates vars needed to update VISP_SCRIPT_CONFIG_LIBS
    #----------------------------------------------------------------------
    # prepend with ViSP own modules first
    set(VISP_SCRIPT_CONFIG_LIBS
      "-L\$PREFIX/${VISP_LIB_INSTALL_PATH}"
      "${_modules}"
    )
    if(BUILD_SHARED_LIBS)
      set(VISP_SCRIPT_CONFIG_LIBS_PRIVATE "${_extra_opt}")
    else()
      set(VISP_SCRIPT_CONFIG_LIBS_PRIVATE
        "-L\$PREFIX/3rdparty/lib"
        "${_3rdparty}"
        "${_extra_opt}"
      )
    endif()

    vp_list_remove_separator(VISP_SCRIPT_CONFIG_LIBS)
    vp_list_remove_separator(VISP_SCRIPT_CONFIG_LIBS_PRIVATE)

    set(VISP_SCRIPT_CONFIG_ECHO_NO_NEWLINE_CHARACTER "")
    set(VISP_SCRIPT_CONFIG_ECHO_NO_NEWLINE_OPTION "")
    if(APPLE)
      set(VISP_SCRIPT_CONFIG_ECHO_NO_NEWLINE_CHARACTER "\\c")
    else()
      set(VISP_SCRIPT_CONFIG_ECHO_NO_NEWLINE_OPTION "-n")
    endif()

    #---------------------------------------------------------------------
    # Updates vars needed to update VISP_SCRIPT_CONFIG_CFLAGS
    # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
    #    VISP_SCRIPT_CONFIG_CFLAGS = _cxx_flags + _includes_extra + _includes_modules
    # 2/ For usage with the install tree
    #    VISP_SCRIPT_CONFIG_CFLAGS = _cxx_flags + _includes_extra
    #----------------------------------------------------------------------
    set(VISP_SCRIPT_CONFIG_CFLAGS
      "${_cxx_flags}"
      "-I$PREFIX/${VISP_INC_INSTALL_PATH}"
      "${_includes_modules}"
      "${_includes_extra}")

    # Format the string to suppress CMake separators ";"
    vp_list_remove_separator(VISP_SCRIPT_CONFIG_CFLAGS)
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_SCRIPT_CONFIG_IN}" "${FILE_VISP_SCRIPT_CONFIG}" @ONLY)

    #---------------------------------------------------------------------
    # Configure to create <build dir>/unix-install/visp-config shell script
    # that will call visp.pc
    #----------------------------------------------------------------------
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_SCRIPT_CONFIG_INSTALL_IN}" "${FILE_VISP_SCRIPT_CONFIG_INSTALL}" @ONLY)

    #---------------------------------------------------------------------
    # Updates the <build dir>/install/visp.pc pkg-config file
    # Updates VISP_SCRIPT_PC_CFLAGS (for visp.pc used by pkg-config)
    # Updates VISP_SCRIPT_PC_LIBS (for visp.pc used by pkg-config)
    #----------------------------------------------------------------------
    set(exec_prefix "\${prefix}")
    if(IS_ABSOLUTE ${VISP_INC_INSTALL_PATH})
      set(includedir  "${VISP_INC_INSTALL_PATH}")
    else()
      set(includedir  "\${prefix}/${VISP_INC_INSTALL_PATH}")
    endif()
    if(IS_ABSOLUTE ${VISP_LIB_INSTALL_PATH})
      set(libdir      "${VISP_LIB_INSTALL_PATH}")
    else()
      set(libdir      "\${prefix}/${VISP_LIB_INSTALL_PATH}")
    endif()

    # prepend with ViSP own include dir
    set(VISP_SCRIPT_PC_CFLAGS
      "${_cxx_flags}"
      "-I\${includedir}"
      "${_includes_extra}")

    # Format the string to suppress CMake separators ";"
    vp_list_remove_separator(VISP_SCRIPT_PC_CFLAGS)

    # prepend with ViSP own modules first
    if(IS_ABSOLUTE ${VISP_LIB_INSTALL_PATH})
      set(VISP_SCRIPT_PC_LIBS
        "-L${VISP_LIB_INSTALL_PATH}"
        "${_modules}"
      )
    else()
      set(VISP_SCRIPT_PC_LIBS
        "-L\${exec_prefix}/${VISP_LIB_INSTALL_PATH}"
        "${_modules}"
      )
    endif()
    if(BUILD_SHARED_LIBS)
      set(VISP_SCRIPT_PC_LIBS_PRIVATE "${_extra_opt}")
    else()
      if(IS_ABSOLUTE ${VISP_3P_LIB_INSTALL_PATH})
        set(VISP_SCRIPT_PC_LIBS_PRIVATE
          "-L${VISP_3P_LIB_INSTALL_PATH}"
          "${_3rdparty}"
          "${_extra_opt}"
        )
      else()
        set(VISP_SCRIPT_PC_LIBS_PRIVATE
          "-L\${exec_prefix}/${VISP_3P_LIB_INSTALL_PATH}"
          "${_3rdparty}"
          "${_extra_opt}"
        )
      endif()
    endif()

    vp_list_remove_separator(VISP_SCRIPT_PC_LIBS)
    vp_list_remove_separator(VISP_SCRIPT_PC_LIBS_PRIVATE)

    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_SCRIPT_PC_INSTALL_IN}" "${FILE_VISP_SCRIPT_PC_INSTALL}" @ONLY)

  else()
    #######################################################################
    #
    # for windows platforms
    #
    #######################################################################
    set(FILE_VISP_SCRIPT_CONFIG_IN "cmake/templates/visp-config.bat.in")
    set(FILE_VISP_SCRIPT_CONFIG_INSTALL_IN "cmake/templates/visp-config.bat.in")

    #---------------------------------------------------------------------
    # Updates VISP_SCRIPT_CONFIG_SCRIPT_DEF
    #----------------------------------------------------------------------
    set(VISP_SCRIPT_CONFIG_DEFS
    "_SCL_SECURE_NO_DEPRECATE")

    #---------------------------------------------------------------------
    # Updates VISP_SCRIPT_CONFIG_INC
    # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
    #    VISP_SCRIPT_CONFIG_INC = %PREFIX%/${VISP_INC_INSTALL_PATH} + _includes_extra
    # 2/ For usage with the install tree
    #    VISP_SCRIPT_CONFIG_INC = %PREFIX%/${VISP_INC_INSTALL_PATH} + _includes_extra + _includes_modules
    #----------------------------------------------------------------------

    #---------------------------------------------------------------------
    # Updates VISP_SCRIPT_CONFIG_SCRIPT_LIBDIR
    # 1/ For usage with the build tree
    # 2/ For usage with the install tree
    #
    # and updates VISP_SCRIPT_CONFIG_LIBS_${config}
    #----------------------------------------------------------------------

    fix_suffix_win(_modules 1 _modules_dbg_libname _modules_opt_libname)
    fix_suffix_win(_3rdparty 1 _3rdparty_dbg_libname _3rdparty_opt_libname)
    # We suppose that _modules_dbg_libname and _modules_opt_libname have the same libdir
    get_libdir_win(_extra_opt _extra_opt_libdir)

    get_libname_win(_extra_opt _extra_opt_libname)
    get_libname_win(_extra_dbg _extra_dbg_libname)

    vp_list_unique(_extra_opt_libdir)

    if(BUILD_SHARED_LIBS)
      set(VISP_SCRIPT_CONFIG_LIBS_DEBUG
        "${_modules_dbg_libname}"
        "${_extra_dbg_libname}")
      set(VISP_SCRIPT_CONFIG_LIBS_OPTIMIZED
        "${_modules_opt_libname}"
        "${_extra_opt_libname}")
    else()
      set(VISP_SCRIPT_CONFIG_LIBS_DEBUG
        "${_modules_dbg_libname}"
        "${_3rdparty_dbg_libname}"
        "${_extra_dbg_libname}")
      set(VISP_SCRIPT_CONFIG_LIBS_OPTIMIZED
        "${_modules_opt_libname}"
        "${_3rdparty_opt_libname}"
        "${_extra_opt_libname}")
    endif()

    vp_list_replace_separator(VISP_SCRIPT_CONFIG_LIBS_DEBUG "; ")
    vp_list_replace_separator(VISP_SCRIPT_CONFIG_LIBS_OPTIMIZED "; ")

    # Propagate openmp compiler option if enabled during ViSP build
    set(VISP_SCRIPT_CONFIG_OPENMP_SUPPORT "no")
    if(VISP_HAVE_OPENMP)
      set(VISP_SCRIPT_CONFIG_OPENMP_SUPPORT "yes")
    endif()

    # 1/ For usage with the build tree
    set(VISP_SCRIPT_CONFIG_INC
      "%PREFIX%/${VISP_INC_INSTALL_PATH}"
      "${_includes_modules}"
      "${_includes_extra}")

    if(BUILD_SHARED_LIBS)
      set(VISP_SCRIPT_CONFIG_LIBDIR
        "%PREFIX%/lib/$(ConfigurationName)"
        "${_extra_opt_libdir}")
    else()
      set(VISP_SCRIPT_CONFIG_LIBDIR
        "%PREFIX%/lib/$(ConfigurationName)"
        "%PREFIX%/3rdparty/lib/$(ConfigurationName)"
        "${_extra_opt_libdir}")
    endif()

    vp_list_replace_separator(VISP_SCRIPT_CONFIG_INC "; ")
    vp_list_replace_separator(VISP_SCRIPT_CONFIG_LIBDIR "; ")

    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_SCRIPT_CONFIG_IN}"
      "${FILE_VISP_SCRIPT_CONFIG}" @ONLY)

    # 2/ For usage with the install tree
    set(VISP_SCRIPT_CONFIG_INC
      "%PREFIX%/${VISP_INC_INSTALL_PATH}"
      "${_includes_extra}")

    if(BUILD_SHARED_LIBS)
      set(VISP_SCRIPT_CONFIG_LIBDIR
        "%PREFIX%/${VISP_ARCH}/${VISP_RUNTIME}/lib"
        "${_extra_opt_libdir}")
    else()
      set(VISP_SCRIPT_CONFIG_LIBDIR
        "%PREFIX%/${VISP_ARCH}/${VISP_RUNTIME}/staticlib"
        "${_extra_opt_libdir}")
    endif()

    vp_list_replace_separator(VISP_SCRIPT_CONFIG_INC "; ")
    vp_list_replace_separator(VISP_SCRIPT_CONFIG_LIBDIR "; ")

    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_SCRIPT_CONFIG_INSTALL_IN}"
      "${FILE_VISP_SCRIPT_CONFIG_INSTALL}" @ONLY)
  endif()

endif() # DEFINED CMAKE_HELPER_SCRIPT
