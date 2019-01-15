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
# This file generates the ViSP library config shell scripts:
# - visp-config in <build dir>/bin from visp-config.in
# - visp-config in <build dir>/install from visp-config.install.in
#   When make install, this file is copied in <install dir>/bin
# - visp.pc in <build dir>/install from visp.pc.in
#   When make install, this file is copied in <install dir>/lib/pkgconfig
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(MINGW OR IOS)
  # not implemented yet
  return()
endif()

if(NOT DEFINED CMAKE_HELPER_SCRIPT)

  if(UNIX)
    # for unix/osx platforms
    set(FILE_VISP_CONFIG_SCRIPT         "${BINARY_OUTPUT_PATH}/visp-config")
    set(FILE_VISP_CONFIG_SCRIPT_INSTALL "${VISP_BINARY_DIR}/unix-install/visp-config")
    set(FILE_VISP_CONFIG_PC_INSTALL     "${VISP_BINARY_DIR}/unix-install/visp.pc")
  else()
    # for windows platforms
    set(FILE_VISP_CONFIG_SCRIPT         "${BINARY_OUTPUT_PATH}/visp-config.bat")
    set(FILE_VISP_CONFIG_SCRIPT_INSTALL "${VISP_BINARY_DIR}/win-install/visp-config-${VISP_ARCH}-${VISP_RUNTIME}.bat")
  endif()

  #---------------------------------------------------------------------
  # Get cxx_flags to propagate in scripts
  #----------------------------------------------------------------------
  if(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    list(APPEND _cxx_flags "-ftest-coverage")
    list(APPEND _cxx_flags "-fprofile-arcs")
  endif()

  # Propagate C++11 compiler option if enabled during ViSP build
  if(USE_CXX11 AND CXX11_CXX_FLAGS)
    list(APPEND _cxx_flags ${CXX11_CXX_FLAGS})
  endif()
  # Propagate openmp compiler option if enabled during ViSP build
  set(VISP_OPENMP_SUPPORT "no")
  if(VISP_HAVE_OPENMP)
    list(APPEND _cxx_flags ${OpenMP_CXX_FLAGS})
    set(VISP_OPENMP_SUPPORT "yes")
  endif()

  #---------------------------------------------------------------------
  # Get include_flags to propagate in scripts
  #----------------------------------------------------------------------
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND _include_flags_deps ${VISP_MODULE_${m}_INC_DEPS})
    if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
      list(APPEND _include_flags_src "${VISP_MODULE_${m}_LOCATION}/include")
    endif()
  endforeach()

  #---------------------------------------------------------------------
  # Get system_include_dirs to propagate in scripts for SonarQube
  #----------------------------------------------------------------------
  set(SYSTEM_HEADERS vector iostream type_traits limits.h stdarg.h endian.h)
  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    list(APPEND COMPILER_INCLUDE_DIRS /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include/c++/v1)
    list(APPEND COMPILER_INCLUDE_DIRS /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/lib/clang/10.0.0/include)
    list(APPEND COMPILER_INCLUDE_DIRS /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include/i386)
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # Todo
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    # Todo
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # Todo
  endif()

  vp_find_path(SYSTEM_HEADERS _system_include_dirs PATHS ${COMPILER_INCLUDE_DIRS})

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
    VISP_DEBUG_POSTFIX
    VISP_OPENMP_SUPPORT

    _cxx_flags
    _include_flags_deps
    _include_flags_src
    _system_include_dirs

    FILE_VISP_CONFIG_SCRIPT
    FILE_VISP_CONFIG_SCRIPT_INSTALL
    FILE_VISP_CONFIG_PC_INSTALL

    VISP_MODULES_BUILD
  )

  foreach(item ${VISP_MODULES_BUILD})
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
    foreach(dep ${VISP_MODULE_${item}_LINK_DEPS})
      if(dep MATCHES "framework$")
        get_filename_component(_fpath "${dep}" DIRECTORY)
        get_filename_component(_fname "${dep}" NAME_WE)
        list(APPEND _deps "-F${_fpath} -framework ${_fname}")
      else()
        list(APPEND _deps "${dep}")
      endif()
    endforeach()

    vp_list_unique(_deps)
    set(HELPER_SCRIPT "${HELPER_SCRIPT}
set(VISP_MODULE_${item}_LINK_DEPS \"${_deps}\")
")
  endforeach()

  set(CMAKE_HELPER_SCRIPT "${CMAKE_BINARY_DIR}/VISPGenerateConfigScript.info.cmake")
  file(GENERATE OUTPUT "${CMAKE_HELPER_SCRIPT}" CONTENT "${HELPER_SCRIPT}")

  add_custom_command(
    OUTPUT "${FILE_VISP_CONFIG_SCRIPT}"
    COMMAND ${CMAKE_COMMAND} "-DCMAKE_HELPER_SCRIPT=${CMAKE_HELPER_SCRIPT}" -P "${VISP_SOURCE_DIR}/cmake/VISPGenerateConfigScript.cmake"
    DEPENDS "${CMAKE_BINARY_DIR}/VISPGenerateConfigScript.info.cmake"
            "${VISP_SOURCE_DIR}/cmake/VISPGenerateConfigScript.cmake"
    COMMENT "Generate visp-config"
  )
  add_custom_target(developer_scripts ALL SOURCES "${FILE_VISP_CONFIG_SCRIPT}")

  #----------------------------------------------------------------------
  # customize install target
  #----------------------------------------------------------------------
  if(NOT ANDROID)
  # install rule for visp-config shell script
  install(FILES ${FILE_VISP_CONFIG_SCRIPT_INSTALL}
    DESTINATION ${VISP_BIN_INSTALL_PATH}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
    OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    OWNER_WRITE
    COMPONENT dev
  )

  # install rule for visp.pc pkg-config file
  if(UNIX)
    install(FILES ${FILE_VISP_CONFIG_PC_INSTALL}
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

  cmake_minimum_required(VERSION 2.8.12.2)
  include("${CMAKE_HELPER_SCRIPT}")
  include("${VISP_SOURCE_DIR}/cmake/VISPUtils.cmake")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  set(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")

  if(UNIX)
    #######################################################################
    #
    # for Unix platforms: Linux, OSX
    #
    #######################################################################
    set(FILE_VISP_CONFIG_SCRIPT_IN "cmake/templates/visp-config.in")
    set(FILE_VISP_CONFIG_SCRIPT_INSTALL_IN "cmake/templates/visp-config.install.in")
    set(FILE_VISP_CONFIG_PC_INSTALL_IN "cmake/templates/visp.pc.in")

    #---------------------------------------------------------------------
    # Updates vars needed to update VISP_CONFIG_LIBS_SCRIPT
    #----------------------------------------------------------------------
    # prepend with ViSP own modules first
    set(VISP_CONFIG_LIBS_SCRIPT "")
    foreach(m ${VISP_MODULES_BUILD})
      get_filename_component(m_libname "${TARGET_LOCATION_${m}}" NAME)
      list(APPEND VISP_CONFIG_LIBS_SCRIPT "$PREFIX/${VISP_LIB_INSTALL_PATH}/${m_libname}")
    endforeach()
    # append deps
    foreach(m ${VISP_MODULES_BUILD})
      list(APPEND VISP_CONFIG_LIBS_SCRIPT ${VISP_MODULE_${m}_LINK_DEPS})
    endforeach()
    vp_list_unique(VISP_CONFIG_LIBS_SCRIPT)
    vp_list_filterout(VISP_CONFIG_LIBS_SCRIPT "debug")
    vp_list_filterout(VISP_CONFIG_LIBS_SCRIPT "optimized")
    # Format the string to suppress CMake separators ";"
    vp_list_remove_separator(VISP_CONFIG_LIBS_SCRIPT)

    set(VISP_ECHO_NO_NEWLINE_CHARACTER "")
    set(VISP_ECHO_NO_NEWLINE_OPTION "")
    if(APPLE)
      set(VISP_ECHO_NO_NEWLINE_CHARACTER "\\c")
    else()
      set(VISP_ECHO_NO_NEWLINE_OPTION "-n")
    endif()

    #---------------------------------------------------------------------
    # Updates vars needed to update VISP_CONFIG_CFLAGS_SCRIPT
    # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
    #    VISP_CONFIG_CFLAGS_SCRIPT = _cxx_flags + _include_flags_deps + _include_flags_src
    # 2/ For usage with the install tree
    #    VISP_CONFIG_CFLAGS_SCRIPT = _cxx_flags + _include_flags_deps
    #----------------------------------------------------------------------
    set(VISP_CONFIG_CFLAGS_SCRIPT ${_cxx_flags})
    list(APPEND VISP_CONFIG_CFLAGS_SCRIPT "-I$PREFIX/${VISP_INC_INSTALL_PATH}")
    foreach(_inc ${_include_flags_src})
      list(APPEND VISP_CONFIG_CFLAGS_SCRIPT "-I${_inc}")
    endforeach()
    foreach(_inc ${_include_flags_deps})
      list(APPEND VISP_CONFIG_CFLAGS_SCRIPT "-I${_inc}")
    endforeach()
    vp_list_unique(VISP_CONFIG_CFLAGS_SCRIPT)
    # Format the string to suppress CMake separators ";"
    vp_list_remove_separator(VISP_CONFIG_CFLAGS_SCRIPT)
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_CONFIG_SCRIPT_IN}" "${FILE_VISP_CONFIG_SCRIPT}" @ONLY)

    #---------------------------------------------------------------------
    # Configure to create <build dir>/unix-install/visp-config shell script
    # that will call visp.pc
    #----------------------------------------------------------------------
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_CONFIG_SCRIPT_INSTALL_IN}" "${FILE_VISP_CONFIG_SCRIPT_INSTALL}" @ONLY)

    #---------------------------------------------------------------------
    # Updates the <build dir>/install/visp.pc pkg-config file
    # Updates VISP_CONFIG_CFLAGS_PC (for libvisp.pc used by pkg-config)
    # Updates VISP_CONFIG_LIBS_PC (for libvisp.pc used by pkg-config)
    #----------------------------------------------------------------------
    set(exec_prefix "\${prefix}")
    set(includedir  "\${prefix}/${VISP_INC_INSTALL_PATH}")
    set(libdir  "\${prefix}/${VISP_LIB_INSTALL_PATH}")

    # prepend with ViSP own include dir
    set(VISP_CONFIG_CFLAGS_PC "-I\${includedir}")
    list(APPEND VISP_CONFIG_CFLAGS_PC ${_cxx_flags})
    foreach(_inc ${_include_flags_deps})
      list(APPEND VISP_CONFIG_CFLAGS_PC "-I${_inc}")
    endforeach()

    vp_list_unique(VISP_CONFIG_CFLAGS_PC)
    # Format the string to suppress CMake separators ";"
    vp_list_remove_separator(VISP_CONFIG_CFLAGS_PC)

    # prepend with ViSP own modules first
    set(VISP_CONFIG_LIBS_PC "")
    foreach(m ${VISP_MODULES_BUILD})
      get_filename_component(m_libname "${TARGET_LOCATION_${m}}" NAME)
      list(APPEND VISP_CONFIG_LIBS_PC "\${libdir}/${m_libname}")
    endforeach()
    # append deps
    foreach(m ${VISP_MODULES_BUILD})
      list(APPEND VISP_CONFIG_LIBS_PC ${VISP_MODULE_${m}_LINK_DEPS})
    endforeach()
    vp_list_filterout(VISP_CONFIG_LIBS_PC "optimized")
    vp_list_filterout(VISP_CONFIG_LIBS_PC "debug")
    vp_list_unique(VISP_CONFIG_LIBS_PC)
    vp_list_remove_separator(VISP_CONFIG_LIBS_PC)

    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_CONFIG_PC_INSTALL_IN}" "${FILE_VISP_CONFIG_PC_INSTALL}" @ONLY)

    #----------------------------------------------------------------------
    # Generate SonarQube config file
    #----------------------------------------------------------------------
    set(VISP_SONARQUBE_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/${VISP_INC_INSTALL_PATH}")
    # Include system path
    foreach(_inc ${_system_include_dirs})
      set(VISP_SONARQUBE_INCLUDE_DIRS "${VISP_SONARQUBE_INCLUDE_DIRS}, ${_inc}")
    endforeach()
    foreach(_inc ${_include_flags_src})
      set(VISP_SONARQUBE_INCLUDE_DIRS "${VISP_SONARQUBE_INCLUDE_DIRS}, ${_inc}")
    endforeach()
    foreach(_inc ${_include_flags_deps})
      set(VISP_SONARQUBE_INCLUDE_DIRS "${VISP_SONARQUBE_INCLUDE_DIRS}, ${_inc}")
    endforeach()
    configure_file("${VISP_SOURCE_DIR}/cmake/templates/sonar-project.properties.in"
      "${VISP_BINARY_DIR}/sonar-project.properties"
      @ONLY )

  else()
    #######################################################################
    #
    # for windows platforms
    #
    #######################################################################
    set(FILE_VISP_CONFIG_SCRIPT_IN "cmake/templates/visp-config.bat.in")
    set(FILE_VISP_CONFIG_SCRIPT_INSTALL_IN "cmake/templates/visp-config.bat.in")

    #---------------------------------------------------------------------
    # Updates VISP_CONFIG_SCRIPT_DEF
    #----------------------------------------------------------------------
    vp_list_replace_separator(_cxx_flags ", ")
    set(VISP_CONFIG_SCRIPT_DEFS ${_cxx_flags})

    #---------------------------------------------------------------------
    # Updates VISP_CONFIG_SCRIPT_INC
    # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
    #    VISP_CONFIG_SCRIPT_INC = VISP_CONFIG_SCRIPT_INC_BUILD + _include_flags_deps
    # 2/ For usage with the install tree
    #    VISP_CONFIG_SCRIPT_INC = VISP_CONFIG_SCRIPT_INC_BUILD + _include_flags_deps + _include_flags_src
    #----------------------------------------------------------------------
    set(VISP_CONFIG_SCRIPT_INC_BUILD "%PREFIX%/${VISP_INC_INSTALL_PATH}")
    set(VISP_CONFIG_SCRIPT_INC_SRC_TREE ${_include_flags_src})

    #---------------------------------------------------------------------
    # Updates VISP_CONFIG_SCRIPT_LIBDIR
    # 1/ For usage with the build tree
    # 2/ For usage with the install tree
    #
    # and updates VISP_CONFIG_SCRIPT_LIBS_${config}
    #----------------------------------------------------------------------

    # prepend with ViSP own modules first
    set(TMP_SCRIPT_LIBS_DBG "")
    set(TMP_SCRIPT_LIBS_OPT "")
    foreach(m ${VISP_MODULES_BUILD})
      get_filename_component(m_libname "${TARGET_LOCATION_${m}}" NAME_WE)
      list(APPEND TMP_SCRIPT_LIBS_DBG "${m_libname}${VISP_DEBUG_POSTFIX}.lib")
      list(APPEND TMP_SCRIPT_LIBS_OPT "${m_libname}.lib")
    endforeach()

    # append deps
    set(TMP_SCRIPT_LIBS_DEPS "")
    foreach(m ${VISP_MODULES_BUILD})
      list(APPEND TMP_SCRIPT_LIBS_DEPS ${VISP_MODULE_${m}_LINK_DEPS})
    endforeach()
    vp_list_unique(TMP_SCRIPT_LIBS_DEPS)

    set(TMP_IS_DBG FALSE)
    set(TMP_IS_OPT FALSE)
    foreach(lib ${TMP_SCRIPT_LIBS_DEPS})
      if("${lib}" MATCHES "[d][e][b][u][g]")
        set(TMP_IS_DBG TRUE)
      elseif("${lib}" MATCHES "[o][p][t][i][m][i][z][e][d]")
        set(TMP_IS_OPT TRUE)
      else()
        # Get the library name
        get_filename_component(libname ${lib} NAME)
        if("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
          #MESSAGE("${libname} matches .lib or .Lib")
        else()
          # We need to add .lib suffix
          #MESSAGE("For ${libname} we add .lib suffix")
          set(libname "${libname}.lib")
        endif()

        # Get the library path
        get_filename_component(libpath ${lib} PATH)
        list(APPEND VISP_CONFIG_SCRIPT_LIBDIR_ "${libpath}")

        if(TMP_IS_DBG)
          set(TMP_IS_DBG FALSE)
          list(APPEND TMP_SCRIPT_LIBS_DBG ${libname})
        elseif(TMP_IS_OPT)
          set(TMP_IS_OPT FALSE)
          list(APPEND TMP_SCRIPT_LIBS_OPT ${libname})
        else()
          list(APPEND TMP_SCRIPT_LIBS_DBG ${libname})
          list(APPEND TMP_SCRIPT_LIBS_OPT ${libname})
        endif()
      endif()
    endforeach(lib)

    vp_list_unique(TMP_SCRIPT_LIBS_DBG)
    vp_list_unique(TMP_SCRIPT_LIBS_OPT)

    # Format the string
    set(VISP_CONFIG_SCRIPT_LIBS_DEBUG "${TMP_SCRIPT_LIBS_DBG}")
    set(VISP_CONFIG_SCRIPT_LIBS_OPTIMIZED "${TMP_SCRIPT_LIBS_OPT}")

    # Format the string
    string(REGEX REPLACE "lib/Release" "lib/$(ConfigurationName)" VISP_CONFIG_SCRIPT_LIBDIR_ "${VISP_CONFIG_SCRIPT_LIBDIR_}")
    string(REGEX REPLACE "lib/Debug" "lib/$(ConfigurationName)" VISP_CONFIG_SCRIPT_LIBDIR_ "${VISP_CONFIG_SCRIPT_LIBDIR_}")

    # 1/ For usage with the build tree
    set(VISP_CONFIG_SCRIPT_INC "${VISP_CONFIG_SCRIPT_INC_BUILD}")
    list(APPEND VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC_SRC_TREE})
    list(APPEND VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC_DEPS})
    vp_list_unique(VISP_CONFIG_SCRIPT_INC)
    set(VISP_CONFIG_SCRIPT_LIBDIR "%PREFIX%/lib")
    list(APPEND VISP_CONFIG_SCRIPT_LIBDIR "%PREFIX%/lib/$(ConfigurationName)")
    list(APPEND VISP_CONFIG_SCRIPT_LIBDIR ${VISP_CONFIG_SCRIPT_LIBDIR_})
    vp_list_unique(VISP_CONFIG_SCRIPT_LIBDIR)
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_CONFIG_SCRIPT_IN}" "${FILE_VISP_CONFIG_SCRIPT}" @ONLY)

    # 2/ For usage with the install tree
    set(VISP_CONFIG_SCRIPT_INC "${VISP_CONFIG_SCRIPT_INC_BUILD}")
    list(APPEND VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC_DEPS})
    vp_list_unique(VISP_CONFIG_SCRIPT_INC)
    set(VISP_CONFIG_SCRIPT_LIBDIR "%PREFIX%/${VISP_ARCH}/${VISP_RUNTIME}/lib")
    list(APPEND VISP_CONFIG_SCRIPT_LIBDIR ${VISP_CONFIG_SCRIPT_LIBDIR_})
    vp_list_unique(VISP_CONFIG_SCRIPT_LIBDIR)
    configure_file("${VISP_SOURCE_DIR}/${FILE_VISP_CONFIG_SCRIPT_INSTALL_IN}" "${FILE_VISP_CONFIG_SCRIPT_INSTALL}" @ONLY)
  endif()

endif() # DEFINED CMAKE_HELPER_SCRIPT
