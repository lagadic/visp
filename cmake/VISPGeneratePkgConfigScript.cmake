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

if(MINGW)
  # not implemented yet
  return()
endif()

if(UNIX)
  #######################################################################
  #
  # for Unix platforms: Linux, OSX
  #
  ####################################################################### 
  set(FILE_VISP_CONFIG_SCRIPT_IN "cmake/templates/visp-config.in")
  set(FILE_VISP_CONFIG_SCRIPT    "${BINARY_OUTPUT_PATH}/visp-config")

  set(FILE_VISP_CONFIG_SCRIPT_INSTALL_IN "cmake/templates/visp-config.install.in")
  set(FILE_VISP_CONFIG_SCRIPT_INSTALL    "${VISP_BINARY_DIR}/unix-install/visp-config")
  
  set(FILE_VISP_CONFIG_PC_INSTALL_IN "cmake/templates/visp.pc.in")
  set(FILE_VISP_CONFIG_PC_INSTALL    "${VISP_BINARY_DIR}/unix-install/visp.pc")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  set(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")
 
  #---------------------------------------------------------------------
  # Updates vars needed to update VISP_CONFIG_CFLAGS_SCRIPT
  # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
  #    VISP_CONFIG_CFLAGS_SCRIPT = VISP_CONFIG_CFLAGS + VISP_CONFIG_CFLAGS_DEPS + VISP_CONFIG_CFLAGS_SRC_TREE
  # 2/ For usage with the install tree
  #    VISP_CONFIG_CFLAGS_SCRIPT = VISP_CONFIG_CFLAGS + VISP_CONFIG_CFLAGS_DEPS
  #----------------------------------------------------------------------
  if(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    list(APPEND VISP_CONFIG_CFLAGS "-ftest-coverage")
    list(APPEND VISP_CONFIG_CFLAGS "-fprofile-arcs")
  endif()

  # Propagate cxx11 compiler option if enabled during ViSP build
  if(VISP_HAVE_CPP11_COMPATIBILITY)
    list(APPEND VISP_CONFIG_CFLAGS ${CPP11_CXX_FLAGS})
  endif()
  # Propagate openmp compiler option if enabled during ViSP build
  if(VISP_HAVE_OPENMP)
    list(APPEND VISP_CONFIG_CFLAGS ${OpenMP_CXX_FLAGS})
  endif()

  # include ViSP own include dir
  set(VISP_CONFIG_CFLAGS_SRC_TREE "-I$PREFIX/${VISP_INC_INSTALL_PATH}")

  foreach(m ${VISP_MODULES_BUILD})
    if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
      list(APPEND VISP_CONFIG_CFLAGS_SRC_TREE "-I${VISP_MODULE_${m}_LOCATION}/include")
    endif()
  endforeach()

  foreach(m ${VISP_MODULES_BUILD})
    foreach(inc ${VISP_MODULE_${m}_INC_DEPS})
      list(APPEND VISP_CONFIG_CFLAGS_DEPS "-I${inc}")
    endforeach()
  endforeach()
 
  #---------------------------------------------------------------------
  # Updates vars needed to update VISP_CONFIG_LIBS_SCRIPT
  #----------------------------------------------------------------------
  # prepend with ViSP own modules first
  set(VISP_CONFIG_LIBS_SCRIPT "")
  foreach(m ${VISP_MODULES_BUILD})
    # need to be improved
    if(POLICY CMP0026)
      cmake_policy(PUSH)
      cmake_policy(SET CMP0026 OLD)
      get_target_property(m_libpath ${m} LOCATION_Release)
      cmake_policy(POP)
    else()
      get_target_property(m_libpath ${m} LOCATION_Release)
    endif()
    get_filename_component(m_libname "${m_libpath}" NAME)
    list(APPEND VISP_CONFIG_LIBS_SCRIPT "$PREFIX/${VISP_LIB_INSTALL_PATH}/${m_libname}")
  endforeach()
  # append deps
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_CONFIG_LIBS_SCRIPT ${VISP_MODULE_${m}_LINK_DEPS})
  endforeach()
  vp_list_unique(VISP_CONFIG_LIBS_SCRIPT)
  vp_list_filterout(VISP_CONFIG_LIBS_SCRIPT "debug")
  vp_list_filterout(VISP_CONFIG_LIBS_SCRIPT "optimized")
#  message("VISP_CONFIG_LIBS_SCRIPT: ${VISP_CONFIG_LIBS_SCRIPT}")
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
  # Configure to create <build dir>/bin/visp-config shell script
  # Updates VISP_CONFIG_CFLAGS_SCRIPT before
  #----------------------------------------------------------------------
  set(VISP_CONFIG_CFLAGS_SCRIPT ${VISP_CONFIG_CFLAGS})
  list(APPEND VISP_CONFIG_CFLAGS_SCRIPT ${VISP_CONFIG_CFLAGS_SRC_TREE})
  list(APPEND VISP_CONFIG_CFLAGS_SCRIPT ${VISP_CONFIG_CFLAGS_DEPS})
  vp_list_unique(VISP_CONFIG_CFLAGS_SCRIPT)
  # Format the string to suppress CMake separators ";"
  vp_list_remove_separator(VISP_CONFIG_CFLAGS_SCRIPT)
  configure_file(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})

  #---------------------------------------------------------------------
  # Configure to create <build dir>/unix-install/visp-config shell script
  # that will call visp.pc
  #----------------------------------------------------------------------
  configure_file(${FILE_VISP_CONFIG_SCRIPT_INSTALL_IN} ${FILE_VISP_CONFIG_SCRIPT_INSTALL})

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
  list(APPEND VISP_CONFIG_CFLAGS_PC ${VISP_CONFIG_CFLAGS})
  list(APPEND VISP_CONFIG_CFLAGS_PC ${VISP_CONFIG_CFLAGS_DEPS})
  vp_list_unique(VISP_CONFIG_CFLAGS_PC)
  # Format the string to suppress CMake separators ";"
  vp_list_remove_separator(VISP_CONFIG_CFLAGS_PC)

  # prepend with ViSP own modules first
  set(VISP_CONFIG_LIBS_PC "")
  foreach(m ${VISP_MODULES_BUILD})
    # need to be improved
    if(POLICY CMP0026)
      cmake_policy(PUSH)
      cmake_policy(SET CMP0026 OLD)
      get_target_property(m_libpath ${m} LOCATION_Release)
      cmake_policy(POP)
    else()
      get_target_property(m_libpath ${m} LOCATION_Release)
    endif()
    get_filename_component(m_libname "${m_libpath}" NAME)
    list(APPEND VISP_CONFIG_LIBS_PC "\${libdir}/${m_libname}")
  endforeach()
  # append deps
  foreach(m ${VISP_MODULES_BUILD})
    list(APPEND VISP_CONFIG_LIBS_PC ${VISP_MODULE_${m}_LINK_DEPS})
  endforeach()
  vp_list_filterout(VISP_CONFIG_LIBS_PC "optimized")
  vp_list_filterout(VISP_CONFIG_LIBS_PC "debug")
  vp_list_remove_separator(VISP_CONFIG_LIBS_PC)

# message("VISP_CONFIG_LIBS_PC: ${VISP_CONFIG_LIBS_PC}")

  configure_file(${FILE_VISP_CONFIG_PC_INSTALL_IN} ${FILE_VISP_CONFIG_PC_INSTALL})

else()
  #######################################################################
  #
  # for windows platforms
  #
  ####################################################################### 
  set(FILE_VISP_CONFIG_SCRIPT_IN "cmake/templates/visp-config.bat.in")
  set(FILE_VISP_CONFIG_SCRIPT    "${BINARY_OUTPUT_PATH}/visp-config.bat")
    
  set(FILE_VISP_CONFIG_SCRIPT_INSTALL_IN "cmake/templates/visp-config.bat.in")
  set(FILE_VISP_CONFIG_SCRIPT_INSTALL    "${VISP_BINARY_DIR}/win-install/visp-config-${VISP_ARCH}-${VISP_RUNTIME}.bat")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  set(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_DEF
  #----------------------------------------------------------------------
  set(VISP_CONFIG_SCRIPT_DEFS "")
  set(VISP_OPENMP_SUPPORT "no")
  if(VISP_HAVE_OPENMP)
    set(VISP_CONFIG_SCRIPT_DEFS "${OpenMP_CXX_FLAGS}")
    set(VISP_OPENMP_SUPPORT "yes")
  endif()
  if(VISP_HAVE_CPP11_COMPATIBILITY)
    set(VISP_CONFIG_SCRIPT_DEFS "${CPP11_CXX_FLAGS}, ${VISP_CONFIG_SCRIPT_DEFS}")
  endif()

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_INC
  # 1/ For usage with the build tree (where location of the <source tree>/<module>/include should be added)
  #    VISP_CONFIG_SCRIPT_INC = VISP_CONFIG_SCRIPT_INC_BUILD + VISP_CONFIG_SCRIPT_INC_DEPS
  # 2/ For usage with the install tree
  #    VISP_CONFIG_SCRIPT_INC = VISP_CONFIG_SCRIPT_INC_BUILD + VISP_CONFIG_SCRIPT_INC_SRC_TREE + VISP_CONFIG_SCRIPT_INC_DEPS
  #----------------------------------------------------------------------
  set(VISP_CONFIG_SCRIPT_INC_BUILD "%PREFIX%/${VISP_INC_INSTALL_PATH}")
  set(VISP_CONFIG_SCRIPT_INC_SRC_TREE "")
  foreach(m ${VISP_MODULES_BUILD})
    if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
      list(APPEND VISP_CONFIG_SCRIPT_INC_SRC_TREE "${VISP_MODULE_${m}_LOCATION}/include")
    endif()
  endforeach()
  set(VISP_CONFIG_SCRIPT_INC_DEPS "")
  foreach(m ${VISP_MODULES_BUILD})
    foreach(inc ${VISP_MODULE_${m}_INC_DEPS})
      list(APPEND VISP_CONFIG_SCRIPT_INC_DEPS "${inc}")
    endforeach()
  endforeach()

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
    # need to be improved
    if(POLICY CMP0026)
      cmake_policy(PUSH)
      cmake_policy(SET CMP0026 OLD)
      get_target_property(m_libpath ${m} LOCATION_Release)
      cmake_policy(POP)
    else()
      get_target_property(m_libpath ${m} LOCATION_Release)
    endif()
    get_filename_component(m_libname "${m_libpath}" NAME_WE)
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
  list(REMOVE_DUPLICATES VISP_CONFIG_SCRIPT_LIBDIR)
  configure_file(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})

  # 2/ For usage with the install tree
  set(VISP_CONFIG_SCRIPT_INC "${VISP_CONFIG_SCRIPT_INC_BUILD}")
  list(APPEND VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC_DEPS})
  vp_list_unique(VISP_CONFIG_SCRIPT_INC)
  set(VISP_CONFIG_SCRIPT_LIBDIR "%PREFIX%/${VISP_ARCH}/${VISP_RUNTIME}/lib")
  list(APPEND VISP_CONFIG_SCRIPT_LIBDIR ${VISP_CONFIG_SCRIPT_LIBDIR_})
  list(REMOVE_DUPLICATES VISP_CONFIG_SCRIPT_LIBDIR)
  configure_file(${FILE_VISP_CONFIG_SCRIPT_INSTALL_IN} ${FILE_VISP_CONFIG_SCRIPT_INSTALL})
endif()

#----------------------------------------------------------------------
# customize install target
#----------------------------------------------------------------------
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


