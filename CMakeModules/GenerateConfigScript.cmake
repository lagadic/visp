#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
# This file generates the ViSP-2 library config shell script: "visp-config"
# from visp-config.in (in VISP_SOURCE_DIR).
#
# Authors:
# Fabien Spindler
#
#############################################################################


IF (UNIX)
  #######################################################################
  #
  # for Unix platforms: Linux, OSX
  #
  ####################################################################### 

  SET(FILE_VISP_CONFIG_SCRIPT_IN "${VISP_SOURCE_DIR}/CMakeModules/visp-config.in")
  SET(FILE_VISP_CONFIG_SCRIPT    "${BINARY_OUTPUT_PATH}/visp-config")
  
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")
 
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_CFLAGS
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_CFLAGS ${VISP_OPENMP_FLAGS})
  LIST(APPEND VISP_CONFIG_SCRIPT_CFLAGS "${VISP_DEFS}")
  LIST(APPEND VISP_CONFIG_SCRIPT_CFLAGS "-I$PREFIX/${CMAKE_INSTALL_INCLUDEDIR}")

  FOREACH(INCDIR ${VISP_EXTERN_INCLUDE_DIRS})
    LIST(APPEND VISP_CONFIG_SCRIPT_CFLAGS "-I${INCDIR}")
  ENDFOREACH(INCDIR)

  # Suppress twins
  LIST(REMOVE_DUPLICATES VISP_CONFIG_SCRIPT_CFLAGS)

  # Format the string to suppress CMake separators ";"
  SET(VISP_CONFIG_SCRIPT_CFLAGS_REFORMATED "")
  FOREACH(element ${VISP_CONFIG_SCRIPT_CFLAGS})
    SET(VISP_CONFIG_SCRIPT_CFLAGS_REFORMATED "${VISP_CONFIG_SCRIPT_CFLAGS_REFORMATED} ${element}")
  ENDFOREACH(element)
  SET(VISP_CONFIG_SCRIPT_CFLAGS ${VISP_CONFIG_SCRIPT_CFLAGS_REFORMATED})
#  MESSAGE(": ${VISP_CONFIG_SCRIPT_CFLAGS}")


  IF(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler 
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static 
    # library build
    SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} -ftest-coverage -fprofile-arcs")
  ENDIF(BUILD_TEST_COVERAGE)


  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_EXTERN_LIBS
  #
  # add "-l" to library names
  # skip *.so, *.a, *.dylib, -framework*, -l*
  #
  #----------------------------------------------------------------------
 
  # Manage the lib path
  SET(VISP_CONFIG_SCRIPT_TMP_LDFLAGS)
  FOREACH(dir ${VISP_EXTERN_LINK_DIR})
    LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LDFLAGS "-L${dir}") 
  ENDFOREACH(dir)
  #MESSAGE("VISP_EXTERN_LINK_DIR: ${VISP_EXTERN_LINK_DIR}")
  #MESSAGE("VISP_CONFIG_SCRIPT_TMP_LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
  # convert semicolon-separated vector to space-separated string 
  FOREACH(val ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS})
     SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_EXTERN_LIBS} ${val}")
  ENDFOREACH(val)

  # Manage the libs	
  LIST(REMOVE_ITEM VISP_EXTERN_LIBRARIES "debug")
  LIST(REMOVE_ITEM VISP_EXTERN_LIBRARIES "optimized")
  SET(TMP_LIBS)
  FOREACH(lib ${VISP_EXTERN_LIBRARIES})
    IF("${lib}" MATCHES "[-][f][r][a][m][e][w][o][r][k]+.")
      # does nothing
      LIST(APPEND TMP_LIBS ${lib})
    ELSEIF("${lib}" MATCHES ".[.][f][r][a][m][e][w][o][r][k]+$")
      # replace /path/name.framework by -framework name
      GET_FILENAME_COMPONENT(FRAMEWORK ${lib} NAME_WE)
      #MESSAGE("add -framework ${FRAMEWORK}")
      LIST(APPEND TMP_LIBS "-framework ${FRAMEWORK}")
    ELSEIF("${lib}" MATCHES ".[.][s][o]+$" OR "${lib}" MATCHES ".[.][a]+$")
      # does nothing
      LIST(APPEND TMP_LIBS ${lib})
    ELSEIF("${lib}" MATCHES ".[.][d][y][l][i][b]+$")
      # does nothing
      LIST(APPEND TMP_LIBS ${lib})
    ELSEIF("${lib}" MATCHES "[-][l]+.")
      # does nothing
      LIST(APPEND TMP_LIBS ${lib})
    ELSE()
      # add -l prefix
      #MESSAGE("add -l${lib}")
      LIST(APPEND TMP_LIBS "-l${lib}")
    ENDIF()
  ENDFOREACH(lib)

  FOREACH(val ${TMP_LIBS})
     SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_EXTERN_LIBS} ${val}")
  ENDFOREACH(val)

  #MESSAGE("EXTERN LIBS : ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
 
  # prepend with ViSP own lib dir
  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-L$PREFIX/${CMAKE_INSTALL_LIBDIR} -l${VISP_INTERN_LIBRARY} ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
  IF(UNIX)
    IF(NOT APPLE)
      SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-Wl,-rpath,$PREFIX/${CMAKE_INSTALL_LIBDIR} ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
    ENDIF(NOT APPLE)
  ENDIF(UNIX)

  SET(VISP_ECHO_NO_NEWLINE_CHARACTER "")
  SET(VISP_ECHO_NO_NEWLINE_OPTION "")
  IF(APPLE)
    SET(VISP_ECHO_NO_NEWLINE_CHARACTER "\\c")
  ELSE(APPLE)
    SET(VISP_ECHO_NO_NEWLINE_OPTION "-n")
  ENDIF(APPLE)

  #---------------------------------------------------------------------
  # Updates the visp-config shell script
  #----------------------------------------------------------------------
  CONFIGURE_FILE(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})

ELSE(UNIX)
  #######################################################################
  #
  # for windows platforms
  #
  ####################################################################### 
  SET(FILE_VISP_CONFIG_SCRIPT_IN "${VISP_SOURCE_DIR}/CMakeModules/visp-config.bat.in")
  SET(FILE_VISP_CONFIG_SCRIPT    "${BINARY_OUTPUT_PATH}/visp-config.bat")
  
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_DEF
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_DEFS "")
  FOREACH(def ${VISP_DEFS})
    #MESSAGE("def to process: ${def}")
    IF("${def}" MATCHES "[-][D]+.")
    	#MESSAGE("${def} matches -D")
        STRING(REGEX REPLACE "[-][D]" "" def ${def})
        STRING(REGEX REPLACE "[ ]" ";" def ${def})
	#MESSAGE("new ${def} without -D")
    ENDIF("${def}" MATCHES "[-][D]+.")
    SET(VISP_CONFIG_SCRIPT_DEFS "${def}, ${VISP_CONFIG_SCRIPT_DEFS}")
  ENDFOREACH(def)

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_INCLUDE
  #----------------------------------------------------------------------
  LIST(APPEND VISP_EXTERN_INCLUDE_DIRS "%PREFIX%/${CMAKE_INSTALL_INCLUDEDIR}")
  LIST(REMOVE_DUPLICATES VISP_EXTERN_INCLUDE_DIRS)

  # Format the string
  FOREACH(element ${VISP_EXTERN_INCLUDE_DIRS})
    SET(VISP_CONFIG_SCRIPT_INC "\"${element}\"; ${VISP_CONFIG_SCRIPT_INC}")
  ENDFOREACH(element)

#  MESSAGE(VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC})

  #---------------------------------------------------------------------
  # Updates VISP_OPENMP_SUPPORT
  #----------------------------------------------------------------------
  IF(VISP_OPENMP_FLAGS)
    SET(VISP_OPENMP_SUPPORT "OpenMP support: Yes")
  ELSE()
    SET(VISP_OPENMP_SUPPORT "OpenMP support: No")
  ENDIF()
  
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_LIBDIR
  #----------------------------------------------------------------------
  SET(TMP_SCRIPT_LIBDIR "%PREFIX%\\lib")
  #SET(TMP_SCRIPT_LIBDIR ${TMP_SCRIPT_LIBDIR} "%PREFIX%\\lib\\$(Outdir)")
  SET(TMP_SCRIPT_LIBDIR ${TMP_SCRIPT_LIBDIR} "%PREFIX%\\lib\\$(ConfigurationName)")
  #MESSAGE("VISP_EXTERN_LINK_DIR ${VISP_EXTERN_LINK_DIR}")
  FOREACH(var ${VISP_EXTERN_LINK_DIR})
    #MESSAGE("var to process: ${var}")
    IF("${var}" MATCHES "[-][L]+.")
    	#MESSAGE("${var} matches -L")
        STRING(REGEX REPLACE "[-][L]" "" var ${var})
	#MESSAGE("new ${newvar} without -L")
    ENDIF("${var}" MATCHES "[-][L]+.")
    LIST(APPEND TMP_SCRIPT_LIBDIR ${var})
    LIST(APPEND TMP_SCRIPT_LIBDIR "${var}\\$(ConfigurationName)")
  ENDFOREACH(var)

  FOREACH(lib ${VISP_EXTERN_LIBRARIES})
    # Get the library path 
    GET_FILENAME_COMPONENT(libpath ${lib} PATH)
    #MESSAGE("library path: ${libpath}")

    LIST(APPEND TMP_SCRIPT_LIBDIR ${libpath})
  ENDFOREACH(lib)

  #MESSAGE("TMP_SCRIPT_LIBDIR: ${TMP_SCRIPT_LIBDIR}")

  # Suppress twins
  IF(TMP_SCRIPT_LIBDIR)
    LIST(REMOVE_DUPLICATES TMP_SCRIPT_LIBDIR)
  ENDIF()
  # Format the string
  FOREACH(element ${TMP_SCRIPT_LIBDIR})
    SET(VISP_CONFIG_SCRIPT_LIBDIR "\"${element}\"; ${VISP_CONFIG_SCRIPT_LIBDIR}")
  ENDFOREACH(element)

#  MESSAGE("VISP_CONFIG_SCRIPT_LIBDIR: ${VISP_CONFIG_SCRIPT_LIBDIR}")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_LIBS
  #----------------------------------------------------------------------
  SET(TMP_SCRIPT_LIBS_DEBUG "${VISP_INTERN_LIBRARY}${CMAKE_DEBUG_POSTFIX}.lib")
  SET(TMP_SCRIPT_LIBS_OPTIMIZED "${VISP_INTERN_LIBRARY}.lib")

  #MESSAGE(VISP_EXTERN_LIBRARIES: ${VISP_EXTERN_LIBRARIES})
  SET(TMP_IS_DEBUG FALSE)
  SET(TMP_IS_OPTIMIZED FALSE)
  FOREACH(lib ${VISP_EXTERN_LIBRARIES})
    IF("${lib}" MATCHES "[d][e][b][u][g]")
	  SET(TMP_IS_DEBUG TRUE)
	ELSEIF("${lib}" MATCHES "[o][p][t][i][m][i][z][e][d]")
	  SET(TMP_IS_OPTIMIZED TRUE)
	ELSE()
	  
      # Get the library name
      GET_FILENAME_COMPONENT(libname ${lib} NAME)
      IF("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
        #MESSAGE("${libname} matches .lib or .Lib")
      ELSE("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
        # We need to add .lib suffix
        #MESSAGE("For ${libname} we add .lib suffix")
        SET(libname "${libname}.lib")
      ENDIF("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
	  
	  IF(TMP_IS_DEBUG)
        SET(TMP_IS_DEBUG FALSE)
		LIST(APPEND TMP_SCRIPT_LIBS_DEBUG ${libname})
	  ELSEIF(TMP_IS_OPTIMIZED)
	    SET(TMP_IS_OPTIMIZED FALSE)
		LIST(APPEND TMP_SCRIPT_LIBS_OPTIMIZED ${libname})
      ELSE()
		LIST(APPEND TMP_SCRIPT_LIBS_DEBUG ${libname})
		LIST(APPEND TMP_SCRIPT_LIBS_OPTIMIZED ${libname})
	  ENDIF()
    ENDIF()

  ENDFOREACH(lib)

  # Format the string
  FOREACH(element ${TMP_SCRIPT_LIBS_DEBUG})
    SET(VISP_CONFIG_SCRIPT_LIBS_DEBUG "${VISP_CONFIG_SCRIPT_LIBS_DEBUG}; ${element}")
  ENDFOREACH(element)
  FOREACH(element ${TMP_SCRIPT_LIBS_OPTIMIZED})
    SET(VISP_CONFIG_SCRIPT_LIBS_OPTIMIZED "${VISP_CONFIG_SCRIPT_LIBS_OPTIMIZED}; ${element}")
  ENDFOREACH(element)

  #MESSAGE(VISP_CONFIG_SCRIPT_LIBS: ${VISP_CONFIG_SCRIPT_LIBS})

  #---------------------------------------------------------------------
  # Updates the visp-config shell script
  #----------------------------------------------------------------------
  CONFIGURE_FILE(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})
ENDIF(UNIX)

