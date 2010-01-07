#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit.
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# This file generates the ViSP-2 library config shell script: "visp2-config"
# from visp2-config.in (in VISP_SOURCE_DIR).
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
  SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_DEFS}")
  LIST(APPEND VISP_CONFIG_SCRIPT_CFLAGS "-I$PREFIX/include")

  FOREACH(INCDIR ${VISP_EXTERN_INCLUDE_DIR})
    LIST(APPEND VISP_CONFIG_SCRIPT_CFLAGS "-I${INCDIR}")
  ENDFOREACH(INCDIR)

  # Suppress twins
  SET(myvar "")
  FOREACH(element ${VISP_CONFIG_SCRIPT_CFLAGS})
#     MESSAGE("element: ${element}")
     SET(toadd 1)
     FOREACH(elementadded ${myvar})
#       MESSAGE("compare ${element} and ${elementadded}")
       STRING(COMPARE EQUAL "${element}" "${elementadded}" iseq)
       IF(${iseq})
#	 MESSAGE("is equal")
         SET(toadd 0)
       ENDIF(${iseq})
     ENDFOREACH(elementadded ${myvar})
     IF(${toadd})
#       MESSAGE("need to add ${element}")
       LIST(APPEND myvar ${element})
     ENDIF(${toadd})
  ENDFOREACH(element ${VISP_CONFIG_SCRIPT_CFLAGS})
  SET(VISP_CONFIG_SCRIPT_CFLAGS ${myvar})

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
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
  ENDIF(BUILD_TEST_COVERAGE)


  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_EXTERN_LIBS
  #
  # add "-l" to "real" library names ending with .so or .a
  # add "-L" to library path
  # skip *.so  and *.a
  #
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS)
  SET(VISP_CONFIG_SCRIPT_TMP_LDFLAGS)
  FOREACH(dir ${VISP_EXTERN_LINK_DIR})
    LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LDFLAGS "-L${dir}") 
  ENDFOREACH(dir)
  #MESSAGE("VISP_EXTERN_LINK_DIR: ${VISP_EXTERN_LINK_DIR}")
  #MESSAGE("VISP_CONFIG_SCRIPT_TMP_LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
  SET(VISP_CONFIG_SCRIPT_TMP_LIBS "")
  #MESSAGE("VISP_EXTERN_LIBS: ${VISP_EXTERN_LIBS}")
  FOREACH(lib ${VISP_EXTERN_LIBS})
    #MESSAGE("library to process: ${lib}")
    IF("${lib}" MATCHES ".[.][s][o]+$" OR "${lib}" MATCHES ".[.][a]+$")
      #MESSAGE("${lib} matches .so or .a, processing to create -L and -l")

      # If the string lib to process is like: /usr/lib/libgtk-x11-2.0.so
      # with GET_FILENAME_COMPONENT(libname_we ${LIB} NAME_WE) we get: 
      # libgtk-x11-2 (it should be libgtk-x11-2.0)
      # with GET_FILENAME_COMPONENT(libext ${LIB} EXT) we get: .0.so
      # (it should be .so)
      # So we use GET_FILENAME_COMPONENT(libname ${LIB} NAME) to get: 
      # libgtk-x11-2.0.so which is good
      # The idea is than to suppress the extension .so (or .a) and to replace
      # lib by -l

      # Get the library path 
      GET_FILENAME_COMPONENT(path ${lib} PATH)
      #MESSAGE("library path: ${path}")

      # Get the library name
      GET_FILENAME_COMPONENT(libname ${lib} NAME)
      #MESSAGE("library name: ${libname}")
      # Suppress the .so or .a suffix extension
      IF("${libname}" MATCHES ".+[.][s][o]")
        #STRING(REGEX REPLACE ".so" "" libname ${libname})
        STRING(REGEX REPLACE "[.][s][o]" "" libname ${libname})
      ENDIF("${libname}" MATCHES ".+[.][s][o]")
      IF("${libname}" MATCHES ".+[.][a]")
        #STRING(REGEX REPLACE ".a" "" libname ${libname})
        STRING(REGEX REPLACE "[.][a]" "" libname ${libname})
      ENDIF("${libname}" MATCHES ".+[.][a]")
      #MESSAGE("library name without extension: ${libname}")
      # In all cases, replace lib prefix by -l
      STRING(REGEX REPLACE "^[l][i][b]" "-l" libname ${libname})
      #MESSAGE("processed library name: ${libname}")

      LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LIBS ${libname})
      #----
      # Add test to be sure that the library path is unique in 
      # VISP_CONFIG_SCRIPT_TMP_LDFLAGS
      SET(LDFLAGS_ADDED 0)

      #MESSAGE("actual LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")

      FOREACH(ldflags ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS})
	SET(ldflags_to_add "-L${path}")
	#MESSAGE("compare:")
	#MESSAGE("dummy${ldflags_to_add}dummy")
	#MESSAGE("dummy${ldflags}dummy")
	STRING(COMPARE EQUAL "${ldflags_to_add}" "${ldflags}" INLDFLAGS)
	  
	IF(INLDFLAGS)
	  SET(LDFLAGS_ADDED 1)
	  #MESSAGE("equal----")
	#ELSE(INLDFLAGS)
	  #MESSAGE("not equal----")
	ENDIF(INLDFLAGS)
      ENDFOREACH(ldflags)
      IF(NOT ${LDFLAGS_ADDED})
	#MESSAGE("add ${path} ")
	# Add the path to ldflags
	SET(LDFLAGS_TO_ADD "-L${path}")
	LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LDFLAGS ${LDFLAGS_TO_ADD})

      ENDIF(NOT ${LDFLAGS_ADDED})

      #MESSAGE("LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
      #MESSAGE("LIBS: ${VISP_CONFIG_SCRIPT_TMP_LIBS}")
#    ELSEIF("${lib}" MATCHES ".+[.][f][r][a][m][e][w][o][r][k]")
    ELSEIF("${lib}" MATCHES ".[.][f][r][a][m][e][w][o][r][k]+$")
      # Specific case of APPLE frameworks
      #MESSAGE("Framework case: ${lib}")
      # Get the framework name
      GET_FILENAME_COMPONENT(framework ${lib} NAME_WE)
      #MESSAGE("Framework name: ${framework}")
      LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LIBS "-framework ${framework}")
    ELSE("${lib}" MATCHES ".[.][s][o]+$" OR "${lib}" MATCHES ".[.][a]+$")
      #MESSAGE("${lib} is not a library in .so or .a, to keep")
      LIST(APPEND VISP_CONFIG_SCRIPT_TMP_LIBS ${lib})
    ENDIF("${lib}" MATCHES ".[.][s][o]+$" OR "${lib}" MATCHES ".[.][a]+$")
  ENDFOREACH(lib)

  #MESSAGE("LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
  #MESSAGE("LIBS: ${VISP_CONFIG_SCRIPT_TMP_LIBS}")

  # convert semicolon-separated vector to space-separated string 
  FOREACH(val ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS})
     SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_EXTERN_LIBS} ${val}")
  ENDFOREACH(val)
  FOREACH(val ${VISP_CONFIG_SCRIPT_TMP_LIBS})
     SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_EXTERN_LIBS} ${val}")
  ENDFOREACH(val)

  #MESSAGE("EXTERN LIBS: ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")

  # prepend with ViSP own lib dir
  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-L$PREFIX/lib -l${VISP_INTERN_LIBS} ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
  IF(UNIX)
    IF(NOT APPLE)
      SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-Wl,-rpath,$PREFIX/lib ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
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
  SET(TMP_SCRIPT_INC "%PREFIX%/include")
#  MESSAGE(TMP_SCRIPT_INC: ${TMP_SCRIPT_INC})

  # Suppress twins
  FOREACH(element ${VISP_EXTERN_INCLUDE_DIR})
#     MESSAGE("element: ${element}")
     SET(toadd 1)
     FOREACH(elementadded ${TMP_SCRIPT_INC})
#       MESSAGE("compare ${element} and ${elementadded}")
       STRING(COMPARE EQUAL "${element}" "${elementadded}" iseq)
       IF(${iseq})
#	 MESSAGE("is equal")
         SET(toadd 0)
       ENDIF(${iseq})
     ENDFOREACH(elementadded ${TMP_SCRIPT_INC})
     IF(${toadd})
#       MESSAGE("need to add ${element}")
       LIST(APPEND TMP_SCRIPT_INC ${element})
     ENDIF(${toadd})
  ENDFOREACH(element ${VISP_EXTERN_INCLUDE_DIR})

  # Format the string
  FOREACH(element ${TMP_SCRIPT_INC})
    SET(VISP_CONFIG_SCRIPT_INC "\"${element}\"; ${VISP_CONFIG_SCRIPT_INC}")
  ENDFOREACH(element)

#  MESSAGE(VISP_CONFIG_SCRIPT_INC ${VISP_CONFIG_SCRIPT_INC})

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_LIBDIR
  #----------------------------------------------------------------------
  SET(TMP_SCRIPT_LIBDIR "%PREFIX%\\lib")
  SET(TMP_SCRIPT_LIBDIR ${TMP_SCRIPT_LIBDIR} "%PREFIX%\\lib\\$(Outdir)")
  SET(TMP_SCRIPT_LIBDIR ${TMP_SCRIPT_LIBDIR} "%PREFIX%\\lib\\$(ConfigurationName)")
  FOREACH(var ${VISP_EXTERN_LINK_DIR})
    #MESSAGE("var to process: ${var}")
    IF("${var}" MATCHES "[-][L]+.")
    	#MESSAGE("${var} matches -L")
        STRING(REGEX REPLACE "[-][L]" "" var ${var})
	#MESSAGE("new ${newvar} without -L")
    ENDIF("${var}" MATCHES "[-][L]+.")
    LIST(APPEND TMP_SCRIPT_LIBDIR ${var})
  ENDFOREACH(var)

  FOREACH(lib ${VISP_EXTERN_LIBS})
    # Get the library path 
    GET_FILENAME_COMPONENT(libpath ${lib} PATH)
    #MESSAGE("library path: ${libpath}")

    LIST(APPEND TMP_SCRIPT_LIBDIR ${libpath})
  ENDFOREACH(lib)

  # MESSAGE("TMP_SCRIPT_LIBDIR: ${TMP_SCRIPT_LIBDIR}")

  # Suppress twins
  SET(TMP_TMP_SCRIPT_LIBDIR "")
  FOREACH(element ${TMP_SCRIPT_LIBDIR})
#     MESSAGE("element: ${element}")
     SET(toadd 1)
     FOREACH(elementadded ${TMP_TMP_SCRIPT_LIBDIR})
#       MESSAGE("compare ${element} and ${elementadded}")
       STRING(COMPARE EQUAL "${element}" "${elementadded}" iseq)
       IF(${iseq})
#	 MESSAGE("is equal")
         SET(toadd 0)
       ENDIF(${iseq})
     ENDFOREACH(elementadded)
     IF(${toadd})
#       MESSAGE("need to add ${element}")
       LIST(APPEND TMP_TMP_SCRIPT_LIBDIR ${element})
     ENDIF(${toadd})
  ENDFOREACH(element)

  # Format the string
  FOREACH(element ${TMP_TMP_SCRIPT_LIBDIR})
    SET(VISP_CONFIG_SCRIPT_LIBDIR "\"${element}\", ${VISP_CONFIG_SCRIPT_LIBDIR}")
  ENDFOREACH(element)

#  MESSAGE("VISP_CONFIG_SCRIPT_LIBDIR: ${VISP_CONFIG_SCRIPT_LIBDIR}")

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_LIBS
  #----------------------------------------------------------------------
  SET(TMP_SCRIPT_LIBS "${VISP_INTERN_LIBS}.lib")

  #MESSAGE(VISP_EXTERN_LIBS: ${VISP_EXTERN_LIBS})
  FOREACH(lib ${VISP_EXTERN_LIBS})
    # Get the library name
    GET_FILENAME_COMPONENT(libname ${lib} NAME)
    IF("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
      #MESSAGE("${libname} matches .lib or .Lib")
    ELSE("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")
      # We need to add .lib suffix
      #MESSAGE("For ${libname} we add .lib suffix")
      SET(libname "${libname}.lib")
    ENDIF("${libname}" MATCHES ".+[.][l][i][b]" OR "${libname}" MATCHES ".+[.][L][i][b]")


    LIST(APPEND TMP_SCRIPT_LIBS ${libname})
  ENDFOREACH(lib)

  # Format the string
  FOREACH(element ${TMP_SCRIPT_LIBS})
    SET(VISP_CONFIG_SCRIPT_LIBS "${VISP_CONFIG_SCRIPT_LIBS} ${element}")
  ENDFOREACH(element)

  #MESSAGE(VISP_CONFIG_SCRIPT_LIBS: ${VISP_CONFIG_SCRIPT_LIBS})

  #---------------------------------------------------------------------
  # Updates the visp-config shell script
  #----------------------------------------------------------------------
  CONFIGURE_FILE(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})
ENDIF(UNIX)

