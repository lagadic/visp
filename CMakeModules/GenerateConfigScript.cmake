##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## This file generates the ViSP-2 library config shell script: "visp2-config"
## from visp2-config.in (in VISP_SOURCE_DIR) 
##
IF (UNIX)
  SET(FILE_VISP_CONFIG_SCRIPT_IN "${VISP_SOURCE_DIR}/CMakeModules/visp-config.in")
  SET(FILE_VISP_CONFIG_SCRIPT    "${VISP_BINARY_DIR}/visp-config")
  
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_PREFIX
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_PREFIX "${CMAKE_INSTALL_PREFIX}")
 
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_CFLAGS
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_CFLAGS "")
  SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} ${VISP_DEFS}")
  SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} -I${CMAKE_INSTALL_PREFIX}/include")

  FOREACH(INCDIR ${VISP_EXTERN_INCLUDE_DIR})
    SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} -I${INCDIR}")
  ENDFOREACH(INCDIR)

  IF(BIT3_FOUND)
    SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} -DBT1003")
  ENDIF(BIT3_FOUND)

  # Add build options for test coverage. Currently coverage is only supported
  # on gcc compiler 
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(VISP_CONFIG_SCRIPT_CFLAGS "${VISP_CONFIG_SCRIPT_CFLAGS} -fprofile-arcs -ftest-coverage")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
  
  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_EXTERN_LIBS
  #
  # add "-l" to "real" library names ending with .so or .a
  # add "-L" to library path
  # skip *.so  and *.a
  #
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "")
  SET(VISP_CONFIG_SCRIPT_TMP_LDFLAGS "")
  SET(VISP_CONFIG_SCRIPT_TMP_LIBS "")
  FOREACH(LIB ${VISP_EXTERN_LIBS})
    #MESSAGE("LIB to process: ${LIB}")
    IF("${LIB}" MATCHES ".+[.][s][o]" OR "${LIB}" MATCHES ".+[.][a]")
      #MESSAGE("${LIB} matches .so or .a, processing to create -L and -l")
      GET_FILENAME_COMPONENT(path ${LIB} PATH)

      #----
      # Add test to be sure that the library path is unique
      SET(LDFLAGS_ADDED 0)
      FOREACH(ldflags ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS})
	SET(tmp_ldflags " -L${path}")
	#MESSAGE("compare:")
	#MESSAGE("dummy${tmp_ldflags}dummy")
	#MESSAGE("dummy${ldflags}dummy")
	STRING(COMPARE EQUAL "${tmp_ldflags}" "${ldflags}" INLDFLAGS)
	  
	IF(INLDFLAGS)
	  SET(LDFLAGS_ADDED 1)
#	  MESSAGE("equal----")
	#ELSE(INLDFLAGS)
	 # MESSAGE("not equal----")
	ENDIF(INLDFLAGS)
      ENDFOREACH(ldflags)
      IF(NOT ${LDFLAGS_ADDED})
	#MESSAGE("add ${path} ")
	# Add the path to ldflags
	SET(VISP_CONFIG_SCRIPT_TMP_LDFLAGS "${VISP_CONFIG_SCRIPT_TMP_LDFLAGS} -L${path}")

      ENDIF(NOT ${LDFLAGS_ADDED})


      #MESSAGE("LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
      GET_FILENAME_COMPONENT(libs ${LIB} NAME_WE)
      STRING(REGEX REPLACE "lib" "-l" libs ${libs})
      
      SET(VISP_CONFIG_SCRIPT_TMP_LIBS "${VISP_CONFIG_SCRIPT_TMP_LIBS} ${libs}")
      #MESSAGE("LIBS: ${VISP_CONFIG_SCRIPT_TMP_LIBS}")
      
      
      
      # convert semicolon-separated vector to space-separated string 
      SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_EXTERN_LIBS} -l${LIB}")
      
    ELSE("${LIB}" MATCHES ".+[.][s][o]" OR "${LIB}" MATCHES ".+[.][a]")
      #MESSAGE("${LIB} is not a lib in .so or .a, to keep")
      SET(VISP_CONFIG_SCRIPT_TMP_LIBS "${VISP_CONFIG_SCRIPT_TMP_LIBS} ${LIB}")
    ENDIF("${LIB}" MATCHES ".+[.][s][o]" OR "${LIB}" MATCHES ".+[.][a]")
  ENDFOREACH(LIB)

  #MESSAGE("LDFLAGS: ${VISP_CONFIG_SCRIPT_TMP_LDFLAGS}")
  #MESSAGE("LIBS: ${VISP_CONFIG_SCRIPT_TMP_LIBS}")


  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS "${VISP_CONFIG_SCRIPT_TMP_LDFLAGS} ${VISP_CONFIG_SCRIPT_TMP_LIBS}")

  # prepend with ViSP own lib dir
  SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-L${CMAKE_INSTALL_PREFIX}/lib -l${VISP_INTERN_LIBS} ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
  IF(UNIX)
    IF(NOT APPLE)
      SET(VISP_CONFIG_SCRIPT_EXTERN_LIBS  "-Wl,-rpath,${CMAKE_INSTALL_PREFIX}/lib ${VISP_CONFIG_SCRIPT_EXTERN_LIBS}")
    ENDIF(NOT APPLE)
  ENDIF(UNIX)

  #---------------------------------------------------------------------
  # Updates VISP_CONFIG_SCRIPT_VERSION
  #----------------------------------------------------------------------
  SET(VISP_CONFIG_SCRIPT_VERSION "${VISP_VERSION_FULL}")

  #---------------------------------------------------------------------
  # Updates the visp-config shell script
  #----------------------------------------------------------------------
  CONFIGURE_FILE(${FILE_VISP_CONFIG_SCRIPT_IN} ${FILE_VISP_CONFIG_SCRIPT})

ELSE(UNIX)
  #MESSAGE("GenerateConfigScript works only on Unix platforms, sorry.")
ENDIF(UNIX)

