#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
# Try to find gnu scientific library GSL 
# (see http://www.gnu.org/software/gsl/)
# Once run this will define: 
# 
# GSL_FOUND       = system has GSL lib
#
# GSL_LIBRARIES   = full path to the libraries
#    on Unix/Linux with additional linker flags from "gsl-config --libs"
# 
# GSL_INCLUDE_DIRS      = where to find headers 
#
# GSL_LINK_DIRECTORIES = link directories, useful for rpath on Unix
# GSL_EXE_LINKER_FLAGS = rpath on Unix
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(WIN32)
 FIND_LIBRARY(GSL_gsl_LIBRARY
     NAMES gsl
     PATHS 
     "$ENV{GSL_HOME}/lib"
     "$ENV{GSL_DIR}/lib"
     DOC "Where can the GSL (gsl.lib) library be found"
     )
 FIND_LIBRARY(GSL_cblas_LIBRARY
     NAMES cblas
     PATHS 
     "$ENV{GSL_HOME}/lib"
     "$ENV{GSL_DIR}/lib"
     DOC "Where can the GSL (cblas.lib) library be found"
      )
 FIND_LIBRARY(GSL_gsl_LIBRARY_DEBUG
     NAMES gsl_d
     PATHS 
     "$ENV{GSL_HOME}/lib"
     "$ENV{GSL_DIR}/lib"
     DOC "Where can the GSL (gsl.lib) library be found"
     )
 FIND_LIBRARY(GSL_cblas_LIBRARY_DEBUG
     NAMES cblas_d
     PATHS 
     "$ENV{GSL_HOME}/lib"
     "$ENV{GSL_DIR}/lib"
     DOC "Where can the GSL (cblas.lib) library be found"
      )
  SET(GSL_LIBRARIES "optimized" ${GSL_cblas_LIBRARY}
                    "optimized" ${GSL_gsl_LIBRARY}
                    "debug" ${GSL_cblas_LIBRARY_DEBUG}
                    "debug" ${GSL_gsl_LIBRARY_DEBUG})

  FIND_PATH(GSL_INCLUDE_DIR gsl/gsl_linalg.h
      $ENV{GSL_HOME}/include
      $ENV{GSL_DIR}/include
      )

  IF(GSL_INCLUDE_DIR AND GSL_gsl_LIBRARY AND GSL_cblas_LIBRARY
      AND GSL_gsl_LIBRARY_DEBUG AND GSL_cblas_LIBRARY_DEBUG)
    SET(GSL_FOUND TRUE)
  ELSE(GSL_INCLUDE_DIR AND GSL_gsl_LIBRARY AND GSL_cblas_LIBRARY
      AND GSL_gsl_LIBRARY_DEBUG AND GSL_cblas_LIBRARY_DEBUG)
    SET(GSL_FOUND FALSE) 
  ENDIF(GSL_INCLUDE_DIR AND GSL_gsl_LIBRARY AND GSL_cblas_LIBRARY
      AND GSL_gsl_LIBRARY_DEBUG AND GSL_cblas_LIBRARY_DEBUG)

  MARK_AS_ADVANCED(
    GSL_gsl_LIBRARY
    GSL_cblas_LIBRARY
    GSL_gsl_LIBRARY_DEBUG
    GSL_cblas_LIBRARY_DEBUG
    GSL_INCLUDE_DIR
    GSL_LIBRARIES
    GSL_LINK_DIRECTORIES
  )  
ELSE(WIN32)
  IF(UNIX) 
    FIND_PROGRAM(GSL_CONFIG gsl-config
      $ENV{GSL_HOME}/bin
      $ENV{GSL_DIR}/bin
      /usr/bin
      /usr/local/bin
      )
     #MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")
    
    IF (GSL_CONFIG) 
      # set INCLUDE_DIRS to prefix+include
      EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --prefix
	OUTPUT_VARIABLE GSL_PREFIX)
      SET(GSL_INCLUDE_DIR ${GSL_PREFIX}/include)
      #MESSAGE(STATUS "Using GSL from ${GSL_PREFIX}")
      #MESSAGE("GSL_INCLUDE_DIR ${GSL_INCLUDE_DIR}")
    
      ## extract link lib path and name  for rpath  
      EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --libs
	OUTPUT_VARIABLE GSL_CONFIG_LIBS )
      #MESSAGE("GSL_CONFIG_LIBS: ${GSL_CONFIG_LIBS}")

      ## split off the link dirs (for rpath)
      ## use regular expression to match wildcard equivalent "-L*<endchar>"
      ## with <endchar> is a space or a semicolon
      STRING(REGEX MATCHALL "[-][L]([^ ;])+" 
	GSL_LINK_DIRECTORIES_WITH_PREFIX 
	"${GSL_CONFIG_LIBS}" )
      SET(GSL_LINK_DIRECTORIES "")

      #MESSAGE("DBG  GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")
      
      ## remove prefix -L because we need the pure directory for LINK_DIRECTORIES
     
      IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
	STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
      ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
      ## Check if link directory is empty. This can occur with GSL-1.6
      ## In that case we force the link directory to be ${GSL_PREFIX}/lib
      IF(GSL_LINK_DIRECTORIES MATCHES "")
        #MESSAGE("DBG GSL_LINK_DIRECTORIES is empty")
        SET(GSL_LINK_DIRECTORIES "${GSL_PREFIX}/lib")
      ENDIF(GSL_LINK_DIRECTORIES MATCHES "")

      #MESSAGE("DBG GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
#       IF(NOT APPLE)
#       	SET(GSL_EXE_LINKER_FLAGS "-Wl,-rpath,${GSL_LINK_DIRECTORIES}")
#       ENDIF(NOT APPLE)

      #MESSAGE("DBG GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")
      ## use regular expression to match wildcard equivalent "-l*<endchar>"
      ## with <endchar> is a space or a semicolon
      STRING(REGEX MATCHALL "[-][l]([^ ;])+" 
	GSL_LINK_LIBRARIES 
	"${GSL_CONFIG_LIBS}" )
      #MESSAGE("DBG  GSL_LINK_LIBRARIES=${GSL_LINK_LIBRARIES}")
      # Because in GSL_LINK_LIBRARIES defs are separated by ";", parse the
      # GSL_LINK_LIBRARIES in order to build a space separated string
      #message(GSL_LINK_LIBRARIES ${GSL_LINK_LIBRARIES})
      SET(GSL_LIB_ONES "")
      FOREACH(libs ${GSL_LINK_LIBRARIES})
	STRING(REGEX REPLACE "[-][l]" "" GSL_LIB_ONES ${libs})
    	#MESSAGE("GSL_LIB_ONES=${GSL_LIB_ONES} -  ${libs}")
  	MARK_AS_ADVANCED(LIBGSL_${GSL_LIB_ONES})
	FIND_LIBRARY(LIBGSL_${GSL_LIB_ONES}
	  NAMES ${GSL_LIB_ONES}
	  PATHS
	  ${GSL_LINK_DIRECTORIES}
	  /usr/lib
	  /usr/local/lib
	)
	#MESSAGE("LIBGSL_${GSL_LIB_ONES} ${LIBGSL_${GSL_LIB_ONES}}")
	IF(LIBGSL_${GSL_LIB_ONES})
     	  #MESSAGE("LIB_GSL=${LIB_GSL} -  ${libs}")
	  LIST(APPEND GSL_LIBRARIES ${LIBGSL_${GSL_LIB_ONES}})
        ENDIF()
      ENDFOREACH(libs)

      #MESSAGE("GSL_LIBRARIES=${GSL_LIBRARIES}")
      
    #ELSE(GSL_CONFIG)
    #  MESSAGE("FindGSL.cmake: gsl-config not found. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
    ENDIF(GSL_CONFIG)

  IF(GSL_INCLUDE_DIR AND GSL_LIBRARIES)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${GSL_LIBRARIES})
    SET(CMAKE_REQUIRED_INCLUDES ${GSL_INCLUDE_DIR})
    CHECK_CXX_SOURCE_COMPILES("
      #include <gsl/gsl_linalg.h> // Contrib for GSL library
      #include <gsl/gsl_math.h>
      #include <gsl/gsl_eigen.h>
     int main()
      {
        gsl_matrix *A = gsl_matrix_alloc(6, 6) ;
      }
      " GSL_BUILD_TEST) 
    #MESSAGE("GSL_BUILD_TEST: ${GSL_BUILD_TEST}")
    IF(GSL_BUILD_TEST)
      SET(GSL_INCLUDE_DIRS ${GSL_INCLUDE_DIR})
      SET(GSL_LIBRARIES ${GSL_LIBRARIES})
      SET(GSL_FOUND TRUE)
    ELSE()
      SET(GSL_FOUND FALSE)
      #MESScAGE("libgsl library found but not compatible with architecture.")
    ENDIF()
  ELSE(GSL_INCLUDE_DIR AND GSL_LIBRARIES)
    SET(GSL_FOUND FALSE) 
  ENDIF(GSL_INCLUDE_DIR AND GSL_LIBRARIES)

  MARK_AS_ADVANCED(
    GSL_INCLUDE_DIR
    GSL_INCLUDE_DIRS
    GSL_LIBRARIES
    GSL_LINK_DIRECTORIES
    GSL_CONFIG
  )

  ENDIF(UNIX)
ENDIF(WIN32)

