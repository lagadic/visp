#############################################################################
#
# $Id: FindGSL.cmake,v 1.4 2006-11-30 10:44:31 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
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
# Try to find gnu scientific library GSL 
# (see http://www.gnu.org/software/gsl/)
# Once run this will define: 
# 
# GSL_FOUND       = system has GSL lib
#
# GSL_LIBRARIES   = full path to the libraries
#    on Unix/Linux with additional linker flags from "gsl-config --libs"
# 
# GSL_INCLUDE_DIR      = where to find headers 
#
# GSL_LINK_DIRECTORIES = link directories, useful for rpath on Unix
# GSL_EXE_LINKER_FLAGS = rpath on Unix
#
# Authors:
# Fabien Spindler
#
#############################################################################

#IF(WIN32)
#  MESSAGE("FindGSL.cmake: gnu scientific library GSL not (yet) supported on WIN32")
  
#ELSE(WIN32)
  IF(UNIX) 
  SET(GSL_CONFIG_PREFER_PATH "$ENV{GSL_HOME}/bin" CACHE STRING "preferred path to OpenSG (osg-config)")
    FIND_PROGRAM(GSL_CONFIG gsl-config
      ${GSL_CONFIG_PREFER_PATH}
      /usr/bin
      /usr/local/bin
      )
    # MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")
    
    IF (GSL_CONFIG) 
      # set CXXFLAGS to be fed into CXX_FLAGS by the user:
      SET(GSL_CXX_FLAGS "`${GSL_CONFIG} --cflags`")
      
      # set INCLUDE_DIRS to prefix+include
      EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --prefix
	OUTPUT_VARIABLE GSL_PREFIX)
      SET(GSL_INCLUDE_DIR ${GSL_PREFIX}/include CACHE STRING INTERNAL)
      
      ## extract link dirs for rpath  
      EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --libs
	OUTPUT_VARIABLE GSL_CONFIG_LIBS )
      #MESSAGE("GSL_CONFIG_LIBS: ${GSL_CONFIG_LIBS}")

      ## use regular expression to match wildcard equivalent "-l*<endchar>"
      ## with <endchar> is a space or a semicolon
      STRING(REGEX MATCHALL "[-][l]([^ ;])+" 
	GSL_LINK_LIBRARIES 
	"${GSL_CONFIG_LIBS}" )
      #MESSAGE("DBG  GSL_LINK_LIBRARIES=${GSL_LINK_LIBRARIES}")
      # Because in GSL_LINK_LIBRARIES defs are separated by ";", parse the
      # GSL_LINK_LIBRARIES in order to build a space separated string
      FOREACH(libs ${GSL_LINK_LIBRARIES})
	SET(GSL_LIBRARIES "${GSL_LIBRARIES} ${libs}")
      ENDFOREACH(libs)
      #MESSAGE("GSL_LIBRARIES: ${GSL_LIBRARIES}")

      ## split off the link dirs (for rpath)
      ## use regular expression to match wildcard equivalent "-L*<endchar>"
      ## with <endchar> is a space or a semicolon
      STRING(REGEX MATCHALL "[-][L]([^ ;])+" 
	GSL_LINK_DIRECTORIES_WITH_PREFIX 
	"${GSL_CONFIG_LIBS}" )
      #MESSAGE("DBG  GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")
      
      ## remove prefix -L because we need the pure directory for LINK_DIRECTORIES
     
      IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
	STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
      ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
      IF(NOT APPLE)
      	     SET(GSL_EXE_LINKER_FLAGS "-Wl,-rpath,${GSL_LINK_DIRECTORIES}" CACHE STRING INTERNAL)
      ENDIF(NOT APPLE)
      #MESSAGE("DBG  GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
      #MESSAGE("DBG  GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")

#      ADD_DEFINITIONS("-DHAVE_GSL")
#      SET(GSL_DEFINITIONS "-DHAVE_GSL")
      MARK_AS_ADVANCED(
	GSL_CXX_FLAGS
	GSL_INCLUDE_DIR
	GSL_LIBRARIES
	GSL_LINK_DIRECTORIES
	GSL_DEFINITIONS
	)
      #MESSAGE(STATUS "Using GSL from ${GSL_PREFIX}")
      
    #ELSE(GSL_CONFIG)
    #  MESSAGE("FindGSL.cmake: gsl-config not found. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
    ENDIF(GSL_CONFIG)

  ENDIF(UNIX)
#ENDIF(WIN32)


IF(GSL_LIBRARIES)
  IF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
    SET(GSL_FOUND TRUE)
  ELSE(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
    SET(GSL_FOUND FALSE)    
  ENDIF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
ENDIF(GSL_LIBRARIES)

