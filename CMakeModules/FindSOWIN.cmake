#############################################################################
#
# $Id$
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
# Try to find SoQt library.
# Once run this will define: 
#
# SOWIN_FOUND
# SOWIN_LIBRARIES
# SOWIN_LIBRARY_RELEASE
# SOWIN_LIBRARY_DEBUG
#
# Authors:
# Fabien Spindler
# Anthony Saunier
#
#############################################################################

IF(WIN32)

  FIND_LIBRARY(SOWIN_LIBRARY_RELEASE
    NAMES sowin1 #only shared libraries under windows
    PATHS
    "$ENV{COINDIR}/lib"
    "$ENV{COIN_DIR}/lib"
    "$ENV{SOWIN_DIR}/lib"
    )
  FIND_LIBRARY(SOWIN_LIBRARY_DEBUG
    NAMES sowin1d #only shared libraries under windows
    PATHS
    "$ENV{COINDIR}/lib"
    "$ENV{COIN_DIR}/lib"
    "$ENV{SOWIN_DIR}/lib"
    )  

 FIND_PATH(SOWIN_INCLUDE_PATH Inventor/Win/SoWin.h
    "$ENV{COINDIR}/include"
    "$ENV{COIN_DIR}/include"
    "$ENV{SOWIN_DIR}/include"
    )
 MARK_AS_ADVANCED(
 	SOWIN_LIBRARY_DEBUG
	SOWIN_LIBRARY_RELEASE
	SOWIN_INCLUDE_PATH
 )

 ## --------------------------------
  
  IF(SOWIN_LIBRARY OR SOWIN_LIBRARY_DEBUG OR SOWIN_LIBRARY_RELEASE)
    

	 	IF(SOWIN_LIBRARY_RELEASE AND NOT SOWIN_LIBRARY_DEBUG)
			 SET(SOWIN_LIBRARY_RELEASE ${SOWIN_LIBRARY_RELEASE})			 
		ENDIF(SOWIN_LIBRARY_RELEASE AND NOT SOWIN_LIBRARY_DEBUG)
		IF(SOWIN_LIBRARY_DEBUG AND NOT SOWIN_LIBRARY_RELEASE)
			 SET(SOWIN_LIBRARY_DEBUG ${SOWIN_LIBRARY_DEBUG})			 
		ENDIF(SOWIN_LIBRARY_DEBUG AND NOT SOWIN_LIBRARY_RELEASE)
		IF(SOWIN_LIBRARY_RELEASE AND SOWIN_LIBRARY_DEBUG)
			 SET(SOWIN_LIBRARY_RELEASE ${SOWIN_LIBRARY_RELEASE})
			 SET(SOWIN_LIBRARY_DEBUG ${SOWIN_LIBRARY_DEBUG})			 		 
		ENDIF(SOWIN_LIBRARY_RELEASE AND SOWIN_LIBRARY_DEBUG)

    SET(SOWIN_FOUND TRUE)
  ELSE(SOWIN_LIBRARY OR SOWIN_LIBRARY_DEBUG OR SOWIN_LIBRARY_RELEASE)
    SET(SOWIN_FOUND FALSE)
    #MESSAGE("SoQt library not found.")
  ENDIF(SOWIN_LIBRARY OR SOWIN_LIBRARY_DEBUG OR SOWIN_LIBRARY_RELEASE)

 
  IF(SOWIN_INCLUDE_PATH)
	set(SOWIN_INCLUDE_DIR ${SOWIN_INCLUDE_PATH})
  ELSE(SOWIN_INCLUDE_PATH)
	#MESSAGE("Can not find SoQt includes")
  ENDIF(SOWIN_INCLUDE_PATH)
  MARK_AS_ADVANCED(
    	SOWIN_INCUDE_DIR
  	)

  #MESSAGE(STATUS "SOWIN_FOUND : ${SOWIN_FOUND}")

ELSE(WIN32)
  SET(SOWIN_FOUND FALSE)
ENDIF(WIN32)
