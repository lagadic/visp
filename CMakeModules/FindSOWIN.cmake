#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
