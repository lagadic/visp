#############################################################################
#
# $Id: FindCOIN.cmake,v 1.3 2007-03-21 13:54:39 asaunier Exp $
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
# Try to find Coin library .
# Once run this will define: 
#
# COIN_FOUND
# COIN_LIBRARIES
# COIN_LIBRARY_DEBUG
# COIN_LIBRARY_RELEASE
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(UNIX OR WIN32) 
  
IF(WIN32)
 FIND_LIBRARY(COIN_LIBRARY_RELEASE
    NAMES coin2 #only shared libraries under windows
    PATHS
     "$ENV{COINDIR}/lib"	  
    )

 FIND_LIBRARY(COIN_LIBRARY_DEBUG
    NAMES coin2d #only shared libraries under windows
    PATHS
    "$ENV{COINDIR}/lib"	  
    )

ENDIF(WIN32)
  FIND_LIBRARY(COIN_LIBRARY
    NAMES Coin #only shared libraries under windows
    PATHS
     "$ENV{COINDIR}/lib"	  
    )

  #MESSAGE(STATUS "DBG COIN_LIBRARY=${COIN_LIBRARY}")
  
  ## --------------------------------
  
  IF(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG)
    	IF(WIN32)
	 	IF(COIN_LIBRARY_RELEASE AND NOT COIN_LIBRARY_DEBUG)
			 SET(COIN_LIBRARY_RELEASE ${COIN_LIBRARY_RELEASE})			 
		ENDIF(COIN_LIBRARY_RELEASE AND NOT COIN_LIBRARY_DEBUG)
		IF(COIN_LIBRARY_DEBUG AND NOT COIN_LIBRARY_RELEASE)
			 SET(COIN_LIBRARY_DEBUG ${COIN_LIBRARY_DEBUG})			 
		ENDIF(COIN_LIBRARY_DEBUG AND NOT COIN_LIBRARY_RELEASE)
		IF(COIN_LIBRARY_RELEASE AND COIN_LIBRARY_DEBUG)
			 SET(COIN_LIBRARY_RELEASE ${COIN_LIBRARY_RELEASE})
			 SET(COIN_LIBRARY_DEBUG ${COIN_LIBRARY_DEBUG})			
		ENDIF(COIN_LIBRARY_RELEASE AND COIN_LIBRARY_DEBUG)
	ELSE(WIN32)
		SET(COIN_LIBRARIES ${COIN_LIBRARY})
	ENDIF(WIN32)
    SET(COIN_FOUND TRUE)
  ELSE(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG)
    SET(COIN_FOUND FALSE)
    #MESSAGE("Coin library not found.")
  ENDIF(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG)
    
  MARK_AS_ADVANCED(
    COIN_LIBRARIES
    COIN_LIBRARY_RELEASE
    COIN_LIBRARY_DEBUG
    )
  #MESSAGE(STATUS "COIN_FOUND : ${COIN_FOUND}")

ELSE(UNIX OR WIN32)
	SET(COIN_FOUND FALSE)
ENDIF(UNIX OR WIN32)
