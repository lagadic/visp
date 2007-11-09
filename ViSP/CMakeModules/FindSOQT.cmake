#############################################################################
#
# $Id: FindSOQT.cmake,v 1.5 2007-11-09 15:41:17 asaunier Exp $
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
# SOQT_FOUND
# SOQT_LIBRARIES
# SOQT_LIBRARY_RELEASE
# SOQT_LIBRARY_DEBUG
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(UNIX OR WIN32)

  

IF(WIN32)
  FIND_LIBRARY(SOQT_LIBRARY_RELEASE
    NAMES soqt1 #only shared libraries under windows
    PATHS
    "$ENV{COINDIR}/lib"
    )
  FIND_LIBRARY(SOQT_LIBRARY_DEBUG
    NAMES soqt1d #only shared libraries under windows
    PATHS
    "$ENV{COINDIR}/lib"
    )  

 FIND_PATH(SOQT_INCLUDE_PATH Inventor/Qt/SoQt.h
    "$ENV{COINDIR}/include"
    "$ENV{INCLUDE}"
    )
 MARK_AS_ADVANCED(
 	SOQT_LIBRARY_DEBUG
	SOQT_LIBRARY_RELEASE
	SOQT_INCLUDE_PATH
 )

ELSE(WIN32)
FIND_LIBRARY(SOQT_LIBRARY
    NAMES SoQt
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    "$ENV{COINDIR}/lib"
    )

  #MESSAGE(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")

ENDIF(WIN32)
  
  ## --------------------------------
  
  IF(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)
    
	IF(WIN32)
	 	IF(SOQT_LIBRARY_RELEASE AND NOT SOQT_LIBRARY_DEBUG)
			 SET(SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY_RELEASE})			 
		ENDIF(SOQT_LIBRARY_RELEASE AND NOT SOQT_LIBRARY_DEBUG)
		IF(SOQT_LIBRARY_DEBUG AND NOT SOQT_LIBRARY_RELEASE)
			 SET(SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY_DEBUG})			 
		ENDIF(SOQT_LIBRARY_DEBUG AND NOT SOQT_LIBRARY_RELEASE)
		IF(SOQT_LIBRARY_RELEASE AND SOQT_LIBRARY_DEBUG)
			 SET(SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY_RELEASE})
			 SET(SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY_DEBUG})			 		 
		ENDIF(SOQT_LIBRARY_RELEASE AND SOQT_LIBRARY_DEBUG)
	ELSE(WIN32)
		SET(SOQT_LIBRARIES ${SOQT_LIBRARY})
		MARK_AS_ADVANCED(
    			SOQT_LIBRARIES
			    SOQT_LIBRARY
    		)
	ENDIF(WIN32)
    SET(SOQT_FOUND TRUE)
  ELSE(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)
    SET(SOQT_FOUND FALSE)
    #MESSAGE("SoQt library not found.")
  ENDIF(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)

  IF(WIN32)
  IF(SOQT_INCLUDE_PATH)
	set(SOQT_INCLUDE_DIR ${SOQT_INCLUDE_PATH})
  ELSE(SOQT_INCLUDE_PATH)
	#MESSAGE("Can not find SoQt includes")
  ENDIF(SOQT_INCLUDE_PATH)
  MARK_AS_ADVANCED(
    	SOQT_INCUDE_DIR
  	)
  ENDIF(WIN32)
 
  #MESSAGE(STATUS "SOQT_FOUND : ${SOQT_FOUND}")

ELSE(UNIX OR WIN32)
  SET(SOQT_FOUND FALSE)
ENDIF(UNIX OR WIN32)
