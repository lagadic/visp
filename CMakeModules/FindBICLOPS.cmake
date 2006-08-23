#############################################################################
#
# $Id: FindBICLOPS.cmake,v 1.3 2006-08-23 10:41:55 brenier Exp $
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
# Try to find libBiclops, libPMD and libUtils for Biclops head.
# Once run this will define: 
#
# BICLOPS_FOUND
# BICLOPS_INCLUDE_DIR
# BICLOPS_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX AND NOT WIN32)
  # MESSAGE("FindBICLOPS.cmake: macro only for Unix and Windows for the moment.")
  SET(BICLOPS_FOUND FALSE)
ELSE(NOT UNIX AND NOT WIN32)

  FIND_PATH(BICLOPS_INCLUDE_DIR Biclops.h
    $ENV{BICLOPS_HOME}/include
    /usr/include )
  	
  #MESSAGE("DBG BICLOPS_INCLUDE_DIR=${BICLOPS_INCLUDE_DIR}")  

  FIND_PATH(PMD_INCLUDE_DIR PMD.h
    $ENV{BICLOPS_HOME}/include
    $ENV{BICLOPS_PMD_HOME}/include
    /usr/include )

  FIND_PATH(UTILS_INCLUDE_DIR utility.h
    $ENV{BICLOPS_HOME}/include
    $ENV{BICLOPS_UTILS_HOME}/include
    /usr/include )

  FIND_LIBRARY(BICLOPS_LIBRARY
    NAMES Biclops libBiclops libBiclopsD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    /usr/lib
    )

  FIND_LIBRARY(PMD_LIBRARY
    NAMES PMD libPMD libPMDD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    $ENV{BICLOPS_PMD_HOME}/lib
    /usr/lib
    )

  FIND_LIBRARY(UTILS_LIBRARY
    NAMES Utils libUtils libUtilsD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    $ENV{BICLOPS_UTILS_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG BICLOPS_LIBRARY=${BICLOPS_LIBRARY}")

  ## --------------------------------

  IF(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)
    SET(BICLOPS_LIBRARIES ${BICLOPS_LIBRARY} ${PMD_LIBRARY} ${UTILS_LIBRARY})
  ELSE(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)
    MESSAGE(SEND_ERROR "Biclops library not found.")
  ENDIF(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)

  IF(NOT BICLOPS_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Biclops include dir not found.")
  ENDIF(NOT BICLOPS_INCLUDE_DIR)
 
  IF(NOT PMD_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "PMD include dir not found.")
  ENDIF(NOT PMD_INCLUDE_DIR)
  
  IF(NOT UTILS_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Utils include dir not found.")
  ENDIF(NOT UTILS_INCLUDE_DIR)

  IF(BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    SET(BICLOPS_INCLUDE_DIR ${BICLOPS_INCLUDE_DIR} ${PMD_INCLUDE_DIR} ${UTILS_INCLUDE_DIR})
  ENDIF(BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)

   IF(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    SET(BICLOPS_FOUND TRUE)
  ELSE(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    SET(BICLOPS_FOUND FALSE)
  ENDIF(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)

  MARK_AS_ADVANCED(
    BICLOPS_INCLUDE_DIR
    BICLOPS_LIBRARIES
    BICLOPS_LIBRARY
    PMD_INCLUDE_DIR
    UTILS_INCLUDE_DIR
    PMD_LIBRARY
    UTILS_LIBRARY
    )
ENDIF(NOT UNIX AND NOT WIN32)
