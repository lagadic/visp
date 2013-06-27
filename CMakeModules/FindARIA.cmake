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
# Try to find Aria libraries and headers to control Pioneer mobile robots.
# Once run this will define: 
#
# ARIA_FOUND
# ARIA_INCLUDE_DIRS
# ARIA_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

FIND_PATH(ARIA_INCLUDE_DIR Aria.h
    $ENV{ARIA_HOME}/include
    /usr/local/Aria/include
    /usr/Aria/include
    /usr/include
    /usr/local/include
	"C:/Program Files/MobileRobots/Aria/include"
    )
#MESSAGE("DBG ARIA_INCLUDE_DIR=${ARIA_INCLUDE_DIR}")  

if(UNIX)  
  FIND_LIBRARY(ARIA_LIBRARY
    NAMES Aria
    PATHS 
    $ENV{ARIA_HOME}/lib
    /usr/local/Aria/lib
    /usr/Aria/lib
    /usr/lib
    /usr/local/lib
	"C:/Program Files/MobileRobots/Aria/lib"
    )
  #MESSAGE("DBG ARIA_LIBRARY=${ARIA_LIBRARY}")
else()
  if(MSVC11)
    set(ARIA_SUFFIX_NAME VC11)
  elseif(MSVC10)
    set(ARIA_SUFFIX_NAME VC10)
  elseif(MSVC90)
    set(ARIA_SUFFIX_NAME VC9)
  elseif(MSVC80)
    set(ARIA_SUFFIX_NAME VC8)
  elseif(MSVC71)
    set(ARIA_SUFFIX_NAME VC71)
  else()
    set(ARIA_SUFFIX_NAME "")
  endif()
  
  FIND_LIBRARY(ARIA_LIBRARY_DEBUG
    NAMES AriaDebug${ARIA_SUFFIX_NAME}
    PATHS 
    $ENV{ARIA_HOME}/lib
    /usr/local/Aria/lib
    /usr/Aria/lib
    /usr/lib
    /usr/local/lib
	"C:/Program Files/MobileRobots/Aria/lib"
    )
  FIND_LIBRARY(ARIA_LIBRARY_RELEASE
    NAMES Aria${ARIA_SUFFIX_NAME}
    PATHS 
    $ENV{ARIA_HOME}/lib
    /usr/local/Aria/lib
    /usr/Aria/lib
    /usr/lib
    /usr/local/lib
	"C:/Program Files/MobileRobots/Aria/lib"
    )
endif()
    
if(UNIX)  
  IF(ARIA_LIBRARY AND ARIA_INCLUDE_DIR)
    SET(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    SET(ARIA_LIBRARIES ${ARIA_LIBRARY})
    SET(ARIA_FOUND TRUE)
  ELSE()
    SET(ARIA_FOUND FALSE)
  ENDIF()
else()
  set(ARIA_LIBRARIES "")
  IF(ARIA_LIBRARY_RELEASE AND ARIA_INCLUDE_DIR)
    SET(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    LIST(APPEND ARIA_LIBRARIES optimized ${ARIA_LIBRARY_RELEASE})
    SET(ARIA_FOUND TRUE)
  ENDIF()
  IF(ARIA_LIBRARY_DEBUG AND ARIA_INCLUDE_DIR)
    SET(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    LIST(APPEND ARIA_LIBRARIES debug ${ARIA_LIBRARY_DEBUG})
    SET(ARIA_FOUND TRUE)
  ENDIF()

endif()
  
MARK_AS_ADVANCED(
    ARIA_INCLUDE_DIR
    ARIA_LIBRARY
    ARIA_LIBRARY_DEBUG
    ARIA_LIBRARY_RELEASE
    )

