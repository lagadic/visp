#############################################################################
#
# $Id: FindXML2.cmake 5316 2015-02-12 10:58:18Z fspindle $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
# Try to find XML library.
# Once run this will define: 
#
# XML2_FOUND
# XML2_INCLUDE_DIRS
# XML2_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################


if(WIN32)
  find_package(ICONV)
  if(MINGW)
    find_path(XML2_INCLUDE_DIR libxml/xmlmemory.h
      "$ENV{MINGW_DIR}/include/libxml2"
      C:/mingw/include/libxml2
    )
    find_library(XML2_LIBRARY libxml2
      "$ENV{MINGW_DIR}/lib"
      "$ENV{MINGW_DIR}/lib64"
      C:/mingw/lib64
    )
  else()
    find_path(XML2_INCLUDE_DIR libxml/xmlmemory.h
      "$ENV{XML2_DIR}/include"
      "$ENV{XML2_DIR}/include/libxml2"
      "$ENV{XML2_HOME}/include"
      "$ENV{XML2_HOME}/include/libxml2"
    )
    find_library(XML2_LIBRARY libxml2
      "$ENV{XML2_DIR}/lib"
      "$ENV{XML2_HOME}/lib"
      /usr/lib
      /usr/local/lib
      "c:/libxml2/lib"
    )
  endif()
else(WIN32) 
  find_path(XML2_INCLUDE_DIR libxml/xmlmemory.h
    "$ENV{XML2_DIR}/include/libxml2"
    "$ENV{XML2_HOME}/include/libxml2"
    /usr/include/libxml2
    /usr/local/include/libxml2
    )
  find_library(XML2_LIBRARY xml2
    "$ENV{XML2_DIR}/lib"
    "$ENV{XML2_HOME}/lib"
    /usr/lib
    /usr/local/lib
    )
endif(WIN32)
#MESSAGE("DBG XML2_INCLUDE_DIR=${XML2_INCLUDE_DIR}")  


#MESSAGE("DBG XML2_LIBRARY=${XML2_LIBRARY}")

## --------------------------------

IF(NOT XML2_INCLUDE_DIR)
  #MESSAGE(SEND_ERROR "xml include dir not found.")
ENDIF(NOT XML2_INCLUDE_DIR)

IF(XML2_LIBRARY)
  SET(XML2_LIBRARIES ${XML2_LIBRARY})
ELSE(XML2_LIBRARY)
  #MESSAGE(SEND_ERROR "xml library not found.")
ENDIF(XML2_LIBRARY)


IF(XML2_LIBRARIES AND XML2_INCLUDE_DIR)
  SET(XML2_INCLUDE_DIRS ${XML2_INCLUDE_DIR})
  SET(XML2_FOUND TRUE)

  IF(WIN32 AND ICONV_FOUND)
    LIST(APPEND XML2_INCLUDE_DIRS ${ICONV_INCLUDE_DIRS})
    SET(XML2_LIBRARIES ${XML2_LIBRARIES} ${ICONV_LIBRARIES})
  ENDIF()

ELSE(XML2_LIBRARIES AND XML2_INCLUDE_DIR)
  SET(XML2_FOUND FALSE)
ENDIF(XML2_LIBRARIES AND XML2_INCLUDE_DIR)

MARK_AS_ADVANCED(
  XML2_INCLUDE_DIR
  XML2_LIBRARIES
  XML2_LIBRARY
  )
