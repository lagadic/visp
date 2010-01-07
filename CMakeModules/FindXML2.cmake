#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
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
# Try to find XML library.
# Once run this will define: 
#
# XML2_FOUND
# XML2_INCLUDE_DIR
# XML2_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################


IF(WIN32)
  FIND_PACKAGE(ICONV)
  IF(ICONV_FOUND)
    FIND_PATH(XML2_INCLUDE_DIR libxml/xmlmemory.h
      $ENV{XML2_DIR}/include
      $ENV{XML2_HOME}/include
      )
    FIND_LIBRARY(XML2_LIBRARY libxml2
      $ENV{XML2_DIR}/lib
      $ENV{XML2_HOME}/lib
      /usr/lib
      /usr/local/lib
      "c:/libxml2/lib"
      )
  ENDIF(ICONV_FOUND)  
ELSE(WIN32) 
  FIND_PATH(XML2_INCLUDE_DIR libxml/xmlmemory.h
    $ENV{XML2_DIR}/include/libxml2
    $ENV{XML2_HOME}/include/libxml2
    /usr/include/libxml2
    /usr/local/include/libxml2
    )
  FIND_LIBRARY(XML2_LIBRARY xml2
    $ENV{XML2_DIR}/lib
    $ENV{XML2_HOME}/lib
    /usr/lib
    /usr/local/lib
    )
ENDIF(WIN32)
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
  SET(XML2_INCLUDE_DIR ${XML2_INCLUDE_DIR})
  SET(XML2_FOUND TRUE)

  IF(WIN32)
    SET(XML2_INCLUDE_DIR ${XML2_INCLUDE_DIR} ${ICONV_INCLUDE_DIR})
    SET(XML2_LIBRARIES ${XML2_LIBRARIES} ${ICONV_LIBRARIES})
  ENDIF(WIN32)

ELSE(XML2_LIBRARIES AND XML2_INCLUDE_DIR)
  SET(XML2_FOUND FALSE)
ENDIF(XML2_LIBRARIES AND XML2_INCLUDE_DIR)

MARK_AS_ADVANCED(
  XML2_INCLUDE_DIR
  XML2_LIBRARIES
  XML2_LIBRARY
  )
