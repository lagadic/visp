#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
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
# XML2_VERSION_STRING
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
    /usr/local/opt/libxml2/include/libxml2
    )
  find_library(XML2_LIBRARY xml2
    "$ENV{XML2_DIR}/lib"
    "$ENV{XML2_HOME}/lib"
    /usr/lib
    /usr/local/lib
    /usr/local/opt/libxml2/lib
    )
endif(WIN32)

if(XML2_LIBRARY)
  set(XML2_LIBRARIES ${XML2_LIBRARY})
endif()

if(XML2_LIBRARIES AND XML2_INCLUDE_DIR)
  set(XML2_INCLUDE_DIRS ${XML2_INCLUDE_DIR})
  set(XML2_FOUND TRUE)

  vp_parse_header2(XML2 "${XML2_INCLUDE_DIR}/libxml/xmlversion.h" LIBXML_DOTTED_VERSION)

  if(WIN32 AND ICONV_FOUND)
    list(APPEND XML2_INCLUDE_DIRS ${ICONV_INCLUDE_DIRS})
    set(XML2_LIBRARIES ${XML2_LIBRARIES} ${ICONV_LIBRARIES})
  endif()

else()
  set(XML2_FOUND FALSE)
endif()

mark_as_advanced(
  XML2_INCLUDE_DIR
  XML2_LIBRARIES
  XML2_LIBRARY
  )
