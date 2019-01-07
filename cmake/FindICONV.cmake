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
# Try to find ICONV library.
# Once run this will define: 
#
# ICONV_FOUND
# ICONV_INCLUDE_DIRS
# ICONV_LIBRARIES
#
# Authors:
# Anthony Saunier
#
#############################################################################

if(MINGW)
  find_path(ICONV_INCLUDE_DIR iconv.h
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
  )
  find_library(ICONV_LIBRARY iconv
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
  )
else()
  find_path(ICONV_INCLUDE_DIR iconv.h
    $ENV{ICONV_DIR}/include
    $ENV{ICONV_HOME}/include
    $ENV{XML2_DIR}/include
    $ENV{XML2_HOME}/include
    "c:/libxml2/include"
    "c:/iconv/include"
  )
  find_library(ICONV_LIBRARY iconv
    $ENV{ICONV_DIR}/lib
    $ENV{ICONV_HOME}/lib
    $ENV{XML2_DIR}/lib
    $ENV{XML2_HOME}/lib
    "c:/libxml2/lib"
    "c:/iconv/lib"
  )
endif()

if(ICONV_LIBRARY)
  SET(ICONV_LIBRARIES ${ICONV_LIBRARY})
endif()

if(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)
  SET(ICONV_INCLUDE_DIRS ${ICONV_INCLUDE_DIR})
  SET(ICONV_FOUND TRUE)
else()
  SET(ICONV_FOUND FALSE)
endif()

mark_as_advanced(
  ICONV_INCLUDE_DIR
  ICONV_LIBRARIES
  ICONV_LIBRARY
)
