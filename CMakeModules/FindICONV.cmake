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


IF(WIN32)
  FIND_PATH(ICONV_INCLUDE_DIR iconv.h
    $ENV{ICONV_DIR}/include
    $ENV{ICONV_HOME}/include
    $ENV{XML2_DIR}/include
    $ENV{XML2_HOME}/include
    "c:/libxml2/include"
    "c:/iconv/include"
  )
  FIND_LIBRARY(ICONV_LIBRARY iconv
    $ENV{ICONV_DIR}/lib
    $ENV{ICONV_HOME}/lib
    $ENV{XML2_DIR}/lib
    $ENV{XML2_HOME}/lib
    "c:/libxml2/lib"
    "c:/iconv/lib"
  )
ELSE(WIN32) 
ENDIF(WIN32)

IF(ICONV_LIBRARY)
  SET(ICONV_LIBRARIES ${ICONV_LIBRARY})
ENDIF(ICONV_LIBRARY)

IF(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)
  SET(ICONV_INCLUDE_DIRS ${ICONV_INCLUDE_DIR})
  SET(ICONV_FOUND TRUE)
ELSE(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)
  SET(ICONV_FOUND FALSE)
ENDIF(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)

MARK_AS_ADVANCED(
  ICONV_INCLUDE_DIR
  ICONV_LIBRARIES
  ICONV_LIBRARY
)
