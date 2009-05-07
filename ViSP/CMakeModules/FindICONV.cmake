#############################################################################
#
# $Id$
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
# Try to find ICONV library.
# Once run this will define: 
#
# ICONV_FOUND
# ICONV_INCLUDE_DIR
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
  SET(ICONV_INCLUDE_DIR ${ICONV_INCLUDE_DIR})
  SET(ICONV_FOUND TRUE)
ELSE(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)
  SET(ICONV_FOUND FALSE)
ENDIF(ICONV_LIBRARIES AND ICONV_INCLUDE_DIR)

MARK_AS_ADVANCED(
  ICONV_INCLUDE_DIR
  ICONV_LIBRARIES
  ICONV_LIBRARY
)
