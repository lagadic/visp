#############################################################################
#
# $Id: FindLIBJPEG.cmake 2179 2009-06-09 16:33:50Z fspindle $
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
# Try to find libpng library.
# To use the libpng library, the zlib library is required.
# Once run this will define: 
#
# LIBPNG_FOUND
# LIBPNG_INCLUDE_DIR
# LIBPNG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libpng headers location
  FIND_PATH(LIBPNG_INCLUDE_PATH 
    NAMES
    png.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{LIBPNG_DIR}/include
    $ENV{LIBPNG_DIR}
    )
  #MESSAGE("LIBPNG_HEADER=${LIBPNG_INCLUDE_PATH}")

  # Detection of the Libpng library on Unix
  FIND_LIBRARY(LIBPNG_LIBRARY
    NAMES
    png libpng
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{LIBPNG_DIR}/lib
    $ENV{LIBPNG_DIR}/Release
    $ENV{LIBPNG_DIR}
    )
  #MESSAGE("LIBPNG_LIBRARY=${LIBPNG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBPNG_LIBRARY
    LIBPNG_INCLUDE_PATH
  )
  
## --------------------------------
  
IF(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
  FIND_PACKAGE(ZLIB)
  IF(ZLIB_FOUND)
    SET(LIBPNG_INCLUDE_DIR ${LIBPNG_INCLUDE_PATH})
    SET(LIBPNG_LIBRARIES  ${LIBPNG_LIBRARY})
    SET(LIBPNG_FOUND TRUE)
  ELSE(ZLIB_FOUND)
    MESSAGE("To use the libpng library, the zlib library is required")
  ENDIF(ZLIB_FOUND)

ELSE(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
  SET(LIBPNG_FOUND FALSE)
ENDIF(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
