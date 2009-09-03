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
# Try to find libjpeg library.
# Once run this will define: 
#
# LIBJPEG_FOUND
# LIBJPEG_INCLUDE_DIR
# LIBJPEG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libjpeg headers location
  FIND_PATH(LIBJPEG_INCLUDE_PATH 
    NAMES
    jpeglib.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{LIBJPEG_DIR}/include
    $ENV{LIBJPEG_DIR}
    $ENV{GTK2_DIR}/include
    "C:/Program Files/GnuWin32/include"
    )
  #MESSAGE("LIBJPEG_HEADER=${LIBJPEG_INCLUDE_PATH}")

  # Detection of the Libjpeg library on Unix
  FIND_LIBRARY(LIBJPEG_LIBRARY
    NAMES
    libjpeg.so jpeg.lib
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{LIBJPEG_DIR}/lib
    $ENV{LIBJPEG_DIR}/Release
    $ENV{LIBJPEG_DIR}
    $ENV{GTK2_DIR}/lib
    "C:/Program Files/GnuWin32/lib"
    )
  #MESSAGE("LIBJPEG_LIBRARY=${LIBJPEG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBJPEG_LIBRARY
    LIBJPEG_INCLUDE_PATH
  )
  
## --------------------------------
  
IF(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)

  SET(LIBJPEG_INCLUDE_DIR ${LIBJPEG_INCLUDE_PATH})

  SET(LIBJPEG_LIBRARIES  ${LIBJPEG_LIBRARY})
	SET(LIBJPEG_FOUND TRUE)

ELSE(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)
  SET(LIBJPEG_FOUND FALSE)
ENDIF(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)
