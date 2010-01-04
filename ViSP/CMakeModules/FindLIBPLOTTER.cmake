#############################################################################
#
# $Id: FindLibplot.cmake 2179 2009-06-09 16:33:50Z fspindle $
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
# Try to find Libplot library.
# Once run this will define: 
#
# LIBPLOTTER_FOUND
# LIBPLOTTER_INCLUDE_DIR
# LIBPLOTTER_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libplot headers location
  FIND_PATH(LIBPLOTTER_INCLUDE_PATH 
    NAMES
    plotter.h
    PATHS
    "$ENV(LIBPLOTTER_DIR)/include"
    "/usr/include"
	"/usr/local/include"
	"C:/Program Files/GnuWin32/include"
    )

  # Detection of the Libplot library on Unix
  FIND_LIBRARY(LIBPLOTTER_LIBRARY
    NAMES
    libplotter plotter
    PATHS
    "$ENV(LIBPLOTTER_DIR)/lib"
    /usr/lib
    /usr/local/lib
    /lib
	"C:/Program Files/GnuWin32/lib"
    )
  #MESSAGE("LIBPLOTTER_LIBRARY=${LIBPLOTTER_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBPLOTTER_LIBRARY
    LIBPLOTTER_INCLUDE_PATH
  )
  
## --------------------------------
  
IF(LIBPLOTTER_LIBRARY AND LIBPLOTTER_INCLUDE_PATH)

  SET(LIBPLOTTER_INCLUDE_DIR ${LIBPLOTTER_INCLUDE_PATH})
  
  IF(UNIX)
    FIND_LIBRARY(X11_Xaw_LIBRARY
    NAMES
    Xaw
    PATHS
    /usr/lib
    /usr/local/lib
    )

    FIND_LIBRARY(X11_Xmu_LIBRARY
    NAMES
    Xmu
    PATHS
    /usr/lib
    /usr/local/lib
    )

    FIND_LIBRARY(X11_Xt_LIBRARY
    NAMES
    Xt
    PATHS
    /usr/lib
    /usr/local/lib
    )
	IF(X11_Xaw_LIBRARY AND X11_Xmu_LIBRARY AND X11_Xt_LIBRARY)
	  SET(LIBPLOTTER_LIBRARIES  ${LIBPLOTTER_LIBRARY} ${X11_Xaw_LIBRARY} ${X11_Xmu_LIBRARY} ${X11_Xt_LIBRARY})
	  SET(LIBPLOTTER_FOUND TRUE)
	ELSE(X11_Xaw_LIBRARY AND X11_Xmu_LIBRARY AND X11_Xt_LIBRARY)
	  SET(LIBPLOTTER_FOUND FALSE)
	ENDIF(X11_Xaw_LIBRARY AND X11_Xmu_LIBRARY AND X11_Xt_LIBRARY)

  MARK_AS_ADVANCED(
    X11_Xaw_LIBRARY
    X11_Xmu_LIBRARY
    X11_Xt_LIBRARY
  )

  ELSE(UNIX)

  	SET(LIBPLOTTER_LIBRARIES  ${LIBPLOTTER_LIBRARY})
	SET(LIBPLOTTER_FOUND TRUE)

  ENDIF(UNIX)
ELSE(LIBPLOTTER_LIBRARY AND LIBPLOTTER_INCLUDE_PATH)
  SET(LIBPLOTTER_FOUND FALSE)
ENDIF(LIBPLOTTER_LIBRARY AND LIBPLOTTER_INCLUDE_PATH)
