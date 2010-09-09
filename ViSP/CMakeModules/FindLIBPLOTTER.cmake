#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.GPL at the root directory of this source
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
