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
# Try to find libraw1394 for IEEE1394 camera under Linux.
# Once run this will define: 
#
# RAW1394_FOUND
# RAW1394_INCLUDE_DIR
# RAW1394_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindRAW1394.cmake: libraw1394 only available for Unix.")
  SET(RAW1394_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(RAW1394_INCLUDE_DIR libraw1394/raw1394.h
    $ENV{RAW1394_HOME}/include
    /usr/include )
  #MESSAGE("DBG RAW1394_INCLUDE_DIR=${RAW1394_INCLUDE_DIR}")  
  
  FIND_LIBRARY(RAW1394_LIBRARY
    NAMES raw1394
    PATHS 
    $ENV{RAW1394_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG RAW1394_LIBRARY=${RAW1394_LIBRARY}")
  
  ## --------------------------------
  
  IF(RAW1394_LIBRARY)
    SET(RAW1394_LIBRARIES ${RAW1394_LIBRARY})
  ELSE(RAW1394_LIBRARY)
    #MESSAGE("libraw1394 library not found.")
  ENDIF(RAW1394_LIBRARY)
  
  IF(NOT RAW1394_INCLUDE_DIR)
    #MESSAGE("libraw1394 include dir not found.")
  ENDIF(NOT RAW1394_INCLUDE_DIR)
  
  IF(RAW1394_LIBRARIES AND RAW1394_INCLUDE_DIR)
    SET(RAW1394_FOUND TRUE)
  ELSE(RAW1394_LIBRARIES AND RAW1394_INCLUDE_DIR)
    SET(RAW1394_FOUND FALSE)
  ENDIF(RAW1394_LIBRARIES AND RAW1394_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    RAW1394_INCLUDE_DIR
    RAW1394_LIBRARIES
    RAW1394_LIBRARY
    )
ENDIF(NOT UNIX)
