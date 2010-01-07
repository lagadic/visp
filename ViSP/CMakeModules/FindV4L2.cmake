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
# Try to find linux/videodev.h for Video For Linux Two framegrabbing 
# capabilities.
# Once run this will define: 
#
# V4L_FOUND
# V4L_INCLUDE_DIR
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindV4L2.cmake: only available for Unix.")
  SET(V4L2_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(V4L2_INCLUDE_DIR linux/videodev2.h
    $ENV{V4L2_HOME}/include
    $ENV{V4L2_DIR}/include
    /usr/include 
    /usr/src/linux/include)
  #MESSAGE("DBG V4L2_INCLUDE_DIR=${V4L2_INCLUDE_DIR}")  
    
  ## --------------------------------
    
  # IF(NOT V4L2_INCLUDE_DIR)
  #  MESSAGE(SEND_ERROR "Video For Linux Two include dir not found.")
  #ENDIF(NOT V4L2_INCLUDE_DIR)
  
  IF(V4L2_INCLUDE_DIR)
    SET(V4L2_FOUND TRUE)
  ELSE(V4L2_INCLUDE_DIR)
    SET(V4L2_FOUND FALSE)
  ENDIF(V4L2_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    V4L2_INCLUDE_DIR
    )
ENDIF(NOT UNIX)
