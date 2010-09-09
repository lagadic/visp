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
