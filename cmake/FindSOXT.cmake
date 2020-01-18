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
# Try to find SoXt library.
# Once run this will define: 
#
# SOXT_FOUND
# SOXT_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(UNIX)

FIND_LIBRARY(SOXT_LIBRARY
    NAMES SoXt
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    "$ENV{COIN_DIR}/lib"
    "$ENV{COINDIR}/lib"
    )
MARK_AS_ADVANCED(
    SOXT_LIBRARY
    )
  #MESSAGE(STATUS "DBG SOXT_LIBRARY=${SOXT_LIBRARY}")

  ## --------------------------------
  
  IF(SOXT_LIBRARY)
    
		SET(SOXT_LIBRARIES ${SOXT_LIBRARY})
    SET(SOXT_FOUND TRUE)
    MARK_AS_ADVANCED(
    SOXT_LIBRARIES
    )
  ELSE(SOXT_LIBRARY)
    SET(SOXT_FOUND FALSE)
    #MESSAGE("SoXt library not found.")
  ENDIF(SOXT_LIBRARY)

  #MESSAGE(STATUS "SOXT_FOUND : ${SOXT_FOUND}")

ELSE(UNIX)
  SET(SOXT_FOUND FALSE)
ENDIF(UNIX)
