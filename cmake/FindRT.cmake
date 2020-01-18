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
# Try to find RT library material 
#
# RT_FOUND
# RT_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindRT.cmake: only available for Unix.")
  SET(RT_FOUND FALSE)
ELSE(NOT UNIX)
    
  FIND_LIBRARY(RT_LIBRARY
    NAMES rt
    PATHS 
    $ENV{RT_HOME}/lib
    /usr/lib
    /usr/local/lib
    )

  #MESSAGE("DBG RT_LIBRARY=${RT_LIBRARY}")
  
  ## --------------------------------
  
  IF(RT_LIBRARY)
    SET(RT_LIBRARIES ${RT_LIBRARY})
    SET(RT_FOUND TRUE)
  ELSE(RT_LIBRARY)
    SET(RT_FOUND FALSE)
    #MESSAGE(SEND_ERROR "rt library not found.")
  ENDIF(RT_LIBRARY)
  
  MARK_AS_ADVANCED(
    RT_LIBRARIES
    RT_LIBRARY
    )
ENDIF(NOT UNIX)
