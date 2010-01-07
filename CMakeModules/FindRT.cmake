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
