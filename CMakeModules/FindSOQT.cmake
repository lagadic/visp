#############################################################################
#
# $Id: FindSOQT.cmake,v 1.2 2006-05-30 08:35:01 fspindle Exp $
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
# Try to find SoQt library.
# Once run this will define: 
#
# SOQT_FOUND
# SOQT_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  SET(SOQT_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_LIBRARY(SOQT_LIBRARY
    NAMES SoQt
    PATHS
    /usr/lib
    /usr/local/lib
    /lib    
    )

  #MESSAGE(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")
  
  ## --------------------------------
  
  IF(SOQT_LIBRARY)
    SET(SOQT_LIBRARIES ${SOQT_LIBRARY})
    SET(SOQT_FOUND TRUE)
  ELSE(SOQT_LIBRARY)
    SET(SOQT_FOUND FALSE)
    #MESSAGE("SoQt library not found.")
  ENDIF(SOQT_LIBRARY)
  
  
  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    )
  #MESSAGE(STATUS "SOQT_FOUND : ${SOQT_FOUND}")

ENDIF(NOT UNIX)
