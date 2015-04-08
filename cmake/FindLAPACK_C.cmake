#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
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
# Try to find lapack, the Linear Algebra PACKage.
# Since the FindLAPACK.cmake provided with CMake is for Fortran language,
# this file is able to detect lapack for C language.
# Once run this will define:
#
# LAPACK_FOUND
# LAPACK_LIBRARIES
#
# Authors:
# Filip Novotny
# Fabien Spindler
#
#############################################################################

SET(LAPACK_FOUND FALSE)
SET(LAPACK_LIBRARIES "")
IF(WIN32)
    FIND_LIBRARY(LAPACK_LIBRARY_LAPACK_RELEASE
            NAMES lapack
            PATHS
            $ENV{LAPACK_HOME}           
            $ENV{LAPACK_DIR}
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
        )

    FIND_LIBRARY(LAPACK_LIBRARY_BLAS_RELEASE
            NAMES blas
            PATHS
            $ENV{LAPACK_HOME}                   
            $ENV{LAPACK_DIR}                   
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
        )
    
	FIND_LIBRARY(LAPACK_LIBRARY_F2C_RELEASE
            NAMES libf2c 
            PATHS
            $ENV{LAPACK_HOME}                   
            $ENV{LAPACK_DIR}                   
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
        )
			
    FIND_LIBRARY(LAPACK_LIBRARY_LAPACK_DEBUG
            NAMES lapackd 
            PATHS
            $ENV{LAPACK_HOME}           
            $ENV{LAPACK_DIR}
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
        )

    FIND_LIBRARY(LAPACK_LIBRARY_BLAS_DEBUG
            NAMES blasd 
            PATHS
            $ENV{LAPACK_HOME}                   
            $ENV{LAPACK_DIR}                   
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
       )
		
	FIND_LIBRARY(LAPACK_LIBRARY_F2C_DEBUG
            NAMES libf2cd 
            PATHS
            $ENV{LAPACK_HOME}                   
            $ENV{LAPACK_DIR}                   
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
        )
				 
    IF((LAPACK_LIBRARY_LAPACK_RELEASE AND LAPACK_LIBRARY_BLAS_RELEASE AND LAPACK_LIBRARY_F2C_RELEASE))
      LIST(APPEND LAPACK_LIBRARIES optimized ${LAPACK_LIBRARY_LAPACK_RELEASE})
      LIST(APPEND LAPACK_LIBRARIES optimized ${LAPACK_LIBRARY_BLAS_RELEASE})
 	  LIST(APPEND LAPACK_LIBRARIES optimized ${LAPACK_LIBRARY_F2C_RELEASE})

      SET(LAPACK_FOUND TRUE)
    ENDIF()
    IF((LAPACK_LIBRARY_LAPACK_DEBUG AND LAPACK_LIBRARY_BLAS_DEBUG AND LAPACK_LIBRARY_F2C_DEBUG))
      LIST(APPEND LAPACK_LIBRARIES debug ${LAPACK_LIBRARY_LAPACK_DEBUG})
      LIST(APPEND LAPACK_LIBRARIES debug ${LAPACK_LIBRARY_BLAS_DEBUG})
	  LIST(APPEND LAPACK_LIBRARIES debug ${LAPACK_LIBRARY_F2C_DEBUG})
      SET(LAPACK_FOUND TRUE)
    ENDIF()


ELSE(WIN32)
    FIND_LIBRARY(LAPACK_LIBRARY_LAPACK
            NAMES lapack
            PATHS
            $ENV{LAPACK_HOME}
            $ENV{LAPACK_DIR}
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
            /usr/lib
            /usr/lib64
            /usr/local/lib
            /usr/local/lib64
        )

    FIND_LIBRARY(LAPACK_LIBRARY_BLAS
            NAMES blas
            PATHS
            $ENV{LAPACK_DIR}
            $ENV{LAPACK_HOME}
            $ENV{LAPACK_HOME}/lib                   
            $ENV{LAPACK_DIR}/lib                   
            /usr/lib
            /usr/lib64
            /usr/local/lib
            /usr/local/lib64        
        )
  IF((LAPACK_LIBRARY_LAPACK AND LAPACK_LIBRARY_BLAS))
    SET(LAPACK_LIBRARIES ${LAPACK_LIBRARY_LAPACK} ${LAPACK_LIBRARY_BLAS})
    SET(LAPACK_FOUND TRUE)
  ENDIF()
ENDIF(WIN32)
## --------------------------------


MARK_AS_ADVANCED(
  LAPACK_LIBRARIES
  LAPACK_LIBRARY_LAPACK
  LAPACK_LIBRARY_BLAS
  LAPACK_LIBRARY_LAPACK_RELEASE
  LAPACK_LIBRARY_BLAS_RELEASE
  LAPACK_LIBRARY_LAPACK_DEBUG
  LAPACK_LIBRARY_BLAS_DEBUG
  LAPACK_LIBRARY_F2C_DEBUG
  LAPACK_LIBRARY_F2C_RELEASE
)

