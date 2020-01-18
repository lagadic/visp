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
# Try to find lapack, the Linear Algebra PACKage.
# Since the FindLAPACK.cmake provided with CMake is for Fortran language,
# this file is able to detect lapack for C language.
# Once run this will define:
#
# LAPACK_C_FOUND
# LAPACK_C_LIBRARIES
# LAPACK_C_VERSION
#
# Authors:
# Filip Novotny
# Fabien Spindler
#
#############################################################################

set(LAPACK_C_FOUND FALSE)
set(LAPACK_C_LIBRARIES "")
set(LAPACK_C_VERSION "n/a")
if(WIN32)
  set(LAPACK_C_LIB_SEARCH_PATH
    $ENV{LAPACK_C_HOME}
    $ENV{LAPACK_C_DIR}
    $ENV{LAPACK_C_HOME}/lib
    $ENV{LAPACK_C_DIR}/lib
    $ENV{LAPACK_HOME}
    $ENV{LAPACK_DIR}
    $ENV{LAPACK_HOME}/lib
    $ENV{LAPACK_DIR}/lib
  )

  find_library(LAPACK_C_LIBRARY_LAPACK_C_RELEASE
    NAMES lapack
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )

  find_library(LAPACK_C_LIBRARY_BLAS_RELEASE
    NAMES blas
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )
    
  find_library(LAPACK_C_LIBRARY_F2C_RELEASE
    NAMES libf2c
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )
			
  find_library(LAPACK_C_LIBRARY_LAPACK_C_DEBUG
    NAMES lapackd
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )

  find_library(LAPACK_C_LIBRARY_BLAS_DEBUG
    NAMES blasd
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )
		
  find_library(LAPACK_C_LIBRARY_F2C_DEBUG
    NAMES libf2cd
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )
				 
  if((LAPACK_C_LIBRARY_LAPACK_C_RELEASE AND LAPACK_C_LIBRARY_BLAS_RELEASE AND LAPACK_C_LIBRARY_F2C_RELEASE))
    list(APPEND LAPACK_C_LIBRARIES optimized ${LAPACK_C_LIBRARY_LAPACK_C_RELEASE})
    list(APPEND LAPACK_C_LIBRARIES optimized ${LAPACK_C_LIBRARY_BLAS_RELEASE})
    list(APPEND LAPACK_C_LIBRARIES optimized ${LAPACK_C_LIBRARY_F2C_RELEASE})
    set(LAPACK_C_FOUND TRUE)
  endif()
  if((LAPACK_C_LIBRARY_LAPACK_C_DEBUG AND LAPACK_C_LIBRARY_BLAS_DEBUG AND LAPACK_C_LIBRARY_F2C_DEBUG))
    list(APPEND LAPACK_C_LIBRARIES debug ${LAPACK_C_LIBRARY_LAPACK_C_DEBUG})
    list(APPEND LAPACK_C_LIBRARIES debug ${LAPACK_C_LIBRARY_BLAS_DEBUG})
    list(APPEND LAPACK_C_LIBRARIES debug ${LAPACK_C_LIBRARY_F2C_DEBUG})
    set(LAPACK_C_FOUND TRUE)
  endif()

else(WIN32)

  set(LAPACK_C_LIB_SEARCH_PATH
    $ENV{LAPACK_C_HOME}
    $ENV{LAPACK_C_DIR}
    $ENV{LAPACK_C_HOME}/lib
    $ENV{LAPACK_C_DIR}/lib
    $ENV{LAPACK_HOME}
    $ENV{LAPACK_DIR}
    $ENV{LAPACK_HOME}/lib
    $ENV{LAPACK_DIR}/lib
    /usr/lib
    /usr/lib64
    /usr/local/lib
    /usr/local/lib64
  )
  find_library(LAPACK_C_LIBRARY_LAPACK
    NAMES lapack
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )

  find_library(LAPACK_C_LIBRARY_BLAS
    NAMES blas
    PATHS ${LAPACK_C_LIB_SEARCH_PATH}
  )

  if((LAPACK_C_LIBRARY_LAPACK AND LAPACK_C_LIBRARY_BLAS))
    set(LAPACK_C_LIBRARIES ${LAPACK_C_LIBRARY_LAPACK} ${LAPACK_C_LIBRARY_BLAS})
    set(LAPACK_C_FOUND TRUE)

    get_filename_component(LAPACK_C_LIB_DIR ${LAPACK_C_LIBRARY_LAPACK} PATH)
    vp_get_version_from_pkg("lapack" "${LAPACK_C_LIB_DIR}/pkgconfig" LAPACK_C_VERSION)

  endif()
endif(WIN32)

mark_as_advanced(
  LAPACK_C_LIBRARIES
  LAPACK_C_LIBRARY_LAPACK
  LAPACK_C_LIBRARY_BLAS
  LAPACK_C_LIBRARY_LAPACK_C_RELEASE
  LAPACK_C_LIBRARY_BLAS_RELEASE
  LAPACK_C_LIBRARY_LAPACK_C_DEBUG
  LAPACK_C_LIBRARY_BLAS_DEBUG
  LAPACK_C_LIBRARY_F2C_DEBUG
  LAPACK_C_LIBRARY_F2C_RELEASE
)

