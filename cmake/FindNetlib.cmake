#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# See https://visp.inria.fr for more information.
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
# Try to find Netlib Linear Algebra PACKage.
# Once run this will define:
#
# NETLIB_FOUND
# NETLIB_LIBRARIES
# NETLIB_VERSION
#
#############################################################################


set(NETLIB_FOUND FALSE)
set(NETLIB_LIBRARIES "")
set(NETLIB_VERSION "n/a")

set(NETLIB_SEARCH_HINTS
  ${NETLIB_HOME}
  $ENV{NETLIB_HOME}
  ${NETLIB_DIR}
  $ENV{NETLIB_DIR}
  ${LAPACK_HOME}
  $ENV{LAPACK_HOME}
  ${LAPACK_DIR}
  $ENV{LAPACK_DIR}
)

if(WIN32)
  find_library(NETLIB_LIBRARY_NETLIB_RELEASE
    NAMES lapack
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(NETLIB_LIBRARY_BLAS_RELEASE
    NAMES blas
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(NETLIB_LIBRARY_NETLIB_DEBUG
    NAMES lapackd
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(NETLIB_LIBRARY_BLAS_DEBUG
    NAMES blasd
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  if(NETLIB_LIBRARY_NETLIB_RELEASE AND NETLIB_LIBRARY_BLAS_RELEASE)
    list(APPEND NETLIB_LIBRARIES optimized ${NETLIB_LIBRARY_NETLIB_RELEASE})
    list(APPEND NETLIB_LIBRARIES optimized ${NETLIB_LIBRARY_BLAS_RELEASE})
    set(NETLIB_FOUND TRUE)
  endif()

  if(NETLIB_LIBRARY_NETLIB_DEBUG AND NETLIB_LIBRARY_BLAS_DEBUG)
    list(APPEND NETLIB_LIBRARIES debug ${NETLIB_LIBRARY_NETLIB_DEBUG})
    list(APPEND NETLIB_LIBRARIES debug ${NETLIB_LIBRARY_BLAS_DEBUG})
    set(NETLIB_FOUND TRUE)
  endif()
else()
  find_library(NETLIB_LIBRARY_LAPACK
    NAMES lapack
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(NETLIB_LIBRARY_BLAS
    NAMES blas
    HINTS ${NETLIB_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  if(NETLIB_LIBRARY_LAPACK AND NETLIB_LIBRARY_BLAS)
    set(NETLIB_LIBRARIES ${NETLIB_LIBRARY_LAPACK} ${NETLIB_LIBRARY_BLAS})
    set(NETLIB_FOUND TRUE)

    get_filename_component(NETLIB_LIB_DIR ${NETLIB_LIBRARY_LAPACK} PATH)
    vp_get_version_from_pkg("lapack" "${NETLIB_LIB_DIR}/pkgconfig" NETLIB_VERSION)
  endif()
endif()

mark_as_advanced(
  NETLIB_LIBRARY_LAPACK
  NETLIB_LIBRARY_BLAS
  NETLIB_LIBRARY_NETLIB_RELEASE
  NETLIB_LIBRARY_BLAS_RELEASE
  NETLIB_LIBRARY_NETLIB_DEBUG
  NETLIB_LIBRARY_BLAS_DEBUG
)
