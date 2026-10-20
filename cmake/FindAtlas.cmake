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
# Find the Atlas (and Lapack) libraries
#
# The following variables are optionally searched for defaults
#  Atlas_ROOT_DIR:            Base directory where all Atlas components are found
#
# The following are set after configuration is done:
#  Atlas_FOUND
#  Atlas_INCLUDE_DIR
#  Atlas_LIBRARIES
#
#############################################################################

# Layouts handled, relative to each standard prefix (/usr, /usr/local,
# ${CMAKE_PREFIX_PATH}, ...) and to Atlas_ROOT_DIR:
#   <prefix>/include/atlas, <prefix>/include/atlas-base
#   <prefix>/lib/atlas, <prefix>/lib/atlas-base (also lib/<arch>/atlas)
set(Atlas_SEARCH_HINTS
  ${Atlas_ROOT_DIR}
  $ENV{Atlas_ROOT_DIR}
)
set(Atlas_INCLUDE_SUFFIXES include atlas atlas-base)
set(Atlas_LIB_SUFFIXES lib atlas atlas-base)

find_path(Atlas_CBLAS_INCLUDE_DIR
  NAMES cblas.h
  HINTS ${Atlas_SEARCH_HINTS}
  PATH_SUFFIXES ${Atlas_INCLUDE_SUFFIXES}
)
find_path(Atlas_CLAPACK_INCLUDE_DIR
  NAMES clapack.h
  HINTS ${Atlas_SEARCH_HINTS}
  PATH_SUFFIXES ${Atlas_INCLUDE_SUFFIXES}
)

find_library(Atlas_CBLAS_LIBRARY
  NAMES  ptcblas_r ptcblas cblas_r cblas
  HINTS ${Atlas_SEARCH_HINTS}
  PATH_SUFFIXES ${Atlas_LIB_SUFFIXES}
)
find_library(Atlas_BLAS_LIBRARY
  NAMES   atlas_r   atlas
  HINTS ${Atlas_SEARCH_HINTS}
  PATH_SUFFIXES ${Atlas_LIB_SUFFIXES}
)
find_library(Atlas_LAPACK_LIBRARY
  NAMES lapack alapack_r alapack lapack_atlas
  HINTS ${Atlas_SEARCH_HINTS}
  PATH_SUFFIXES ${Atlas_LIB_SUFFIXES}
)

set(LOOKED_FOR
  Atlas_CBLAS_INCLUDE_DIR
  Atlas_CLAPACK_INCLUDE_DIR

  Atlas_CBLAS_LIBRARY
  Atlas_BLAS_LIBRARY
  Atlas_LAPACK_LIBRARY
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Atlas DEFAULT_MSG ${LOOKED_FOR})

if(ATLAS_FOUND)
  set(Atlas_INCLUDE_DIR ${Atlas_CBLAS_INCLUDE_DIR} ${Atlas_CLAPACK_INCLUDE_DIR})
  set(Atlas_LIBRARIES ${Atlas_LAPACK_LIBRARY} ${Atlas_CBLAS_LIBRARY} ${Atlas_BLAS_LIBRARY})

  message(STATUS "Found Atlas (include: ${Atlas_CBLAS_INCLUDE_DIR}, library: ${Atlas_BLAS_LIBRARY})")
endif()

mark_as_advanced(${LOOKED_FOR})
