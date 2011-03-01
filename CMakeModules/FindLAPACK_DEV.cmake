#############################################################################
#
# $Id: ${Id}
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
# Try to find linux/videodev.h for Video For Linux Two framegrabbing
# capabilities.
# Once run this will define:
#
# LAPACK_DEV_FOUND
# LAPACK_DEV_LIBRARIES
#
# Authors:
# Filip Novotny
#
#############################################################################


FIND_LIBRARY(LAPACK_DEV_LIBRARY_LAPACK
        NAMES lapack
        PATHS
        $ENV{CLAPACK_DEV_HOME}/lib
        $ENV{CLAPACK_DEV_DIR}/lib
        /usr/lib
        /usr/lib64
        /usr/local/lib
        /usr/local/lib64
    )

FIND_LIBRARY(LAPACK_DEV_LIBRARY_BLAS
        NAMES blas
        PATHS
        $ENV{CLAPACK_DEV_HOME}/lib
        $ENV{CLAPACK_DEV_DIR}/lib
        /usr/lib
        /usr/lib64
        /usr/local/lib
        /usr/local/lib64
    )

## --------------------------------


IF(LAPACK_DEV_LIBRARY_LAPACK AND LAPACK_DEV_LIBRARY_BLAS)
SET(LAPACK_DEV_LIBRARIES ${LAPACK_DEV_LIBRARY_LAPACK_DEV} ${LAPACK_DEV_LIBRARY_BLAS})
SET(LAPACK_DEV_INCLUDE_DIR ${LAPACK_DEV_INCLUDE_LAPACK})
SET(LAPACK_DEV_FOUND TRUE)
ELSE()
SET(LAPACK_DEV_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
LAPACK_DEV_LIBRARIES
)
