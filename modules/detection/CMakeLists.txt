#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# ViSP configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

if(USE_ZBAR)
  list(APPEND opt_incs ${ZBAR_INCLUDE_DIRS})
  list(APPEND opt_libs ${ZBAR_LIBRARIES})
endif()
if(USE_DMTX)
  list(APPEND opt_incs ${DMTX_INCLUDE_DIRS})
  list(APPEND opt_libs ${DMTX_LIBRARIES})
endif()

if(USE_ZBAR)
  # Add specific build flag to turn off warnings coming from zbar 3rd party
  vp_set_source_file_compile_flag(src/barcode/vpDetectorQRCode.cpp -Wno-unused-parameter)
endif()

# April Tag is private
include_directories(${APRILTAG_INCLUDE_DIRS})

vp_add_module(detection visp_core visp_vision PRIVATE_OPTIONAL ${APRILTAG_LIBRARIES})
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

