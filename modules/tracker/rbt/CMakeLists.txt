#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# ViSP configuration file.
#
#############################################################################

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")
set(opt_libs_private "")

if(NOT USE_PANDA3D OR VISP_CXX_STANDARD LESS VISP_CXX_STANDARD_11)
  return()
endif()

if(USE_NLOHMANN_JSON)
  get_target_property(_inc_dirs "nlohmann_json::nlohmann_json" INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND opt_incs ${_inc_dirs})
endif()

if(WITH_SIMDLIB)
  # Simd lib is private
  include_directories(${SIMDLIB_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${SIMDLIB_LIBRARIES})
endif()

vp_add_module(rbt visp_mbt visp_vision visp_core visp_me visp_visual_features visp_ar OPTIONAL  visp_klt visp_gui PRIVATE_OPTIONAL ${opt_libs_private})
vp_glob_module_sources()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

vp_add_tests(DEPENDS_ON visp_core visp_gui visp_io)
