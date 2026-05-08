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
# ViSP configuration file.
#
#############################################################################

# Catch2 for testing
if(USE_CATCH2)
  if(BUILD_CATCH2)
    list(APPEND opt_test_incs ${CATCH2_INCLUDE_DIRS})
    list(APPEND opt_test_libs ${CATCH2_LIBRARIES})
  else()
    set(_inc_dirs "")
    set(_lnk_libs "")
    vp_get_interface_include_dirs(CATCH2_LIBRARIES _inc_dirs)
    vp_get_interface_link_libraries(CATCH2_LIBRARIES _lnk_libs)
    list(APPEND opt_test_incs ${_inc_dirs})
    list(APPEND opt_test_libs ${_lnk_libs})
  endif()
endif()

# visp_robot is optional to run testFeatureSegment.cpp
vp_add_module(vs visp_core visp_visual_features)
vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()
vp_add_tests(DEPENDS_ON visp_blob visp_io visp_gui visp_robot visp_visual_features PRIVATE_INCLUDE_DIRS ${opt_test_incs} PRIVATE_LIBRARIES ${opt_test_libs})
