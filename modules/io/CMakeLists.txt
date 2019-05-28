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
# ViSP configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

# Image i/o: jpeg, png, OpenCV (already included by core)
if(USE_JPEG)
  list(APPEND opt_incs ${JPEG_INCLUDE_DIR})
  list(APPEND opt_libs ${JPEG_LIBRARIES})
  # If Qt3 and libjpeg are used, we add here QT_CLEAN_NAMESPACE define to
  # handle a compilation error "conflicting declaration 'typedef long int INT32'
  if(VISP_HAVE_QT3_FOUND)
    add_definitions("-DQT_CLEAN_NAMESPACE")
  endif()
endif()
if(USE_PNG)
  list(APPEND opt_incs ${PNG_INCLUDE_DIR})
  list(APPEND opt_libs ${PNG_LIBRARIES})
  add_definitions(${PNG_DEFINITIONS})
endif()

if (ANDROID)
  vp_add_module(io visp_core)
else()
  vp_add_module(io visp_core WRAP java)
endif()

vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(DEPENDS_ON visp_features)

vp_set_source_file_compile_flag(src/tools/vpParseArgv.cpp -Wno-strict-overflow)

