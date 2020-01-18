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

# Display X11, GTK, GDI, D3D9. OpenCV already handled by core module
if(USE_X11)
  list(APPEND opt_incs ${X11_INCLUDE_DIR})
  list(APPEND opt_libs ${X11_LIBRARIES})
endif()
if(USE_GTK2)
  list(APPEND opt_incs ${GTK2_INCLUDE_DIRS})
  list(APPEND opt_libs ${GTK2_LIBRARIES})
  # FS next lines removed until they proved to be useful
#  if(UNIX)
#    ## ignore gtk pedantic warnings by gcc specific -isystem :
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK2_gtk_INCLUDE_PATH}")
#  endif()
endif()
if(USE_GDI)
  list(APPEND opt_libs ${GDI_LIBRARIES})
endif()
if(USE_DIRECT3D)
  list(APPEND opt_incs ${DIRECT3D_INCLUDE_DIRS})
  list(APPEND opt_libs ${DIRECT3D_LIBRARIES})
endif()

vp_add_module(gui visp_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(DEPENDS_ON visp_io)

vp_set_source_file_compile_flag(src/display/vpDisplayX.cpp -Wno-strict-overflow)
vp_set_source_file_compile_flag(src/display/vpDisplayOpenCV.cpp -Wno-strict-overflow)
