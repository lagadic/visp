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

# Display X11, GTK, GDI, D3D9. OpenCV already handled by core module
if(USE_X11)
  # X11 is private
  include_directories(${X11_INCLUDE_DIR})
  list(APPEND opt_libs_private ${X11_LIBRARIES})
  mark_as_advanced(X11_xcb_icccm_INCLUDE_PATH)
  mark_as_advanced(X11_xcb_icccm_LIB)
  mark_as_advanced(X11_xcb_xkb_INCLUDE_PATH)
endif()

if(USE_GTK2)
  # gtk2 is private
  include_directories(${GTK2_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${GTK2_LIBRARIES})
endif()

if(USE_GDI)
  list(APPEND opt_libs ${GDI_LIBRARIES})
endif()

if(USE_DIRECT3D)
  list(APPEND opt_incs ${DIRECT3D_INCLUDE_DIRS})
  list(APPEND opt_libs ${DIRECT3D_LIBRARIES})
endif()

if(USE_PCL)
  list(APPEND opt_incs ${PCL_INCLUDE_DIRS})
  # To ensure to build with VTK and other PCL 3rd parties we are not using PCL_LIBRARIES but PCL_DEPS_INCLUDE_DIRS
  # and PCL_DEPS_LIBRARIES instead
  list(APPEND opt_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

vp_add_module(gui visp_core PRIVATE_OPTIONAL ${opt_libs_private})
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(DEPENDS_ON visp_io)

if(USE_X11)
  vp_set_source_file_compile_flag(src/display/vpDisplayX.cpp -Wno-strict-overflow)
endif()
if(USE_OPENCV)
  vp_set_source_file_compile_flag(src/display/vpDisplayOpenCV.cpp -Wno-strict-overflow -Wno-float-equal)
  vp_set_source_file_compile_flag(src/forward-projection/vpProjectionDisplay.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/plot/vpPlot.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/plot/vpPlotCurve.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/plot/vpPlotGraph.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display/testDisplayPolygonLines.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display/testDisplayRoi.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display/testDisplays.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display/testVideoDevice.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display/testVideoDeviceDual.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display-with-dataset/testMouseEvent.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display-with-dataset/testDisplayScaled.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/display-with-dataset/testClick.cpp -Wno-float-equal)
endif()
if(USE_GTK2)
  vp_set_source_file_compile_flag(src/display/vpDisplayGTK.cpp -Wno-deprecated-declarations)
endif()
if(USE_PCL)
  vp_set_source_file_compile_flag(src/pointcloud/vpDisplayPCL.cpp -Wno-unused-parameter)
  vp_set_source_file_compile_flag(src/pointcloud/vpPclViewer.cpp -Wno-unused-parameter)
endif()
