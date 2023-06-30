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

# Display X11, GTK, GDI, D3D9. OpenCV already handled by core module
if(USE_X11)
  list(APPEND opt_incs ${X11_INCLUDE_DIR})
  list(APPEND opt_libs ${X11_LIBRARIES})
  mark_as_advanced(X11_xcb_icccm_INCLUDE_PATH)
  mark_as_advanced(X11_xcb_icccm_LIB)
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
if(USE_PCL)
  list(APPEND opt_incs ${PCL_INCLUDE_DIRS})

  # list(APPEND opt_libs ${PCL_LIBRARIES})
  # Using PCL_LIBRARIES works to build visp library, examples, demos and test thanks to the components,
  # but not tutorials when they are build outside ViSP as they are stand alone CMake projects that use
  # ViSP as a 3rd party.
  # To be clear PCL_LIBRARIES contains VTK 3rd party such as vtkalglib and not /usr/local/Cellar/vtk/6.3.0/lib/libvtkalglib-6.3.1.dylib
  # full path as requested to use ViSP as 3rd party. This is the case for all VTK libraries that are PCL dependencies.
  # The build of ViSP works with PCL_LIBRARIES since in that case thanks to vtkalglib properties, CMake
  # is able to find the real name and location of the libraries.
  # But when ViSP is used as a 3rd party where it should import PCL libraries, it doesn't work with
  # PCL_LIBRARIES and especially with VTK_LIBRARIES.
  # The solution here is to get the full location of VTK_LIBRARIES libraries thanks to the properties and link
  # with these names.
  # An other way could be to include PCLConfig.cmake, but in that case, visp-config and visp.pc
  # will be not able to give the names of PCL libraries when used without CMake.
  vp_find_pcl(PCL_LIBRARIES PCL_DEPS_INCLUDE_DIRS PCL_DEPS_LIBRARIES)
  list(APPEND opt_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

vp_add_module(gui visp_core)
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
