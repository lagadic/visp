#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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

if(USE_AFMA4 OR USE_AFMA6 OR USE_VIPER650 OR USE_VIPER850)
  list(APPEND opt_incs ${RAW1394_INCLUDE_DIRS})
  list(APPEND opt_incs ${CALINUX_INCLUDE_DIRS})
  list(APPEND opt_incs ${IRISA_INCLUDE_DIRS})
  list(APPEND opt_libs ${IRISA_LIBRARIES})
  list(APPEND opt_libs ${CALINUX_LIBRARIES})
  list(APPEND opt_libs ${RAW1394_LIBRARIES})
  list(APPEND opt_libs ${RT_LIBRARIES})
endif()

if(USE_BICLOPS)
  list(APPEND opt_incs ${BICLOPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${BICLOPS_LIBRARIES})
endif()

if(USE_PTU46)
  list(APPEND opt_incs ${PTU46_INCLUDE_DIRS})
  list(APPEND opt_libs ${PTU46_LIBRARIES})
endif()

if(USE_ARIA AND UNIX AND USE_PTHREAD AND RT_FOUND AND DL_FOUND)
  # Under Unix we need Aria, pthread, dl and rt libraries
  list(APPEND opt_incs ${ARIA_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARIA_LIBRARIES})
  list(APPEND opt_incs ${PTHREAD_INCLUDE_DIRS})
  list(APPEND opt_libs ${PTHREAD_LIBRARIES})
  list(APPEND opt_libs ${RT_LIBRARIES})
  list(APPEND opt_libs ${DL_LIBRARIES})
elseif(USE_ARIA AND NOT UNIX)
  list(APPEND opt_incs ${ARIA_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARIA_LIBRARIES})
endif()

# for the wireframe simulator we can use optional opengl and coin
if(USE_COIN3D)
  if(WIN32)
    add_definitions("-DCOIN_DLL")
  endif()
  list(APPEND opt_incs ${COIN3D_INCLUDE_DIRS})
  # On OSX cmake 2.8 found OpenGL but OPENGL_INCLUDE_DIR was set to NOT_FOUND
  # We add a test to be sure that the OPENGL vars exist.
  if(OPENGL_INCLUDE_DIR)
    list(APPEND opt_incs ${OPENGL_INCLUDE_DIR})
  endif()
  if(OPENGL_LIBRARIES)
    list(APPEND opt_libs ${OPENGL_LIBRARIES})
  endif()
  list(APPEND opt_libs ${COIN3D_LIBRARIES})
endif()

vp_add_module(robot visp_core OPTIONAL visp_io visp_gui)
vp_glob_module_sources()

# copy robot and wireframe simulator data
vp_glob_module_copy_data("src/robot-simulator/arms/*.bnd"      data/robot-simulator)
vp_glob_module_copy_data("src/wireframe-simulator/scene/*.bnd" data/wireframe-simulator)
vp_glob_module_copy_data("src/wireframe-simulator/scene/*.sld" data/wireframe-simulator)

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(DEPENDS_ON visp_sensor visp_vision visp_blob visp_gui)
