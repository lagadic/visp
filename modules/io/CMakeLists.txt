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

# videos: ffmpeg
if(USE_FFMPEG)
  list(APPEND opt_incs ${FFMPEG_INCLUDE_DIRS})
  list(APPEND opt_libs ${FFMPEG_LIBRARIES})
endif()

vp_add_module(io visp_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(DEPENDS_ON visp_features)
