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

#vp_define_module(sensor)

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

# camera devices: v4l, dc1394, cmu1394, directshow ?, freenect
if(USE_V4L2)
  list(APPEND opt_incs ${V4L2_INCLUDE_DIRS})
  list(APPEND opt_libs ${V4L2_LIBRARIES})
endif()
if(USE_DC1394)
  list(APPEND opt_incs ${DC1394_INCLUDE_DIRS})
  list(APPEND opt_libs ${DC1394_LIBRARY})
endif()
if(USE_CMU1394)
  list(APPEND opt_incs ${CMU1394_INCLUDE_DIRS})
  list(APPEND opt_libs ${CMU1394_LIBRARIES})
endif()
if(USE_DIRECTSHOW)
  list(APPEND opt_incs ${DIRECTSHOW_INCLUDE_DIRS})
  list(APPEND opt_libs ${DIRECTSHOW_LIBRARIES})
endif()
if(USE_LIBFREENECT)
  list(APPEND opt_incs ${LIBFREENECT_INCLUDE_DIRS})
  list(APPEND opt_libs ${LIBFREENECT_LIBRARIES})
endif()
if(USE_LIBUSB_1)
  list(APPEND opt_incs ${LIBUSB_1_INCLUDE_DIRS})
  list(APPEND opt_libs ${LIBUSB_1_LIBRARIES})
endif()

vp_add_module(sensor visp_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(CTEST_EXCLUDE_PATH framegrabber DEPENDS_ON visp_io)
