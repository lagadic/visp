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

#vp_define_module(sensor)

# Add optional 3rd parties
set(opt_incs "")
set(opt_system_incs "")
set(opt_libs "")

# camera devices: v4l, dc1394, cmu1394, directshow ?, freenect,
# flycapture, pylon
if(USE_V4L2)
  list(APPEND opt_system_incs ${V4L2_INCLUDE_DIRS})
  list(APPEND opt_libs ${V4L2_LIBRARIES})
endif()
if(USE_DC1394)
  list(APPEND opt_system_incs ${DC1394_INCLUDE_DIRS})
  list(APPEND opt_libs ${DC1394_LIBRARY})
endif()
if(USE_CMU1394)
  list(APPEND opt_system_incs ${CMU1394_INCLUDE_DIRS})
  list(APPEND opt_libs ${CMU1394_LIBRARIES})
endif()
if(USE_DIRECTSHOW)
  list(APPEND opt_system_incs ${DIRECTSHOW_INCLUDE_DIRS})
  list(APPEND opt_libs ${DIRECTSHOW_LIBRARIES})
endif()
if(USE_LIBFREENECT)
  list(APPEND opt_system_incs ${LIBFREENECT_INCLUDE_DIRS})
  list(APPEND opt_libs ${LIBFREENECT_LIBRARIES})
endif()
if(USE_LIBUSB_1)
  list(APPEND opt_system_incs ${LIBUSB_1_INCLUDE_DIRS})
  list(APPEND opt_libs ${LIBUSB_1_LIBRARIES})
endif()
if(USE_FLYCAPTURE)
  list(APPEND opt_system_incs ${FLYCAPTURE_INCLUDE_DIRS})
  list(APPEND opt_libs ${FLYCAPTURE_LIBRARIES})
endif()
if(USE_PYLON)
  list(APPEND opt_system_incs ${PYLON_INCLUDE_DIRS})
  list(APPEND opt_libs ${PYLON_LIBRARIES})
endif()
if(USE_UEYE)
  list(APPEND opt_system_incs ${UEYE_INCLUDE_DIRS})
  list(APPEND opt_libs ${UEYE_LIBRARIES})
endif()
if(USE_COMEDI)
  list(APPEND opt_system_incs ${COMEDI_INCLUDE_DIRS})
  list(APPEND opt_libs ${COMEDI_LIBRARIES})
endif()
if(USE_REALSENSE)
  list(APPEND opt_system_incs ${REALSENSE_INCLUDE_DIRS})
  list(APPEND opt_libs ${REALSENSE_LIBRARIES})
endif()
if(USE_REALSENSE2)
  list(APPEND opt_system_incs ${REALSENSE2_INCLUDE_DIRS})
  list(APPEND opt_libs ${REALSENSE2_LIBRARIES})
endif()
if(USE_OCCIPITAL_STRUCTURE)
  list(APPEND opt_system_incs ${OCCIPITAL_STRUCTURE_INCLUDE_DIRS})
  list(APPEND opt_libs ${OCCIPITAL_STRUCTURE_LIBRARIES})
endif()
if(USE_FTIITSDK)
  list(APPEND opt_system_incs ${FTIITSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${FTIITSDK_LIBRARIES})
endif()
if(USE_PCL)
  list(APPEND opt_system_incs ${PCL_INCLUDE_DIRS})
  # To ensure to build with VTK and other PCL 3rd parties we are not using PCL_LIBRARIES but PCL_DEPS_INCLUDE_DIRS
  # and PCL_DEPS_LIBRARIES instead
  list(APPEND opt_system_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

if(USE_QUALISYS)
  list(APPEND opt_system_incs ${QUALISYS_INCLUDE_DIRS})
  list(APPEND opt_libs ${QUALISYS_LIBRARIES})
endif()

if(USE_VICON)
  list(APPEND opt_system_incs ${VICON_INCLUDE_DIRS})
  list(APPEND opt_libs ${VICON_LIBRARIES})
endif()

if(WITH_ATIDAQ)
  # atidac is private
  include_directories(SYSTEM ${ATIDAQ_INCLUDE_DIRS})
endif()

vp_add_module(sensor visp_core PRIVATE_OPTIONAL ${ATIDAQ_LIBRARIES})
vp_glob_module_sources()

vp_module_include_directories(${opt_incs} SYSTEM ${opt_system_incs})
vp_create_module(${opt_libs})
vp_add_tests(CTEST_EXCLUDE_PATH force-torque framegrabber mocap rgb-depth DEPENDS_ON visp_io visp_gui)

# Add configuration file for IIT FT sensor besides testForceTorqueIitSensor.cpp binary
configure_file(test/force-torque/configurationSettings.ini configurationSettings.ini COPYONLY)
