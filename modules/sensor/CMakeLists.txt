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
set(opt_libs "")

# camera devices: v4l, dc1394, cmu1394, directshow ?, freenect,
# flycapture, pylon
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
if(USE_FLYCAPTURE)
  list(APPEND opt_incs ${FLYCAPTURE_INCLUDE_DIRS})
  list(APPEND opt_libs ${FLYCAPTURE_LIBRARIES})
endif()
if(USE_PYLON)
  list(APPEND opt_incs ${PYLON_INCLUDE_DIRS})
  list(APPEND opt_libs ${PYLON_LIBRARIES})
endif()
if(USE_UEYE)
  list(APPEND opt_incs ${UEYE_INCLUDE_DIRS})
  list(APPEND opt_libs ${UEYE_LIBRARIES})
endif()
if(USE_COMEDI)
  list(APPEND opt_incs ${COMEDI_INCLUDE_DIRS})
  list(APPEND opt_libs ${COMEDI_LIBRARIES})
endif()
if(USE_REALSENSE)
  list(APPEND opt_incs ${REALSENSE_INCLUDE_DIRS})
  list(APPEND opt_libs ${REALSENSE_LIBRARIES})
endif()
if(USE_REALSENSE2)
  list(APPEND opt_incs ${REALSENSE2_INCLUDE_DIRS})
  list(APPEND opt_libs ${REALSENSE2_LIBRARIES})
endif()
if(USE_OCCIPITAL_STRUCTURE)
  list(APPEND opt_incs ${OCCIPITAL_STRUCTURE_INCLUDE_DIRS})
  list(APPEND opt_libs ${OCCIPITAL_STRUCTURE_LIBRARIES})
endif()
if(USE_FTIITSDK)
  list(APPEND opt_incs ${FTIITSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${FTIITSDK_LIBRARIES})
endif()
if(USE_PCL)
  list(APPEND opt_incs ${PCL_INCLUDE_DIRS})
  # To ensure to build with VTK and other PCL 3rd parties we are not using PCL_LIBRARIES but PCL_DEPS_INCLUDE_DIRS
  # and PCL_DEPS_LIBRARIES instead
  list(APPEND opt_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

if(USE_QUALISYS)
  list(APPEND opt_incs ${QUALISYS_INCLUDE_DIRS})
  list(APPEND opt_libs ${QUALISYS_LIBRARIES})
endif()

if(USE_VICON)
  list(APPEND opt_incs ${VICON_INCLUDE_DIRS})
  list(APPEND opt_libs ${VICON_LIBRARIES})
endif()

if(WITH_ATIDAQ)
  # atidac is private
  include_directories(${ATIDAQ_INCLUDE_DIRS})
endif()

vp_add_module(sensor visp_core PRIVATE_OPTIONAL ${ATIDAQ_LIBRARIES})
vp_glob_module_sources()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(CTEST_EXCLUDE_PATH force-torque framegrabber mocap rgb-depth DEPENDS_ON visp_io visp_gui)

# Add configuration file for IIT FT sensor besides testForceTorqueIitSensor.cpp binary
configure_file(test/force-torque/configurationSettings.ini configurationSettings.ini COPYONLY)

if(USE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unknown-pragmas")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
endif()

if(USE_PYLON)
  # Add specific build flag to turn off warnings coming from Basler
  # pylon headers
  if(MSVC)
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  else()
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unknown-pragmas")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-variable")
  endif()
endif()

if(USE_REALSENSE)
  # Add specific build flag to turn off warnings coming from RealSense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-strict-aliasing")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
endif()

if(USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-inconsistent-missing-override")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-conversion")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-float-equal")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-comment")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")
  else()
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  endif()
endif()

if(USE_LIBFREENECT)
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
endif()

if(USE_UEYE)
  if(UNIX)
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  else()
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4996")
  endif()
endif()

if(USE_OPENCV)
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-float-equal")
endif()

if(USE_OCCIPITAL_STRUCTURE)
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-maybe-uninitialized")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-inconsistent-missing-override")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-conversion")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-float-equal")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
endif()

vp_set_source_file_compile_flag(src/force-torque/vpForceTorqueAtiNetFTSensor.cpp "-Wno-strict-aliasing")

if(CXX_FLAGS_MUTE_WARNINGS)
  # Add specific build flag to turn off warnings
  vp_set_source_file_compile_flag(src/framegrabber/1394/vp1394TwoGrabber.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/directshow/vpDirectShowSampleGrabberI.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/flycapture/vpFlyCaptureGrabber.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonFactory.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberGigE.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberUsb.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/framegrabber/ueye/vpUeyeGrabber.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/rgb-depth/kinect/vpKinect.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/rgb-depth/occipital_structure/vpOccipitalStructure.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense2.cpp ${CXX_FLAGS_MUTE_WARNINGS})

  vp_set_source_file_compile_flag(test/framegrabber/test1394TwoResetBus.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/framegrabber/test1394TwoGrabber.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/framegrabber/testPylonGrabber.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_images.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_imu.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_pcl.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_SR300.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_align.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_opencv.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_pcl.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry_async.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_imu.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_odometry.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_undistort.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()
