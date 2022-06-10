#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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

if(WITH_ATIDAQ)
  # atidac is private
  include_directories(${ATIDAQ_INCLUDE_DIRS})
endif()

vp_add_module(sensor visp_core PRIVATE_OPTIONAL ${ATIDAQ_LIBRARIES})
vp_glob_module_sources()

if(USE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  vp_set_source_file_compile_flag(src/framegrabber/flycapture/vpFlyCaptureGrabber.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()
if(USE_PYLON)
  # Add specific build flag to turn off warnings coming from Basler
  # pylon headers
  if(MSVC)
  if(BUILD_SHARED_LIBS)
      vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberGigE.cpp /wd4244 /wd4267)
      vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberUsb.cpp /wd4244 /wd4267)
  else()
    vp_warnings_disable(CMAKE_CXX_FLAGS /wd4244 /wd4267)
  endif()
  else()
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonFactory.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual -Wno-deprecated-copy -Wno-unused-variable)
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberGigE.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual -Wno-deprecated-copy -Wno-unused-variable)
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberUsb.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual -Wno-deprecated-copy -Wno-unused-variable)
    vp_set_source_file_compile_flag(test/framegrabber/testPylonGrabber.cpp -Wno-unknown-pragmas -Wno-overloaded-virtual -Wno-unused-parameter -Wno-deprecated-copy -Wno-unused-variable)
  endif()
endif()
if(USE_REALSENSE)
  # Add specific build flag to turn off warnings coming from RealSense 3rd party
  vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense.cpp -Wno-strict-aliasing -Wno-pessimizing-move -Wno-unused-parameter)
endif()
if(USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from RealSense2 3rd party
    vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense2.cpp -Wno-reorder -Wno-unused-function -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers -Wno-deprecated-copy)
  else()
    vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense2.cpp /wd4244 /wd4267)
  endif()
endif()
if(USE_REALSENSE OR USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_R200.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_SR300.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_SR300.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_align.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_opencv.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_pcl.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry_async.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_imu.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_odometry.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_undistort.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers -Wno-deprecated-copy)
  else()
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_SR300.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_align.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_opencv.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435_pcl.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry_async.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_imu.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_odometry.cpp /wd4244 /wd4267)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_undistort.cpp /wd4244 /wd4267)
  endif()
endif()
if(USE_LIBFREENECT)
  vp_set_source_file_compile_flag(src/rgb-depth/kinect/vpKinect.cpp -Wno-unused-parameter)
endif()
if(USE_UEYE)
  if(UNIX)
    vp_set_source_file_compile_flag(src/framegrabber/ueye/vpUeyeGrabber.cpp -Wno-deprecated-declarations)
  else()
    vp_set_source_file_compile_flag(src/framegrabber/ueye/vpUeyeGrabber.cpp /wd4996)
  endif()
endif()

vp_set_source_file_compile_flag(src/force-torque/vpForceTorqueAtiNetFTSensor.cpp -Wno-strict-aliasing)

if(USE_OCCIPITAL_STRUCTURE)
  vp_set_source_file_compile_flag(src/rgb-depth/occipital_structure/vpOccipitalStructure.cpp -Wno-reorder -Wno-unused-function -Wno-unused-parameter -Wno-sign-compare -Wno-overloaded-virtual -Wno-ignored-qualifiers -Wno-maybe-uninitialized)
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_images.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers)
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_imu.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers)
  vp_set_source_file_compile_flag(test/rgb-depth/testOccipitalStructure_Core_pcl.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment -Wno-ignored-qualifiers)
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(CTEST_EXCLUDE_PATH framegrabber force-torque rgb-depth DEPENDS_ON visp_io visp_gui)

# Add configuration file for IIT FT sensor besides testForceTorqueIitSensor.cpp binary
configure_file(test/force-torque/configurationSettings.ini configurationSettings.ini COPYONLY)

if(USE_OPENCV)
  vp_set_source_file_compile_flag(src/framegrabber/1394/vp1394TwoGrabber.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/framegrabber/directshow/vpDirectShowSampleGrabberI.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/rgb-depth/kinect/vpKinect.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/framegrabber/test1394TwoResetBus.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/framegrabber/test1394TwoGrabber.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_SR300.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_imu.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_undistort.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry_async.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry_async.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_R200.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_odometry.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry.cpp -Wno-float-equal)
  vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_T265_images_odometry.cpp -Wno-float-equal)
endif()
