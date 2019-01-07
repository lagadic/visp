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
  foreach(lib_ ${PCL_LIBRARIES})
    # check if ${lib_} is in VTK_LIBRARIES and is a TARGET. If this is the case, we retrieve the full location
    list(FIND VTK_LIBRARIES ${lib_} lib_is_vtk_)
    if(NOT ${lib_is_vtk_} EQUAL -1 AND TARGET ${lib_})
      # This is a VTK library
      list(APPEND PCL_VTK_LIBRARIES ${lib_})
    else()
      # Other libraries sqlite3, boost..., optimized, debug
      if(EXISTS ${lib_} OR ${lib_} MATCHES "optimized" OR ${lib_} MATCHES "debug")
        list(APPEND opt_libs ${lib_})
      else()
        find_library(${lib_}_LIBRARY ${lib_} QUIET)
        mark_as_advanced(${lib_}_LIBRARY)
        if(${lib_}_LIBRARY)
          list(APPEND opt_libs ${${lib_}_LIBRARY})
        else()
          list(APPEND opt_libs ${lib_})
        endif()
      endif()
    endif()
  endforeach()

  find_package(VTK QUIET)
  if (VTK_FOUND AND NOT ANDROID)
    # Fix for Ubuntu 16.04 to add vtkFiltering as dependency. Note that vtkFiltering doesn't exists on OSX
    list(FIND VTK_LIBRARIES "vtkFiltering" vtkFiltering_exists_)
    if(NOT ${vtkFiltering_exists_} EQUAL -1)
      list(APPEND PCL_VTK_LIBRARIES "vtkFiltering") # seems required on Ubuntu 16.04
    endif()
    if(VTK_VERSION VERSION_EQUAL 6.2.0)
      # Work arround to avoid build issue on ubuntu 16.04 with libvtk6-dev package
      # cannot find -lvtkproj4
      # See https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
      list(REMOVE_ITEM opt_libs "vtkproj4")
    endif()
    # When VTK kits are enabled we need to get imported targets
    vp_list_unique(PCL_VTK_LIBRARIES)
    if(NOT VTK_ENABLE_KITS)
      set(PCL_VTK_LIBS ${PCL_VTK_LIBRARIES})
    else()
      foreach(lib_ ${PCL_VTK_LIBRARIES})
        get_target_property(imported_libs_ ${lib_} INTERFACE_LINK_LIBRARIES)
        if(imported_libs_)
          list(APPEND PCL_VTK_IMPORTED_LIBS ${imported_libs_} ${prop})
          if(${lib_}_KIT)
            list(APPEND PCL_VTK_LIBS ${${lib_}_KIT})
          endif()
        else()
          list(APPEND PCL_VTK_LIBS ${lib_})
        endif()
      endforeach()
      vp_list_unique(PCL_VTK_IMPORTED_LIBS)
      while(PCL_VTK_IMPORTED_LIBS)
        vp_list_pop_front(PCL_VTK_IMPORTED_LIBS elt)
        if(elt MATCHES "^vtk") # to avoid precessing -framework ApplicationServices -framework CoreServices
          get_target_property(imported_libs_ ${elt} INTERFACE_LINK_LIBRARIES)
          if(imported_libs_)
            list(APPEND PCL_VTK_IMPORTED_LIBS ${imported_libs_})
          else()
            list(APPEND PCL_VTK_LIBS ${elt})
          endif()
        else()
          list(APPEND PCL_VTK_LIBS ${elt})
        endif()
        vp_list_unique(PCL_VTK_IMPORTED_LIBS)
      endwhile()
      vp_list_unique(PCL_VTK_LIBS)
    endif()

    set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
    foreach(lib_ ${PCL_VTK_LIBS})
      if(TARGET ${lib_})
        foreach(imp_config_ ${config_})
          get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_IMPLIB_${imp_config_})
          if(NOT EXISTS ${lib_property_${imp_config_}_})
            get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})
          endif()
          # Under Unix, there is no specific suffix for PCL/VTK libraries.         # Under Windows, we add the "optimized", "debug" specific keywords
          if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
            list(APPEND opt_libs optimized "${lib_property_${imp_config_}_}")
          elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
            list(APPEND opt_libs debug     "${lib_property_${imp_config_}_}")
          elseif(EXISTS "${lib_property_${imp_config_}_}")
            list(APPEND opt_libs "${lib_property_${imp_config_}_}")
          endif()
        endforeach()
      else()
        list(APPEND opt_libs "${lib_}")
      endif()
    endforeach()
  endif()
  mark_as_advanced(Qt5Core_DIR Qt5Gui_DIR Qt5Network_DIR Qt5WebKit_DIR Qt5Widgets_DIR Qt5Sql_DIR)
  mark_as_advanced(DAVIDSDK_INCLUDE_DIR DAVIDSDK_LIBRARY RSSDK_DIR DSSDK_DIR VTK_DIR)
  mark_as_advanced(ENSENSO_INCLUDE_DIR ENSENSO_LIBRARY)
  mark_as_advanced(QHULL_INCLUDE_DIRS QHULL_LIBRARY QHULL_LIBRARY_DEBUG)
  mark_as_advanced(GLEW_INCLUDE_DIR GLEW_GLEW_LIBRARY GLEW_cocoa_LIBRARY)
  mark_as_advanced(pkgcfg_lib_PC_FLANN_flann pkgcfg_lib_PC_FLANN_flann_cpp)
  mark_as_advanced(pkgcfg_lib_PC_OPENNI2_OpenNI2 pkgcfg_lib_PC_OPENNI2_OpenNI2 pkgcfg_lib_PC_OPENNI_OpenNI)
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
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonFactory.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual)
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberGigE.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual -Wno-non-pod-varargs)
    vp_set_source_file_compile_flag(src/framegrabber/pylon/vpPylonGrabberUsb.cpp -Wno-unknown-pragmas -Wno-unused-parameter -Wno-overloaded-virtual -Wno-non-pod-varargs)
    vp_set_source_file_compile_flag(test/framegrabber/testPylonGrabber.cpp -Wno-unknown-pragmas -Wno-overloaded-virtual -Wno-unused-parameter)
  endif()
endif()
if(USE_REALSENSE)
  # Add specific build flag to turn off warnings coming from RealSense 3rd party
  vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense.cpp -Wno-strict-aliasing -Wno-pessimizing-move -Wno-unused-parameter)
endif()
if(USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from RealSense2 3rd party
    vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense2.cpp -Wno-reorder -Wno-unused-function -Wno-sign-compare -Wno-overloaded-virtual)
  else()
    vp_set_source_file_compile_flag(src/rgb-depth/realsense/vpRealSense2.cpp /wd4244)
  endif()
endif()
if(USE_REALSENSE OR USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_R200.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-comment)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense_SR300.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-comment)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_SR300.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-pessimizing-move -Wno-unused-parameter -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual -Wno-comment)
  else()
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_SR300.cpp /wd4244)
    vp_set_source_file_compile_flag(test/rgb-depth/testRealSense2_D435.cpp /wd4244)
  endif()
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests(CTEST_EXCLUDE_PATH framegrabber force-torque rgb-depth DEPENDS_ON visp_io visp_gui)
