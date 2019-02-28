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

if(USE_COIN3D AND NOT HAVE_visp_ar)
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
  if(USE_SOWIN)
    add_definitions("-DSOWIN_DLL")
    list(APPEND opt_incs ${SOWIN_INCLUDE_DIRS})
    list(APPEND opt_libs ${SOWIN_LIBRARIES})
  endif()

  if(USE_SOQT AND ((SoQt_VERSION VERSION_EQUAL "1.6.0") OR (SoQt_VERSION VERSION_GREATER "1.6.0")))
    # Nothing to do; Coin 4 is not linked to SoQt and Qt5
  elseif(USE_SOQT AND USE_QT)
    list(APPEND opt_incs ${SOQT_INCLUDE_DIRS})
    list(APPEND opt_incs ${QT_INCLUDE_DIR})
    list(APPEND opt_incs ${QT_INCLUDES})
    list(APPEND opt_libs ${SOQT_LIBRARIES})
    if(WIN32)
      add_definitions("-DSOQT_DLL")
    endif()

    # We manage QT libraries
    if(DESIRED_QT_VERSION MATCHES 3)
      #Add Qt3 libraries
      #message("QT_QT_LIBRARY ${QT_QT_LIBRARY}")
      list(APPEND opt_libs ${QT_QT_LIBRARY})
    elseif(DESIRED_QT_VERSION MATCHES 4)
      #Add Qt4 libraries
      #message("QT_QTGUI_LIBRARY ${QT_QTGUI_LIBRARY}")
      #message("QT_QTGUI_LIBRARY_RELEASE ${QT_QTGUI_LIBRARY_RELEASE}")
      #message("QT_QTGUI_LIBRARY_DEBUG ${QT_QTGUI_LIBRARY_DEBUG}")
      if(QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE AND QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
        list(APPEND opt_libs optimized ${QT_QTGUI_LIBRARY_RELEASE})
        list(APPEND opt_libs optimized ${QT_QTCORE_LIBRARY_RELEASE})
        list(APPEND opt_libs debug ${QT_QTGUI_LIBRARY_DEBUG})
        list(APPEND opt_libs debug ${QT_QTCORE_LIBRARY_DEBUG})
      elseif(QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
        list(APPEND opt_libs ${QT_QTGUI_LIBRARY_RELEASE})
        list(APPEND opt_libs ${QT_QTCORE_LIBRARY_RELEASE})
      elseif(QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
        list(APPEND opt_libs ${QT_QTGUI_LIBRARY_DEBUG})
        list(APPEND opt_libs ${QT_QTCORE_LIBRARY_DEBUG})
      endif()
    endif()

    # Because in QT_DEFINITIONS defs are separated by ";", parse the
    # QT_DEFINITIONS in order to build a space separated string
    vp_list_remove_separator(QT_DEFINITIONS)
    add_definitions(${QT_DEFINITIONS})
    add_definitions("-DQT_DLL")
  endif()

  if(USE_SOXT)
    # OpenGL and SoXt are found
    list(APPEND opt_libs ${SOXT_LIBRARIES})
  endif()
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

if(WITH_CLIPPER)
  # clipper is private
  include_directories(${CLIPPER_INCLUDE_DIRS})
endif()

vp_add_module(mbt visp_vision visp_core visp_me visp_visual_features OPTIONAL visp_ar visp_klt visp_gui PRIVATE_OPTIONAL ${CLIPPER_LIBRARIES})
vp_glob_module_sources()

if(USE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-declarations)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCircle.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceLine.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-strict-overflow)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltPoints.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/vpMbTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-strict-overflow)
  vp_set_source_file_compile_flag(src/depth/vpMbtFaceDepthDense.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/depth/vpMbtFaceDepthNormal.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-declarations)
  vp_set_source_file_compile_flag(src/depth/vpMbDepthDenseTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/depth/vpMbDepthNormalTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/vpMbtXmlGenericParser.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/vpMbGenericTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
else()
  vp_set_source_file_compile_flag(src/vpMbTracker.cpp -Wno-strict-overflow)
  vp_set_source_file_compile_flag(src/klt/vpMbKltTracker.cpp -Wno-strict-overflow)
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeTracker.cpp -Wno-deprecated-declarations)
  vp_set_source_file_compile_flag(src/depth/vpMbtFaceDepthNormal.cpp -Wno-deprecated-declarations)
endif()
if(MSVC)
  if(BUILD_SHARED_LIBS)
    vp_set_source_file_compile_flag(src/depth/vpMbtTukeyEstimator.cpp /wd4244)
    if(BUILD_DEPRECATED_FUNCTIONS)
      # Disable Visual C++ C4996 warning for MBT multi deprecation
      vp_set_source_file_compile_flag(src/edge/vpMbEdgeMultiTracker.cpp /wd4996)
      vp_set_source_file_compile_flag(src/klt/vpMbKltMultiTracker.cpp /wd4996)
      vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltMultiTracker.cpp /wd4996)
    endif()
  else()
    if(BUILD_DEPRECATED_FUNCTIONS)
      vp_warnings_disable(CMAKE_CXX_FLAGS /wd4244 /wd4996)
    else()
      vp_warnings_disable(CMAKE_CXX_FLAGS /wd4244)
    endif()
  endif()
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

if(BUILD_TESTS)
  vp_set_source_file_compile_flag(test/testGenericTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(test/testGenericTrackerDepth.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
endif()

# Improvement: remove hack to glob the test folder with vp_add_tests
# TODO: re-enable tests after PR #365 (make MBT edges deterministic)
vp_add_tests(DEPENDS_ON visp_core visp_gui visp_io)

# TODO: re-enable tests after PR #365 (make MBT edges deterministic)
#add_test(testGenericTracker-edge                            testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 1) #already added by vp_add_tests
add_test(testGenericTracker-edge-scanline                           testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 1 -l)
add_test(testGenericTracker-KLT                                     testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 2)
add_test(testGenericTracker-KLT-scanline                            testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 2 -l)
add_test(testGenericTracker-edge-KLT                                testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3)
add_test(testGenericTracker-edge-KLT-scanline                       testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3 -l)
add_test(testGenericTracker-edge-depth-dense                        testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 1 -D -e 20)
add_test(testGenericTracker-edge-depth-dense-scanline               testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 1 -D -l -e 20)
add_test(testGenericTracker-KLT-depth-dense                         testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 2 -D -e 20)
add_test(testGenericTracker-KLT-depth-dense-scanline                testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 2 -D -l -e 20)
add_test(testGenericTracker-edge-KLT-depth-dense                    testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3 -D -e 20)
add_test(testGenericTracker-edge-KLT-depth-dense-scanline           testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3 -D -l -e 20)
add_test(testGenericTracker-edge-KLT-depth-dense-scanline-color     testGenericTracker -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3 -D -l -e 20 -C)

#add_test(testGenericTrackerDepth            testGenericTrackerDepth -c ${OPTION_TO_DESACTIVE_DISPLAY}) #already added by vp_add_tests
add_test(testGenericTrackerDepth-scanline           testGenericTrackerDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -l -e 20)
add_test(testGenericTrackerDepth-scanline-color     testGenericTrackerDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -l -e 20 -C)
