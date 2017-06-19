#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

  if(USE_SOQT AND USE_QT)
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
  endif(USE_SOQT AND USE_QT)

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
    # Fix for Ubuntu 16.04 to add vtkFiltering as dependency. Note that vtkFiltering does't exists on OSX
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
    set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
    foreach(lib_ ${PCL_VTK_LIBRARIES})
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
    endforeach()
    if(VTK_VERSION VERSION_EQUAL 6.2.0)
      mark_as_advanced(Qt5Core_DIR Qt5Gui_DIR Qt5Network_DIR Qt5WebKit_DIR Qt5Widgets_DIR)
    elseif(VTK_VERSION VERSION_EQUAL 7.1.0)
      mark_as_advanced(DAVIDSDK_INCLUDE_DIR DAVIDSDK_LIBRARY DSSDK_DIR ENSENSO_INCLUDE_DIR ENSENSO_LIBRARY)
      mark_as_advanced(GLEW_INCLUDE_DIR GLEW_GLEW_LIBRARY)
      mark_as_advanced(Qt5Core_DIR Qt5Gui_DIR Qt5Sql_DIR Qt5Widgets_DIR RSSDK_DIR)
    endif()
  endif()
endif()

# clipper is private
include_directories(${CLIPPER_INCLUDE_DIRS})

vp_add_module(mbt visp_vision visp_core visp_me visp_visual_features OPTIONAL visp_ar visp_klt visp_gui PRIVATE_OPTIONAL ${CLIPPER_LIBRARIES})
vp_glob_module_sources()

if(USE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCircle.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceLine.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltPoints.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  vp_set_source_file_compile_flag(src/vpMbTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
