#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
set(opt_system_incs "")
set(opt_libs "")
set(opt_libs_private "")

if(USE_COIN3D AND NOT HAVE_visp_ar)
  if(WIN32)
    add_definitions("-DCOIN_DLL")
  endif()
  list(APPEND opt_system_incs ${COIN3D_INCLUDE_DIRS})
  # On OSX cmake 2.8 found OpenGL but OPENGL_INCLUDE_DIR was set to NOT_FOUND
  # We add a test to be sure that the OPENGL vars exist.
  if(OPENGL_INCLUDE_DIR)
    list(APPEND opt_system_incs ${OPENGL_INCLUDE_DIR})
  endif()
  if(OPENGL_LIBRARIES)
    list(APPEND opt_libs ${OPENGL_LIBRARIES})
  endif()

  list(APPEND opt_libs ${COIN3D_LIBRARIES})
  if(USE_SOWIN)
    add_definitions("-DSOWIN_DLL")
    list(APPEND opt_system_incs ${SOWIN_INCLUDE_DIRS})
    list(APPEND opt_libs ${SOWIN_LIBRARIES})
  endif()

  if(USE_SOQT AND ((SoQt_VERSION VERSION_EQUAL "1.6.0") OR (SoQt_VERSION VERSION_GREATER "1.6.0")))
    # Nothing to do; Coin 4 is not linked to SoQt and Qt5
  elseif(USE_SOQT AND USE_QT)
    list(APPEND opt_system_incs ${SOQT_INCLUDE_DIRS})
    list(APPEND opt_system_incs ${QT_INCLUDE_DIR})
    list(APPEND opt_system_incs ${QT_INCLUDES})
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
  list(APPEND opt_system_incs ${PCL_INCLUDE_DIRS})
  # To ensure to build with VTK and other PCL 3rd parties we are not using PCL_LIBRARIES but PCL_DEPS_INCLUDE_DIRS
  # and PCL_DEPS_LIBRARIES instead
  list(APPEND opt_system_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

if(USE_NLOHMANN_JSON AND NOT VISP_HAVE_NLOHMANN_JSON_FROM_VTK)
  get_target_property(_inc_dirs "nlohmann_json::nlohmann_json" INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND opt_system_incs ${_inc_dirs})
endif()

if(WITH_CLIPPER)
  # clipper is private
  include_directories(SYSTEM ${CLIPPER_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${CLIPPER_LIBRARIES})
endif()

# pugixml is always enabled to provide default XML I/O capabilities
# pugixml is private
include_directories(SYSTEM ${PUGIXML_INCLUDE_DIRS})
list(APPEND opt_libs_private ${PUGIXML_LIBRARIES})

if(WITH_SIMDLIB)
  # Simd lib is private
  include_directories(SYSTEM ${SIMDLIB_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${SIMDLIB_LIBRARIES})
endif()

vp_add_module(mbt visp_vision visp_core visp_me visp_visual_features OPTIONAL visp_ar visp_klt visp_gui PRIVATE_OPTIONAL ${opt_libs_private} WRAP java)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs} SYSTEM ${opt_system_incs})
vp_create_module(${opt_libs})

set(opt_test_incs "")
set(opt_test_libs "")

if(WITH_CATCH2)
  # catch2 is private
  list(APPEND opt_test_incs ${CATCH2_INCLUDE_DIRS})
  list(APPEND opt_test_libs ${CATCH2_LIBRARIES})
endif()

vp_add_tests(DEPENDS_ON visp_core visp_gui visp_io PRIVATE_INCLUDE_DIRS ${opt_test_incs} PRIVATE_LIBRARIES ${opt_test_libs})

if(VISP_DATASET_FOUND)
  #add_test(testGenericTracker-edge                            testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 1) #already added by vp_add_tests
  add_test(testGenericTracker-edge-scanline                           testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 1 -l)
  add_test(testGenericTracker-KLT                                     testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 2)
  add_test(testGenericTracker-KLT-scanline                            testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 2 -l)
  add_test(testGenericTracker-edge-KLT                                testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 3)
  add_test(testGenericTracker-edge-KLT-scanline                       testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 3 -l)
  add_test(testGenericTracker-edge-depth-dense                        testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 1 -D -e 20)
  add_test(testGenericTracker-edge-depth-dense-scanline               testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 1 -D -l -e 20)
  add_test(testGenericTracker-KLT-depth-dense                         testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 2 -D -e 20)
  add_test(testGenericTracker-KLT-depth-dense-scanline                testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 2 -D -l -e 20)
  add_test(testGenericTracker-edge-KLT-depth-dense                    testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 3 -D -e 20)
  add_test(testGenericTracker-edge-KLT-depth-dense-scanline           testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 3 -D -l -e 20)
  add_test(testGenericTracker-edge-KLT-depth-dense-scanline-color     testGenericTracker -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -t 3 -D -l -e 20 -C)

  #add_test(testGenericTrackerDepth            testGenericTrackerDepth -c ${SHORT_OPTION_TO_DISABLE_DISPLAY}) #already added by vp_add_tests
  add_test(testGenericTrackerDepth-scanline           testGenericTrackerDepth -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -l -e 20)
  add_test(testGenericTrackerDepth-scanline-color     testGenericTrackerDepth -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} -l -e 20 -C)
endif()
