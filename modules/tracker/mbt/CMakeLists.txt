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

# If
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

vp_add_module(mbt visp_vision visp_core visp_me visp_visual_features OPTIONAL visp_ar visp_klt visp_gui)
vp_glob_module_sources()

if(USE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/edge/vpMbEdgeTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCircle.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/edge/vpMbtDistanceLine.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/hybrid/vpMbEdgeKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/klt/vpMbKltMultiTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/klt/vpMbKltTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltCylinder.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/klt/vpMbtDistanceKltPoints.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  vp_set_source_file_compile_flag(src/vpMbTracker.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
