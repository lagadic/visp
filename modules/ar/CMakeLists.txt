#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
set(opt_libs "")

if(USE_OGRE)
  # With Ubuntu 20.04, OGRE_LIBRARIES contains Boost::thread that cannot
  # be exported as is on pain of producing a build issue when ViSP is used
  # as a 3rd party. That's why here we get its imported location.
  vp_filter_libraries_with_imported_location(OGRE_LIBRARIES)
  mark_as_advanced(OGRE_SAMPLES_INCLUDEPATH)
  #message("OGRE_SAMPLES_INCLUDEPATH: ${OGRE_SAMPLES_INCLUDEPATH}")
  if(OGRE_SAMPLES_INCLUDEPATH)
    list(APPEND opt_incs ${OGRE_SAMPLES_INCLUDEPATH})
  endif()

  # hack to fix possible presence of NOTFOUND in OGRE_INCLUDE_DIRS
  #message("OGRE_INCLUDE_DIRS: ${OGRE_INCLUDE_DIRS}")
  foreach(inc_ ${OGRE_INCLUDE_DIRS})
    if(NOT ${inc_} MATCHES "NOTFOUND")
      list(APPEND opt_incs ${inc_})
    endif()
  endforeach()
  if(WIN32)
    foreach(lib_ ${OGRE_LIBRARIES})
      if(${lib_} MATCHES "^Ogre")
        list(APPEND opt_libs "${OGRE_LIBRARY_DIRS}/${lib_}.lib")
      else()
        list(APPEND opt_libs ${lib_})
      endif()
    endforeach()
  else()
    list(APPEND opt_libs ${OGRE_LIBRARIES})
  endif()
endif(USE_OGRE)

if(USE_OIS AND USE_OGRE)
  list(APPEND opt_incs ${OIS_INCLUDE_DIR})
  list(APPEND opt_libs ${OIS_LIBRARIES})
  if(APPLE)
    # With Ogre 1.7.4 and 1.8.1 to be able to link with libOIS.a, Cocoa framework is requested.
    # This is a work around since it should come with FindOGRE.cmake
    list(APPEND opt_libs "-framework Cocoa")
  endif()
endif()

if(USE_COIN3D)
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
    set(SoQt_incs ${SoQt_INCLUDE_DIRS})
    # Handle SoQt_LIBRARIES to extract the imported location
    set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
    foreach(lib_ ${SoQt_LIBRARIES})
        if(${lib_} MATCHES "^(Coin::Coin)" OR ${lib_} MATCHES "^(SoQt::SoQt)" OR ${lib_} MATCHES "^(Qt5::)")
          foreach(imp_config_ ${config_})
            get_target_property(lib_property_${imp_config_}_ ${lib_} IMPORTED_LOCATION_${imp_config_})
            get_target_property(lib_property_inc ${lib_} INTERFACE_INCLUDE_DIRECTORIES)
            list(APPEND opt_incs ${lib_property_inc})
            # Under Unix, there is no specific suffix for libraries.
            # Under Windows, we add the "optimized", "debug" specific keywords
            if(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
              list(APPEND SoQt_libs optimized "${lib_property_${imp_config_}_}")
            elseif(WIN32 AND EXISTS "${lib_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
              list(APPEND SoQt_libs debug     "${lib_property_${imp_config_}_}")
            elseif(EXISTS "${lib_property_${imp_config_}_}")
              list(APPEND SoQt_libs "${lib_property_${imp_config_}_}")
            endif()
          endforeach()
        else() # not a vtk library
          list(APPEND SoQt_libs "${lib_}")
        endif()
    endforeach()
    vp_list_unique(SoQt_incs)
    vp_list_unique(SoQt_libs)
    list(APPEND opt_incs ${SoQt_incs})
    list(APPEND opt_libs ${SoQt_libs})
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

if(USE_PANDA3D)
  if(Panda3D_INCLUDE_DIRS)
    list(APPEND opt_incs ${Panda3D_INCLUDE_DIRS})
  endif()
  if(Panda3D_LIBRARIES)
  list(APPEND opt_libs ${Panda3D_LIBRARIES})
  endif()
endif()

vp_add_module(ar visp_core)
vp_glob_module_sources()

if(USE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  vp_set_source_file_compile_flag(src/ogre-simulator/vpAROgre.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-float-equal -Wno-deprecated-copy -Wno-register)
endif()

if(USE_PANDA3D)
  set(PANDA3D_CXX_FLAGS -Wno-unused-parameter -Wno-unused-variable -Wno-extra -Wno-reorder-ctor)
  set(PANDA3D_MODULE_SOURCES
    vpPanda3DBaseRenderer.cpp vpPanda3DGeometryRenderer.cpp
    vpPanda3DRGBRenderer.cpp vpPanda3DRenderParameters.cpp
    vpPanda3DRendererSet.cpp vpPanda3DPostProcessFilter.cpp vpPanda3DCommonFilters.cpp
  )
  foreach(panda_src_name ${PANDA3D_MODULE_SOURCES})
    vp_set_source_file_compile_flag(src/panda3d-simulator/${panda_src_name} ${PANDA3D_CXX_FLAGS})
  endforeach()
endif()

vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
