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
# ViSP overall configuration file. Some useful tools for Ogre3D.
#
#############################################################################

if(NOT OGRE_FOUND)
  return()
endif()

# add the path to detect Ogre3D FindPkgMacros.cmake
if(WIN32)
  if(EXISTS "${OGRE_DIR}/cmake")
    list(APPEND CMAKE_MODULE_PATH "${OGRE_DIR}/cmake")
  endif()
  if(EXISTS "${OGRE_DIR}")
    list(APPEND CMAKE_MODULE_PATH "${OGRE_DIR}")
  endif()
endif(WIN32)

if(UNIX)
  if(EXISTS "${OGRE_DIR}/cmake")
    list(APPEND CMAKE_MODULE_PATH "${OGRE_DIR}/cmake")
  endif()
  if(EXISTS "${OGRE_DIR}/CMake")
    list(APPEND CMAKE_MODULE_PATH "${OGRE_DIR}/CMake")
  endif()
  if(EXISTS "${OGRE_DIR}")
    list(APPEND CMAKE_MODULE_PATH "${OGRE_DIR}")
  endif()
  if(EXISTS "/usr/local/lib/OGRE/cmake")
    list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake")
  endif()
  if(EXISTS "/usr/lib/OGRE/cmake")
    list(APPEND CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake")
  endif()
  if(EXISTS "/usr/local/lib64/OGRE/cmake")
    list(APPEND CMAKE_MODULE_PATH "/usr/local/lib64/OGRE/cmake")
  endif()
  if(EXISTS "/usr/lib64/OGRE/cmake")
    list(APPEND CMAKE_MODULE_PATH "/usr/lib64/OGRE/cmake")
  endif()
  if(EXISTS "/usr/share/OGRE/cmake/modules")
    list(APPEND CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules")
  endif()
endif(UNIX)

# If OGRE_CONFIG_DIR/plugins.cfg exists create a copy in data/ogre-simulator/plugins.cfg
# with PluginFolder=@VISP_OGRE_PLUGIN_DIR@
# - on unix, VISP_OGRE_PLUGIN_DIR=OGRE_PLUGIN_DIR
# - on windws, VISP_OGRE_PLUGIN_DIR=OGRE_PLUGIN_DIR with "/" replaced with "\"
macro(vp_set_ogre_plugin)
  if(EXISTS "${OGRE_CONFIG_DIR}/plugins.cfg")
    # Prepare var used in plugind.cfg
    set(VISP_OGRE_PLUGIN_DIR ${OGRE_PLUGIN_DIR})

    # Get all the lines that contains "Plugin="
    file(STRINGS "${OGRE_CONFIG_DIR}/plugins.cfg" OGRE_PLUGIN_LINES REGEX "^[ \t]*#?[ \t]*Plugin=")

    if (WIN32)
      # Replace / with \
      string(REPLACE "/" "\\" VISP_OGRE_PLUGIN_DIR "${VISP_OGRE_PLUGIN_DIR}")

      # Remove Rendering Systems under Windows that are known to be not installed with Ogre 14.4.1 SDK
      set(RENDERING_SYSTEMS_TO_REMOVE "RenderSystem_Direct3D9" "Plugin_CgProgramManager")

      # Process OGRE_PLUGIN_LINES to add # if needed
      set(MODIFIED_PLUGIN_LINES "")
      foreach(line ${OGRE_PLUGIN_LINES})
        set(should_comment FALSE)

        # Check if this line contains any plugin to remove
        foreach(plugin_to_remove ${RENDERING_SYSTEMS_TO_REMOVE})
          if(line MATCHES "${plugin_to_remove}")
            set(should_comment TRUE)
            break()
          endif()
        endforeach()

        # Add # if needed and line is not already commented
        if(should_comment AND NOT line MATCHES "^[ \t]*#")
          string(REGEX REPLACE "^([ \t]*)" "\\1# " modified_line "${line}")
          list(APPEND MODIFIED_PLUGIN_LINES "${modified_line}")
        else()
          list(APPEND MODIFIED_PLUGIN_LINES "${line}")
        endif()
      endforeach()

      # Replace original variable with modified one
      set(OGRE_PLUGIN_LINES "${MODIFIED_PLUGIN_LINES}")
    endif()

    # Convert list to newline-separated string
    string(REPLACE ";" "\n" OGRE_PLUGIN_LINES "${OGRE_PLUGIN_LINES}")

    set(OGRE_DATA_ROOT_DIR "${VISP_BINARY_DIR}/data/ogre-simulator")

    configure_file(
      "cmake/templates/plugins.cfg.in"
      "${OGRE_DATA_ROOT_DIR}/plugins.cfg"
      @ONLY
    )

    install(FILES
      "${OGRE_DATA_ROOT_DIR}/plugins.cfg"
      DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT dev
    )

    # For vpConfig.h
    set(VISP_HAVE_OGRE_PLUGINS_PATH "${VISP_BINARY_DIR}/data/ogre-simulator" CACHE INTERNAL "Ogre plugins.cfg location")
  endif()
endmacro()

# If OGRE_CONFIG_DIR/resources.cfg exists create a copy in data/ogre-simulator/resources.cfg
# with an updated path to media dir
macro(vp_set_ogre_resources)
  if(EXISTS "${OGRE_CONFIG_DIR}/resources.cfg")
    file(STRINGS "${OGRE_CONFIG_DIR}/resources.cfg" RESOURCE_LINES)

    set(OGRE_RESOURCE_LINES "")

    foreach(line ${RESOURCE_LINES})
      string(REPLACE "=../Media" "=${OGRE_MEDIA_DIR}" line "${line}")
      string(REPLACE "=../Tests/Media" "=${OGRE_MEDIA_DIR}" line "${line}")
      string(REPLACE "=./Media" "=${OGRE_MEDIA_DIR}" line "${line}")
      string(REPLACE "=./Tests/Media" "=${OGRE_MEDIA_DIR}" line "${line}")
      list(APPEND OGRE_RESOURCE_LINES ${line})
    endforeach()

    string(REPLACE ";" "\n" OGRE_RESOURCE_LINES "${OGRE_RESOURCE_LINES}")

    set(OGRE_DATA_ROOT_DIR "${VISP_BINARY_DIR}/data/ogre-simulator")

    configure_file(
      "cmake/templates/resources.cfg.in"
      "${OGRE_DATA_ROOT_DIR}/resources.cfg"
      @ONLY
    )

    install(FILES
      "${OGRE_DATA_ROOT_DIR}/resources.cfg"
      DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT dev
    )

    # For vpConfig.h
    set(VISP_HAVE_OGRE_RESOURCES_PATH "${VISP_BINARY_DIR}/data/ogre-simulator" CACHE INTERNAL "Ogre resources.cfg location")
  endif()
endmacro()

macro(vp_set_ogre_advanced_var)
  set(ogre_components_ Paging Terrain Plugin_BSPSceneManager Plugin_CgProgramManager Plugin_OctreeSceneManager)
  list(APPEND ogre_components_ Plugin_OctreeZone Plugin_PCZSceneManager Plugin_ParticleFX RenderSystem_Direct3D11)
  list(APPEND ogre_components_ RenderSystem_Direct3D9 RenderSystem_GLES2 RenderSystem_GLES RenderSystem_GL RTShaderSystem)
  foreach(component_ ${ogre_components_})
    mark_as_advanced(OGRE_${component_}_INCLUDE_DIR)
    mark_as_advanced(OGRE_${component_}_LIBRARY_DBG)
    mark_as_advanced(OGRE_${component_}_LIBRARY_REL)
    mark_as_advanced(OGRE_${component_}_LIBRARY_FWK)
  endforeach()
  mark_as_advanced(pkgcfg_lib_OGRE_PKGC_OgreMain)
  mark_as_advanced(pkgcfg_lib_OGRE_PKGC_pthread)
  mark_as_advanced(pkgcfg_lib_OGRE_PKGC_boost_system)
  mark_as_advanced(pkgcfg_lib_OIS_PKGC_OIS)
endmacro()
