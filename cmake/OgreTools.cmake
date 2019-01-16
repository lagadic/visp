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
# ViSP overall configuration file. Some useful tools for Ogre3D.
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(WIN32)
  mark_as_advanced(OGRE_FRAMEWORK_PATH)
endif()

include(FindPkgMacros OPTIONAL)

#########################################################
# Find Ogre plugins
#
# This is a modified version of the macro provided with Ogre
# except that it should be used only in a desperate way when the original
# one doesn't detect anything
#########################################################

macro(vp_ogre_find_plugin_lib_visp PLUGIN)
  # On Unix, the plugins might have no prefix
  if (CMAKE_FIND_LIBRARY_PREFIXES)
    set(TMP_CMAKE_LIB_PREFIX ${CMAKE_FIND_LIBRARY_PREFIXES})
    set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
  endif()

  # strip RenderSystem_ or Plugin_ prefix from plugin name
  string(REPLACE "RenderSystem_" "" PLUGIN_TEMP ${PLUGIN})
  string(REPLACE "Plugin_" "" PLUGIN_NAME ${PLUGIN_TEMP})

  set(OGRE_PLUGIN_PATH_SUFFIXES
    PlugIns PlugIns/${PLUGIN_NAME} Plugins Plugins/${PLUGIN_NAME} ${PLUGIN}
    RenderSystems RenderSystems/${PLUGIN_NAME} ${ARGN})
  # find link libraries for plugins
  set(OGRE_${PLUGIN}_LIBRARY_NAMES "${PLUGIN}${OGRE_LIB_SUFFIX}")
  get_debug_names(OGRE_${PLUGIN}_LIBRARY_NAMES)
  find_library(OGRE_${PLUGIN}_LIBRARY_REL NAMES ${OGRE_${PLUGIN}_LIBRARY_NAMES}
    HINTS ${OGRE_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}/OGRE ${OGRE_LIBRARY_DIRS}/OGRE-${OGRE_VERSION_MAJOR}.${OGRE_VERSION_MINOR}.${OGRE_VERSION_PATCH}
    PATH_SUFFIXES "" OGRE opt release release/opt relwithdebinfo relwithdebinfo/opt minsizerel minsizerel/opt)
  find_library(OGRE_${PLUGIN}_LIBRARY_DBG NAMES ${OGRE_${PLUGIN}_LIBRARY_NAMES_DBG}
    HINTS ${OGRE_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}/OGRE ${OGRE_LIBRARY_DIRS}/OGRE-${OGRE_VERSION_MAJOR}.${OGRE_VERSION_MINOR}.${OGRE_VERSION_PATCH}
    PATH_SUFFIXES "" OGRE opt debug debug/opt)
  make_library_set(OGRE_${PLUGIN}_LIBRARY)

  if (OGRE_${PLUGIN}_LIBRARY)
    set(OGRE_${PLUGIN}_FOUND TRUE)
  endif ()

  mark_as_advanced(OGRE_${PLUGIN}_LIBRARY_REL OGRE_${PLUGIN}_LIBRARY_DBG OGRE_${PLUGIN}_LIBRARY_FWK)
endmacro()

macro(vp_create_ogre_plugin_config_file)
  set(VISP_HAVE_OGRE_PLUGINS_PATH "${VISP_BINARY_DIR}/data/ogre-simulator" CACHE INTERNAL "Ogre plugins location")

  # If OGRE_PLUGIN_DIR_REL and OGRE_PLUGIN_DIR_DBG are not defined we
  # try to find them manually
  if(NOT OGRE_PLUGIN_DIR_REL AND NOT OGRE_PLUGIN_DIR_DBG)
    vp_ogre_find_plugin_lib_visp(RenderSystem_Direct3D9)
    vp_ogre_find_plugin_lib_visp(RenderSystem_Direct3D10)
    vp_ogre_find_plugin_lib_visp(RenderSystem_Direct3D11)
    vp_ogre_find_plugin_lib_visp(RenderSystem_GL)
    vp_ogre_find_plugin_lib_visp(RenderSystem_GLES)
    vp_ogre_find_plugin_lib_visp(Plugin_ParticleFX)
    vp_ogre_find_plugin_lib_visp(Plugin_BSPSceneManager)
    vp_ogre_find_plugin_lib_visp(Plugin_CgProgramManager)
    vp_ogre_find_plugin_lib_visp(Plugin_PCZSceneManager)
    vp_ogre_find_plugin_lib_visp(Plugin_OctreeSceneManager)
    vp_ogre_find_plugin_lib_visp(Plugin_OctreeZone)

    if(OGRE_RenderSystem_GL_LIBRARY_REL)
      get_filename_component(OGRE_PLUGIN_DIR_REL ${OGRE_RenderSystem_GL_LIBRARY_REL} PATH)
      #message("set manually OGRE_PLUGIN_DIR_REL to ${OGRE_PLUGIN_DIR_REL}")
    endif()
    if(OGRE_RenderSystem_GL_LIBRARY_DBG)
      get_filename_component(OGRE_PLUGIN_DIR_DBG ${OGRE_RenderSystem_GL_LIBRARY_DBG} PATH)
      #message("set manually OGRE_PLUGIN_DIR_DBG to ${OGRE_PLUGIN_DIR_DBG}")
    endif()
  endif()

  if(OGRE_PLUGIN_DIR_REL)
    list(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D9_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D10_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D11_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_RenderSystem_GL_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_RenderSystem_GLES_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_Plugin_ParticleFX_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_Plugin_BSPSceneManager_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_Plugin_CgProgramManager_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_Plugin_PCZSceneManager_LIBRARY_REL})
    list(APPEND PLUGIN_REL ${OGRE_Plugin_OctreeSceneManager_LIBRARY_REL})
    if(NOT APPLE)
      # Since the plugin Plugin_Octree causes problems on OSX, we take
      # it only into account on non Apple platforms
      list(APPEND PLUGIN_REL ${OGRE_Plugin_OctreeZone_LIBRARY_REL})
    endif()

    set(PLUGINS_CONTENT_REL "# Defines plugins to load\n\n")
    list(APPEND PLUGINS_CONTENT_REL "# Define plugin folder\n")

    list(APPEND PLUGINS_CONTENT_REL "PluginFolder=${OGRE_PLUGIN_DIR_REL}/\n\n")
    list(APPEND PLUGINS_CONTENT_REL "# Define plugins\n")
    foreach(PLUGIN ${PLUGIN_REL})
      if(PLUGIN)
      get_filename_component(PLUGIN_NAME ${PLUGIN} NAME_WE)
        list(APPEND PLUGINS_CONTENT_REL " Plugin=${PLUGIN_NAME}\n")
      endif()
    endforeach()
    #MESSAGE("PLUGINS_CONTENT_REL: ${PLUGINS_CONTENT_REL}")
    file(WRITE "${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg" ${PLUGINS_CONTENT_REL})
  endif()

  if(OGRE_PLUGIN_DIR_DBG)
    list(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D9_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D10_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D11_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_RenderSystem_GL_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_RenderSystem_GLES_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_Plugin_ParticleFX_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_Plugin_BSPSceneManager_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_Plugin_CgProgramManager_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_Plugin_PCZSceneManager_LIBRARY_DBG})
    list(APPEND PLUGIN_DBG ${OGRE_Plugin_OctreeSceneManager_LIBRARY_DBG})
    if(NOT APPLE)
      # Since the plugin Plugin_Octree causes problems on OSX, we take
      # it only into account on non Apple platforms
      list(APPEND PLUGIN_DBG ${OGRE_Plugin_OctreeZone_LIBRARY_DBG})
    endif()

    set(PLUGINS_CONTENT_DBG "# Defines plugins to load\n\n")
    list(APPEND PLUGINS_CONTENT_DBG "# Define plugin folder\n")
    list(APPEND PLUGINS_CONTENT_DBG "PluginFolder=${OGRE_PLUGIN_DIR_DBG}/\n\n")
    list(APPEND PLUGINS_CONTENT_DBG "# Define plugins\n")
    foreach(PLUGIN ${PLUGIN_DBG})
      if(PLUGIN)
        get_filename_component(PLUGIN_NAME ${PLUGIN} NAME_WE)
        list(APPEND PLUGINS_CONTENT_DBG " Plugin=${PLUGIN_NAME}\n")
      endif()
    endforeach()

    #MESSAGE("PLUGINS_CONTENT_DBG: ${PLUGINS_CONTENT_DBG}")
    file(WRITE "${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg" ${PLUGINS_CONTENT_DBG})
  endif()
endmacro()

function(vp_set_ogre_media)
  # if OGRE_MEDIA_DIR is empty, try to find the path by searching for plugins.cfg
  # Introduced since Ubuntu 12.04
  if(NOT OGRE_MEDIA_DIR)
    find_path(OGRE_MEDIA_DIR ../plugins.cfg
      $ENV{OGRE_MEDIA_DIR}
      /usr/share/OGRE-1.7.4/media
      /usr/share/OGRE-1.8.0/media
      /usr/share/OGRE-1.8.1/media
      /usr/share/OGRE-1.9.0/media
    )
  endif()

  # If Ogre media are not available we provide the minimal material to run the examples:
  # - resources.cfg
  # - plugins.cfg
  # - media/materials/...
  # - media/models/...
  #
  # We need to introduce OGRE_MEDIA_NOT_AVAILABLE to memorize when OGRE_MEDIA_DIR is not set.
  # Because in that case, OGRE_MEDIA_DIR should be set first to VISP_HAVE_OGRE_RESOURCES_PATH
  #  (for the "make all" case) then to VISP_INSTALL_DIR_OGRE_RESOURCES (for the "make install" case)
  if(NOT OGRE_MEDIA_DIR)
    set(OGRE_MEDIA_NOT_AVAILABLE "TRUE")
  endif()

  # Try to search for an existing plugins.cfg file
  # Here we cannot use OGRE_PLUGIN_DIR_REL or OGRE_PLUGIN_DIR_DBG where
  # we may find an existing plugins.cfg file, since under Windows in these
  # files the PluginFolder is set to a relative path. We need an absolute
  # path to avoid recopy of the plugins in ViSP.
  # Under Linux or OSX, we may find plugins.cfg with a PluginFolder set
  # to an absolute path in OGRE_MEDIA_DIR/..
  find_path(ogre_plugings_cfg_exists
    NAMES plugins.cfg
    PATHS ${OGRE_MEDIA_DIR}/..
    NO_SYSTEM_ENVIRONMENT_PATH
  )
  mark_as_advanced(ogre_plugings_cfg_exists)

  #message("OGRE_PLUGIN_DIR_REL: ${OGRE_PLUGIN_DIR_REL}")
  #message("OGRE_PLUGIN_DIR_DBG: ${OGRE_PLUGIN_DIR_DBG}")
  if(ogre_plugings_cfg_exists)
    set(VISP_HAVE_OGRE_PLUGINS_PATH "${ogre_plugings_cfg_exists}" CACHE INTERNAL "Ogre plugins location")
  else(NOT ogre_plugings_cfg_exists)
    # If no plugins.cfg file is found, we create one with absolute path

    # case 1: normal case
    #--------------
    vp_create_ogre_plugin_config_file()

    # case 2: install or packaging case
    #--------------
    # install rule for plugins.cfg:
    if(UNIX)
      if(OGRE_PLUGIN_DIR_REL)
        install(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg
          DESTINATION ${VISP_LIB_INSTALL_PATH}/visp/data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()
      if(OGRE_PLUGIN_DIR_DBG)
        install(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg
          DESTINATION ${VISP_LIB_INSTALL_PATH}/visp/data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()

    else()
      if(OGRE_PLUGIN_DIR_REL)
        install(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg
          DESTINATION data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()
      if(OGRE_PLUGIN_DIR_DBG)
        install(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg
          DESTINATION data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()
    endif()
  endif()

  # Try to search for an existing resources.cfg file
  find_path(ogre_resources_cfg_exists
    NAMES resources.cfg
    PATHS ${OGRE_MEDIA_DIR}/..
    NO_SYSTEM_ENVIRONMENT_PATH
  )
  mark_as_advanced(ogre_resources_cfg_exists)
  # Here we copy all the minimal media files
  # - media/materials/...
  # - media/models/...
  if(OGRE_MEDIA_NOT_AVAILABLE)
    file(COPY modules/ar/data/ogre-simulator/media DESTINATION ${VISP_BINARY_DIR}/data/ogre-simulator)
  endif()

  if(ogre_resources_cfg_exists)
    set(VISP_HAVE_OGRE_RESOURCES_PATH "${ogre_resources_cfg_exists}" CACHE INTERNAL "Ogre resources location")
  else()
    # Here we create a resources.cfg if it was not found

    # we create a resources.cfg file for vpAROgre.cpp
    # case 1: normal case
    #         If OGRE_MEDIA_DIR is not found, we set it to VISP_HAVE_OGRE_RESOURCES_PATH in order to use
    #         the minimal requested media to run the examples
    #--------------
    set(VISP_HAVE_OGRE_RESOURCES_PATH "${VISP_BINARY_DIR}/data/ogre-simulator" CACHE INTERNAL "Ogre resources location")

    if(OGRE_MEDIA_NOT_AVAILABLE)
      set(OGRE_MEDIA_DIR ${VISP_HAVE_OGRE_RESOURCES_PATH}/media)
    endif()

    # Here we add all the subdirs in @OGRE_MEDIA_DIR@/* as resource location.
    vp_get_relative_subdirs(media_subdirs ${OGRE_MEDIA_DIR})
    set(OGRE_RESOURCES_FileSystem "FileSystem=${OGRE_MEDIA_DIR}\n")
    foreach(m ${media_subdirs})
      set(OGRE_RESOURCES_FileSystem "${OGRE_RESOURCES_FileSystem}FileSystem=${OGRE_MEDIA_DIR}/${m}\n")
    endforeach()

    configure_file(
      ${VISP_SOURCE_DIR}/cmake/templates/resources.cfg.in
      ${VISP_HAVE_OGRE_RESOURCES_PATH}/resources.cfg
      IMMEDIATE @ONLY
    )

    # case 2: install or packaging case
    #         If OGRE_MEDIA_DIR is not found, we set it to VISP_INSTALL_DIR_OGRE_RESOURCES in order to use
    #         the minimal requested media to run the examples
    #--------------
    set(VISP_INSTALL_DIR_OGRE_RESOURCES "${CMAKE_INSTALL_PREFIX}/${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator")

    # make the var global
    set(VISP_INSTALL_DIR_OGRE_RESOURCES ${VISP_INSTALL_DIR_OGRE_RESOURCES} CACHE INTERNAL "Ogre media install dir")

    if(OGRE_MEDIA_NOT_AVAILABLE)
      set(OGRE_MEDIA_DIR ${VISP_INSTALL_DIR_OGRE_RESOURCES}/media)
    endif()

    # Here we add all the subdirs in @OGRE_MEDIA_DIR@/* as resource location.
    set(OGRE_RESOURCES_FileSystem "FileSystem=${OGRE_MEDIA_DIR}\n")
    foreach(m ${media_subdirs})
      set(OGRE_RESOURCES_FileSystem "${OGRE_RESOURCES_FileSystem}FileSystem=${OGRE_MEDIA_DIR}/${m}\n")
    endforeach()

    # install rule for resources.cfg and Ogre media if they are not available:
    if(UNIX)
      configure_file(
        ${VISP_SOURCE_DIR}/cmake/templates/resources.cfg.in
        ${VISP_BINARY_DIR}/unix-install/resources.cfg
        IMMEDIATE @ONLY
      )
      install(FILES
        ${VISP_BINARY_DIR}/unix-install/resources.cfg
        DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT dev
      )
      if(OGRE_MEDIA_NOT_AVAILABLE)
        install(DIRECTORY
          ${VISP_BINARY_DIR}/data/ogre-simulator/media
          DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
          FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()
    else()
      configure_file(
        ${VISP_SOURCE_DIR}/cmake/templates/resources.cfg.in
        ${VISP_BINARY_DIR}/win-install/resources.cfg
        IMMEDIATE @ONLY
      )
      install(FILES
        ${VISP_BINARY_DIR}/win-install/resources.cfg
        DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT dev
      )
      if(OGRE_MEDIA_NOT_AVAILABLE)
        install(DIRECTORY
          ${VISP_BINARY_DIR}/data/ogre-simulator/media
          DESTINATION ${VISP_INSTALL_DATAROOTDIR}/data/ogre-simulator
          FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT dev
        )
      endif()
    endif()
  endif()
endfunction()

macro(vp_set_ogre_advanced_var)
  set(ogre_components_ Paging Terrain Plugin_BSPSceneManager Plugin_CgProgramManager Plugin_OctreeSceneManager Plugin_OctreeZone Plugin_PCZSceneManager Plugin_ParticleFX RenderSystem_Direct3D11 RenderSystem_Direct3D9 RenderSystem_GLES2 RenderSystem_GLES RenderSystem_GL)
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
