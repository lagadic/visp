#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
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

MACRO(CREATE_OGRE_PLUGIN_CONFIG_FILE)
    	SET(VISP_HAVE_OGRE_PLUGINS_PATH ${VISP_BINARY_DIR}/data/ogre-simulator)

	# If OGRE_PLUGIN_DIR_REL and OGRE_PLUGIN_DIR_DBG are not defined we 
        # try to find these path manually
	IF(NOT OGRE_PLUGIN_DIR_REL AND NOT OGRE_PLUGIN_DIR_DBG)
	  IF(OGRE_RenderSystem_GL_LIBRARY_REL)
	    GET_FILENAME_COMPONENT(OGRE_PLUGIN_DIR_REL ${OGRE_RenderSystem_GL_LIBRARY_REL} PATH)
	    #message("set manually OGRE_PLUGIN_DIR_REL to ${OGRE_PLUGIN_DIR_REL}")
          ELSEIF(OGRE_RenderSystem_GL_LIBRARY_DBG)
	    GET_FILENAME_COMPONENT(OGRE_PLUGIN_DIR_DBG ${OGRE_RenderSystem_GL_LIBRARY_DBG} PATH)
	    #message("set manually OGRE_PLUGIN_DIR_DBG to ${OGRE_PLUGIN_DIR_DBG}")
 	  ENDIF()
	ENDIF()

        IF(OGRE_PLUGIN_DIR_REL)
	  LIST(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D9_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D10_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_RenderSystem_Direct3D11_LIBRARY_REL})
          LIST(APPEND PLUGIN_REL ${OGRE_RenderSystem_GL_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_RenderSystem_GLES_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_Plugin_ParticleFX_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_Plugin_BSPSceneManager_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_Plugin_CgProgramManager_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_Plugin_PCZSceneManager_LIBRARY_REL})
	  LIST(APPEND PLUGIN_REL ${OGRE_Plugin_OctreeSceneManager_LIBRARY_REL})
	  IF (NOT APPLE)
	    # Since the plugin Plugin_Octree causes problems on OSX, we take 
            # it only into account on non Apple platforms
	    LIST(APPEND PLUGIN_REL ${OGRE_Plugin_OctreeZone_LIBRARY_REL})
	  ENDIF()

 	  SET(PLUGINS_CONTENT_REL "# Defines plugins to load\n\n")
	  LIST(APPEND PLUGINS_CONTENT_REL "# Define plugin folder\n")

	  LIST(APPEND PLUGINS_CONTENT_REL "PluginFolder=${OGRE_PLUGIN_DIR_REL}/\n\n")
	  LIST(APPEND PLUGINS_CONTENT_REL "# Define plugins\n")
	  foreach(PLUGIN ${PLUGIN_REL})
	    if(PLUGIN)
	      GET_FILENAME_COMPONENT(PLUGIN_NAME ${PLUGIN} NAME_WE)
  	      LIST(APPEND PLUGINS_CONTENT_REL " Plugin=${PLUGIN_NAME}\n")
	    endif()
	  endforeach()
	  #MESSAGE("PLUGINS_CONTENT_REL: ${PLUGINS_CONTENT_REL}")
          FILE(WRITE "${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg" ${PLUGINS_CONTENT_REL})
        ENDIF()
	  
        IF(OGRE_PLUGIN_DIR_DBG)
	  LIST(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D9_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D10_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_RenderSystem_Direct3D11_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_RenderSystem_GL_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_RenderSystem_GLES_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_ParticleFX_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_BSPSceneManager_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_CgProgramManager_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_PCZSceneManager_LIBRARY_DBG})
	  LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_OctreeSceneManager_LIBRARY_DBG})
	  IF (NOT APPLE)
	    # Since the plugin Plugin_Octree causes problems on OSX, we take 
            # it only into account on non Apple platforms
	    LIST(APPEND PLUGIN_DBG ${OGRE_Plugin_OctreeZone_LIBRARY_DBG})
	  ENDIF()

 	  SET(PLUGINS_CONTENT_DBG "# Defines plugins to load\n\n")
	  LIST(APPEND PLUGINS_CONTENT_DBG "# Define plugin folder\n")
	  LIST(APPEND PLUGINS_CONTENT_DBG "PluginFolder=${OGRE_PLUGIN_DIR_DBG}/\n\n")
	  LIST(APPEND PLUGINS_CONTENT_DBG "# Define plugins\n")
	  foreach(PLUGIN ${PLUGIN_DBG})
	    if(PLUGIN)
	      GET_FILENAME_COMPONENT(PLUGIN_NAME ${PLUGIN} NAME_WE)
  	      LIST(APPEND PLUGINS_CONTENT_DBG " Plugin=${PLUGIN_NAME}\n")
	    endif()
	  endforeach()
	  
	  #MESSAGE("PLUGINS_CONTENT_DBG: ${PLUGINS_CONTENT_DBG}")
          FILE(WRITE "${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg" ${PLUGINS_CONTENT_DBG})
        ENDIF()

ENDMACRO()
