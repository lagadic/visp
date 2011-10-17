#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
# This file creates the ViSP library.
#
# Authors:
# Fabien Spindler
#
#############################################################################

#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the ViSP project .cpp files
INCLUDE(${VISP_SOURCE_DIR}/CMakeSourceFileList.cmake)
# include all the ViSP project .h files
INCLUDE(${VISP_SOURCE_DIR}/CMakeHeaderFileList.cmake)


#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/visp
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
FOREACH(header ${HEADER_ALL})
  GET_FILENAME_COMPONENT(headerName ${header} NAME) 
  ADD_CUSTOM_COMMAND(
    OUTPUT ${VISP_BINARY_DIR}/include/visp/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${VISP_BINARY_DIR}/include/visp/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
    )
  SET(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${VISP_BINARY_DIR}/include/visp/${headerName}
    )
ENDFOREACH(header)

#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/visp
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(header ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${VISP_INTERN_LIBRARY} 
  ${SRC_ALL} 
  ${HEADER_ALL} ${VISP_INCLUDE_DIR}/vpConfig.h) 

# create the headers in include/visp before compiling the lib
ADD_DEPENDENCIES(${VISP_INTERN_LIBRARY} header)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${VISP_INTERN_LIBRARY} PROPERTIES
  VERSION ${VISP_VERSION}
  SOVERSION ${VISP_VERSION_MAJOR}
  )

# Link with external libs especially under windows for winmm.lib
# needed to create visp-2.dll. Why ? Because for binaries, winmm.lib
# is considered in the link stage. Probably a CMake bug ! 
#IF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
#  TARGET_LINK_LIBRARIES(${VISP_INTERN_LIBRARY} "winmm")
#ENDIF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
TARGET_LINK_LIBRARIES(${VISP_INTERN_LIBRARY} ${VISP_EXTERN_LIBRARIES})

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL(TARGETS ${VISP_INTERN_LIBRARY}
  # Destination is either lib or lib64 on Unix 64 bits architectures
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  COMPONENT libraries
  )
# install rule for all the headers
INSTALL(FILES ${HEADER_ALL}
  DESTINATION include/visp
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  COMPONENT headers
  )
