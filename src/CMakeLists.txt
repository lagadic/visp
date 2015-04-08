#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
include(${VISP_CMAKE_MODULE_PATH}/CMakeSourceFileList.cmake)
# include all the ViSP project .h files
include(${VISP_CMAKE_MODULE_PATH}/CMakeHeaderFileList.cmake)

#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/visp
#----------------------------------------------------------------------
# For each header, we create a rule
set(HEADER_IN_INCLUDE_DIR "")
foreach(header_ ${HEADER_ALL})
  get_filename_component(headerName ${header_} NAME)
  add_custom_command(
    OUTPUT ${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/visp/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header_} ${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/visp/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
    )
  set(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/visp/${headerName}
    )
endforeach()

#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/visp
#----------------------------------------------------------------------
add_custom_target(header ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )

#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------

if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

add_library(${VISP_INTERN_LIBRARY}
  ${SRC_ALL} 
  ${HEADER_ALL} ${VISP_INCLUDE_DIR}/vpConfig.h) 

# create the headers in include/visp before compiling the lib
add_dependencies(${VISP_INTERN_LIBRARY} header)

# extra target
add_dependencies(visp_library ${VISP_INTERN_LIBRARY})

# Append the library version information to the library target
# properties. 
set_target_properties(${VISP_INTERN_LIBRARY} PROPERTIES
  VERSION ${VISP_VERSION}
  SOVERSION ${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}
  PUBLIC_HEADER "${HEADER_ALL};${VISP_INCLUDE_DIR}/vpConfig.h"
  OUTPUT_NAME "${VISP_INTERN_LIBRARY}${VISP_DLLVERSION}"
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  RUNTIME_OUTPUT_DIRECTORY "${BINARY_OUTPUT_PATH}"
  LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  )
set_property(TARGET ${VISP_INTERN_LIBRARY} APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${VISP_EXTERN_INCLUDE_DIRS}
)
if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${VISP_INTERN_LIBRARY} PROPERTIES FOLDER "library")
  set_target_properties(header PROPERTIES FOLDER "library")
endif()



# Link with external libs especially under windows for winmm.lib
# needed to create visp.dll. Why ? Because for binaries, winmm.lib
# is considered in the link stage. Probably a CMake bug ! 
#IF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
#  TARGET_LINK_LIBRARIES(${VISP_INTERN_LIBRARY} "winmm")
#ENDIF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
target_link_libraries(${VISP_INTERN_LIBRARY} ${VISP_EXTERN_LIBRARIES})

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
install(
  TARGETS ${VISP_INTERN_LIBRARY}
  EXPORT VISPTargets
  # install rule for all the headers
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp
  # Destination is either lib or lib64 on Unix 64 bits architectures
  RUNTIME DESTINATION ${VISP_BIN_INSTALL_PATH} COMPONENT libraries
  LIBRARY DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT libraries
  ARCHIVE DESTINATION ${VISP_LIB_INSTALL_PATH} COMPONENT libraries
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
)
