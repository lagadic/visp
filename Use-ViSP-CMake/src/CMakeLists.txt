#############################################################################
#
# $Id: CMakeLists.txt,v 1.3 2007-02-06 15:37:09 fspindle Exp $
#
# Copyright (C) 2007 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP software.
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# This small project under CMake shows how to use ViSP as a third party library
# in this development.
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#############################################################################

#----------------------------------------------------------------------
# Include project files 
#----------------------------------------------------------------------
# include all the project .cpp files
INCLUDE(${PROJECT_SOURCE_DIR}/CMakeSourceFileList.cmake)
# include all the project .h files
INCLUDE(${PROJECT_SOURCE_DIR}/CMakeHeaderFileList.cmake)


#----------------------------------------------------------------------
# Create rule to copy all the headers from src to include/PROJECT_NAME_LOWER
#----------------------------------------------------------------------
# For each header, we create a rule
SET(HEADER_IN_INCLUDE_DIR "")
SET(ADDED_SRC_DEPEND_PROPERTIES "")
FOREACH(header ${HEADER_ALL})
  GET_FILENAME_COMPONENT(headerName ${header} NAME) 
  ADD_CUSTOM_COMMAND(
    OUTPUT ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME_LOWER}/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME_LOWER}/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
    )
  SET(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME_LOWER}/${headerName}
    )
ENDFOREACH(header)

#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to 
# include/PROJECT_NAME_LOWER
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(header ALL
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${PROJECT_INTERN_LIBS} ${SRC_ALL}) 

# create the headers in include/PROJECT_NAME_LOWER before compiling the lib
ADD_DEPENDENCIES(${PROJECT_INTERN_LIBS} header)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${PROJECT_INTERN_LIBS} PROPERTIES
  VERSION ${PROJECT_VERSION_MAJOR}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  )

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL(TARGETS ${PROJECT_INTERN_LIBS}
  DESTINATION lib
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
# install rule for all the headers
INSTALL(FILES ${HEADER_ALL}
  DESTINATION include/${PROJECT_NAME_LOWER}
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
