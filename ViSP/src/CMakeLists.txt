#############################################################################
#
# $Id: CMakeLists.txt,v 1.11 2007-05-02 12:43:33 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
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
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
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
SET(ADDED_SRC_DEPEND_PROPERTIES "")
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
ADD_LIBRARY (${VISP_INTERN_LIBS} ${SRC_ALL} ${HEADER_ALL}) 

# create the headers in include/visp before compiling the lib
ADD_DEPENDENCIES(${VISP_INTERN_LIBS} header)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${VISP_INTERN_LIBS} PROPERTIES
  VERSION ${VISP_VERSION}
  SOVERSION ${VISP_VERSION_MAJOR}
  )

# Link with external libs especially under windows for winmm.lib
# needed to create visp-2.dll. Why ? Because for binaries, winmm.lib
# is considered in the link stage. Probably a CMake bug ! 
#IF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
#  TARGET_LINK_LIBRARIES(${VISP_INTERN_LIBS} "winmm")
#ENDIF(WIN32 AND BUILD_SHARED_LIBS AND HAVE_LIBWINMM)
TARGET_LINK_LIBRARIES(${VISP_INTERN_LIBS} ${VISP_EXTERN_LIBS})

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL(TARGETS ${VISP_INTERN_LIBS}
  DESTINATION lib
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
# install rule for all the headers
INSTALL(FILES ${HEADER_ALL}
  DESTINATION include/visp
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
