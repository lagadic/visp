#
# Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
# www: http://www.irisa.fr/lagadic
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#
# This file creates the ViSP library

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
    OUTPUT ${CMAKE_SOURCE_DIR}/include/visp/${headerName}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${CMAKE_SOURCE_DIR}/include/visp/${headerName}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${header}
    )
  SET(HEADER_IN_INCLUDE_DIR ${HEADER_IN_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include/visp/${headerName}
    )
ENDFOREACH(header)

#----------------------------------------------------------------------
# Create a top level rule to copy all the headers from src to include/visp
#----------------------------------------------------------------------
ADD_CUSTOM_TARGET(header
  DEPENDS ${HEADER_IN_INCLUDE_DIR}
  )


#----------------------------------------------------------------------
# build rule for the library
#----------------------------------------------------------------------
ADD_LIBRARY (${VISP_INTERN_LIBS} ${SRC_ALL}) 

# create the headers in include/visp before compiling the lib
ADD_DEPENDENCIES(${VISP_INTERN_LIBS} header)

# Append the library version information to the library target
# properties. 
SET_TARGET_PROPERTIES(${VISP_INTERN_LIBS}
  PROPERTIES
  VERSION ${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}
  SOVERSION ${VISP_VERSION_PATCH}
  )

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for the library
INSTALL(TARGETS ${VISP_INTERN_LIBS}
  DESTINATION lib
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
  )
# install rule for all the headers
INSTALL(FILES ${HEADER_ALL}
  DESTINATION include/visp
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
  )
