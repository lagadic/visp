#############################################################################
#
# $Id: CMakeLists.txt,v 1.3 2007-02-06 15:34:30 fspindle Exp $
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
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# This small project under CMake shows how to use ViSP as a third party library
# in this development.
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#############################################################################

# You can tune this project given as example by modifying the CMake variables
# below:
#
#   PROJECT_NAME:          The name of your project
#   PROJECT_VERSION_MAJOR: The major version.
#   PROJECT_VERSION_MINOR: The minor version. An even number mean release.
#   PROJECT_VERSION_PATCH: Patch version.
#
# If you change the PROJECT_NAME, you have to edit the following files
# - src/example/exExample.cpp
# - example/example.cpp
# in order to replace #include <example/exExample.h> 
# by #include <your new project name in lower characters/exExample.h>

#-----------------------------------------------------------------------------
# Main project settings: name and version numbers
#-----------------------------------------------------------------------------
SET(PROJECT_NAME "EXAMPLE")
SET(PROJECT_VERSION_MAJOR 1)
SET(PROJECT_VERSION_MINOR 0)
SET(PROJECT_VERSION_PATCH 0)

#-----------------------------------------------------------------------------
# Set the project name
#-----------------------------------------------------------------------------
PROJECT(${PROJECT_NAME})

#-----------------------------------------------------------------------------
# the project name in lower characters
#-----------------------------------------------------------------------------
STRING(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#-----------------------------------------------------------------------------
# PROJECT version number.  An even minor number corresponds to releases.
#-----------------------------------------------------------------------------
SET(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
SET(PROJECT_VERSION_FULL "${PROJECT_VERSION}.${PROJECT_VERSION_PATCH}")      


#-----------------------------------------------------------------------------
# Project settings
#-----------------------------------------------------------------------------
# The location in which to copy the headers
SET(PROJECT_INCLUDE_DIR "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME_LOWER}")

# The location of the documentation
SET(PROJECT_DOC_DIR "${PROJECT_BINARY_DIR}/doc")

# The location in which to install PROJECT library.
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# The include directory we depend on (usefull with -I compiler option)
SET(PROJECT_INTERN_INCLUDE_DIR ${PROJECT_BINARY_DIR}/include)

# The internal PROJECT library name we want to create
SET(PROJECT_INTERN_LIBS ${PROJECT_NAME_LOWER})

# The third party include directories we depend on
SET(PROJECT_EXTERN_INCLUDE_DIR "")

# The third party libraries we depend on
SET(PROJECT_EXTERN_LIBS "")

# Additional defines (-D compiler option)
SET(PROJECT_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

# where are user-specific cmake modules
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${PROJECT_INCLUDE_DIR})

#---------------------------------------------------------------------
# Choose static or shared libraries.
#---------------------------------------------------------------------
OPTION(BUILD_SHARED_LIBS "Build a dynamic or shared library." OFF)

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(PROJECT_DEFS "${PROJECT_DEFS} -DWIN32")
  SET(PROJECT_DEFS "${PROJECT_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    SET(PROJECT_EXTERN_LIBS ${PROJECT_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(PROJECT_DEFS "${PROJECT_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(PROJECT_DEFS "${PROJECT_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

#----------------------------------------------------------------------
# Third party libraries
#----------------------------------------------------------------------
# ViSP
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${PROJECT_INTERN_INCLUDE_DIR} ${PROJECT_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add library path
#----------------------------------------------------------------------
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# add html-doc target for doxygen documentation building
#----------------------------------------------------------------------
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)  
  ## we need latex for doxygen because of the formulas
  FIND_PACKAGE(LATEX)
  IF    (NOT LATEX_COMPILER)
    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run.")
  ENDIF (NOT LATEX_COMPILER)
  IF    (NOT MAKEINDEX_COMPILER)
    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  ENDIF (NOT MAKEINDEX_COMPILER)
  IF    (NOT DVIPS_CONVERTER)
    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  ENDIF (NOT DVIPS_CONVERTER)
   
  CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/doc/config-doxygen.in
    ${PROJECT_BINARY_DIR}/doc/config-doxygen
    @ONLY )
  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doc/config-doxygen) 
ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${PROJECT_DEFS})

#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(src)
SUBDIRS(example)

#----------------------------------------------------------------------
# customize uninstall target 
#----------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

#----------------------------------------------------------------------
# For CPack packaging tool
#----------------------------------------------------------------------

# InstallRequiredSystemLibraries: Hack for Visual Studio support Search for
# system runtime libraries based on the platform.
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_NAME}: An example of CMake project using ViSP as a third party library")
SET(CPACK_PACKAGE_VENDOR "Inria, French National Institute for Research in Computer Science and Control")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
SET(CPACK_SOURCE_IGNORE_FILES
"~$"
"^${PROJECT_SOURCE_DIR}.*/CVS/"
)

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME} ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
IF(WIN32 AND NOT UNIX)
  # There is a bug in NSI that does not handle full unix paths properly. Make
  # sure there is at least one set of four (4) backlasshes.
#  SET(CPACK_PACKAGE_ICON "${CMake_SOURCE_DIR}/Utilities/Release\\\\InstallIcon.bmp")
  # CPACK_NSIS_INSTALLED_ICON_NAME icon is the one that shows up in the 
  # Add/Remove control panel
  #SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\ViSP.exe")
  SET(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} My Famous Project")
  SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.http://www.irisa.fr/lagadic/visp")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.http://www.irisa.fr/lagadic/visp/what.html")
  SET(CPACK_NSIS_CONTACT "visp@irisa.fr")
  SET(CPACK_NSIS_MODIFY_PATH ON)
ELSE(WIN32 AND NOT UNIX)
  SET(CPACK_STRIP_FILES "")
  SET(CPACK_SOURCE_STRIP_FILES "")
ENDIF(WIN32 AND NOT UNIX)
SET(CPACK_PACKAGE_EXECUTABLES ${PROJECT_NAME} ${PROJECT_NAME})

INCLUDE(CPack)
