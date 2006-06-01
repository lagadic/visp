#############################################################################
#
# $Id: CMakeLists.txt,v 1.11 2006-06-01 08:48:58 fspindle Exp $
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
# ViSP overall configuration file. Detect third party libraries (X11, GTK, ...)
#
# Authors:
# Fabien Spindler
#
#############################################################################

PROJECT(VISP)

#-----------------------------------------------------------------------------
# CMake 2.3 is required for INSTALL command functionnality
# ccmake --help-command INSTALL to kwow more are INSTALL command
CMAKE_MINIMUM_REQUIRED(VERSION 2.3)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_MAJOR_VERSION 2)
SET(VISP_MINOR_VERSION 1)
SET(VISP_PATCH_VERSION 2)
SET(VISP_VERSION "${VISP_MAJOR_VERSION}.${VISP_MINOR_VERSION}")
SET(VISP_VERSION_FULL "${VISP_VERSION}.${VISP_VERSION_PATCH}")      

#SET(VISP_INCLUDE_DIR "${VISP_SOURCE_DIR}/include/visp-${VISP_VERSION_FULL}")
SET(VISP_INCLUDE_DIR "${VISP_BINARY_DIR}/include/visp")

#-----------------------------------------------------------------------------

# The location in which to install VISP libraries.
SET(LIBRARY_OUTPUT_PATH ${VISP_BINARY_DIR}/lib)
# The location in which to install some VISP binaries.
SET(BINARY_OUTPUT_PATH ${VISP_BINARY_DIR}/bin)

# the include directory we depend on
SET(VISP_INTERN_INCLUDE_DIR ${VISP_BINARY_DIR}/include)
# the third party include directories we depend on
SET(VISP_EXTERN_INCLUDE_DIR "")
# the internal visp libs we depend on
SET(VISP_INTERN_LIBS visp-2)
# the third party libraries we depend on
SET(VISP_EXTERN_LIBS "")
# the third party libraries we depend on
SET(VISP_DEFS "")

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

## where are user-specific cmake modules
## make_dist.sh relies on $ENV{CMAKE_MODULES_PATH} (TODO, FIXME)
SET(CMAKE_MODULE_PATH ${VISP_SOURCE_DIR}/CMakeModules)

INCLUDE(CheckLibraryExists)

#---------------------------------------------------------------------
# Platform dependent 
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(VISP_DEFS "${VISP_DEFS} -DWIN32")
  SET(VISP_DEFS "${VISP_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm"  getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    #MESSAGE("have winmm.lib")
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
ENDIF(WIN32 AND NOT CYGWIN)
IF(UNIX)
  SET(VISP_DEFS "${VISP_DEFS} -DUNIX")
  IF(APPLE)
    # We will not use rpath support.  Tell CMake to skip it.
    SET(CMAKE_SKIP_RPATH 1 CACHE INTERNAL "Whether to build with rpath." FORCE)
    SET(VISP_DEFS "${VISP_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)


#----------------------------------------------------------------------
# Unix libraries : Coin, Qt, SoQt, X11, pthread, cfox (MAC OS X) 
#----------------------------------------------------------------------
IF(UNIX)
  #--------------------------------------------------------------------
  # Coin lib
  #--------------------------------------------------------------------
  FIND_PACKAGE(COIN)
  IF(COIN_FOUND)  
    SET(VISP_HAVE_COIN TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARIES})
    #MESSAGE(STATUS "COIN_LIBRARIES = ${COIN_LIBRARIES}")
  ELSE(COIN_FOUND)  
    MESSAGE(STATUS "COIN_LIBRARIES no found")
  ENDIF(COIN_FOUND)

  #--------------------------------------------------------------------
  # Qt lib
  #--------------------------------------------------------------------
  INCLUDE (${CMAKE_ROOT}/Modules/FindQt.cmake)
  IF(QT_FOUND)  
    SET(VISP_HAVE_QT TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${QT_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${QT_QT_LIBRARY})
    # Because in QT_DEFINITIONS defs are separated by ";", parse the
    # QT_DEFINITIONS in order to build a space separated string
    # SET(VISP_DEFS "${VISP_DEFS} ${QT_DEFINITIONS}") did not work
    # in third party projects which use visp-config shell script (due to ";"
    # separate char)
    FOREACH(DEF ${QT_DEFINITIONS})
      #MESSAGE("DEF: ${DEF}")
      SET(VISP_DEFS "${VISP_DEFS} ${DEF}")
    ENDFOREACH(DEF)

    IF(WIN32)
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${QT_QTMAIN_LIBRARY})
      SET(VISP_DEFS "${VISP_DEFS} -DQT_DLL")
    ENDIF(WIN32)
  ELSE(QT_FOUND)  
    MESSAGE(STATUS "QT_LIBRARIES no found")
  ENDIF(QT_FOUND)

  #--------------------------------------------------------------------
  # SoQt lib
  #--------------------------------------------------------------------
  FIND_PACKAGE(SOQT)
  IF(SOQT_FOUND)  
    SET(VISP_HAVE_SOQT TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOQT_LIBRARIES})
    #MESSAGE(STATUS "SOQT_LIBRARIES = ${SOQT_LIBRARIES}")
  ELSE(SOQT_FOUND)  
    MESSAGE(STATUS "SOQT_LIBRARIES no found")
  ENDIF(SOQT_FOUND)

  #--------------------------------------------------------------------
  # X11 lib
  #--------------------------------------------------------------------
  FIND_PACKAGE(X11)
  IF(X11_FOUND)
    SET(VISP_HAVE_X11 TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${X11_LIBRARIES} -lm)
    #MESSAGE("X11: ${X11_LIBRARIES}")
  ENDIF(X11_FOUND)
  #--------------------------------------------------------------------
  # Apple MAC OS X cfox library 
  #--------------------------------------------------------------------
  IF(APPLE AND USE_CFOX)
    FIND_PACKAGE(CFOX)
    IF(CFOX_FOUND)
      SET(VISP_HAVE_CFOX TRUE) # for header vpConfig.h
      #MESSAGE("DBG VISP_HAVE_CFOX= ${VISP_HAVE_CFOX}")
      #MESSAGE("DBG CFOX_INCLUDE_DIR=${CFOX_INCLUDE_DIR}")
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
	${CFOX_INCLUDE_DIR})
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} 
	${CFOX_LIBRARIES} -framework IOKit -framework Carbon)
      #MESSAGE("CFOX_LIBRARIES = ${CFOX_LIBRARIES}")
    ELSE(CFOX_FOUND)
      MESSAGE("cfox not found.")
      # SET(USE_CFOX OFF)
    ENDIF(CFOX_FOUND)
  ENDIF(APPLE AND USE_CFOX)
ENDIF(UNIX)


  #--------------------------------------------------------------------
  # pthread lib
  #--------------------------------------------------------------------
  FIND_PACKAGE(PTHREAD)
  IF(PTHREAD_FOUND)  
    SET(VISP_HAVE_PTHREAD TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
	${PTHREAD_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${PTHREAD_LIBRARIES})
    SET(VISP_HAVE_PTHREAD TRUE) # for header vpConfig.h
    #MESSAGE("PTHREAD_LIBRARIES = ${PTHREAD_LIBRARIES}")
  ELSE(PTHREAD_FOUND)  
    #MESSAGE("PTHREAD_LIBRARIES no found")
  ENDIF(PTHREAD_FOUND)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${VISP_INCLUDE_DIR})



# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build ViSP with shared libraries." OFF)

IF (UNIX)
#  OPTION(USE_DC1394  "Compile ViSP with firewire 1394 cameras usage" OFF)
#  OPTION(USE_V4L2    "Compile ViSP with Video For Linux Two (V4L2) usage" OFF)
  OPTION(USE_ICCOMP  "Compile ViSP with ICcomp framegrabber usage" OFF)
  OPTION(USE_AFMA4   "Compile ViSP for Afma4 robot usage" OFF)
  OPTION(USE_AFMA6   "Compile ViSP for Afma6 robot usage" OFF)
  OPTION(USE_BICLOPS "Compile ViSP for Biclops head usage" OFF)
  OPTION(USE_PTU46   "Compile ViSP for Ptu-46 head usage" OFF)
#  OPTION(USE_GSL     "Compile ViSP with gnu scientific library (GSL) usage" OFF)
#  OPTION(USE_GTK1    "Compile ViSP with GTK 1.0 usage ?" OFF)
#  OPTION(USE_GTK2    "Compile ViSP with GTK 2.x usage ?" OFF)
   IF(APPLE)
     OPTION(USE_CFOX   "Compile ViSP with cfox usage" OFF)
   ENDIF(APPLE)
ENDIF(UNIX)

# Default 
SET(USE_DC1394 ON)
SET(USE_V4L2   ON)
SET(USE_GSL    ON)
SET(USE_GTK2   ON)

#----------------------------------------------------------------------
# Third party software : 
#----------------------------------------------------------------------

# gnu scientific library
IF(USE_GSL)
  FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    SET(VISP_HAVE_GSL  TRUE)  # for header vpConfig.h
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GSL_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${GSL_INCLUDE_DIR})
    SET(VISP_EXTERN_LINK_DIR ${VISP_EXTERN_LINK_DIR} ${GSL_LINK_DIRECTORIES})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${GSL_LIBRARIES})
    LINK_DIRECTORIES(${GSL_LINK_DIRECTORIES})
      #MESSAGE("GSL_LINK_DIRECTORIES = ${GSL_LINK_DIRECTORIES}")
      #MESSAGE("GSL_LIBRARIE = ${GSL_LIBRARIES}")
      #MESSAGE("GSL_DEFINITIONS = ${GSL_DEFINITIONS}")
  ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

# strategy for gtk: search first gtk2. If not found search gtk1
# gtk-2 library: 
IF(USE_GTK2)
  IF(UNIX) 
    FIND_PACKAGE(GTK2) 
    IF(GTK2_FOUND)
      SET(VISP_HAVE_GTK TRUE)  # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${GTK2_INCLUDE_DIR})      
      ## ignore gtk pedantic warnings by gcc specific -isystem : 
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK2_gtk_INCLUDE_PATH}")
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${GTK2_LIBRARIES})
      # MESSAGE(STATUS "DBG CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
      #MESSAGE("GTK2 includes: ${GTK2_INCLUDE_DIR}")
      #MESSAGE("GTK2 Libs: ${GTK2_LIBRARIES}")
      SET(USE_GTK1 OFF) # no need to found gtk-1
    ELSE(GTK2_FOUND)
      #MESSAGE("GTK2 requested but not found.")
      SET(USE_GTK1 ON) # try to found gtk-1
    ENDIF(GTK2_FOUND)
#    IF(USE_GTK1)
#      MESSAGE(FATAL_ERROR "gtk1 and gtk2 cannot be used together")
#    ENDIF(USE_GTK1)
  ENDIF(UNIX)
ENDIF(USE_GTK2)

# gtk-1 library: is search only ig gtk-2 was not found
SET(USE_GTK2 OFF)
#MESSAGE("USE_GTK1: ${USE_GTK1}")
IF(USE_GTK1)
  IF(UNIX) 
    FIND_PACKAGE(GTK)
    IF(GTK_FOUND)
      SET(VISP_HAVE_GTK TRUE) # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${GTK_INCLUDE_DIR})
      ## ignore gtk pedantic warnings by gcc specific -isystem : 
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK_gtk_INCLUDE_PATH}")
      # MESSAGE("DBG CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${GTK2_LIBRARIES})
    ELSE(GTK_FOUND)
      #MESSAGE("GTK requested but not found.")
      # SET(USE_GTK OFF)
    ENDIF(GTK_FOUND)
    IF(USE_GTK2)
      MESSAGE(FATAL_ERROR "gtk1 and gtk2 cannot be used together 1" )
    ENDIF(USE_GTK2)
  ENDIF(UNIX)
ENDIF(USE_GTK1)

#----------------------------------------------------------------------
# Specific hardware : framegrabbers and cameras
#----------------------------------------------------------------------

# library for firewire cameras
IF(USE_DC1394)
  FIND_PACKAGE(RAW1394)
  FIND_PACKAGE(DC1394)
  IF(RAW1394_FOUND AND DC1394_FOUND)
    SET(VISP_HAVE_DC1394 TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_DC1394= ${VISP_HAVE_DC1394}")
     #MESSAGE("DBG DC1394_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")
     #MESSAGE("DBG RAW1394_INCLUDE_DIR=${RAW1394_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
      ${RAW1394_INCLUDE_DIR} ${DC1394_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} 
      ${RAW1394_LIBRARIES} ${DC1394_LIBRARIES})
     #MESSAGE("DC1394_LIBRARIES = ${DC1394_LIBRARIES}")
     #MESSAGE("RAW1394_LIBRARIES = ${RAW1394_LIBRARIES}")
  ELSE(RAW1394_FOUND AND DC1394_FOUND)
    #MESSAGE("Firewire dc1394 requested but not found. Turn off USE_DC1394!")
    # SET(USE_DC1394 OFF)
  ENDIF(RAW1394_FOUND AND DC1394_FOUND)
ENDIF(USE_DC1394)

# library for Video For Linux Two framegrabber devices
IF(USE_V4L2)
  FIND_PACKAGE(V4L2)
  IF(V4L2_FOUND)
    SET(VISP_HAVE_V4L2 TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_V4L2= ${VISP_HAVE_V4L2}")
     #MESSAGE("DBG V4L2_INCLUDE_DIR=${V4L2_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
      ${V4L2_INCLUDE_DIR})
  #ELSE(V4L2_FOUND)
    #MESSAGE("Video For Linux Two requested but not found.")
    # SET(USE_V4L2 OFF)
  ENDIF(V4L2_FOUND)
ENDIF(USE_V4L2)

# library for Irisa's ICcomp framegrabber devices
IF(USE_ICCOMP)
  FIND_PACKAGE(ICCOMP)
  IF(ICCOMP_FOUND)
    SET(VISP_HAVE_ICCOMP TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_ICCOMP= ${VISP_HAVE_ICCOMP}")
     #MESSAGE("DBG ICCOMP_INCLUDE_DIR=${ICCOMP_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
      ${ICCOMP_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${ICCOMP_LIBRARIES} ${VISP_EXTERN_LIBS})
     #MESSAGE("ICCOMP_LIBRARIES = ${ICCOMP_LIBRARIES}")
  ELSE(ICCOMP_FOUND)
    MESSAGE("ICcomp requested but not found.")
    # SET(USE_ICCOMP OFF)
  ENDIF(ICCOMP_FOUND)
ENDIF(USE_ICCOMP)


#----------------------------------------------------------------------
# Specific hardware : robots
#----------------------------------------------------------------------

# library for Irisa's Afma4 cylindrical robot
SET(AFMA_LIBS "")
IF(USE_AFMA4)
  FIND_PACKAGE(BIT3)
  FIND_PACKAGE(AFMA4)
  IF(AFMA4_FOUND AND BIT3_FOUND)
    SET(VISP_HAVE_AFMA4 TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_AFMA4= ${VISP_HAVE_AFMA4}")
     #MESSAGE("DBG AFMA4_INCLUDE_DIR=${AFMA4_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${AFMA4_INCLUDE_DIR}
      )
    SET(AFMA_LIBS ${AFMA_LIBS} ${AFMA4_LIBRARIES})
     #MESSAGE("AFMA4_LIBRARIES = ${AFMA4_LIBRARIES}")
  ELSE(AFMA4_FOUND AND BIT3_FOUND)
    MESSAGE("Afma4 robot requested but not found. Turn off USE_AFMA4 option!")
    # SET(USE_AFMA4 OFF)
  ENDIF(AFMA4_FOUND AND BIT3_FOUND)
ENDIF(USE_AFMA4)

# library for Irisa's Afma6 cartesian robot
IF(USE_AFMA6)
  # Test if bit3 was added previously for Afma4
  IF(NOT USE_AFMA4)
    FIND_PACKAGE(BIT3)
  ENDIF(NOT USE_AFMA4)
  FIND_PACKAGE(AFMA6)
  IF(AFMA6_FOUND AND BIT3_FOUND)
    SET(VISP_HAVE_AFMA6 TRUE) # for header vpConfig.h
    #MESSAGE("DBG VISP_HAVE_AFMA6= ${VISP_HAVE_AFMA6}")
    #MESSAGE("DBG AFMA6_INCLUDE_DIR=${AFMA6_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${AFMA6_INCLUDE_DIR}
      )
    SET(AFMA_LIBS ${AFMA_LIBS} ${AFMA6_LIBRARIES})
     #MESSAGE("AFMA6_LIBRARIES = ${AFMA6_LIBRARIES}")
  ELSE(AFMA6_FOUND AND BIT3_FOUND)
    MESSAGE("Afma6 robot requested but not found. Turn off USE_AFMA6 option!")
    # SET(USE_AFMA6 OFF)
  ENDIF(AFMA6_FOUND AND BIT3_FOUND)
ENDIF(USE_AFMA6)

# For Irisa's Afma4 and Afma6 libraries addition of common bit3 libraries.
IF(VISP_HAVE_AFMA4 OR VISP_HAVE_AFMA6)
  SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${BIT3_INCLUDE_DIR})
  # Afma4 or Afma6 api is build using basic ViSP functionalities. Because
  # of cross references problems with static libraries, we need to add
  # -lvisp-2 after Afma4 libs
  SET(AFMA_LIBS ${AFMA_LIBS} ${BIT3_LIBRARIES})
  SET(VISP_EXTERN_LIBS ${AFMA_LIBS} 
    ${VISP_EXTERN_LIBS} -l${VISP_INTERN_LIBS} )
ENDIF(VISP_HAVE_AFMA4 OR VISP_HAVE_AFMA6)

# library for Biclops pan-tilt head
IF(USE_BICLOPS)
  FIND_PACKAGE(BICLOPS)
  IF(BICLOPS_FOUND)
    SET(VISP_HAVE_BICLOPS TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_BICLOPS= ${VISP_HAVE_BICLOPS}")
     #MESSAGE("DBG BICLOPS_INCLUDE_DIR=${BICLOPS_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
      ${BICLOPS_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${BICLOPS_LIBRARIES} ${VISP_EXTERN_LIBS})
     #MESSAGE("BICLOPS_LIBRARIES = ${BICLOPS_LIBRARIES}")
  ELSE(BICLOPS_FOUND)
    MESSAGE("Biclops requested but not found. Turn off USE_BICLOPS option!")
    # SET(USE_BICLOPS OFF)
  ENDIF(BICLOPS_FOUND)
ENDIF(USE_BICLOPS)

# library for Ptu-46 pan-tilt head
IF(USE_PTU46)
  FIND_PACKAGE(PTU46)
  IF(PTU46_FOUND)
    SET(VISP_HAVE_PTU46 TRUE) # for header vpConfig.h
     #MESSAGE("DBG VISP_HAVE_PTU46= ${VISP_HAVE_PTU46}")
     #MESSAGE("DBG PTU46_INCLUDE_DIR=${PTU46_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} 
      ${PTU46_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${PTU46_LIBRARIES} ${VISP_EXTERN_LIBS})
     #MESSAGE("PTU46_LIBRARIES = ${PTU46_LIBRARIES}")
  ELSE(PTU46_FOUND)
    MESSAGE("Ptu-46 requested but not found. Turn off USE_PTU46 option!")
    # SET(USE_PTU46 OFF)
  ENDIF(PTU46_FOUND)
ENDIF(USE_PTU46)

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
   
  CONFIGURE_FILE(${VISP_SOURCE_DIR}/doc/config-doxygen.in
    ${VISP_SOURCE_DIR}/doc/config-doxygen
    @ONLY )
  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${VISP_SOURCE_DIR}/doc/config-doxygen) 
ENDIF(DOXYGEN_FOUND)


#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${VISP_INTERN_INCLUDE_DIR} ${VISP_EXTERN_INCLUDE_DIR})
LINK_DIRECTORIES(${VISP_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
# Generate the package dependent file include/visp/vpConfig.h
#----------------------------------------------------------------------
CONFIGURE_FILE(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake 
  ${VISP_INCLUDE_DIR}/vpConfig.h
)

#----------------------------------------------------------------------
# Generate the package dependent visp-config shell script for projects which
# are not using CMake:
# Usage: 
#    visp-config --include ...
#----------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/GenerateConfigScript.cmake)

#----------------------------------------------------------------------
# customize install target 
#----------------------------------------------------------------------
# install rule for visp-config shell script
INSTALL(FILES ${BINARY_OUTPUT_PATH}/visp-config 
  DESTINATION bin
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ 
  OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  )
# install rule for vpConfig.h
INSTALL(FILES ${VISP_BINARY_DIR}/include/visp/vpConfig.h
  DESTINATION include/visp 
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
  )
#----------------------------------------------------------------------
# add distclean target 
#----------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/TargetDistclean.cmake)

#----------------------------------------------------------------------
# customize clean target 
#----------------------------------------------------------------------
INCLUDE(${VISP_SOURCE_DIR}/CMakeHeaderFileList.cmake)

SET(FILE_TO_CLEAN "core*;*~;gmon.out;DartTestfile.txt")
FOREACH(header ${HEADER_ALL})
  GET_FILENAME_COMPONENT(headerName ${header} NAME) 
  SET(FILE_TO_CLEAN ${FILE_TO_CLEAN} ${CMAKE_SOURCE_DIR}/include/visp/${headerName})
ENDFOREACH(header)

SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "${FILE_TO_CLEAN}"
)

#----------------------------------------------------------------------
# customize uninstall target 
#----------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/VISPBuildSettings.cmake")
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/VISPLibraryDepends.cmake")


#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${VISP_DEFS})

#----------------------------------------------------------------------
# Configure the file describing how to use ViSP. VISPConfig.cmake
# is the main file configuring a CMake package.
# . Exports build settings and dependencies for projects using ViSP as a
#   third party project.
# . Create and install files for simple use of FIND_PACKAGE(VISP) 
#   by other cmakified "user" projects and libraries depending on ViSP. 
#   (see "Mastering CMake", pp.72)
# . To use ViSP in a third party project based on CMake:
#   FIND_PACKAGE(VISP REQUIRED)
#   IF(VISP_FOUND)
#     INCLUDE(${VISP_USE_FILE})
#   ENDIF(VISP_FOUND)
#----------------------------------------------------------------------
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${LIBRARY_OUTPUT_PATH}/VISPConfig.cmake
  IMMEDIATE @ONLY)

# Just to copy ./CMakeModule/VISPUse.cmake.in in ./lib/UseVISP.cmake
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${LIBRARY_OUTPUT_PATH}/VISPUse.cmake
  IMMEDIATE @ONLY)

# Export our build settings and library dependencies for use by the third party
# user projects
INCLUDE(${CMAKE_ROOT}/Modules/CMakeExportBuildSettings.cmake)
CMAKE_EXPORT_BUILD_SETTINGS(
  ${LIBRARY_OUTPUT_PATH}/VISPBuildSettings.cmake
  )
EXPORT_LIBRARY_DEPENDENCIES(
  ${LIBRARY_OUTPUT_PATH}/VISPLibraryDepends.cmake
  )

# Install the packaging files for use by FIND_PACKAGE(VISP) in user projects
INSTALL(FILES
  ${LIBRARY_OUTPUT_PATH}/VISPUse.cmake
  ${LIBRARY_OUTPUT_PATH}/VISPConfig.cmake
  ${LIBRARY_OUTPUT_PATH}/VISPBuildSettings.cmake
  ${LIBRARY_OUTPUT_PATH}/VISPLibraryDepends.cmake
  DESTINATION lib
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
  )
  

#----------------------------------------------------------------------
# For Dart server and tests
#----------------------------------------------------------------------
ENABLE_TESTING()
INCLUDE(DartConfig.cmake)
INCLUDE(Dart)

# Add build options for test coverage. Currently coverage is only supported on
# gcc compiler 
# Because using -fprofile-arcs with shared lib can cause problems like:
# hidden symbol `__bb_init_func', we add this option only for static library
# build
IF(CMAKE_COMPILER_IS_GNUCXX AND BUILD_STATIC_LIBS)
  SET(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND BUILD_STATIC_LIBS)

#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(src)
SUBDIRS(example)
IF(BUILD_TESTING)
  SUBDIRS(test)
ENDIF(BUILD_TESTING)
