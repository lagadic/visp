#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
# ViSP overall configuration file. Detect third party libraries (X11, GTK, ...)
#
# Authors:
# Fabien Spindler
#
#############################################################################

PROJECT(VISP)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_VERSION_MAJOR "2")
SET(VISP_VERSION_MINOR "6")
SET(VISP_VERSION_PATCH "0")
SET(VISP_VERSION "${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}.${VISP_VERSION_PATCH}")

SET(VISP_INCLUDE_DIR "${VISP_BINARY_DIR}/include/visp")
SET(VISP_DOC_DIR "${VISP_BINARY_DIR}/doc")

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

# where are user-specific cmake modules
SET(CMAKE_MODULE_PATH ${VISP_SOURCE_DIR}/CMakeModules)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${VISP_INCLUDE_DIR})

#-----------------------------------------------------------------------------
# Add extra compilation flags under UNIX
#-----------------------------------------------------------------------------
if(UNIX)
  include(${CMAKE_MODULE_PATH}/AddExtraCompilationFlags.cmake)
  ADD_EXTRA_COMPILATION_FLAGS()
endif()

#--------------------------------------------------------------------
# By default set release configuration
#--------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

#--------------------------------------------------------------------
# Option management
#--------------------------------------------------------------------

# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build ViSP shared libraries (.dll/.so) instead of static ones (.lib/.a)." OFF)
# Build examples as an option.
OPTION(BUILD_EXAMPLES "Build ViSP examples." ON)
# Build demos as an option.
OPTION(BUILD_DEMOS "Build ViSP demos." ON)
# Build deprecated functions as an option.
OPTION(BUILD_DEPRECATED_FUNCTIONS "Build deprecated functionalities." ON)

IF(BUILD_DEPRECATED_FUNCTIONS)
  SET(VISP_BUILD_DEPRECATED_FUNCTIONS TRUE) # for header vpConfig.h
ENDIF(BUILD_DEPRECATED_FUNCTIONS)

IF (UNIX)
  FIND_PACKAGE(DC1394)
  IF(DC1394_2_FOUND)
    OPTION(USE_DC1394_2    "Compile ViSP with the libdc1394.2 library" ON)
  ELSE(DC1394_2_FOUND)
    SET(USE_DC1394_2 OFF)
  ENDIF(DC1394_2_FOUND)
  IF(DC1394_1_FOUND)
    OPTION(USE_DC1394_1    "Compile ViSP with the libdc1394.1 library" ON)
  ELSE(DC1394_1_FOUND)
    SET(USE_DC1394_1 OFF)
  ENDIF(DC1394_1_FOUND)

  FIND_PACKAGE(V4L2)
  IF(V4L2_FOUND)
    OPTION(USE_V4L2    "Compile ViSP with the v4l2 (video for linux 2) library" ON)
  ELSE(V4L2_FOUND)
    SET(USE_V4L2 OFF)
  ENDIF(V4L2_FOUND)

  IF(NOT RAW1394_FOUND)
    FIND_PACKAGE(RAW1394)
  ENDIF(NOT RAW1394_FOUND)
  FIND_PACKAGE(RT)
  FIND_PACKAGE(CALINUX)
  FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    OPTION(USE_AFMA4   "Compile ViSP for Afma4 robot usage at Irisa" ON)
    OPTION(USE_AFMA6   "Compile ViSP for Afma6 robot usage at Irisa" ON)
    OPTION(USE_VIPER850 "Compile ViSP for Viper S850 robot usage at Irisa" ON)
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    SET(USE_AFMA4 OFF)
    SET(USE_AFMA6 OFF)
    SET(USE_VIPER850 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)

  FIND_PACKAGE(BICLOPS)
  IF(BICLOPS_FOUND)
    OPTION(USE_BICLOPS "Compile ViSP for Biclops head usage at Irisa" ON)
  ELSE(BICLOPS_FOUND)
    SET(USE_BICLOPS OFF)
  ENDIF(BICLOPS_FOUND)

  FIND_PACKAGE(PTU46)
  IF(PTU46_FOUND)
    OPTION(USE_PTU46 "Compile ViSP for Ptu-46 head usage at Irisa" ON)
  ELSE(PTU46_FOUND)
    SET(USE_PTU46 OFF)
  ENDIF(PTU46_FOUND)

  FIND_PACKAGE(CycabTk)
  IF(CycabTk_FOUND)
    OPTION(USE_CYCAB "Compile ViSP for Cycab car-like mobile robot at Irisa" ON)
  ELSE(CycabTk_FOUND)
    SET(USE_CYCAB OFF)
  ENDIF(CycabTk_FOUND)
 
ENDIF(UNIX)

IF(WIN32)
  FIND_PACKAGE(GDI)
  IF(GDI_FOUND)
    OPTION(USE_GDI    "Manages the display on windows" ON)
  ELSE(GDI_FOUND)
    SET(USE_GDI OFF)
  ENDIF(GDI_FOUND)

  FIND_PACKAGE(DIRECT3D)
  IF(DIRECT3D_FOUND)
    OPTION(USE_DIRECT3D    "Manages the display on windows" ON)
  ELSE(DIRECT3D_FOUND)
    SET(USE_DIRECT3D OFF)
  ENDIF(DIRECT3D_FOUND)

  FIND_PACKAGE(DIRECTSHOW)
  IF(DIRECTSHOW_FOUND)
    OPTION(USE_DIRECTSHOW    "Manages the frame grabbing on windows" ON)
  ELSE(DIRECTSHOW_FOUND)
    SET(USE_DIRECTSHOW OFF)
  ENDIF(DIRECTSHOW_FOUND)
ENDIF(WIN32)

INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
FIND_PACKAGE(COIN)
IF (COIN_FOUND AND OPENGL_FOUND)
  OPTION(USE_COIN    "Compile ViSP with Coin3D" ON)
ELSE (COIN_FOUND AND OPENGL_FOUND)
  SET(USE_COIN OFF)
ENDIF (COIN_FOUND AND OPENGL_FOUND)

IF(WIN32 AND USE_COIN)
  FIND_PACKAGE(SOWIN)
  IF (SOWIN_FOUND)
    OPTION(USE_SOWIN    "Compile ViSP with SoWin" OFF)
  ELSE (SOWIN_FOUND)
    SET(USE_SOWIN OFF)
  ENDIF (SOWIN_FOUND)
ENDIF(WIN32 AND USE_COIN)

IF(USE_COIN)
  FIND_PACKAGE(SOQT)
  IF (SOQT_FOUND)
    OPTION(USE_SOQT    "Compile ViSP with SoQt" OFF)
  ELSE (SOQT_FOUND)
    SET(USE_SOQT OFF)
  ENDIF (SOQT_FOUND)
  IF(USE_SOQT)
    INCLUDE (${CMAKE_ROOT}/Modules/FindQt.cmake) 
    IF (QT_FOUND)
      OPTION(USE_QT    "Compile ViSP with Qt" ON)
    ELSE (QT_FOUND)
	  SET(USE_QT OFF)
    ENDIF (QT_FOUND)
  ENDIF (USE_SOQT)
ENDIF(USE_COIN)

IF(USE_COIN)
  FIND_PACKAGE(SOXT)
  IF (SOXT_FOUND)
    OPTION(USE_SOXT    "Compile ViSP with SoXt" OFF)
  ELSE (SOXT_FOUND)
    SET(USE_SOXT OFF)
  ENDIF (SOXT_FOUND)
ENDIF(USE_COIN)

FIND_PACKAGE(PTHREAD)
IF (PTHREAD_FOUND)
  OPTION(USE_PTHREAD    "Compile ViSP with pthread" ON)
ELSE ()
  SET(USE_PTHREAD OFF)
ENDIF ()

FIND_PACKAGE(XML2)
IF(XML2_FOUND)
  OPTION(USE_XML2    "Compile ViSP with the xml2 library" ON)
ELSE(XML2_FOUND)
  SET(USE_XML2 OFF)
ENDIF(XML2_FOUND)

FIND_PACKAGE(OpenCV QUIET)
IF(OpenCV_FOUND)
  OPTION(USE_OPENCV    "Compile ViSP with the OpenCV library" ON)
ELSE(OpenCV_FOUND)
  SET(USE_OPENCV OFF)
ENDIF(OpenCV_FOUND)

FIND_PACKAGE(GSL)
IF(GSL_FOUND)
  OPTION(USE_GSL    "Compile ViSP with the GSL library" ON)
ELSE(GSL_FOUND)
  SET(USE_GSL OFF)
ENDIF(GSL_FOUND)

FIND_PACKAGE(X11)
IF(X11_FOUND)
  OPTION(USE_X11    "Compile ViSP with the X11 library" ON)
ELSE(X11_FOUND)
  SET(USE_X11 OFF)
ENDIF(X11_FOUND)

FIND_PACKAGE(GTK2)
IF(GTK2_FOUND)
  OPTION(USE_GTK2    "Compile ViSP with the GTK+ library" ON)
ELSE(GTK2_FOUND)
  SET(USE_GTK2 OFF)
  FIND_PACKAGE(GTK)
  IF(GTK_FOUND)
    OPTION(USE_GTK1    "Compile ViSP with the GTK library" ON)
  ELSE(GTK_FOUND)
    SET(USE_GTK1 OFF)
  ENDIF(GTK_FOUND)
ENDIF(GTK2_FOUND)

FIND_PACKAGE(LIBJPEG)
IF(LIBJPEG_FOUND)
  OPTION(USE_LIBJPEG    "Compile ViSP with the libjpeg library" ON)
ELSE(LIBJPEG_FOUND)
  SET(USE_LIBJPEG OFF)
ENDIF(LIBJPEG_FOUND)

FIND_PACKAGE(LIBPNG)
IF(LIBPNG_FOUND)
  OPTION(USE_LIBPNG    "Compile ViSP with the libpng library" ON)
ELSE(LIBPNG_FOUND)
  SET(USE_LIBPNG OFF)
ENDIF(LIBPNG_FOUND)

FIND_PACKAGE(FFMPEG)
IF(FFMPEG_FOUND)
  OPTION(USE_FFMPEG    "Compile ViSP with the ffmpeg library" ON)
ELSE(FFMPEG_FOUND)
  SET(USE_FFMPEG OFF)
ENDIF(FFMPEG_FOUND)

# Set other options to default value
#SET(USE_X11        ON) # For Linux/OSX display
#SET(USE_XML2       ON) # For the Xml Parser (vpCameraParameters)
#SET(USE_COIN       ON) # For the simulator
#SET(USE_SOWIN      ON) # For the simulator viewer
#SET(USE_SOQT       ON) # For the simulator viewer
#SET(USE_SOXT       ON) # For the simulator viewer
#SET(USE_PTHREAD    ON) # For Biclops/Simulator
#SET(USE_OPENCV     ON) # For Intel opencv
#SET(USE_GSL        ON) # For matrix manipulations
#SET(USE_GTK1       ON) # For Linux/OSX/Windows display with gtk-1.x
#SET(USE_GTK2       ON) # For Linux/OSX/Windows display with gtk-2.x
#SET(USE_DIRECT3D   ON) # For Windows display
#SET(USE_GDI        ON) # For Windows display
#SET(USE_DC1394_1   ON) # For firewire grabber under Linux and OSX ?
#SET(USE_DC1394_2   ON) # For firewire grabber under Linux and OSX ?
#SET(USE_V4L2       ON) # For Video 4 Linux 2 grabber under Linux
#SET(USE_DIRECTSHOW ON) # For Windows direct show grabber
#SET(USE_LIBJPEG    ON) # For reading jpeg files
#SET(USE_LIBPNG     ON) # For reading png files
SET(USE_ZLIB       ON) # For data compression
#SET(USE_FFMPEG     ON) # For video read and write

IF(BUILD_SHARED_LIBS)
  SET(VISP_BUILD_SHARED_LIBS TRUE)  # for header vpConfig.h
ENDIF(BUILD_SHARED_LIBS)


INCLUDE(CheckLibraryExists)

#---------------------------------------------------------------------
# Try to determine if Inria's NAS server hosting /udd/ is available 
#----------------------------------------------------------------------
FIND_PACKAGE(NAS)
IF(NAS_FOUND)
  SET(VISP_HAVE_ACCESS_TO_NAS TRUE) # for header vpConfig.h
ENDIF(NAS_FOUND)

#---------------------------------------------------------------------
# Platform dependent
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  LIST(APPEND VISP_DEFS "-DWIN32")
  LIST(APPEND VISP_DEFS "-DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm.lib" getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    #MESSAGE("have winmm.lib")
    LIST(APPEND VISP_EXTERN_LIBS "winmm.lib")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set VISP_USE_MSVC accordingly
    SET(VISP_USE_MSVC TRUE)
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)

IF(UNIX)
  LIST(APPEND VISP_DEFS "-DUNIX")
  IF(APPLE)
    LIST(APPEND VISP_DEFS "-DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

INCLUDE(TestBigEndian)
TEST_BIG_ENDIAN(BIGENDIAN)

IF(BIGENDIAN)
  LIST(APPEND VISP_DEFS "-DBIGENDIAN")
ENDIF(BIGENDIAN)

#--------------------------------------------------------------------
# X11 lib
#--------------------------------------------------------------------
SET(VISP_HAVE_X11_FOUND "no")  # for ViSP-third-party.txt
IF(USE_X11)
  IF(X11_FOUND)
    MESSAGE(STATUS "X11 found")
    SET(VISP_HAVE_X11 TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_X11_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${X11_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${X11_LIBRARIES} -lm)
    #MESSAGE("X11: ${X11_LIBRARIES}")
  ELSE(X11_FOUND)
    MESSAGE(STATUS "X11 not found")
    SET(VISP_HAVE_X11_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(X11_FOUND)
ENDIF(USE_X11)


#--------------------------------------------------------------------
# OpenGL, Qt, SoQt and Coin libs
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_SIMULATOR_FOUND "no") # for ViSP-third-party.txt
SET(VISP_HAVE_COIN_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_SOWIN_FOUND "no")     # for ViSP-third-party.txt
SET(VISP_HAVE_SOXT_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_SOQT_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_QT4_FOUND "no")       # for ViSP-third-party.txt
SET(VISP_HAVE_QT3_FOUND "no")       # for ViSP-third-party.txt

IF(USE_COIN)
  IF (COIN_FOUND)
    MESSAGE(STATUS "Coin3D found")
    SET(VISP_HAVE_COIN_FOUND "yes")   # for ViSP-third-party.txt
    SET(VISP_HAVE_COIN TRUE)          # for header vpConfig.h
    SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
    IF(WIN32)
      LIST(APPEND VISP_DEFS "-DCOIN_DLL")
    ENDIF(WIN32)
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OPENGL_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${COIN_INCLUDE_DIR})  
    LIST(APPEND VISP_EXTERN_LIBS ${OPENGL_gl_LIBRARY})

    #MESSAGE("COIN_INCLUDE_DIR = ${COIN_INCLUDE_DIR}")
    IF(UNIX)
      #MESSAGE("COIN_LIBRARIES = ${COIN_LIBRARIES}")
      LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARIES})
    ELSE(UNIX)
      #MESSAGE("COIN_LIBRARY_RELEASE = ${COIN_LIBRARY_RELEASE}")
      #MESSAGE("COIN_LIBRARY_DEBUG = ${COIN_LIBRARY_DEBUG}")
      IF(COIN_LIBRARY_DEBUG)
        LIST(APPEND VISP_EXTERN_LIBS debug ${COIN_LIBRARY_DEBUG})
      ENDIF()
      IF(COIN_LIBRARY_RELEASE)
        LIST(APPEND VISP_EXTERN_LIBS optimized ${COIN_LIBRARY_RELEASE})
      ENDIF()
    ENDIF(UNIX)
    IF (USE_SOWIN AND SOWIN_FOUND)
      # We manage SoWin and Coin
      SET(VISP_HAVE_SOWIN TRUE)         # for header vpConfig.h
      SET(VISP_HAVE_SOWIN_FOUND "yes")  # for ViSP-third-party.txt
      SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    
      
      #MESSAGE("SOWIN_INCLUDE_DIR = ${SOWIN_INCLUDE_DIR}")
      #MESSAGE("SOWIN_LIBRARY_RELEASE = ${SOWIN_LIBRARY_RELEASE}")
      #MESSAGE("SOWIN_LIBRARY_DEBUG = ${SOWIN_LIBRARY_DEBUG}")

      LIST(APPEND VISP_DEFS "-DSOWIN_DLL")
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${SOWIN_INCLUDE_DIR})
      IF(SOWIN_LIBRARY_RELEASE)
          LIST(APPEND VISP_EXTERN_LIBS optimized ${SOWIN_LIBRARY_RELEASE})
      ENDIF(SOWIN_LIBRARY_RELEASE)
      IF(SOWIN_LIBRARY_DEBUG)
          LIST(APPEND VISP_EXTERN_LIBS debug ${SOWIN_LIBRARY_DEBUG})
      ENDIF(SOWIN_LIBRARY_DEBUG)
    ENDIF(USE_SOWIN AND SOWIN_FOUND)

    IF(USE_SOQT AND SOQT_FOUND)
      #MESSAGE("SoQt found")

      IF (USE_QT AND QT_FOUND)
	#MESSAGE("Qt found")
	# OpenGL, QT and SOQT are found
	# We first manage OpenGL
	SET(VISP_HAVE_SOQT   TRUE) # for header vpConfig.h 
	SET(VISP_HAVE_SOQT_FOUND "yes")      # for ViSP-third-party.txt
	SET(VISP_HAVE_QT     TRUE) # for header vpConfig.h
      	SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    

        #MESSAGE(STATUS "SOQT_INCLUDE_DIR = ${SOQT_INCLUDE_DIR}")
	LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${SOQT_INCLUDE_DIR})
	LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${QT_INCLUDE_DIR})
        LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${QT_INCLUDES})

	# Add SoQt libraries
	IF(WIN32)
	  #MESSAGE("SOQT_LIBRARY_RELEASE = ${SOQT_LIBRARY_RELEASE}")
	  #MESSAGE("SOQT_LIBRARY_DEBUG = ${SOQT_LIBRARY_DEBUG}")
	  LIST(APPEND VISP_DEFS "-DSOQT_DLL")
	  IF(SOQT_LIBRARY_RELEASE)
	    LIST(APPEND VISP_EXTERN_LIBS optimized ${SOQT_LIBRARY_RELEASE})
	  ENDIF()
	  IF(SOQT_LIBRARY_DEBUG)
	    LIST(APPEND VISP_EXTERN_LIBS debug ${SOQT_LIBRARY_DEBUG})
	  ENDIF()
	ELSE(WIN32)
	  #MESSAGE("SOQT_LIBRARIES = ${SOQT_LIBRARIES}")
	  LIST(APPEND VISP_EXTERN_LIBS ${SOQT_LIBRARIES})
	ENDIF(WIN32)

	# We manage QT libraries
	IF (DESIRED_QT_VERSION MATCHES 3)
	  #Add Qt3 libraries
	  SET(VISP_HAVE_QT3_FOUND "yes")       # for ViSP-third-party.txt
	  #MESSAGE("QT_QT_LIBRARY ${QT_QT_LIBRARY}")
	  LIST(APPEND VISP_EXTERN_LIBS ${QT_QT_LIBRARY})
	ELSEIF(DESIRED_QT_VERSION MATCHES 4)
	  #Add Qt4 libraries
	  SET(VISP_HAVE_QT4_FOUND "yes")       # for ViSP-third-party.txt
	  #MESSAGE("QT_QTGUI_LIBRARY ${QT_QTGUI_LIBRARY}")
	  #MESSAGE("QT_QTGUI_LIBRARY_RELEASE ${QT_QTGUI_LIBRARY_RELEASE}")
	  #MESSAGE("QT_QTGUI_LIBRARY_DEBUG ${QT_QTGUI_LIBRARY_DEBUG}")
	  IF (QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
	    LIST(APPEND VISP_EXTERN_LIBS optimized ${QT_QTGUI_LIBRARY_RELEASE})
            LIST(APPEND VISP_EXTERN_LIBS optimized ${QT_QTCORE_LIBRARY_RELEASE})
	  ENDIF()
	  IF (QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
	    LIST(APPEND VISP_EXTERN_LIBS debug ${QT_QTGUI_LIBRARY_DEBUG})
            LIST(APPEND VISP_EXTERN_LIBS debug ${QT_QTCORE_LIBRARY_DEBUG})
	  ENDIF()
	ENDIF (DESIRED_QT_VERSION MATCHES 3)

	# Because in QT_DEFINITIONS defs are separated by ";", parse the
	# QT_DEFINITIONS in order to build a space separated string
	#SET(VISP_DEFS "${VISP_DEFS} ${QT_DEFINITIONS}") did not work
	# in third party projects which use visp-config shell script 
	# (due to ";" # separate char)
	FOREACH(DEF ${QT_DEFINITIONS})
	  #MESSAGE("DEF: ${DEF}")
	  LIST(APPEND VISP_DEFS ${DEF})
	ENDFOREACH(DEF)
					  
	LIST(APPEND VISP_DEFS "-DQT_DLL")
      ENDIF (USE_QT AND QT_FOUND)
    ENDIF(USE_SOQT AND SOQT_FOUND)
        
    IF (USE_SOXT AND SOXT_FOUND)
      # OPenGL and SoXt are found
      # We first manage OpenGL
      SET(VISP_HAVE_SOXT TRUE)  # for header vpConfig.h    
      SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    
      SET(VISP_HAVE_SOXT_FOUND "yes")       # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_LIBS ${SOXT_LIBRARIES})
    ENDIF(USE_SOXT AND SOXT_FOUND)

    IF(VISP_HAVE_COIN_AND_GUI)
      SET(VISP_HAVE_COIN_AND_GUI_FOUND "yes")  # for ViSP-third-party.txt
      MESSAGE(STATUS "Coin3D + GUI found")
    ELSE(VISP_HAVE_COIN_AND_GUI)
      MESSAGE(STATUS "Coin3D + GUI not found")
      SET(VISP_HAVE_COIN_AND_GUI_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(VISP_HAVE_COIN_AND_GUI)
  ENDIF (COIN_FOUND)
ENDIF(USE_COIN)

#---------------------------------------------------------------------
# XML2
#---------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_XML2_FOUND "no")  # for ViSP-third-party.txt

IF(USE_XML2)
  #FIND_PACKAGE(XML2)
  IF(XML2_FOUND)
    SET(VISP_HAVE_XML2 TRUE)
    SET(VISP_HAVE_XML2_FOUND "yes")  # for ViSP-third-party.txt
    MESSAGE(STATUS "XML2 found")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${XML2_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${XML2_LIBRARIES})
  ELSE(XML2_FOUND)
    MESSAGE(STATUS "XML2 not found")
    SET(VISP_HAVE_XML2_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(XML2_FOUND)
ENDIF(USE_XML2)

#--------------------------------------------------------------------
# pthread lib
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_PTHREAD_FOUND "no")  # for ViSP-third-party.txt

IF(USE_PTHREAD)
  #FIND_PACKAGE(PTHREAD)
  IF(PTHREAD_FOUND)
    MESSAGE(STATUS "pthread found")
    SET(VISP_HAVE_PTHREAD TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_PTHREAD_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${PTHREAD_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${PTHREAD_LIBRARIES})
    #MESSAGE("PTHREAD_LIBRARIES = ${PTHREAD_LIBRARIES}")
  ELSE(PTHREAD_FOUND)
    MESSAGE(STATUS "pthread not found")
    SET(VISP_HAVE_PTHREAD_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(PTHREAD_FOUND)
ENDIF(USE_PTHREAD)

#--------------------------------------------------------------------
# parallel port usage
#--------------------------------------------------------------------
FIND_PACKAGE(PARPORT)
IF(PARPORT_FOUND) 
  SET(VISP_HAVE_PARPORT TRUE)  # for header vpConfig.h
ENDIF(PARPORT_FOUND)  

#----------------------------------------------------------------------
# Third party software :
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_OPENCV_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GSL_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GTK_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_D3D9_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GDI_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_LIBJPEG_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_LIBPNG_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_ZLIB_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_FFMPEG_FOUND "no")  # for ViSP-third-party.txt

# open CV
IF(USE_OPENCV)
  #SET(OpenCV_FIND_QUIETLY TRUE)
  #FIND_PACKAGE(OpenCV)
  IF(OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV found")
    IF(VISP_BUILD_SHARED_LIBS AND VISP_USE_MSVC)
      ADD_DEFINITIONS("-DCVAPI_EXPORTS")
    ENDIF(VISP_BUILD_SHARED_LIBS AND VISP_USE_MSVC)
    SET(VISP_HAVE_OPENCV  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_OPENCV_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${OpenCV_LIB_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${OpenCV_LIBS})
    #LINK_DIRECTORIES(${OpenCV_LINK_DIRECTORIES})
    #MESSAGE("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
    #MESSAGE("OpenCV_LIBS = ${OpenCV_LIBS}")
    #MESSAGE("OpenCV_LINK_DIRECTORIES = ${OpenCV_LINK_DIRECTORIES}")
  ELSE(OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV not found")
    SET(VISP_HAVE_OPENCV_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(OpenCV_FOUND)
ELSE(USE_OPENCV)
  SET(OpenCV_VERSION_MAJOR 0)
  SET(OpenCV_VERSION_MINOR 0)
  SET(OpenCV_VERSION_PATCH 0)   
ENDIF(USE_OPENCV)


# gnu scientific library
IF(USE_GSL)
  #FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    MESSAGE(STATUS "GSL found")
    SET(VISP_HAVE_GSL  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_GSL_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${GSL_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${GSL_EXE_LINKER_FLAGS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${GSL_LINK_DIRECTORIES})
    LIST(APPEND VISP_EXTERN_LIBS ${GSL_LIBRARIES})
    #LINK_DIRECTORIES(${GSL_LINK_DIRECTORIES})
    #MESSAGE("GSL_INCLUDE_DIR = ${GSL_INCLUDE_DIR}")
    #MESSAGE("GSL_LINK_DIRECTORIES = ${GSL_LINK_DIRECTORIES}")
    #MESSAGE("GSL_LIBRARIES = ${GSL_LIBRARIES}")
  ELSE(GSL_FOUND)
    MESSAGE(STATUS "GSL not found")
    SET(VISP_HAVE_GSL_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

# strategy for gtk: search first gtk2. If not found search gtk1
# gtk-2 library:
IF(USE_GTK2)
   IF(UNIX OR WIN32)
    #FIND_PACKAGE(GTK2)
    IF(GTK2_FOUND)
      MESSAGE(STATUS "GTK2 found")
      SET(VISP_HAVE_GTK TRUE)  # for header vpConfig.h
      SET(VISP_HAVE_GTK_FOUND "yes")  # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${GTK2_INCLUDE_DIR})
      
      IF(UNIX)
	## ignore gtk pedantic warnings by gcc specific -isystem :
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK2_gtk_INCLUDE_PATH}")
      ENDIF(UNIX)

      LIST(APPEND VISP_EXTERN_LIBS ${GTK2_LIBRARIES})
      # MESSAGE(STATUS "DBG CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
      #MESSAGE("GTK2 includes: ${GTK2_INCLUDE_DIR}")
      #MESSAGE("GTK2 Libs: ${GTK2_LIBRARIES}")
      SET(USE_GTK1 OFF) # no need to found gtk-1
    ELSE(GTK2_FOUND)
      MESSAGE(STATUS "GTK2 not found")
      SET(VISP_HAVE_GTK_FOUND "no")  # for ViSP-third-party.txt
      #MESSAGE("GTK2 requested but not found.")
      SET(USE_GTK1 ON) # try to found gtk-1
    ENDIF(GTK2_FOUND)
#    IF(USE_GTK1)
#      MESSAGE(FATAL_ERROR "gtk1 and gtk2 cannot be used together")
#    ENDIF(USE_GTK1)
   ENDIF(UNIX OR WIN32)
ENDIF(USE_GTK2)

# gtk-1 library: is search only if gtk-2 was not found
SET(USE_GTK2 OFF)
#MESSAGE("USE_GTK1: ${USE_GTK1}")
IF(USE_GTK1)
  IF(UNIX)
    #FIND_PACKAGE(GTK)
    IF(GTK_FOUND)
      MESSAGE(STATUS "GTK1 found")
      SET(VISP_HAVE_GTK TRUE) # for header vpConfig.h
      SET(VISP_HAVE_GTK_FOUND "yes")  # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${GTK_INCLUDE_DIR})
      ## ignore gtk pedantic warnings by gcc specific -isystem :
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK_gtk_INCLUDE_PATH}")
      # MESSAGE("DBG CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
      LIST(APPEND VISP_EXTERN_LIBS ${GTK_LIBRARIES})
    ELSE(GTK_FOUND)
      MESSAGE(STATUS "GTK1 not found")
      SET(VISP_HAVE_GTK_FOUND "no")  # for ViSP-third-party.txt
      #MESSAGE("GTK requested but not found.")
      # SET(USE_GTK OFF)
    ENDIF(GTK_FOUND)
    IF(USE_GTK2)
      MESSAGE(FATAL_ERROR "gtk1 and gtk2 cannot be used together 1" )
    ENDIF(USE_GTK2)
  ENDIF(UNIX)
ENDIF(USE_GTK1)

# Under Windows, usage of Direct3D9
IF(USE_DIRECT3D)
  #FIND_PACKAGE(DIRECT3D)
  IF(DIRECT3D_FOUND)
    MESSAGE(STATUS "Direct3D found")
    SET(VISP_HAVE_D3D9 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_D3D9_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_D3D9= ${VISP_HAVE_D3D9}")
    #MESSAGE("DBG DIRECT3D_INCLUDE_DIR=${DIRECT3D_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${DIRECT3D_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${DIRECT3D_LIBRARIES})
    #MESSAGE("DIRECT3D_LIBRARIES = ${DIRECT3D_LIBRARIES}")
  ELSE(DIRECT3D_FOUND)
    MESSAGE(STATUS "Direct3D not found")
    SET(VISP_HAVE_D3D9_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DIRECT3D_FOUND)
ENDIF(USE_DIRECT3D)

# Under Windows, usage of GDI (Graphics Device Interface)
IF(USE_GDI)
  #FIND_PACKAGE(GDI)
  IF(GDI_FOUND)
    MESSAGE(STATUS "GDI found")
    SET(VISP_HAVE_GDI TRUE) # for header vpConfig.h
    SET(VISP_HAVE_GDI_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_GDI= ${VISP_HAVE_GDI}")
    LIST(APPEND VISP_EXTERN_LIBS ${GDI_LIBRARIES})
    #MESSAGE("GDI_LIBRARIES = ${GDI_LIBRARIES}")
  ELSE(GDI_FOUND)
    MESSAGE(STATUS "GDI not found")
    SET(VISP_HAVE_GDI_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(GDI_FOUND)
ENDIF(USE_GDI)

# zlib
IF(USE_ZLIB)
  FIND_PACKAGE(ZLIB)
  IF(ZLIB_FOUND)
    MESSAGE(STATUS "zlib found")
    SET(VISP_HAVE_ZLIB  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_ZLIB_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${ZLIB_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${ZLIB_LIBRARY})

  ELSE(ZLIB_FOUND)
    MESSAGE(STATUS "zlib not found")
    SET(VISP_HAVE_ZLIB_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(ZLIB_FOUND)
ENDIF(USE_ZLIB)

# Libjpeg
IF(USE_LIBJPEG)
  #FIND_PACKAGE(LIBJPEG)
  #FIND_PACKAGE(JPEG)
  #MESSAGE("test ${JPEG_FOUND}")
  IF(LIBJPEG_FOUND)
    MESSAGE(STATUS "Libjpeg found")
    SET(VISP_HAVE_LIBJPEG  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_LIBJPEG_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${LIBJPEG_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${LIBJPEG_LIBRARIES})

    # If Qt3 and libjpeg are used, we add here QT_CLEAN_NAMESPACE define to 
    # handle a compilation error "conflicting declaration 'typedef long int 
    # INT32'"
    IF(VISP_HAVE_QT3_FOUND)
      LIST(APPEND VISP_DEFS "-DQT_CLEAN_NAMESPACE")
      # MESSAGE("add -DQT_CLEAN_NAMESPACE")
    ENDIF(VISP_HAVE_QT3_FOUND)

  ELSE(LIBJPEG_FOUND)
    MESSAGE(STATUS "Libjpeg not found")
    SET(VISP_HAVE_LIBJPEG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(LIBJPEG_FOUND)
ENDIF(USE_LIBJPEG)

# Libpng
IF(USE_LIBPNG)
  #FIND_PACKAGE(LIBPNG)
  IF(LIBPNG_FOUND)
    MESSAGE(STATUS "Libpng found")
    SET(VISP_HAVE_LIBPNG  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_LIBPNG_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${LIBPNG_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${LIBPNG_LIBRARIES})

  ELSE(LIBPNG_FOUND)
    MESSAGE(STATUS "Libpng not found")
    SET(VISP_HAVE_LIBPNG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(LIBPNG_FOUND)
ENDIF(USE_LIBPNG)

# FFMPEG
IF(USE_FFMPEG)
  #FIND_PACKAGE(FFMPEG)
  IF(FFMPEG_FOUND)
    MESSAGE(STATUS "FFMPEG found")
    SET(VISP_HAVE_FFMPEG  TRUE)  # for header vpConfig.h
    IF(FFMPEG_WITH_DECODE_VIDEO2_FOUND)
      SET(VISP_HAVE_FFMPEG_WITH_DECODE_VIDEO2 TRUE)  # for header vpConfig.h
    ENDIF(FFMPEG_WITH_DECODE_VIDEO2_FOUND)
    SET(VISP_HAVE_FFMPEG_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${FFMPEG_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBS ${FFMPEG_LIBRARIES})
  ELSE(LIBFFMPEG_FOUND)
    MESSAGE(STATUS "FFMPEG not found")
    SET(VISP_HAVE_FFMPEG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(FFMPEG_FOUND)
ENDIF(USE_FFMPEG)

#----------------------------------------------------------------------
# Specific hardware : framegrabbers and cameras
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_V4L2_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_DIRECTSHOW_FOUND "no")  # for ViSP-third-party.txt

# library for firewire cameras
IF(USE_DC1394_1 OR USE_DC1394_2)
  #FIND_PACKAGE(DC1394)
  IF(DC1394_FOUND)
    SET(VISP_HAVE_DC1394 TRUE) # for header vpConfig.h
    IF(DC1394_2_FOUND AND USE_DC1394_2)
      MESSAGE(STATUS "libdc1394-2.x found")
      SET(VISP_HAVE_DC1394_2 TRUE) # for header vpConfig.h
      SET(VISP_HAVE_DC1394_2_FOUND "yes")  # for ViSP-third-party.txt
      IF(DC1394_CAMERA_ENUMERATE_FOUND)
        SET(VISP_HAVE_DC1394_2_CAMERA_ENUMERATE TRUE) # for header vpConfig.h
      ENDIF(DC1394_CAMERA_ENUMERATE_FOUND)
      IF(DC1394_FIND_CAMERAS_FOUND)
        SET(VISP_HAVE_DC1394_2_FIND_CAMERAS TRUE) # for header vpConfig.h
      ENDIF(DC1394_FIND_CAMERAS_FOUND)
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${DC1394_2_INCLUDE_DIR})
      LIST(APPEND VISP_EXTERN_LIBS ${DC1394_2_LIBRARY})

      #MESSAGE("DBG VISP_HAVE_DC1394_2= ${VISP_HAVE_DC1394_2}")
    ELSE(DC1394_2_FOUND AND USE_DC1394_2)
      MESSAGE(STATUS "libdc1394-2.x not found")
      SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(DC1394_2_FOUND AND USE_DC1394_2)
    
    IF(DC1394_1_FOUND AND USE_DC1394_1)
      MESSAGE(STATUS "libdc1394-1.x found")
      SET(VISP_HAVE_DC1394_1 TRUE) # for header vpConfig.h
      SET(VISP_HAVE_DC1394_1_FOUND "yes")  # for ViSP-third-party.txt
      #MESSAGE("DBG VISP_HAVE_DC1394_1= ${VISP_HAVE_DC1394_1}")
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${DC1394_1_INCLUDE_DIR})
      LIST(APPEND VISP_EXTERN_LIBS ${DC1394_1_LIBRARY})
    ELSE(DC1394_1_FOUND AND USE_DC1394_1)
      MESSAGE(STATUS "libdc1394-1.x not found")
      SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(DC1394_1_FOUND AND USE_DC1394_1)
    #MESSAGE("DBG DC1394_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")
    

    IF(APPLE)
      #SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} IOkit "-framework Carbon")
      LIST(APPEND VISP_EXTERN_LIBS "-framework IOkit -framework Carbon")
    ENDIF(APPLE)
     #MESSAGE("DC1394_LIBRARIES = ${DC1394_LIBRARIES}")
  ELSE(DC1394_FOUND)
    MESSAGE(STATUS "libdc1394-1.x not found")
    MESSAGE(STATUS "libdc1394-2.x not found")
    SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
    SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
    #MESSAGE("Firewire dc1394 requested but not found. Turn off USE_DC1394!")
    # SET(USE_DC1394 OFF)
  ENDIF(DC1394_FOUND)
ENDIF(USE_DC1394_1 OR USE_DC1394_2)

# library for Video For Linux Two framegrabber devices
IF(USE_V4L2)
  #FIND_PACKAGE(V4L2)
  IF(V4L2_FOUND)
    MESSAGE(STATUS "V4l2 found")
    SET(VISP_HAVE_V4L2 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_V4L2_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_V4L2= ${VISP_HAVE_V4L2}")
     #MESSAGE("DBG V4L2_INCLUDE_DIR=${V4L2_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${V4L2_INCLUDE_DIR})
  ELSE(V4L2_FOUND)
    MESSAGE(STATUS "V4l2 not found")
    SET(VISP_HAVE_V4L2_FOUND "no")  # for ViSP-third-party.txt
    #MESSAGE("Video For Linux Two requested but not found.")
    # SET(USE_V4L2 OFF)
  ENDIF(V4L2_FOUND)
ENDIF(USE_V4L2)

# Under Windows, usage of Direct Show
IF(USE_DIRECTSHOW)
  #FIND_PACKAGE(DIRECTSHOW)
  IF(DIRECTSHOW_FOUND)
    MESSAGE(STATUS "DirectShow found")
    SET(VISP_HAVE_DIRECTSHOW TRUE) # for header vpConfig.h
    SET(VISP_HAVE_DIRECTSHOW_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_DIRECTSHOW= ${VISP_HAVE_PTU46}")
    #MESSAGE("DBG DIRECTSHOW_INCLUDE_DIR=${DIRECTSHOW_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${DIRECTSHOW_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${DIRECTSHOW_LIBRARIES})
    #MESSAGE("DIRECTSHOW_LIBRARIES = ${DIRECTSHOW_LIBRARIES}")
  ELSE(DIRECTSHOW_FOUND)
    MESSAGE(STATUS "DirectShow not found")
    SET(VISP_HAVE_DIRECTSHOW_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DIRECTSHOW_FOUND)
ENDIF(USE_DIRECTSHOW)


#----------------------------------------------------------------------
# The wireframe simulator
#----------------------------------------------------------------------
# include all the wireframe scene files
INCLUDE(${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/CMakeWireframeScenesList.cmake)

SET(WIREFRAME_SCENES_LIST "")
foreach(SCENE ${WIREFRAME_SCENES})
  set(WIREFRAME_SCENES_SRC "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}")
  set(WIREFRAME_SCENES_DST "${VISP_BINARY_DIR}/data/wireframe-simulator/${SCENE}")
  IF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${WIREFRAME_SCENES_DST}
    COMMAND ${CMAKE_COMMAND} -E copy ${WIREFRAME_SCENES_SRC} ${WIREFRAME_SCENES_DST}
    DEPENDS ${WIREFRAME_SCENES_SRC}
    )
  SET(WIREFRAME_SCENES_LIST ${WIREFRAME_SCENES_LIST} ${WIREFRAME_SCENES_DST})
  ELSE(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
    file(COPY ${WIREFRAME_SCENES_SRC}
      DESTINATION data/wireframe-simulator
      FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE
      )
  ENDIF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
endforeach(SCENE)

IF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_TARGET(SCENE ALL
    DEPENDS ${WIREFRAME_SCENES_LIST}
    )
ENDIF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)

SET(VISP_SCENES_DIR ${VISP_BINARY_DIR}/data/wireframe-simulator)

#----------------------------------------------------------------------
# Robot simulators
#----------------------------------------------------------------------
INCLUDE(${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/CMakeRobotArmsList.cmake)

SET(ROBOT_ARMS_LIST "")
foreach(SCENE_ROBOT ${ROBOT_ARMS_SCENES})
  set(ROBOT_ARMS_SRC "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE_ROBOT}")
  set(ROBOT_ARMS_DST "${VISP_BINARY_DIR}/data/robot-simulator/${SCENE_ROBOT}")
  IF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${ROBOT_ARMS_DST}
    COMMAND ${CMAKE_COMMAND} -E copy ${ROBOT_ARMS_SRC} ${ROBOT_ARMS_DST}
    DEPENDS ${ROBOT_ARMS_SRC}
    )
  SET(ROBOT_ARMS_LIST ${ROBOT_ARMS_LIST} ${ROBOT_ARMS_DST})
  ELSE(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
    file(COPY ${ROBOT_ARMS_SRC}
      DESTINATION data/robot-simulator
      FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE
      )
  ENDIF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
endforeach(SCENE_ROBOT)

IF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_TARGET(SCENE_ROBOT ALL
    DEPENDS ${ROBOT_ARMS_LIST}
    )
ENDIF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)

SET(VISP_ROBOT_ARMS_DIR ${VISP_BINARY_DIR}/data/robot-simulator)


#----------------------------------------------------------------------
# Specific hardware : robots
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_AFMA4_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_AFMA6_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_BICLOPS_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_PTU46_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_VIPER850_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_CYCAB_FOUND "no")  # for ViSP-third-party.txt

# library for Irisa's Afma4 cylindrical robot
IF(USE_AFMA4)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma4 found")
    SET(VISP_HAVE_AFMA4 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_AFMA4_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${RAW1394_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${CALINUX_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${IRISA_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma4 not found")
    MESSAGE("Afma4 robot requested but not found. Turn off USE_AFMA4 option!")
    SET(VISP_HAVE_AFMA4_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_AFMA4 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_AFMA4)

# library for Irisa's Afma6 cartesian robot
IF(USE_AFMA6)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma6 found")
    SET(VISP_HAVE_AFMA6 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_AFMA6_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${RAW1394_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${CALINUX_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${IRISA_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma6 not found")
    MESSAGE("Afma6 robot requested but not found. Turn off USE_AFMA6 option!")
    SET(VISP_HAVE_AFMA6_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_AFMA6 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_AFMA6)

# library for Irisa's Viper850 cartesian robot
IF(USE_VIPER850)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper850 found")
    SET(VISP_HAVE_VIPER850 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_VIPER850_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${RAW1394_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${CALINUX_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${IRISA_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBS ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper S850 not found")
    MESSAGE("Viper S850 robot requested but not found. Turn off USE_VIPER850 option!")
    SET(VISP_HAVE_VIPER850_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_VIPER850 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_VIPER850)

# library for Biclops pan-tilt head
IF(USE_BICLOPS)
#   FIND_PACKAGE(BICLOPS)
  IF(BICLOPS_FOUND)
    MESSAGE(STATUS "Biclops found")
    SET(VISP_HAVE_BICLOPS TRUE) # for header vpConfig.h
    SET(VISP_HAVE_BICLOPS_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_BICLOPS= ${VISP_HAVE_BICLOPS}")
     #MESSAGE("DBG BICLOPS_INCLUDE_DIR=${BICLOPS_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${BICLOPS_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${BICLOPS_LIBRARIES})
     #MESSAGE("BICLOPS_LIBRARIES = ${BICLOPS_LIBRARIES}")
  ELSE(BICLOPS_FOUND)
    MESSAGE(STATUS "Biclops not found")
    MESSAGE("Biclops requested but not found. Turn off USE_BICLOPS option!")
    SET(VISP_HAVE_BICLOPS_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_BICLOPS OFF)
  ENDIF(BICLOPS_FOUND)
ENDIF(USE_BICLOPS)

# library for Ptu-46 pan-tilt head
IF(USE_PTU46)
#   FIND_PACKAGE(PTU46)
  IF(PTU46_FOUND)
    MESSAGE(STATUS "Ptu46 found")
    SET(VISP_HAVE_PTU46 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_PTU46_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_PTU46= ${VISP_HAVE_PTU46}")
     #MESSAGE("DBG PTU46_INCLUDE_DIR=${PTU46_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${PTU46_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${PTU46_LIBRARIES})
     #MESSAGE("PTU46_LIBRARIES = ${PTU46_LIBRARIES}")
  ELSE(PTU46_FOUND)
    MESSAGE(STATUS "Ptu46 not found")
    MESSAGE("Ptu-46 requested but not found. Turn off USE_PTU46 option!")
    SET(VISP_HAVE_PTU46_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_PTU46 OFF)
  ENDIF(PTU46_FOUND)
ENDIF(USE_PTU46)

# library for Cycab car-like mobile robot
IF(USE_CYCAB)
  IF(CycabTk_FOUND)
    IF(CycabTk_NEW_FOUND)
      SET(VISP_HAVE_CYCABTK TRUE) # for header vpConfig.h
    ENDIF(CycabTk_NEW_FOUND)
    IF(CycabTk_OLD_FOUND)
      SET(VISP_HAVE_CYCABTK_OLD TRUE) # for header vpConfig.h
    ENDIF(CycabTk_OLD_FOUND)
    SET(VISP_HAVE_CYCAB TRUE) # for header vpConfig.h
    SET(VISP_HAVE_CYCAB_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${CycabTk_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${CycabTk_LIBRARIES})
  ELSE(CycabTk_FOUND)
    MESSAGE(STATUS "Cycab interface not found")
    MESSAGE("Cycab requested but not found. Turn off USE_CYCAB option!")
    SET(VISP_HAVE_CYCAB_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(CycabTk_FOUND)
ENDIF(USE_CYCAB)


#----------------------------------------------------------------------
# add html-doc target for doxygen documentation building
#----------------------------------------------------------------------
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)
  SET(VISP_HAVE_DOXYGEN "yes")        # for header vpConfig.h
  SET(VISP_HAVE_DOXYGEN_FOUND "yes")  # for ViSP-third-party.txt
  IF(DOXYGEN_DOT_EXECUTABLE)
    SET(VISP_HAVE_DOT "yes")        # for header vpConfig.h
    SET(VISP_HAVE_DOT_FOUND "yes")  # for ViSP-third-party.txt
  ELSE(DOXYGEN_DOT_EXECUTABLE)
    SET(VISP_HAVE_DOT "no")        # for header vpConfig.h
    SET(VISP_HAVE_DOT_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DOXYGEN_DOT_EXECUTABLE)
  ## we need latex for doxygen because of the formulas
  FIND_PACKAGE(LATEX)
  IF    (NOT LATEX_COMPILER)
    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user interaction on doxy run.")
  ENDIF (NOT LATEX_COMPILER)
  IF    (NOT MAKEINDEX_COMPILER)
    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  ENDIF (NOT MAKEINDEX_COMPILER)
  IF    (NOT DVIPS_CONVERTER)
    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  ENDIF (NOT DVIPS_CONVERTER)
   
  CONFIGURE_FILE(${VISP_SOURCE_DIR}/doc/config-doxygen.in
    ${VISP_DOC_DIR}/config-doxygen
    @ONLY )

  CONFIGURE_FILE(${VISP_SOURCE_DIR}/doc/mainpage.doc.in
    ${VISP_SOURCE_DIR}/doc/mainpage.doc
    @ONLY )
  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen) 
ELSE(DOXYGEN_FOUND)
  SET(VISP_HAVE_DOXYGEN "no")        # for header vpConfig.h
  SET(VISP_HAVE_DOXYGEN_FOUND "no")  # for ViSP-third-party.txt
  SET(VISP_HAVE_DOT_FOUND "no")      # for ViSP-third-party.txt
ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
# With Visual Studio 2005, Microsoft deprecates the standard C library, for
# example fopen() and sprintf(), to non-portable functions fopen_s() and
# sprintf_s(). These functions are considered by Microsoft more secure. This is
# a worthwhile exercise ! The use of these deprecated functions causes a lot of
# warnings. To suppress it, we add the _CRT_SECURE_NO_DEPRECATE preprocessor
# definition
IF(WIN32)
  IF(MSVC80 OR MSVC90 OR MSVC10)
    #MESSAGE("Add -D_CRT_SECURE_NO_DEPRECATE ")
    LIST(APPEND VISP_DEFS "-D_CRT_SECURE_NO_DEPRECATE")
  ENDIF(MSVC80 OR MSVC90 OR MSVC10)
ENDIF(WIN32)

#MESSAGE("Definitions: ${VISP_DEFS}")
IF(VISP_DEFS)
  LIST(REMOVE_DUPLICATES VISP_DEFS)
ENDIF(VISP_DEFS)
#MESSAGE("Definitions without duplicates: ${VISP_DEFS}")
ADD_DEFINITIONS(${VISP_DEFS})

#----------------------------------------------------------------------
# Add intern and third party include dirs
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_INCLUDE_DIR: ${VISP_EXTERN_INCLUDE_DIR}")
IF(VISP_EXTERN_INCLUDE_DIR)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_INCLUDE_DIR)
ENDIF(VISP_EXTERN_INCLUDE_DIR)
#MESSAGE("VISP_EXTERN_INCLUDE_DIR without duplicates: ${VISP_EXTERN_INCLUDE_DIR}")
INCLUDE_DIRECTORIES(${VISP_INTERN_INCLUDE_DIR} ${VISP_EXTERN_INCLUDE_DIR})

#----------------------------------------------------------------------
# Add link directories
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_LINK_DIR: ${VISP_EXTERN_LINK_DIR}")
IF(VISP_EXTERN_LINK_DIR)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_LINK_DIR)
ENDIF(VISP_EXTERN_LINK_DIR)
#MESSAGE("VISP_EXTERN_LINK_DIR without duplicates: ${VISP_EXTERN_LINK_DIR}")
LINK_DIRECTORIES(${VISP_EXTERN_LINK_DIR})

#----------------------------------------------------------------------
# Cleanify the libraries list
#----------------------------------------------------------------------
IF(APPLE)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_LIBS)
ENDIF()

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
#    visp-config --cflags ...
#----------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/GenerateConfigScript.cmake)

#----------------------------------------------------------------------
# customize install target
#----------------------------------------------------------------------
# install rule for visp-config shell script
IF (UNIX)
  INSTALL(FILES ${BINARY_OUTPUT_PATH}/visp-config
    DESTINATION bin
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
    OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    OWNER_WRITE)
ELSE(UNIX)
  INSTALL(FILES ${BINARY_OUTPUT_PATH}/visp-config.bat
    DESTINATION bin
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
    OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    OWNER_WRITE)
ENDIF (UNIX)

# install rule for vpConfig.h
INSTALL(FILES ${VISP_BINARY_DIR}/include/visp/vpConfig.h
  DESTINATION include/visp
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
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
  LIST(APPEND FILE_TO_CLEAN ${CMAKE_SOURCE_DIR}/include/visp/${headerName})
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

# Installation for ViSPConfig.cmake
# case 1: <binary dir>/ViSPConfig.cmake for use with make
# case 2: <install dir>/ViSPConfig.cmake for use with make install

# case 1
#--------------
set(VISP_ROOT_DIR_CONFIGCMAKE ${VISP_BINARY_DIR})
set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE ${VISP_BINARY_DIR})
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/VISPConfig.cmake
  IMMEDIATE @ONLY)

# Just to copy ./CMakeModule/VISPUse.cmake.in <binary dir>/UseVISP.cmake
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/VISPUse.cmake
  IMMEDIATE @ONLY)

# Export our build settings and library dependencies for use by the third party
# user projects
INCLUDE(${CMAKE_ROOT}/Modules/CMakeExportBuildSettings.cmake)
CMAKE_EXPORT_BUILD_SETTINGS(
  ${VISP_BINARY_DIR}/VISPBuildSettings.cmake
  )
EXPORT_LIBRARY_DEPENDENCIES(
  ${VISP_BINARY_DIR}/VISPLibraryDepends.cmake
  )

# Generate the package dependent file include/visp/vpConfig.h
CONFIGURE_FILE(${VISP_SOURCE_DIR}/ViSP-third-party.txt.cmake
  ${VISP_BINARY_DIR}/ViSP-third-party.txt
)


# case 2
#--------------
set(VISP_ROOT_DIR_CONFIGCMAKE ${CMAKE_INSTALL_PREFIX})
IF(UNIX)
  set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE ${CMAKE_INSTALL_PREFIX}/share/visp)
ELSE()
  set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE ${CMAKE_INSTALL_PREFIX})
ENDIF()
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/install/VISPConfig.cmake
  IMMEDIATE @ONLY)

# Just to copy ./CMakeModule/VISPUse.cmake.in in <install dir>/UseVISP.cmake
CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/install/VISPUse.cmake
  IMMEDIATE @ONLY)

# Export our build settings and library dependencies for use by the third party
# user projects
INCLUDE(${CMAKE_ROOT}/Modules/CMakeExportBuildSettings.cmake)
CMAKE_EXPORT_BUILD_SETTINGS(
  ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
  )
EXPORT_LIBRARY_DEPENDENCIES(
  ${VISP_BINARY_DIR}/install/VISPLibraryDepends.cmake
  )

# Install the packaging files for use by FIND_PACKAGE(VISP) in user projects
IF(UNIX)
  INSTALL(FILES
    ${VISP_BINARY_DIR}/install/VISPUse.cmake
    ${VISP_BINARY_DIR}/install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
    ${VISP_BINARY_DIR}/install/VISPLibraryDepends.cmake
    ${VISP_BINARY_DIR}/ViSP-third-party.txt
    DESTINATION share/visp
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
ELSE()
  INSTALL(FILES
    ${VISP_BINARY_DIR}/install/VISPUse.cmake
    ${VISP_BINARY_DIR}/install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
    ${VISP_BINARY_DIR}/install/VISPLibraryDepends.cmake
    DESTINATION .
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
ENDIF()

#----------------------------------------------------------------------
# For Dart server and tests 
# We use CDash set through CTestConfig.cmake file
# Dashboards are sent to http://cdash.irisa.fr/CDash/index.php?project=ViSP
#----------------------------------------------------------------------
ENABLE_TESTING()
INCLUDE(Dart)

#----------------------------------------------------------------------
# For CPack packaging tool
#----------------------------------------------------------------------

option(BUILD_PACKAGE "Configure ViSP packaging" OFF)
if (BUILD_PACKAGE)
  include (${VISP_SOURCE_DIR}/CMakeModules/CPackConfig.cmake)
endif(BUILD_PACKAGE)


#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(src)
IF(BUILD_DEMOS)
  SUBDIRS(demo)
ENDIF(BUILD_DEMOS)
IF(BUILD_EXAMPLES)
  SUBDIRS(example)
ENDIF(BUILD_EXAMPLES)
IF(BUILD_TESTING)

  #
  # Test coverage specific code
  #
  # set the default value
  SET(BUILD_TEST_COVERAGE FALSE)

  IF(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS AND CMAKE_BUILD_TYPE MATCHES "Debug")

    OPTION(BUILD_TEST_COVERAGE "Enable test coverage." OFF)
  ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS AND CMAKE_BUILD_TYPE MATCHES "Debug")

  IF(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    MESSAGE("Add -ftest-coverage -fprofile-arcs ")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
  ENDIF(BUILD_TEST_COVERAGE)

  SUBDIRS(test)
ENDIF(BUILD_TESTING)
