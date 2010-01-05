#############################################################################
#
# $Id$
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
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_VERSION_MAJOR "2")
SET(VISP_VERSION_MINOR "4")
SET(VISP_VERSION_PATCH "4")
SET(VISP_VERSION "${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}.${VISP_VERSION_PATCH}")

SET(VISP_INCLUDE_DIR "${VISP_BINARY_DIR}/include/visp")
SET(VISP_DOC_DIR "${VISP_BINARY_DIR}/doc")

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

# where are user-specific cmake modules
SET(CMAKE_MODULE_PATH ${VISP_SOURCE_DIR}/CMakeModules)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${VISP_INCLUDE_DIR})


#--------------------------------------------------------------------
# Option management
#--------------------------------------------------------------------

# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build ViSP with shared libraries." OFF)
# Build examples as an option.
OPTION(BUILD_EXAMPLES "Build ViSP examples." ON)
# Build deprecated functions as an option.
OPTION(BUILD_DEPRECATED_FUNCTIONS "Build deprecated functionalities." ON)

IF(BUILD_DEPRECATED_FUNCTIONS)
  SET(VISP_BUILD_DEPRECATED_FUNCTIONS TRUE) # for header vpConfig.h
ENDIF(BUILD_DEPRECATED_FUNCTIONS)

IF (UNIX)
   OPTION(USE_COIN_SIMULATOR    "Compile ViSP with simulation capabilities based on Coin3D suite" OFF)

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

  FIND_PACKAGE(ITIFG8)
  IF(ITIFG8_FOUND)
    OPTION(USE_ITIFG8 "Compile ViSP with itifg8 framegrabber usage at Irisa" ON)
  ELSE(ITIFG8_FOUND)
    SET(USE_ITIFG8 OFF)
  ENDIF(ITIFG8_FOUND)

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

FIND_PACKAGE(XML2)
IF(XML2_FOUND)
  OPTION(USE_XML2    "Compile ViSP with the xml2 library" ON)
ELSE(XML2_FOUND)
  SET(USE_XML2 OFF)
ENDIF(XML2_FOUND)

SET(OpenCV_FIND_QUIETLY TRUE)
FIND_PACKAGE(OpenCV)
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

FIND_PACKAGE(LIBPLOTTER)
IF(LIBPLOTTER_FOUND)
  OPTION(USE_LIBPLOTTER    "Compile ViSP with the libplotter library" ON)
ELSE(LIBPLOTTER_FOUND)
  SET(USE_LIBPLOTTER OFF)
ENDIF(LIBPLOTTER_FOUND)

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
SET(USE_X11        ON) # For Linux/OSX display
#SET(USE_XML2       ON) # For the Xml Parser (vpCameraParameters)
#SET(USE_COIN_SIMULATOR       ON) # For the simulator
SET(USE_SOWIN      ON) # For the simulator viewer
SET(USE_SOQT       ON) # For the simulator viewer
SET(USE_SOXT       ON) # For the simulator viewer
SET(USE_PTHREAD    ON) # For Biclops/Simulator
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
#SET(USE_LIBPLOTTER ON) # For plotting curves
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

#----------------------------------------------------------------------
# Unix libraries :  X11, pthread
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_X11_FOUND "no")  # for ViSP-third-party.txt

IF(UNIX)
  #--------------------------------------------------------------------
  # X11 lib
  #--------------------------------------------------------------------
  IF(USE_X11)
    FIND_PACKAGE(X11)
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
ENDIF(UNIX)

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

IF(USE_COIN_SIMULATOR)
  INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
  IF(OPENGL_FOUND)
    FIND_PACKAGE(COIN)

    IF (COIN_FOUND)
      SET(VISP_HAVE_COIN_FOUND "yes")   # for ViSP-third-party.txt
    ENDIF (COIN_FOUND)

    IF(WIN32 AND USE_SOWIN)
      FIND_PACKAGE(SOWIN)
    ENDIF(WIN32 AND USE_SOWIN)
    IF (WIN32 AND SOWIN_FOUND AND COIN_FOUND)
      #MESSAGE("COIN_LIBRARY_RELEASE = ${COIN_LIBRARY_RELEASE}")
      #MESSAGE("COIN_LIBRARY_DEBUG = ${COIN_LIBRARY_DEBUG}")
      # OPenGL and SoWin are found
      # We first manage OpenGL
      SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OPENGL_INCLUDE_DIR})
      LIST(APPEND VISP_EXTERN_LIBS ${OPENGL_gl_LIBRARY})

      # We manage SoWin and Coin
      SET(VISP_HAVE_SOWIN TRUE)         # for header vpConfig.h
      SET(VISP_HAVE_COIN TRUE)          # for header vpConfig.h
      SET(VISP_HAVE_SOWIN_FOUND "yes")  # for ViSP-third-party.txt
      
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${SOWIN_INCLUDE_DIR})
      #MESSAGE("SOWIN_INCLUDE_DIR = ${SOWIN_INCLUDE_DIR}")
      LIST(APPEND VISP_DEFS "-DSOWIN_DLL")
      LIST(APPEND VISP_DEFS "-DCOIN_DLL")
      IF(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)
	  #MESSAGE("SOWIN AND COIN RELEASE")
	  LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${COIN_INCLUDE_DIR})  
          LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARY_RELEASE})
          LIST(APPEND VISP_EXTERN_LIBS ${SOWIN_LIBRARY_RELEASE})
      ELSE(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)
        IF(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
	  #MESSAGE("SOWIN AND COIN DEBUG")
	  LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${COIN_INCLUDE_DIR})  
          LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARY_DEBUG})
          LIST(APPEND VISP_EXTERN_LIBS ${SOWIN_LIBRARY_DEBUG})
        ELSE(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
	  #MESSAGE("SOWIN AND COIN NOT RELEASE OR DEBUG")
          SET(VISP_HAVE_SOWIN FALSE)  # for header vpConfig.h
          SET(VISP_HAVE_COIN FALSE)  # for header vpConfig.h
        ENDIF(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
      ENDIF(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)

    ENDIF(WIN32 AND SOWIN_FOUND AND COIN_FOUND)

    IF(USE_SOQT AND NOT VISP_HAVE_COIN)
      IF(COIN_FOUND)
        FIND_PACKAGE(SOQT)
        IF(SOQT_FOUND)
	  #MESSAGE("Soqt found")
	  SET(VISP_HAVE_SOQT_FOUND "yes")      # for ViSP-third-party.txt
          INCLUDE (${CMAKE_ROOT}/Modules/FindQt.cmake)    
          IF (QT_FOUND)
	    #MESSAGE("Qt found")
	    # OpenGL, QT and SOQT are found
	    # We first manage OpenGL
            SET(VISP_HAVE_COIN   TRUE) # for header vpConfig.h
	    SET(VISP_HAVE_SOQT   TRUE) # for header vpConfig.h 
	    SET(VISP_HAVE_QT     TRUE) # for header vpConfig.h
	    SET(VISP_HAVE_OPENGL TRUE) # for header vpConfig.h

	    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${SOQT_INCLUDE_DIR})
	    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${QT_INCLUDE_DIR})
	    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${QT_INCLUDES})
	    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OPENGL_INCLUDE_DIR})

	    # Add first Coin and SoQt libraries
	    IF(WIN32)
	      #MESSAGE(STATUS "SOQT_INCLUDE_DIR = ${SOQT_INCLUDE_DIR}")
	      LIST(APPEND VISP_DEFS "-DSOQT_DLL")
	      LIST(APPEND VISP_DEFS "-DCOIN_DLL")
	      IF(SOQT_LIBRARY_RELEASE)
	        IF(COIN_LIBRARY_RELEASE)
	          LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARY_RELEASE})
	          LIST(APPEND VISP_EXTERN_LIBS ${SOQT_LIBRARY_RELEASE})
	        ENDIF(COIN_LIBRARY_RELEASE)
	      ELSE(SOQT_LIBRARY_RELEASE)
	        IF(COIN_LIBRARY_DEBUG)
	          LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARY_DEBUG})
	          LIST(APPEND VISP_EXTERN_LIBS ${SOQT_LIBRARY_DEBUG})
	        ELSE(COIN_LIBRARY_DEBUG)
	          SET(VISP_HAVE_SOQT FALSE)  # for header vpConfig.h
                  SET(VISP_HAVE_COIN FALSE)  # for header vpConfig.h
	        ENDIF(COIN_LIBRARY_DEBUG)
	      ENDIF(SOQT_LIBRARY_RELEASE)
	    ELSE(WIN32)
	      LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARIES})
	      LIST(APPEND VISP_EXTERN_LIBS ${SOQT_LIBRARIES})
	      #MESSAGE("COIN_LIBRARIES = ${COIN_LIBRARIES}")
	      #MESSAGE("SOQT_LIBRARIES = ${SOQT_LIBRARIES}")
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
	        LIST(APPEND VISP_EXTERN_LIBS ${QT_QTGUI_LIBRARY_RELEASE})
		LIST(APPEND VISP_EXTERN_LIBS ${QT_QTCORE_LIBRARY_RELEASE})
	      ELSEIF (QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
	        LIST(APPEND VISP_EXTERN_LIBS ${QT_QTGUI_LIBRARY_DEBUG})
		LIST(APPEND VISP_EXTERN ${QT_QTCORE_LIBRARY_DEBUG})
	      ELSE (QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
	        SET(VISP_HAVE_QT FALSE)
	      ENDIF (QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
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

	    # We add now OpenGL librairies
	    LIST(APPEND VISP_EXTERN_LIBS ${OPENGL_gl_LIBRARY})

          ENDIF (QT_FOUND)
        ENDIF(SOQT_FOUND)
      ENDIF(COIN_FOUND)
    ENDIF(USE_SOQT AND NOT VISP_HAVE_COIN)
  
    #MESSAGE(STATUS "QT_LIBRARIES no found")
    IF(USE_SOXT AND NOT VISP_HAVE_COIN)
      FIND_PACKAGE(SOXT)
    ENDIF(USE_SOXT AND NOT VISP_HAVE_COIN)

    IF (UNIX AND SOXT_FOUND AND COIN_FOUND)
      # OPenGL and SoXt are found
      # We first manage OpenGL
      SET(VISP_HAVE_SOXT_FOUND "yes")       # for ViSP-third-party.txt
      SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OPENGL_INCLUDE_DIR})

      LIST(APPEND VISP_EXTERN_LIBS ${COIN_LIBRARIES})
      LIST(APPEND VISP_EXTERN_LIBS ${SOXT_LIBRARIES})
      LIST(APPEND VISP_EXTERN_LIBS ${OPENGL_gl_LIBRARY})
      
      # We manage SoXt and Coin
      SET(VISP_HAVE_SOXT TRUE)  # for header vpConfig.h
      SET(VISP_HAVE_COIN TRUE)  # for header vpConfig.h
      
    ENDIF(UNIX AND SOXT_FOUND AND COIN_FOUND)
    IF(VISP_HAVE_COIN)
      MESSAGE(STATUS "Coin found")
      SET(VISP_HAVE_SIMULATOR_FOUND "yes")  # for ViSP-third-party.txt
    ELSE(VISP_HAVE_COIN)
      MESSAGE(STATUS "Coin not found")
      SET(VISP_HAVE_SIMULATOR_FOUND "no")  # for ViSP-third-party.txt
      IF (NOT DESIRED_QT_VERSION MATCHES 0)
        # If QT version not set by the user, we don't print the following 
	# message
        MESSAGE("Simulation capabilities based on Coin3D suite requested but not found. Turn off USE_COIN_SIMULATOR option!")
      ENDIF (NOT DESIRED_QT_VERSION MATCHES 0)
    ENDIF(VISP_HAVE_COIN)
  ELSE(OPENGL_FOUND)
    #MESSAGE(STATUS "OPENGL_LIBRARIES no found")
  ENDIF(OPENGL_FOUND)
ENDIF(USE_COIN_SIMULATOR)

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
  FIND_PACKAGE(PTHREAD)
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
SET(VISP_HAVE_LIBPLOTTER_FOUND "no")  # for ViSP-third-party.txt
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
    SET(VISP_HAVE_OPENCV  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_OPENCV_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${OpenCV_EXE_LINKER_FLAGS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${OpenCV_LINK_DIRECTORIES})
    LIST(APPEND VISP_EXTERN_LIBS ${OpenCV_LIBRARIES})
    #LINK_DIRECTORIES(${OpenCV_LINK_DIRECTORIES})
    #MESSAGE("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
    #MESSAGE("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
    #MESSAGE("OpenCV_LINK_DIRECTORIES = ${OpenCV_LINK_DIRECTORIES}")

    # Specific check for cvcam library provided under Windows with versions 
    # prior to Opencv-1.1.0
    IF(OpenCV_CVCAM_INCLUDE_DIR AND OpenCV_CVCAM_LIBRARY)
      SET(VISP_HAVE_OPENCV_CVCAM  TRUE)  # for header vpConfig.h
      LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${OpenCV_CVCAM_INCLUDE_DIR})
      LIST(APPEND VISP_EXTERN_LIBS ${OpenCV_CVCAM_LIBRARY})    
    ENDIF(OpenCV_CVCAM_INCLUDE_DIR AND OpenCV_CVCAM_LIBRARY)

  ELSE(OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV not found")
    SET(VISP_HAVE_OPENCV_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(OpenCV_FOUND)
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

# Libplotter
IF(USE_LIBPLOTTER)
  #FIND_PACKAGE(LIBPLOTTER)
  IF(LIBPLOTTER_FOUND)
    MESSAGE(STATUS "Libplotter found")
    SET(VISP_HAVE_LIBPLOTTER  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_LIBPLOTTER_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${LIBPLOTTER_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${LIBPLOTTER_LIBRARIES})

  ELSE(LIBPLOTTER_FOUND)
    MESSAGE(STATUS "Libplot not found")
    SET(VISP_HAVE_LIBPLOTTER_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(LIBPLOTTER_FOUND)
ENDIF(USE_LIBPLOTTER)

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
SET(VISP_HAVE_ITIFG8_FOUND "no")  # for ViSP-third-party.txt
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

# library itifg8 framegrabber devices
IF(USE_ITIFG8)
#   FIND_PACKAGE(ITIFG8)
  IF(ITIFG8_FOUND)
    MESSAGE(STATUS "Itifg8 found")
    SET(VISP_HAVE_ITIFG8 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_ITIFG8_FOUND "yes")  # for ViSP-third-party.txt
    SET(VISP_HAVE_ITIFG8_VERSION ${ITIFG8_VERSION}) # for header vpConfig.h
    SET(VISP_HAVE_ITIFG8_VERSION_DOXY ${ITIFG8_VERSION}) # for doxygen file doc/config-doxygen
     #MESSAGE("DBG VISP_HAVE_ITIFG8= ${VISP_HAVE_ITIFG8}")
     #MESSAGE("DBG ITIFG8_INCLUDE_DIR=${ITIFG8_INCLUDE_DIR}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIR ${ITIFG8_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBS ${ITIFG8_LIBRARIES})
     #MESSAGE("ITIFG8_LIBRARIES = ${ITIFG8_LIBRARIES}")
  ELSE(ITIFG8_FOUND)
    MESSAGE(STATUS "Itifg8 not found")
    SET(VISP_HAVE_ITIFG8_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_ITIFG8 OFF)
  ENDIF(ITIFG8_FOUND)
ELSE(USE_ITIFG8)
  SET(VISP_HAVE_ITIFG8_VERSION_DOXY 83) # for doxygen file doc/config-doxygen
ENDIF(USE_ITIFG8)

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
# Specific hardware : robots
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_AFMA4_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_AFMA6_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_BICLOPS_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_PTU46_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_VIPER850_FOUND "no")  # for ViSP-third-party.txt

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
  IF(MSVC80 OR MSVC90)
    #MESSAGE("Add -D_CRT_SECURE_NO_DEPRECATE ")
    LIST(APPEND VISP_DEFS "-D_CRT_SECURE_NO_DEPRECATE")
  ENDIF(MSVC80 OR MSVC90)
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
# Cleanify third party libraries
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_LIBS: ${VISP_EXTERN_LIBS}")
IF(VISP_EXTERN_LIBS)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_LIBS)
ENDIF(VISP_EXTERN_LIBS)
#MESSAGE("VISP_EXTERN_LIBS without duplicates: ${VISP_EXTERN_LIBS}")

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
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
  

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

# InstallRequiredSystemLibraries: Hack for Visual Studio support Search for
# system runtime libraries based on the platform.
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ViSP: Visual Servoing Platform")
SET(CPACK_PACKAGE_VENDOR "Inria, French National Institute for Research in Computer Science and Control")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

SET(CPACK_PACKAGE_VERSION ${VISP_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${VISP_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${VISP_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${VISP_VERSION_PATCH})

# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
SET(CPACK_SOURCE_IGNORE_FILES
"~$"
"^${PROJECT_SOURCE_DIR}.*/CVS/"
)

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME} ${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}")
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

INCLUDE(CPack)

#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(src)
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

#----------------------------------------------------------------------
# Generate the package dependent file include/visp/vpConfig.h
#----------------------------------------------------------------------
CONFIGURE_FILE(${VISP_SOURCE_DIR}/ViSP-third-party.txt.cmake
  ${VISP_BINARY_DIR}/ViSP-third-party.txt
)
