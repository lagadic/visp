#############################################################################
#
# $Id: CMakeLists.txt,v 1.65 2008-02-05 09:08:53 fspindle Exp $
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
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.3)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_VERSION_MAJOR "2")
SET(VISP_VERSION_MINOR "4")
SET(VISP_VERSION_PATCH "2")
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

IF (UNIX)
  OPTION(USE_ICCOMP  "Compile ViSP with ICcomp framegrabber usage" OFF)
  OPTION(USE_ITIFG8  "Compile ViSP with itifg8 framegrabber usage" OFF)
  OPTION(USE_AFMA4   "Compile ViSP for Afma4 robot usage" OFF)
  OPTION(USE_AFMA6   "Compile ViSP for Afma6 robot usage" OFF)
  OPTION(USE_BICLOPS "Compile ViSP for Biclops head usage" OFF)
  OPTION(USE_PTU46   "Compile ViSP for Ptu-46 head usage" OFF)
  IF(APPLE)
    OPTION(USE_CFOX   "Compile ViSP with cfox usage" OFF)
  ENDIF(APPLE)
ENDIF(UNIX)

# Set other options to default value
SET(USE_X11       ON) # For Linux/OSX display
SET(USE_XML2      ON) # For the Xml Parser (vpCameraParameters)
SET(USE_COIN      ON) # For the simulator
SET(USE_SOWIN     ON) # For the simulator viewer
SET(USE_SOQT      ON) # For the simulator viewer
SET(USE_SOXT      ON) # For the simulator viewer
SET(USE_PTHREAD   ON) # For Biclops/Simulator
SET(USE_OPENCV    ON) # For Intel opencv
SET(USE_GSL       ON) # For matrix manipulations
SET(USE_GTK1      ON) # For Linux/OSX/Windows display with gtk-1.x
SET(USE_GTK2      ON) # For Linux/OSX/Windows display with gtk-2.x
SET(USE_DIRECT3D  ON) # For Windows display
SET(USE_GDI       ON) # For Windows display
SET(USE_DC1394    ON) # For firewire grabber under Linux and OSX ?
SET(USE_V4L2      ON) # For Video 4 Linux 2 grabber under Linux
SET(USE_DIRECTSHOW ON) # For Windows direct show grabber

IF(BUILD_SHARED_LIBS)
  SET(VISP_BUILD_SHARED_LIBS TRUE)  # for header vpConfig.h
ENDIF(BUILD_SHARED_LIBS)


INCLUDE(CheckLibraryExists)

#---------------------------------------------------------------------
# Platform dependent
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  SET(VISP_DEFS "${VISP_DEFS} -DWIN32")
  SET(VISP_DEFS "${VISP_DEFS} -DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm" getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    #MESSAGE("have winmm.lib")
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} "winmm")
  ENDIF(HAVE_LIBWINMM)
  IF(MSVC) #check if MSVC is the compiler and set VISP_USE_MSVC accordingly
    SET(VISP_USE_MSVC TRUE)
  ENDIF(MSVC)
ENDIF(WIN32 AND NOT CYGWIN)

IF(UNIX)
  SET(VISP_DEFS "${VISP_DEFS} -DUNIX")
  IF(APPLE)
    SET(VISP_DEFS "${VISP_DEFS} -DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

INCLUDE(TestBigEndian)
TEST_BIG_ENDIAN(BIGENDIAN)

IF(BIGENDIAN)
  SET(VISP_DEFS "${VISP_DEFS} -DBIGENDIAN")
ENDIF(BIGENDIAN)

#----------------------------------------------------------------------
# Unix libraries :  X11, pthread, cfox (MAC OS X)
#----------------------------------------------------------------------
IF(UNIX)

  #--------------------------------------------------------------------
  # X11 lib
  #--------------------------------------------------------------------
  IF(USE_X11)
    FIND_PACKAGE(X11)
    IF(X11_FOUND)
      SET(VISP_HAVE_X11 TRUE)  # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
	${X11_INCLUDE_DIR})
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${X11_LIBRARIES} -lm)
      #MESSAGE("X11: ${X11_LIBRARIES}")
    ENDIF(X11_FOUND)
  ENDIF(USE_X11)
  #--------------------------------------------------------------------
  # Apple MAC OS X cfox library
  #--------------------------------------------------------------------
  IF(APPLE AND USE_CFOX)
    IF(USE_CFOX)
      FIND_PACKAGE(CFOX)
      IF(CFOX_FOUND)
	SET(VISP_HAVE_CFOX TRUE) # for header vpConfig.h
	#MESSAGE("DBG VISP_HAVE_CFOX= ${VISP_HAVE_CFOX}")
	#MESSAGE("DBG CFOX_INCLUDE_DIR=${CFOX_INCLUDE_DIR}")
	SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
	  ${CFOX_INCLUDE_DIR})
	SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS}
	  ${CFOX_LIBRARIES} -lIOKit "-framework Carbon")
	#MESSAGE("CFOX_LIBRARIES = ${CFOX_LIBRARIES}")
      ELSE(CFOX_FOUND)
	MESSAGE("cfox not found.")
	# SET(USE_CFOX OFF)
      ENDIF(CFOX_FOUND)
    ENDIF(USE_CFOX)
  ENDIF(APPLE AND USE_CFOX)
ENDIF(UNIX)

#--------------------------------------------------------------------
# OpenGL, Qt, SoQt and Coin libs
#--------------------------------------------------------------------
IF(USE_COIN)
  INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
  IF(OPENGL_FOUND)
    FIND_PACKAGE(COIN)
    IF(WIN32 AND USE_SOWIN)
      FIND_PACKAGE(SOWIN)
    ENDIF(WIN32 AND USE_SOWIN)
    IF (WIN32 AND SOWIN_FOUND AND COIN_FOUND)
      #MESSAGE("COIN_LIBRARY_RELEASE = ${COIN_LIBRARY_RELEASE}")
      #MESSAGE("COIN_LIBRARY_DEBUG = ${COIN_LIBRARY_DEBUG}")
      # OPenGL and SoWin are found
      # We first manage OpenGL
      SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${OPENGL_gl_LIBRARY})

      # We manage SoWin and Coin
      SET(VISP_HAVE_SOWIN TRUE)  # for header vpConfig.h
      SET(VISP_HAVE_COIN TRUE)  # for header vpConfig.h
      
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${SOWIN_INCLUDE_DIR})
      #MESSAGE("SOWIN_INCLUDE_DIR = ${SOWIN_INCLUDE_DIR}")
      SET(VISP_DEFS "${VISP_DEFS} -DSOWIN_DLL")
      SET(VISP_DEFS "${VISP_DEFS} -DCOIN_DLL")
      IF(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)
	  #MESSAGE("SOWIN AND COIN RELEASE")
	  SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${COIN_INCLUDE_DIR})  
          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARY_RELEASE})
          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOWIN_LIBRARY_RELEASE})
      ELSE(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)
        IF(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
	  #MESSAGE("SOWIN AND COIN DEBUG")
	  SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${COIN_INCLUDE_DIR})  
          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARY_DEBUG})
          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOWIN_LIBRARY_DEBUG})
        ELSE(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
	  #MESSAGE("SOWIN AND COIN NOT RELEASE OR DEBUG")
          SET(VISP_HAVE_SOWIN FALSE)  # for header vpConfig.h
          SET(VISP_HAVE_COIN FALSE)  # for header vpConfig.h
        ENDIF(SOWIN_LIBRARY_DEBUG AND COIN_LIBRARY_DEBUG)
      ENDIF(SOWIN_LIBRARY_RELEASE AND COIN_LIBRARY_RELEASE)

    ENDIF(WIN32 AND SOWIN_FOUND AND COIN_FOUND)

    IF(USE_SOQT AND NOT VISP_HAVE_COIN)
      INCLUDE (${CMAKE_ROOT}/Modules/FindQt.cmake)    
      IF(QT_FOUND)
        FIND_PACKAGE(SOQT)
        IF (SOQT_FOUND AND COIN_FOUND)
	        # OPenGL, QT and SOQT are found
	        # We first manage OpenGL
	        SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
	        SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
	        SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${OPENGL_gl_LIBRARY})

	        # We manage QT
	        SET(VISP_HAVE_QT TRUE)  # for header vpConfig.h
	        SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${QT_INCLUDE_DIR} ${QT_INCLUDES})
	        #MESSAGE("QT_QT_LIB ${QT_QT_LIBRARY}")
	        #MESSAGE("QT_LIBS ${QT_LIBRARIES}")
	        #SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${QT_QT_LIBRARY} ${QT_LIBRARIES})
	        SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${QT_QT_LIBRARY})
	        # Because in QT_DEFINITIONS defs are separated by ";", parse the
	        # QT_DEFINITIONS in order to build a space separated string
	        #SET(VISP_DEFS "${VISP_DEFS} ${QT_DEFINITIONS}") did not work
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

	        # We manage now SOQT and Coin
	        SET(VISP_HAVE_SOQT TRUE)  # for header vpConfig.h 
          SET(VISP_HAVE_COIN TRUE)  # for header vpConfig.h
	        IF(WIN32)
	          SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${SOQT_INCLUDE_DIR})
	          #MESSAGE(STATUS "SOQT_INCLUDE_DIR = ${SOQT_INCLUDE_DIR}")
	          SET(VISP_DEFS "${VISP_DEFS} -DSOQT_DLL")
	          SET(VISP_DEFS "${VISP_DEFS} -DCOIN_DLL")
	          IF(SOQT_LIBRARY_RELEASE)
	            IF(COIN_LIBRARY_RELEASE)
		      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${COIN_INCLUDE_DIR})  
	              SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARY_RELEASE})
	              SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOQT_LIBRARY_RELEASE})
	            ENDIF(COIN_LIBRARY_RELEASE)
	          ELSE(SOQT_LIBRARY_RELEASE)
	            IF(COIN_LIBRARY_DEBUG)
		      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${COIN_INCLUDE_DIR})  
	              SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARY_DEBUG})
	              SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOQT_LIBRARY_DEBUG})
	            ELSE(COIN_LIBRARY_DEBUG)
	              SET(VISP_HAVE_SOQT FALSE)  # for header vpConfig.h
                SET(VISP_HAVE_COIN FALSE)  # for header vpConfig.h
	            ENDIF(COIN_LIBRARY_DEBUG)
	          ENDIF(SOQT_LIBRARY_RELEASE)
	        ELSE(WIN32)
	          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOQT_LIBRARIES})
	          SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARIES})
	          #MESSAGE(STATUS "SOQT_LIBRARIES = ${SOQT_LIBRARY_DEBUG}")
	        ENDIF(WIN32)
        ENDIF (SOQT_FOUND AND COIN_FOUND)
      ENDIF(QT_FOUND)
    ENDIF(USE_SOQT AND NOT VISP_HAVE_COIN)
  
    #MESSAGE(STATUS "QT_LIBRARIES no found")
    IF(USE_SOXT AND NOT VISP_HAVE_COIN)
      FIND_PACKAGE(SOXT)
    ENDIF(USE_SOXT AND NOT VISP_HAVE_COIN)

    IF (UNIX AND SOXT_FOUND AND COIN_FOUND)
      # OPenGL and SoXt are found
      # We first manage OpenGL
      SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${OPENGL_gl_LIBRARY})

      # We manage SoXt and Coin
      SET(VISP_HAVE_SOXT TRUE)  # for header vpConfig.h
      SET(VISP_HAVE_COIN TRUE)  # for header vpConfig.h
      
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${SOXT_LIBRARIES})
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${COIN_LIBRARIES})
      
    ENDIF(UNIX AND SOXT_FOUND AND COIN_FOUND)
    IF(NOT VISP_HAVE_COIN)
      MESSAGE(STATUS "COIN_LIBRARIES not found")
    ENDIF(NOT VISP_HAVE_COIN)
  ELSE(OPENGL_FOUND)
    #MESSAGE(STATUS "OPENGL_LIBRARIES no found")
  ENDIF(OPENGL_FOUND)
ENDIF(USE_COIN)

#---------------------------------------------------------------------
# XML2
#---------------------------------------------------------------------
IF(USE_XML2)
  FIND_PACKAGE(XML2)
  IF(XML2_FOUND)
    SET(VISP_HAVE_XML2 TRUE)
  # MESSAGE("XML2 found")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
        ${XML2_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${XML2_LIBRARIES})
  ELSE(XML2_FOUND)
  # MESSAGE("XML2 requested but not found")
  ENDIF(XML2_FOUND)
ENDIF(USE_XML2)

#--------------------------------------------------------------------
# pthread lib
#--------------------------------------------------------------------
IF(USE_PTHREAD)
  FIND_PACKAGE(PTHREAD)
  IF(PTHREAD_FOUND)
    SET(VISP_HAVE_PTHREAD TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
      ${PTHREAD_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${PTHREAD_LIBRARIES})
    #MESSAGE("PTHREAD_LIBRARIES = ${PTHREAD_LIBRARIES}")
  ELSE(PTHREAD_FOUND)
    #MESSAGE("PTHREAD_LIBRARIES no found")
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

# open CV
IF(USE_OPENCV)
  SET(OpenCV_FIND_QUIETLY TRUE)
  FIND_PACKAGE(OpenCV)
  IF(OpenCV_FOUND)
    SET(VISP_HAVE_OPENCV  TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    SET(VISP_EXTERN_LINK_DIR ${VISP_EXTERN_LINK_DIR}
      ${OpenCV_EXE_LINKER_FLAGS} -L${OpenCV_LINK_DIRECTORIES})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${OpenCV_LIBRARIES})
    LINK_DIRECTORIES(${OpenCV_LINK_DIRECTORIES})
    #MESSAGE("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
    #MESSAGE("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
    #MESSAGE("OpenCV_LINK_DIRECTORIES = ${OpenCV_LINK_DIRECTORIES}")
  ENDIF(OpenCV_FOUND)
ENDIF(USE_OPENCV)


# gnu scientific library
IF(USE_GSL)
  FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    SET(VISP_HAVE_GSL  TRUE)  # for header vpConfig.h
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${GSL_INCLUDE_DIR})
    SET(VISP_EXTERN_LINK_DIR ${VISP_EXTERN_LINK_DIR}
      ${GSL_EXE_LINKER_FLAGS} -L${GSL_LINK_DIRECTORIES})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${GSL_LIBRARIES})
    LINK_DIRECTORIES(${GSL_LINK_DIRECTORIES})
    #MESSAGE("GSL_INCLUDE_DIR = ${GSL_INCLUDE_DIR}")
    #MESSAGE("GSL_LINK_DIRECTORIES = ${GSL_LINK_DIRECTORIES}")
    #MESSAGE("GSL_LIBRARIES = ${GSL_LIBRARIES}")
  ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

# strategy for gtk: search first gtk2. If not found search gtk1
# gtk-2 library:
IF(USE_GTK2)
   IF(UNIX OR WIN32)
    FIND_PACKAGE(GTK2)
    IF(GTK2_FOUND)
      SET(VISP_HAVE_GTK TRUE)  # for header vpConfig.h
      SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR} ${GTK2_INCLUDE_DIR})
      
      IF(UNIX)
	## ignore gtk pedantic warnings by gcc specific -isystem :
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK2_gtk_INCLUDE_PATH}")
      ENDIF(UNIX)

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
   ENDIF(UNIX OR WIN32)
ENDIF(USE_GTK2)

# gtk-1 library: is search only if gtk-2 was not found
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
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} ${GTK_LIBRARIES})
    ELSE(GTK_FOUND)
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
  FIND_PACKAGE(DIRECT3D)
  IF(DIRECT3D_FOUND)
    SET(VISP_HAVE_D3D9 TRUE) # for header vpConfig.h
    #MESSAGE("DBG VISP_HAVE_D3D9= ${VISP_HAVE_D3D9}")
    #MESSAGE("DBG DIRECT3D_INCLUDE_DIR=${DIRECT3D_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
      ${DIRECT3D_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${DIRECT3D_LIBRARIES} ${VISP_EXTERN_LIBS})
    #MESSAGE("DIRECT3D_LIBRARIES = ${DIRECT3D_LIBRARIES}")
  ENDIF(DIRECT3D_FOUND)
ENDIF(USE_DIRECT3D)

# Under Windows, usage of GDI (Graphics Device Interface)
IF(USE_GDI)
  FIND_PACKAGE(GDI)
  IF(GDI_FOUND)
    SET(VISP_HAVE_GDI TRUE) # for header vpConfig.h
    #MESSAGE("DBG VISP_HAVE_GDI= ${VISP_HAVE_GDI}")
    SET(VISP_EXTERN_LIBS ${GDI_LIBRARIES} ${VISP_EXTERN_LIBS})
    #MESSAGE("GDI_LIBRARIES = ${GDI_LIBRARIES}")
  ENDIF(GDI_FOUND)
ENDIF(USE_GDI)


#----------------------------------------------------------------------
# Specific hardware : framegrabbers and cameras
#----------------------------------------------------------------------

# library for firewire cameras
IF(USE_DC1394)
  FIND_PACKAGE(DC1394)
  IF(DC1394_FOUND)
    IF(DC1394_VERSION EQUAL 2)
      SET(VISP_HAVE_DC1394_2 TRUE) # for header vpConfig.h
      IF(DC1394_CAMERA_ENUMERATE_FOUND)
        SET(VISP_HAVE_DC1394_2_CAMERA_ENUMERATE TRUE) # for header vpConfig.h
      ENDIF(DC1394_CAMERA_ENUMERATE_FOUND)
      IF(DC1394_FIND_CAMERAS_FOUND)
        SET(VISP_HAVE_DC1394_2_FIND_CAMERAS TRUE) # for header vpConfig.h
      ENDIF(DC1394_FIND_CAMERAS_FOUND)

      #MESSAGE("DBG VISP_HAVE_DC1394_2= ${VISP_HAVE_DC1394_2}")
    ELSE(DC1394_VERSION EQUAL 2)
      SET(VISP_HAVE_DC1394_1 TRUE) # for header vpConfig.h
      #MESSAGE("DBG VISP_HAVE_DC1394_1= ${VISP_HAVE_DC1394_1}")
    ENDIF(DC1394_VERSION EQUAL 2)
    #MESSAGE("DBG DC1394_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
      ${DC1394_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS}
      ${DC1394_LIBRARIES})
    IF(APPLE)
      SET(VISP_EXTERN_LIBS ${VISP_EXTERN_LIBS} IOkit "-framework Carbon")
    ENDIF(APPLE)
     #MESSAGE("DC1394_LIBRARIES = ${DC1394_LIBRARIES}")
  ELSE(DC1394_FOUND)
    #MESSAGE("Firewire dc1394 requested but not found. Turn off USE_DC1394!")
    # SET(USE_DC1394 OFF)
  ENDIF(DC1394_FOUND)
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

# library itifg8 framegrabber devices
IF(USE_ITIFG8)
  FIND_PACKAGE(ITIFG8)
  IF(ITIFG8_FOUND)
    SET(VISP_HAVE_ITIFG8 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_ITIFG8_VERSION ${ITIFG8_VERSION}) # for header vpConfig.h
    SET(VISP_HAVE_ITIFG8_VERSION_DOXY ${ITIFG8_VERSION}) # for doxygen file doc/config-doxygen
     #MESSAGE("DBG VISP_HAVE_ITIFG8= ${VISP_HAVE_ITIFG8}")
     #MESSAGE("DBG ITIFG8_INCLUDE_DIR=${ITIFG8_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
      ${ITIFG8_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${ITIFG8_LIBRARIES} ${VISP_EXTERN_LIBS})
     #MESSAGE("ITIFG8_LIBRARIES = ${ITIFG8_LIBRARIES}")
  ELSE(ITIFG8_FOUND)
    MESSAGE("itifg8 requested but not found.")
    # SET(USE_ITIFG8 OFF)
  ENDIF(ITIFG8_FOUND)
ELSE(USE_ITIFG8)
  SET(VISP_HAVE_ITIFG8_VERSION_DOXY 83) # for doxygen file doc/config-doxygen
ENDIF(USE_ITIFG8)

# Under Windows, usage of Direct Show
IF(USE_DIRECTSHOW)
  FIND_PACKAGE(DIRECTSHOW)
  IF(DIRECTSHOW_FOUND)
    SET(VISP_HAVE_DIRECTSHOW TRUE) # for header vpConfig.h
    #MESSAGE("DBG VISP_HAVE_DIRECTSHOW= ${VISP_HAVE_PTU46}")
    #MESSAGE("DBG DIRECTSHOW_INCLUDE_DIR=${DIRECTSHOW_INCLUDE_DIR}")
    SET(VISP_EXTERN_INCLUDE_DIR ${VISP_EXTERN_INCLUDE_DIR}
      ${DIRECTSHOW_INCLUDE_DIR})
    SET(VISP_EXTERN_LIBS ${DIRECTSHOW_LIBRARIES} ${VISP_EXTERN_LIBS})
    #MESSAGE("DIRECTSHOW_LIBRARIES = ${DIRECTSHOW_LIBRARIES}")
  ENDIF(DIRECTSHOW_FOUND)
ENDIF(USE_DIRECTSHOW)


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
  SET(VISP_DEFS "${VISP_DEFS} ${BIT3_DEFS}")
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

ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Concatenate intern and third party include dirs
#----------------------------------------------------------------------
INCLUDE_DIRECTORIES(${VISP_INTERN_INCLUDE_DIR} ${VISP_EXTERN_INCLUDE_DIR})

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
IF (UNIX)
INSTALL(FILES ${BINARY_OUTPUT_PATH}/visp-config
  DESTINATION bin
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
  OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
  OWNER_WRITE   )
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

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
ADD_DEFINITIONS(${VISP_DEFS})

# With Visual Studio 2005, Microsoft deprecates the standard C library, for
# example fopen() and sprintf(), to non-portable functions fopen_s() and
# sprintf_s(). These functions are considered by Microsoft more secure. This is
# a worthwhile exercise ! The use of these deprecated functions causes a lot of
# warnings. To suppress it, we add the _CRT_SECURE_NO_DEPRECATE preprocessor
# definition
IF(WIN32 AND MSVC80)
  #MESSAGE("Add -D_CRT_SECURE_NO_DEPRECATE ")
  ADD_DEFINITIONS("-D_CRT_SECURE_NO_DEPRECATE")
ENDIF(WIN32 AND MSVC80)


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
#----------------------------------------------------------------------
ENABLE_TESTING()
INCLUDE(DartConfig.cmake)
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
SET(CPACK_PACKAGE_EXECUTABLES "ViSP" "ViSP")

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

