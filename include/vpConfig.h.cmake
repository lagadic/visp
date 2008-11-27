/****************************************************************************
 *
 * $Id: vpConfig.h.cmake,v 1.25 2008-11-27 12:44:24 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * ViSP configuration.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpConfig_h
#define vpConfig_h

// ViSP major version.
#define VISP_VERSION_MAJOR ${VISP_VERSION_MAJOR}

// ViSP minor version.
#define VISP_VERSION_MINOR ${VISP_VERSION_MINOR}

// ViSP patch version.
#define VISP_VERSION_PATCH ${VISP_VERSION_PATCH}

// ViSP version.
#cmakedefine VISP_VERSION ${VISP_VERSION}

// ViSP library is either compiled static or shared
// Used to set declspec(import, export) in headers if required under Windows
#cmakedefine VISP_BUILD_SHARED_LIBS

// Defined if MSVC is the compiler
#cmakedefine VISP_USE_MSVC

// Defined if X11 library available.
#cmakedefine VISP_HAVE_X11

// Defined if XML2 library available.
#cmakedefine VISP_HAVE_XML2

// Defined if pthread library available.
#cmakedefine VISP_HAVE_PTHREAD

// Defined if OpenCV library available.
#cmakedefine VISP_HAVE_OPENCV

// Defined if GTK library available (either gtk or gtk2).
#cmakedefine VISP_HAVE_GTK

// Defined if GDI (Graphics Device Interface) library available
#cmakedefine VISP_HAVE_GDI

// Defined if Direct3D9 library available
#cmakedefine VISP_HAVE_D3D9

// Defined if GSL library available (-lgsl -lgslcblas).
#cmakedefine VISP_HAVE_GSL

// Defined if Coin library available.
#cmakedefine VISP_HAVE_COIN

// Defined if OpenGL library available.
#cmakedefine VISP_HAVE_OPENGL

// Defined if Qt library available (either Qt-3 or Qt-4).
#cmakedefine VISP_HAVE_QT

// Defined if SoQt library available.
#cmakedefine VISP_HAVE_SOQT

// Defined if SoWin library available.
#cmakedefine VISP_HAVE_SOWIN

// Defined if SoXt library available.
#cmakedefine VISP_HAVE_SOXT

// Defined if raw1394 and dc1394-1.x libraries available.
#cmakedefine VISP_HAVE_DC1394_1

// Defined if raw1394 and dc1394-2.x libraries available.
#cmakedefine VISP_HAVE_DC1394_2

// Defined if dc1394_camera_enumerate() is available in dc1394-2.x.
// dc1394_camera_enumerate() was introduced after libdc1394-2.0.0-rc7.
#cmakedefine VISP_HAVE_DC1394_2_CAMERA_ENUMERATE

// Defined if dc1394_find_cameras() is available in dc1394-2.x 
// dc1394_find_cameras() is still present until libdc1394-2.0.0-rc7.
// This function was suppress and replace by dc1394_camera_enumerate()
// in more recent releases.
#cmakedefine VISP_HAVE_DC1394_2_FIND_CAMERAS

// Defined if Video For Linux Two available.
#cmakedefine VISP_HAVE_V4L2

// Defined if itifg-8.x (Coreco Imaging) framegraber driver available.
#cmakedefine VISP_HAVE_ITIFG8
#cmakedefine VISP_HAVE_ITIFG8_VERSION ${VISP_HAVE_ITIFG8_VERSION}

// Defined if DirectShow library is available (only under Windows).
#cmakedefine VISP_HAVE_DIRECTSHOW

// Defined if Irisa's Afma4 robot available.
#cmakedefine VISP_HAVE_AFMA4

// Defined if Irisa's Afma6 robot available.
#cmakedefine VISP_HAVE_AFMA6

// Defined if Biclops pan-tilt head available.
#cmakedefine VISP_HAVE_BICLOPS

// Defined if Irisa's Ptu-46 pan-tilt head available.
#cmakedefine VISP_HAVE_PTU46

// Defined if linux/parport.h is available for parallel port usage.
#cmakedefine VISP_HAVE_PARPORT

// Defined if Inria's NAS server hosting /udd/ is available
// Used for the moment in vpAfma6 class to check if config files are
// available in /udd/fspindle/robot/Afma6/current/include/
#cmakedefine VISP_HAVE_ACCESS_TO_NAS

// Under Windows, for shared libraries (DLL) we need to define export on
// compilation or import on use (like a third party project).
// We exploit here the fact that cmake auto set xxx_EXPORTS (with S) on 
// compilation.
#if defined (WIN32) && defined(VISP_BUILD_SHARED_LIBS) 
#  ifdef visp_2_EXPORTS 
#    define VISP_EXPORT __declspec(dllexport)
#  else  
#    define VISP_EXPORT __declspec(dllimport)
#  endif 
#else
#  define VISP_EXPORT
#endif

#endif


