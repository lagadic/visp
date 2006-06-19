/****************************************************************************
 *
 * $Id: vpConfig.h.cmake,v 1.7 2006-06-19 08:57:55 fspindle Exp $
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
#cmakedefine VISP_MAJOR_VERSION "${VISP_MAJOR_VERSION}"

// ViSP minor version.
#cmakedefine VISP_MINOR_VERSION "${VISP_MINOR_VERSION}"

// ViSP patch version.
#cmakedefine VISP_PATCH_VERSION "${VISP_PATCH_VERSION}"

// ViSP version.
#cmakedefine VISP_VERSION "${VISP_VERSION}"

// ViSP library is either compiled static or shared
// Used to set declspec(import, export) in headers if required under Windows
#cmakedefine VISP_BUILD_SHARED_LIBS

// Defined if MSVC is the compiler
#cmakedefine VISP_USE_MSVC

// Defined if X11 library available.
#cmakedefine VISP_HAVE_X11

// Defined if pthread library available.
#cmakedefine VISP_HAVE_PTHREAD

// Defined if GTK library available (either gtk or gtk2).
#cmakedefine VISP_HAVE_GTK

// Defined if GSL library available (-lgsl -lgslcblas).
#cmakedefine VISP_HAVE_GSL

// Defined if Coin library available.
#cmakedefine VISP_HAVE_COIN

// Defined if Qt library available (either Qt-3 or Qt-4).
#cmakedefine VISP_HAVE_QT

// Defined if SoQt library available.
#cmakedefine VISP_HAVE_SOQT

// Defined if dc1394_control and raw1394 libraries available.
#cmakedefine VISP_HAVE_DC1394

// Defined if cfox library is available (only under MAC OS X).
#cmakedefine VISP_HAVE_CFOX

// Defined if Video For Linux Two available.
#cmakedefine VISP_HAVE_V4L2

// Defined if Irisa's ICcomp framegraber available.
#cmakedefine VISP_HAVE_ICCOMP

// Defined if cfox library is available (only under Windows).
#cmakedefine VISP_HAVE_DIRECTSHOW

// Defined if Irisa's Afma4 robot available.
#cmakedefine VISP_HAVE_AFMA4

// Defined if Irisa's Afma6 robot available.
#cmakedefine VISP_HAVE_AFMA6

// Defined if Biclops pan-tilt head available.
#cmakedefine VISP_HAVE_BICLOPS

// Defined if Irisa's Ptu-46 pan-tilt head available.
#cmakedefine VISP_HAVE_PTU46

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


