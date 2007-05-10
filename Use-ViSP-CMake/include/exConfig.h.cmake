/****************************************************************************
 *
 * $Id: exConfig.h.cmake,v 1.1 2007-05-10 11:41:36 fspindle Exp $
 *
 * Copyright (C) 1998-2007 Inria. All rights reserved.
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
 * Example configuration.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef exConfig_h
#define exConfig_h

// Example major version.
#define EXAMPLE_VERSION_MAJOR ${EXAMPLE_VERSION_MAJOR}

// Example minor version.
#define EXAMPLE_VERSION_MINOR ${EXAMPLE_VERSION_MINOR}

// Example patch version.
#define EXAMPLE_VERSION_PATCH ${EXAMPLE_VERSION_PATCH}

// Example version.
#cmakedefine EXAMPLE_VERSION ${EXAMPLE_VERSION}

// Library is either compiled static or shared
// Used to set declspec(import, export) in headers if required under Windows
#cmakedefine EXAMPLE_BUILD_SHARED_LIBS

// Defined if MSVC is the compiler
#cmakedefine EXAMPLE_USE_MSVC

// Under Windows, for shared libraries (DLL) we need to define export on
// compilation or import on use (like a third party project).
// We exploit here the fact that cmake auto set xxx_EXPORTS (with S) on 
// compilation.
#if defined (WIN32) && defined(EXAMPLE_BUILD_SHARED_LIBS) 
#  ifdef tarot_EXPORTS 
#    define EXAMPLE_EXPORT __declspec(dllexport)
#  else  
#    define EXAMPLE_EXPORT __declspec(dllimport)
#  endif 
#else
#  define EXAMPLE_EXPORT
#endif

#endif


