#############################################################################
#
# $Id: FindGDI.cmake,v 1.2 2007-02-16 16:18:20 asaunier Exp $
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
# Try to find GDI (Graphics Device Interface) on Windows. 
# Once loaded this will define:
#
# GDI_FOUND        - system has GDI
# GDI_LIBRARIES    - libraries you need to link to
#
# Authors:
# Fabien Spindler
#
#############################################################################

SET(GDI_FOUND "NO")

# GDI (Graphics Device Interface)  is only available on Windows platforms
IF(WIN32)
  IF(MINGW)
    FIND_LIBRARY(GDI_LIBRARY gdi32
      "C:/MinGW/lib"
      DOC "Where can the GDI (Graphics Device Interface) library be found"
      NO_DEFAULT_PATH
    )
  ELSE(MINGW)
    FIND_LIBRARY(GDI_LIBRARY gdi32
      "$ENV{DXSDK_DIR}/Lib"
      "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/Lib"
      "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
      "C:/Program Files/Microsoft Platform SDK/Lib"
      "C:/DXSDK/Include/Lib"
      "C:/MinGW/lib"
      DOC "Where can the GDI (Graphics Device Interface) library be found"
    )
  ENDIF(MINGW)
  # if GDI libraries found, then we're ok
  IF(GDI_LIBRARY)
    SET(GDI_FOUND "YES")
    SET(GDI_LIBRARIES ${GDI_LIBRARY}
  )

  ENDIF(GDI_LIBRARY)

  MARK_AS_ADVANCED(
    GDI_LIBRARIES
  )


ENDIF(WIN32)
