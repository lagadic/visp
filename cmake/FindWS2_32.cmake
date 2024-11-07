#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find ws2_32 library.
#
# WS2_32_FOUND - true if ws2_32 library is detected
# WS2_32_LIBRARY - Library name
#
#############################################################################

if(WIN32 AND NOT CYGWIN)
  if(MINGW)
    set(WS2_32_LIBNAME "ws2_32.a")
    check_library_exists(${WS2_32_LIBNAME} getch "" HAVE_LIBWS2_32) # for inet_ntoa() and socket functionalities
    if(HAVE_LIBWS2_32)
      set(WS2_32_LIBRARY ${WS2_32_LIBNAME})
      set(WS2_32_FOUND TRUE)
    else()
      find_library(HAVE_LIBWS2_32 ${WS2_32_LIBNAME}
        "$ENV{MINGW_DIR}/lib"
        "$ENV{MINGW_DIR}/mingw/lib"
        C:/mingw/mingw/lib)
      if(HAVE_LIBWS2_32)
        set(WS2_32_LIBRARY ${WS2_32_LIBNAME})
        set(WS2_32_FOUND TRUE)
      else()
        set(WS2_32_FOUND FALSE)
      endif()
    endif()
  elseif(WINRT)
    # Since check_library_exists() and find_library() does't work to detect ws2_32.lib we add the lib that is part of Windows SDK
    set(WS2_32_LIBNAME "ws2_32.lib")
    set(WS2_32_LIBRARY ${WS2_32_LIBNAME})
    set(WS2_32_FOUND TRUE)
  else() # pure WIN32
    set(WS2_32_LIBNAME "ws2_32.lib")
    check_library_exists(${WS2_32_LIBNAME} getch "" HAVE_LIBWS2_32) # for inet_ntoa() and socket functionalities
    if(HAVE_LIBWS2_32)
      set(WS2_32_LIBRARY ${WS2_32_LIBNAME})
      set(WS2_32_FOUND TRUE)
    else()
      set(WS2_32_FOUND FALSE)
    endif()
  endif()
  mark_as_advanced(WS2_32_LIBNAME)
  mark_as_advanced(HAVE_LIBWS2_32)
endif()
