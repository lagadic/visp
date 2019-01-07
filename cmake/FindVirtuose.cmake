#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# See http://visp.inria.fr for more information.
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
# Try to find Virtuose library API to dial with the robot Haption Virtuose
#
# VIRTUOSE_FOUND
# VIRTUOSE_INCLUDE_DIRS
# VIRTUOSE_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(VIRTUOSE_API $ENV{VIRTUOSE_HOME})
if(UNIX)
  list(APPEND VIRTUOSE_API /home/soft/virtuose/Livraison_3_97/VirtuoseAPI_v3_97)
  list(APPEND VIRTUOSE_API /home/soft/virtuose/Livraison_3_90/VirtuoseAPI_v3_93)
  list(APPEND VIRTUOSE_API /local/virtuose-6D/Livraison_3_90/VirtuoseAPI_v3_93)
  
  set(VIRTUOSE_OS "linux")
  
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    list(APPEND VIRTUOSE_ARCH "x64")
  else()
    list(APPEND VIRTUOSE_ARCH "x86")
  endif()
elseif(MSVC)
  list(APPEND VIRTUOSE_API "C:/VirtuoseAPI_v3_97")
  list(APPEND VIRTUOSE_API "C:/VirtuoseAPI_v3_93")

  set(VIRTUOSE_OS win)
  
  if(CMAKE_CL_64)
    list(APPEND VIRTUOSE_ARCH "x64")
  else()
    list(APPEND VIRTUOSE_ARCH "Win32")
  endif()

  if(MSVC_VERSION EQUAL 1400)
    set(VIRTUOSE_MSVC_RUNTIME VC2005)
  elseif(MSVC_VERSION EQUAL 1500)
    set(VIRTUOSE_MSVC_RUNTIME VC2008)
  elseif(MSVC_VERSION EQUAL 1600)
    set(VIRTUOSE_MSVC_RUNTIME VC2010)
  elseif(MSVC_VERSION EQUAL 1700)
    set(VIRTUOSE_MSVC_RUNTIME VC2012)
  elseif(MSVC_VERSION EQUAL 1800)
    set(VIRTUOSE_MSVC_RUNTIME VC2013)
  elseif(MSVC_VERSION EQUAL 1900)
    set(VIRTUOSE_MSVC_RUNTIME VC2015)
  endif()

else()
  return()
endif()
  
set(VIRTUOSE_API_INC "")
set(VIRTUOSE_API_LIB "")
if(UNIX)
  foreach(api_ ${VIRTUOSE_API})
    list(APPEND VIRTUOSE_API_INC "${api_}")  
    list(APPEND VIRTUOSE_API_LIB "${api_}/${VIRTUOSE_OS}/lib/${VIRTUOSE_ARCH}")  
  endforeach()
elseif(MSVC)
  foreach(api_ ${VIRTUOSE_API})
    list(APPEND VIRTUOSE_API_INC "${api_}")
    list(APPEND VIRTUOSE_API_LIB "${api_}/${VIRTUOSE_OS}/lib/${VIRTUOSE_MSVC_RUNTIME}/${VIRTUOSE_ARCH}")
  endforeach()
endif()

find_path(VIRTUOSE_INCLUDE_DIRS VirtuoseAPI.h
  PATHS
    ${VIRTUOSE_API_INC}
)

if(UNIX)
  find_library(VIRTUOSE_LIBRARIES
    NAMES virtuose
    PATHS 
      ${VIRTUOSE_API_LIB}
  )
elseif(MSVC)
  set(VIRTUOSE_API_LIB_DEBUG "")
  set(VIRTUOSE_API_LIB_RELEASE "")
  foreach(api_ ${VIRTUOSE_API})
    list(APPEND VIRTUOSE_API_LIB_DEBUG "${api_}/${VIRTUOSE_OS}/lib/${VIRTUOSE_MSVC_RUNTIME}/${VIRTUOSE_ARCH}/DebugMD")
    list(APPEND VIRTUOSE_API_LIB_RELEASE "${api_}/${VIRTUOSE_OS}/lib/${VIRTUOSE_MSVC_RUNTIME}/${VIRTUOSE_ARCH}/ReleaseMD")
  endforeach()
  find_library(VIRTUOSE_LIBRARIES_DEBUG
    NAMES virtuoseDLL
    PATHS 
      ${VIRTUOSE_API_LIB_DEBUG}
  )
  find_library(VIRTUOSE_LIBRARIES_RELEASE
    NAMES virtuoseDLL
    PATHS 
      ${VIRTUOSE_API_LIB_RELEASE}
  )
  
  if(VIRTUOSE_LIBRARIES_DEBUG AND VIRTUOSE_LIBRARIES_RELEASE)
    set(VIRTUOSE_LIBRARIES debug     ${VIRTUOSE_LIBRARIES_DEBUG}
	                       optimized ${VIRTUOSE_LIBRARIES_RELEASE})
	                       
  elseif(VIRTUOSE_LIBRARIES_DEBUG)
    set(VIRTUOSE_LIBRARIES ${VIRTUOSE_LIBRARIES_DEBUG})
  elseif(VIRTUOSE_LIBRARIES_RELEASE)
    set(VIRTUOSE_LIBRARIES ${VIRTUOSE_LIBRARIES_RELEASE})
  endif()
endif()  

if(VIRTUOSE_LIBRARIES AND VIRTUOSE_INCLUDE_DIRS)
  set(VIRTUOSE_FOUND TRUE)
else()
  set(VIRTUOSE_FOUND FALSE)
endif()
  
mark_as_advanced(
  VIRTUOSE_INCLUDE_DIRS
  VIRTUOSE_LIBRARIES
  VIRTUOSE_LIBRARIES_DEBUG
  VIRTUOSE_LIBRARIES_RELEASE
  VIRTUOSE_INC_SEARCH_PATH
  VIRTUOSE_LIB_SEARCH_PATH
)
