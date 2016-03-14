#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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

set(VIRTUOSE_INC_SEARCH_PATH /local/virtuose-6D/api)
set(VIRTUOSE_LIB_SEARCH_PATH /usr/lib)

if(CMAKE_CL_64)
    list(APPEND VIRTUOSE_LIB_SEARCH_PATH "/local/virtuose-6D/api/lib/x64")
else()
    list(APPEND VIRTUOSE_LIB_SEARCH_PATH "/local/virtuose-6D/api/lib/x86")
endif()


find_path(VIRTUOSE_INCLUDE_DIRS VirtuoseAPI.h
  PATHS
    $ENV{VIRTUOSE_HOME}/include
    ${VIRTUOSE_INC_SEARCH_PATH}
)

find_library(VIRTUOSE_LIBRARIES
  NAMES virtuose
  PATHS 
    $ENV{VIRTUOSE_HOME}/lib
    ${VIRTUOSE_LIB_SEARCH_PATH}
)

if(VIRTUOSE_LIBRARIES AND VIRTUOSE_INCLUDE_DIRS)
  set(VIRTUOSE_FOUND TRUE)
else()
  set(VIRTUOSE_FOUND FALSE)
endif()
  
mark_as_advanced(
  VIRTUOSE_INCLUDE_DIRS
  VIRTUOSE_LIBRARIES
  VIRTUOSE_INC_SEARCH_PATH
  VIRTUOSE_LIB_SEARCH_PATH
)
