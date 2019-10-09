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
# Try to find Kinova Jaco Robot SDK
#
# JACO2_FOUND
# JACO2_INCLUDE_DIRS
# JACO2_LIBRARIES
#
# Authors:
# Zubair Arif
# HIT University-Harbin China
#############################################################################

set(JACO2_INC_SEARCH_PATH /usr/JACO-SDK/API/include)
set(JACO2_LIB_SEARCH_PATH /usr/JACO-SDK/API/lib)

if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND JACO2_INC_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/include")
    list(APPEND JACO2_LIB_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/lib/x64")
  else()
    list(APPEND JACO2_INC_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/include")
    list(APPEND JACO2_LIB_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/lib/x86")
  endif()
endif()

find_path(JACO2_INCLUDE_DIRS
  NAMES CommandLayer.h CommunicationLayer.h KinovaTypes.h
  PATHS
    $ENV{JACO2_HOME}/include
    ${JACO2_INC_SEARCH_PATH}
)

find_library(JACO2_LIBRARIES
  NAMES CommandLayerWindows CommunicationLayerWindows CommandLayerEthernet CommunicationLayerEthernet 
  PATHS 
    ${JACO2_LIBRARIES}
    ${JACO2_LIB_SEARCH_PATH}
    "C:/Program Files (x86)/JACO-SDK/API/lib/x64"
)

if(JACO2_LIBRARIES AND JACO2_INCLUDE_DIRS)
  set(JACO2_FOUND TRUE)
else()
  set(JACO2_FOUND FALSE)
endif()
  
message("JACO2_INCLUDE_DIRS: ${JACO2_INCLUDE_DIRS}")
message("JACO2_LIBRARIES: ${JACO2_LIBRARIES}")

mark_as_advanced(
  JACO2_INCLUDE_DIRS
  JACO2_LIBRARIES
  JACO2_INC_SEARCH_PATH
  JACO2_LIB_SEARCH_PATH
)

