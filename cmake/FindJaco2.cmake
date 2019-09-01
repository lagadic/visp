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
# Try to find kinova robot jaco2 library API to dial with the robot jaco-2
#
# JACO2_FOUND
# JACO2_INCLUDE_DIRS
# JACO2_LIBRARIES
# JACO2_VERSION
#
# Authors:
# zubair arif
#
#############################################################################

set(JACO2_INC_SEARCH_PATH /usr/include)
set(JACO2_LIB_SEARCH_PATH /usr/lib)


if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND JACO2_INC_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/include")
    list(APPEND JACO2_LIB_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/x64")
  else()
    list(APPEND JACO2_INC_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API/include")
    list(APPEND JACO2_LIB_SEARCH_PATH C:/Program Files (x86)/JACO-SDK/API/x86")
  endif()
endif()

find_path(JACO2_INCLUDE_DIRS .h
  PATHS
    $ENV{JACO2_HOME}/include/CommandLayer.h
    $ENV{JACO2_HOME}/include/CommunicationLayer.h
    $ENV{JACO2_HOME}/include/KinovaTypes.h
    ${JACO2_INC_SEARCH_PATH}
)

find_library(JACO2_LIBRARIES
  NAMES CommandLayerEthernet.dll CommandLayerWindows.dll CommunicationLayerEthernet.dll CommunicationLayerWindows.dll
  PATHS 
    $ENV{JACO2_HOME}/lib/x64
    ${JACO2_LIB_SEARCH_PATH}
)

if(JACO2_LIBRARIES AND JACO2_INCLUDE_DIRS)
  set(JACO2_FOUND TRUE)
else()
  set(JACO2_FOUND FALSE)
endif()
  
mark_as_advanced(
  JACO2_INCLUDE_DIRS
  JACO2_LIBRARIES
  JACO2_INC_SEARCH_PATH
 JACO2E_LIB_SEARCH_PATH
)
