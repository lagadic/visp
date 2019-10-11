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
# Try to find Kinova Jaco SDK
#
# JACOSDK_FOUND
# JACOSDK_INCLUDE_DIRS
# JACOSDK_LIBRARIES
#
# Authors:
# Zubair Arif
# HIT University-Harbin China
#############################################################################

set(JACOSDK_INC_SEARCH_PATH /usr/JACO-SDK/API/include)
list(APPEND JACOSDK_INC_SEARCH_PATH /opt/JACO-SDK/API/include)
list(APPEND JACOSDK_INC_SEARCH_PATH "C:/Program Files (x86)/JACO-SDK/API")

find_path(JACOSDK_INCLUDE_DIRS
  NAMES CommandLayer.h CommunicationLayer.h KinovaTypes.h
  PATHS
    $ENV{JACOSDK_HOME}
    ${JACOSDK_INC_SEARCH_PATH}
)

if(UNIX)
  set(JACOSDK_LIBRARIES ${CMAKE_DL_LIBS})
else()
  set(JACOSDK_LIBRARIES "")
endif()
  
if(JACOSDK_INCLUDE_DIRS)
  set(JACOSDK_FOUND TRUE)
else()
  set(JACOSDK_FOUND FALSE)
endif()

mark_as_advanced(
  JACOSDK_INCLUDE_DIRS
  JACOSDK_LIBRARIES
  JACOSDK_INC_SEARCH_PATH
)

