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
# Try to find libBiclops, libPMD and libUtils for Biclops head.
# Once run this will define:
#
# BICLOPS_FOUND
# BICLOPS_INCLUDE_DIRS
# BICLOPS_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

find_path(ARSDK_INCLUDE_DIR libARCommands/ARCommands.h
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/include"
)

find_library(ARSDK_ARController_LIBRARY
  NAMES arcontroller
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARSAL_LIBRARY
  NAMES arsal
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARNetwork_LIBRARY
  NAMES arnetwork
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARNetworkAL_LIBRARY
  NAMES arnetworkal
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARCommands_LIBRARY
  NAMES arcommands
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARDiscovery_LIBRARY
  NAMES ardiscovery
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARDataTransfer_LIBRARY
  NAMES ardatatransfer
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARMedia_LIBRARY
  NAMES armedia
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARStream_LIBRARY
  NAMES arstream
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ARStream2_LIBRARY
  NAMES arstream2
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_futils_LIBRARY
  NAMES futils
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_mux_LIBRARY
  NAMES mux
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_pomp_LIBRARY
  NAMES pomp
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_rtsp_LIBRARY
  NAMES rtsp
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_ulog_LIBRARY
  NAMES ulog
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

find_library(ARSDK_sdp_LIBRARY
  NAMES sdp
  PATHS
    "$ENV{ARSDK_DIR}/out/arsdk-native/staging/usr/lib"
)

if(ARSDK_INCLUDE_DIR AND ARSDK_ARController_LIBRARY AND ARSDK_ARSAL_LIBRARY AND ARSDK_ARNetwork_LIBRARY
   AND ARSDK_ARNetworkAL_LIBRARY AND ARSDK_ARCommands_LIBRARY AND ARSDK_ARDiscovery_LIBRARY
   AND ARSDK_ARDataTransfer_LIBRARY AND ARSDK_ARMedia_LIBRARY AND ARSDK_ARStream_LIBRARY AND ARSDK_ARStream2_LIBRARY
   AND ARSDK_futils_LIBRARY AND ARSDK_mux_LIBRARY AND ARSDK_pomp_LIBRARY AND ARSDK_rtsp_LIBRARY AND ARSDK_ulog_LIBRARY
   AND ARSDK_sdp_LIBRARY)
  set(ARSDK_FOUND TRUE)
  set(ARSDK_INCLUDE_DIRS ${ARSDK_INCLUDE_DIR})
  set(ARSDK_LIBRARIES ${ARSDK_ARController_LIBRARY} ${ARSDK_ARSAL_LIBRARY} ${ARSDK_ARNetwork_LIBRARY}
      ${ARSDK_ARNetworkAL_LIBRARY} ${ARSDK_ARCommands_LIBRARY} ${ARSDK_ARDiscovery_LIBRARY} ${ARSDK_ARDataTransfer_LIBRARY}
      ${ARSDK_ARMedia_LIBRARY} ${ARSDK_ARStream_LIBRARY} ${ARSDK_ARStream2_LIBRARY} ${ARSDK_futils_LIBRARY} ${ARSDK_mux_LIBRARY}
      ${ARSDK_pomp_LIBRARY} ${ARSDK_rtsp_LIBRARY} ${ARSDK_ulog_LIBRARY} ${ARSDK_sdp_LIBRARY})
else()
  set(ARSDK_FOUND FALSE)
endif()

mark_as_advanced(
  ARSDK_INCLUDE_DIR
  ARSDK_ARController_LIBRARY
  ARSDK_ARSAL_LIBRARY
  ARSDK_ARNetwork_LIBRARY
  ARSDK_ARNetworkAL_LIBRARY
  ARSDK_ARCommands_LIBRARY
  ARSDK_ARDiscovery_LIBRARY
  ARSDK_ARDataTransfer_LIBRARY
  ARSDK_ARMedia_LIBRARY
  ARSDK_ARStream_LIBRARY
  ARSDK_ARStream2_LIBRARY
  ARSDK_futils_LIBRARY
  ARSDK_mux_LIBRARY
  ARSDK_pomp_LIBRARY
  ARSDK_rtsp_LIBRARY
  ARSDK_ulog_LIBRARY
  ARSDK_sdp_LIBRARY
)
