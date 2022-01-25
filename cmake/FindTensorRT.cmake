#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2021 by Inria. All rights reserved.
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
# Try to find TensorRT.
# Once run this will define:
#
# TENSORRT_FOUND
# TENSORRT_INCLUDE_DIRS
# TENSORRT_LIBRARIES
# TENSORRT_VERSION
#
#############################################################################

set(TENSORRT_LIB_SEARCH_PATHS
  $ENV{TENSORRT_HOME}/lib
  $ENV{TENSORRT_DIR}/lib
  ${TENSORRT_HOME}/lib
  ${TENSORRT_DIR}/lib
  /usr/local/lib
  /usr/local/lib64
  /usr/lib
  /usr/lib64
)

set(TENSORRT_INC_SEARCH_PATHS
  $ENV{TENSORRT_HOME}/include
  $ENV{TENSORRT_DIR}/include
  ${TENSORRT_HOME}/include
  ${TENSORRT_DIR}/include
  /usr/local/include
  /usr/include
)


find_package(CUDA)
find_library(NVINFER_LIBRARY NAMES nvinfer PATHS ${TENSORRT_LIB_SEARCH_PATHS})
find_library(NVINFERPLUGIN_LIBRARY NAMES nvinfer_plugin PATHS ${TENSORRT_LIB_SEARCH_PATHS})
find_library(NVPARSERS_LIBRARY NAMES nvparsers PATHS ${TENSORRT_LIB_SEARCH_PATHS})
find_library(NVONNXPARSER_LIBRARY NAMES nvonnxparser PATHS ${TENSORRT_LIB_SEARCH_PATHS})
find_library(NVONNXPARSERRUNTIME_LIBRARY NAMES nvonnxparser_runtime PATHS ${TENSORRT_LIB_SEARCH_PATHS})

find_path(NVINFER_INCLUDE_DIR NvInfer.h PATHS ${TENSORRT_INC_SEARCH_PATHS})

# If it is ALL there, export libraries as a single package
if(CUDA_FOUND AND NVINFER_LIBRARY AND NVINFERPLUGIN_LIBRARY AND NVPARSERS_LIBRARY AND NVONNXPARSER_LIBRARY AND NVINFER_INCLUDE_DIR)
  list(APPEND TENSORRT_LIBRARIES ${CUDA_LIBRARIES} ${NVINFER_LIBRARY} ${NVINFERPLUGIN_LIBRARY} ${NVPARSERS_LIBRARY} ${NVONNXPARSER_LIBRARY})
  vp_parse_header("${NVINFER_INCLUDE_DIR}/NvInferVersion.h" TENSORRT_VERSION_LINES NV_TENSORRT_MAJOR NV_TENSORRT_MINOR NV_TENSORRT_PATCH NV_TENSORRT_BUILD)
  set(TENSORRT_VERSION "${NV_TENSORRT_MAJOR}.${NV_TENSORRT_MINOR}.${NV_TENSORRT_PATCH}.${NV_TENSORRT_BUILD}")
  if(NVONNXPARSERRUNTIME_LIBRARY)
    list(APPEND TENSORRT_LIBRARIES ${NVONNXPARSERRUNTIME_LIBRARY})
  endif()
  list(APPEND TENSORRT_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} ${NVINFER_INCLUDE_DIR})
  set(TENSORRT_FOUND ON)
else()
  set(TENSORRT_FOUND OFF)
endif()

mark_as_advanced(
  NVINFER_INCLUDE_DIR
  NVINFER_LIBRARY
  NVINFERPLUGIN_LIBRARY
  NVPARSERS_LIBRARY
  NVONNXPARSER_LIBRARY
  NVONNXPARSERRUNTIME_LIBRARY
  CUDA_HOST_COMPILER
  CUDA_SDK_ROOT_DIR
  CUDA_TOOLKIT_ROOT_DIR
  CUDA_USE_STATIC_CUDA_RUNTIME
  CUDA_rt_LIBRARY
  )
