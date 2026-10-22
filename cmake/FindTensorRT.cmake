#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# Try to find TensorRT.
# Once run this will define:
#
# TENSORRT_FOUND
# TENSORRT_INCLUDE_DIRS
# TENSORRT_LIBRARIES
# TENSORRT_VERSION
#
#############################################################################


set(TENSORRT_SEARCH_HINTS
  ${TENSORRT_HOME}
  $ENV{TENSORRT_HOME}
  ${TENSORRT_DIR}
  $ENV{TENSORRT_DIR}
)

set(TENSORRT_LIB_SUFFIXES lib)
set(TENSORRT_INC_SUFFIXES include)
# Debian packages install TensorRT in multiarch folders (/usr/lib/x86_64-linux-gnu, /usr/include/x86_64-linux-gnu).
# CMAKE_LIBRARY_ARCHITECTURE is empty with some compilers (e.g. conda ones), so the multiarch suffix is added explicitly.
if(UNIX AND NOT APPLE)
  if(CMAKE_LIBRARY_ARCHITECTURE)
    set(_tensorrt_multiarch ${CMAKE_LIBRARY_ARCHITECTURE})
  else()
    set(_tensorrt_multiarch ${CMAKE_SYSTEM_PROCESSOR}-linux-gnu)
  endif()
  list(APPEND TENSORRT_LIB_SUFFIXES lib/${_tensorrt_multiarch})
  list(APPEND TENSORRT_INC_SUFFIXES include/${_tensorrt_multiarch})
  unset(_tensorrt_multiarch)
endif()

# FindCUDA is deprecated since CMake 3.10 and removed by policy CMP0146 (CMake 3.27).
# Use FindCUDAToolkit when available (CMake 3.17+) and expose the legacy CUDA_* variables.
if(NOT CMAKE_VERSION VERSION_LESS 3.17)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND AND CUDA_cudart_LIBRARY)
    set(CUDA_FOUND ON)
    set(CUDA_VERSION ${CUDAToolkit_VERSION})
    set(CUDA_INCLUDE_DIRS ${CUDAToolkit_INCLUDE_DIRS})
    set(CUDA_LIBRARIES ${CUDA_cudart_LIBRARY})
  else()
    set(CUDA_FOUND OFF)
  endif()
else()
  find_package(CUDA QUIET)
  mark_as_advanced(
    CUDA_HOST_COMPILER
    CUDA_SDK_ROOT_DIR
    CUDA_TOOLKIT_ROOT_DIR
    CUDA_USE_STATIC_CUDA_RUNTIME
    CUDA_rt_LIBRARY
  )
endif()

find_library(NVINFER_LIBRARY
  NAMES nvinfer
  HINTS ${TENSORRT_SEARCH_HINTS}
  PATH_SUFFIXES ${TENSORRT_LIB_SUFFIXES}
)
find_library(NVINFERPLUGIN_LIBRARY
  NAMES nvinfer_plugin
  HINTS ${TENSORRT_SEARCH_HINTS}
  PATH_SUFFIXES ${TENSORRT_LIB_SUFFIXES}
)
find_library(NVONNXPARSER_LIBRARY
  NAMES nvonnxparser
  HINTS ${TENSORRT_SEARCH_HINTS}
  PATH_SUFFIXES ${TENSORRT_LIB_SUFFIXES}
)
find_library(NVONNXPARSERRUNTIME_LIBRARY
  NAMES nvonnxparser_runtime
  HINTS ${TENSORRT_SEARCH_HINTS}
  PATH_SUFFIXES ${TENSORRT_LIB_SUFFIXES}
)

find_path(NVINFER_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_SEARCH_HINTS}
  PATH_SUFFIXES ${TENSORRT_INC_SUFFIXES}
)

# If it is ALL there, export libraries as a single package
if(CUDA_FOUND AND NVINFER_LIBRARY AND NVINFERPLUGIN_LIBRARY AND NVONNXPARSER_LIBRARY AND NVINFER_INCLUDE_DIR)
  set(TENSORRT_LIBRARIES ${CUDA_LIBRARIES} ${NVINFER_LIBRARY} ${NVINFERPLUGIN_LIBRARY} ${NVONNXPARSER_LIBRARY})
  if(NVONNXPARSERRUNTIME_LIBRARY)
    list(APPEND TENSORRT_LIBRARIES ${NVONNXPARSERRUNTIME_LIBRARY})
  endif()
  set(TENSORRT_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} ${NVINFER_INCLUDE_DIR})
  set(TENSORRT_FOUND ON)

  # Version is in NvInferVersion.h since TensorRT 6, in NvInfer.h before
  if(EXISTS "${NVINFER_INCLUDE_DIR}/NvInferVersion.h")
    set(_tensorrt_version_file "${NVINFER_INCLUDE_DIR}/NvInferVersion.h")
  else()
    set(_tensorrt_version_file "${NVINFER_INCLUDE_DIR}/NvInfer.h")
  endif()
  vp_parse_header("${_tensorrt_version_file}" TENSORRT_VERSION_LINES NV_TENSORRT_MAJOR NV_TENSORRT_MINOR NV_TENSORRT_PATCH)
  # Since TensorRT 10.x NV_TENSORRT_MAJOR is an alias of TRT_MAJOR_ENTERPRISE that holds the numeric value
  vp_parse_header("${_tensorrt_version_file}" TENSORRT_VERSION_LINES TRT_MAJOR_ENTERPRISE TRT_MINOR_ENTERPRISE TRT_PATCH_ENTERPRISE)
  if(NV_TENSORRT_MAJOR)
    set(TENSORRT_VERSION "${NV_TENSORRT_MAJOR}.${NV_TENSORRT_MINOR}.${NV_TENSORRT_PATCH}")
  elseif(TRT_MAJOR_ENTERPRISE)
    set(TENSORRT_VERSION "${TRT_MAJOR_ENTERPRISE}.${TRT_MINOR_ENTERPRISE}.${TRT_PATCH_ENTERPRISE}")
  else()
    set(TENSORRT_VERSION "n/a")
  endif()
else()
  set(TENSORRT_FOUND OFF)
endif()

mark_as_advanced(
  NVINFER_INCLUDE_DIR
  NVINFER_LIBRARY
  NVINFERPLUGIN_LIBRARY
  NVONNXPARSER_LIBRARY
  NVONNXPARSERRUNTIME_LIBRARY
)
