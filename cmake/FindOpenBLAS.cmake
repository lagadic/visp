# COPYRIGHT
#
# All contributions by the University of California:
# Copyright (c) 2014-2017 The Regents of the University of California (Regents)
# All rights reserved.
#
# All other contributions:
# Copyright (c) 2014-2017, the respective contributors
# All rights reserved.
#
# Caffe uses a shared copyright model: each contributor holds copyright over
# their contributions to Caffe. The project versioning records all such
# contribution and copyright details. If a contributor wants to further mark
# their specific copyright on a particular contribution, they should indicate
# their copyright solely in the commit message of the change when it is
# committed.
#
# LICENSE
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# CONTRIBUTION AGREEMENT
#
# By contributing to the BVLC/caffe repository through pull-request, comment,
# or otherwise, the contributor releases their content to the
# license and copyright terms herein.
#
#
# Find the OpenBLAS (and Lapack) libraries
#
# The following variables are optionally searched for defaults
#  OpenBLAS_HOME:            Base directory where all OpenBLAS components are found
#
# The following are set after configuration is done:
#  OpenBLAS_FOUND
#  OpenBLAS_INCLUDE_DIR
#  OpenBLAS_LAPACK_LIB
#  OpenBLAS_LIBRARIES
#  OpenBLAS_VERSION

set(OpenBLAS_INCLUDE_SEARCH_PATHS
  $ENV{OpenBLAS_HOME}
  $ENV{OpenBLAS_HOME}/include
  $ENV{OpenBLAS_DIR}
  $ENV{OpenBLAS_DIR}/include
  /opt/OpenBLAS/include
  /usr/local/opt/openblas/include
  /usr/local/include/openblas
  /usr/local/include/openblas-base
  /usr/local/include
  /usr/include/openblas
  /usr/include/openblas-base
  /usr/include
)

set(OpenBLAS_LIB_SEARCH_PATHS
  $ENV{OpenBLAS}
  $ENV{OpenBLAS}/lib
  $ENV{OpenBLAS_HOME}
  $ENV{OpenBLAS_HOME}/lib
  $ENV{OpenBLAS_DIR}
  $ENV{OpenBLAS_DIR}/lib
  /opt/OpenBLAS/lib
  /usr/local/opt/openblas/lib
  /usr/local/lib
  /usr/local/lib64
  /usr/lib/openblas-base
  /usr/lib
  /usr/lib64
  /lib/openblas-base
  /lib/
  /lib64/
)

find_path(OpenBLAS_INCLUDE_DIR NAMES cblas.h PATHS ${OpenBLAS_INCLUDE_SEARCH_PATHS} NO_DEFAULT_PATH)
if(NOT OpenBLAS_INCLUDE_DIR)
  # Search in default system directories, e.g. in Ubuntu 20.04 (0.3.8+ds-1ubuntu0.20.04.1) cblas.h are in
  # /usr/include/x86_64-linux-gnu/cblas.h (will be found)
  # /usr/include/x86_64-linux-gnu/openblas-pthread/cblas.h
  # Both files are identical
  find_path(OpenBLAS_INCLUDE_DIR NAMES cblas.h PATHS ${OpenBLAS_INCLUDE_SEARCH_PATHS})
endif()

# Here we are looking for lapack library to be able to switch between OpenBLAS, Atlas and Netlib
# with update-alternatives --config libblas.so.3-<multiarch>
# and also for openblas library to be sure that openblas is installed
if(UNIX AND NOT APPLE)
  find_library(OpenBLAS_LAPACK_LIB NAMES lapack PATHS ${OpenBLAS_LIB_SEARCH_PATHS})
endif()
find_library(OpenBLAS_LIB NAMES openblas PATHS ${OpenBLAS_LIB_SEARCH_PATHS} NO_DEFAULT_PATH)
if(NOT OpenBLAS_LIB)
  # Search in default system directories, e.g. in Ubuntu 20.04 (0.3.8+ds-1ubuntu0.20.04.1) libopenblas are in
  # /usr/lib/x86_64-linux-gnu/libopenblas.so (will be found)
  # /usr/lib/x86_64-linux-gnu/openblas-pthread/libopenblas.so
  # Both files are identical
  find_library(OpenBLAS_LIB NAMES openblas PATHS ${OpenBLAS_LIB_SEARCH_PATHS})
endif()

set(OpenBLAS_FOUND ON)

# Check include files
if(NOT OpenBLAS_INCLUDE_DIR)
  set(OpenBLAS_FOUND OFF)
  message(STATUS "Could not find OpenBLAS include. Turning OpenBLAS_FOUND off")
endif()

# Check libraries
if(NOT OpenBLAS_LIB)
  set(OpenBLAS_FOUND OFF)
  message(STATUS "Could not find OpenBLAS lib. Turning OpenBLAS_FOUND off")
endif()

if(OpenBLAS_FOUND)
  if(NOT OpenBLAS_LAPACK_LIB)
    set(OpenBLAS_LIBRARIES ${OpenBLAS_LIB})
  else()
    set(OpenBLAS_LIBRARIES ${OpenBLAS_LAPACK_LIB})
  endif()
  if(NOT OpenBLAS_FIND_QUIETLY)
    message(STATUS "Found OpenBLAS libraries: ${OpenBLAS_LAPACK_LIB}")
    message(STATUS "Found OpenBLAS include: ${OpenBLAS_INCLUDE_DIR}")
  endif()

  get_filename_component(OpenBLAS_LIB_DIR ${OpenBLAS_LIB} PATH)
  vp_parse_header3(OpenBLAS "${OpenBLAS_INCLUDE_DIR}/openblas_config.h" "OPENBLAS_VERSION" OpenBLAS_VERSION)
else()
  if(OpenBLAS_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find OpenBLAS")
  endif()
endif()

mark_as_advanced(
  OpenBLAS
  OpenBLAS_INCLUDE_DIR
  OpenBLAS_LAPACK_LIB
  OpenBLAS_LIB
)
