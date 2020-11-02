# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

set(AVAILABLE_VENDOR TRUE)

# Since the FindLAPACK.cmake provided with CMake is for Fortran language,
# we use here specific cmake Find files
find_package(MKL)
find_package(OpenBLAS)
find_package(Atlas)
find_package(GSL)
find_package(Netlib)

set(USE_LAPACK FALSE)
set(USE_MKL FALSE)
set(USE_OPENBLAS FALSE)
set(USE_ATLAS FALSE)
set(USE_GSL FALSE)
set(USE_NETLIB FALSE)

if(MKL_FOUND)
  set(USE_BLAS/LAPACK "MKL" CACHE STRING "Selected BLAS library")
elseif(OpenBLAS_FOUND)
  set(USE_BLAS/LAPACK "OpenBLAS" CACHE STRING "Selected BLAS library")
elseif(ATLAS_FOUND)
  set(USE_BLAS/LAPACK "Atlas" CACHE STRING "Selected BLAS library")
elseif(GSL_FOUND)
  set(USE_BLAS/LAPACK "GSL" CACHE STRING "Selected BLAS library")
elseif(NETLIB_FOUND)
  set(USE_BLAS/LAPACK "Netlib" CACHE STRING "Selected BLAS library")
else()
  set(AVAILABLE_VENDOR FALSE)
endif()

if(AVAILABLE_VENDOR)
  set(USE_LAPACK TRUE)
  set(FIRST_PROPERTY TRUE)

  if(MKL_FOUND)
    if(FIRST_PROPERTY)
      set(FIRST_PROPERTY FALSE)
      set_property(CACHE USE_BLAS/LAPACK PROPERTY STRINGS "MKL")
    else()
      set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";MKL")
    endif()
  endif()
  if(OpenBLAS_FOUND)
    if(FIRST_PROPERTY)
      set(FIRST_PROPERTY FALSE)
      set_property(CACHE USE_BLAS/LAPACK PROPERTY STRINGS "OpenBLAS")
    else()
      set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";OpenBLAS")
    endif()
  endif()
  if(ATLAS_FOUND)
    if(FIRST_PROPERTY)
      set(FIRST_PROPERTY FALSE)
      set_property(CACHE USE_BLAS/LAPACK PROPERTY STRINGS "Atlas")
    else()
      set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";Atlas")
    endif()
  endif()
  if(GSL_FOUND)
    if(FIRST_PROPERTY)
      set(FIRST_PROPERTY FALSE)
      set_property(CACHE USE_BLAS/LAPACK PROPERTY STRINGS "GSL")
    else()
      set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";GSL")
    endif()
  endif()
  if(NETLIB_FOUND)
    if(FIRST_PROPERTY)
      set(FIRST_PROPERTY FALSE)
      set_property(CACHE USE_BLAS/LAPACK PROPERTY STRINGS "Netlib")
    else()
      set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";Netlib")
    endif()
  endif()

  set_property(CACHE USE_BLAS/LAPACK APPEND_STRING PROPERTY STRINGS ";OFF")

  if(USE_BLAS/LAPACK STREQUAL "MKL" OR USE_BLAS/LAPACK STREQUAL "mkl")
    if(MKL_FOUND)
      set(USE_MKL TRUE)
    endif()
  elseif(USE_BLAS/LAPACK STREQUAL "OpenBLAS" OR USE_BLAS/LAPACK STREQUAL "openblas")
    if(OpenBLAS_FOUND)
      set(USE_OPENBLAS TRUE)
    endif()
  elseif(USE_BLAS/LAPACK STREQUAL "Atlas" OR USE_BLAS/LAPACK STREQUAL "atlas")
    if(ATLAS_FOUND)
      set(USE_ATLAS TRUE)
    endif()
  elseif(USE_BLAS/LAPACK STREQUAL "GSL" OR USE_BLAS/LAPACK STREQUAL "gsl")
    if(GSL_FOUND)
      set(USE_GSL TRUE)
    endif()
  elseif(USE_BLAS/LAPACK STREQUAL "Netlib" OR USE_BLAS/LAPACK STREQUAL "netlib")
    if(NETLIB_FOUND)
      set(USE_NETLIB TRUE)
    endif()
  elseif(USE_BLAS/LAPACK STREQUAL "OFF")
    set(USE_MKL FALSE)
    set(USE_OPENBLAS FALSE)
    set(USE_ATLAS FALSE)
    set(USE_GSL FALSE)
    set(USE_NETLIB FALSE)
    set(USE_LAPACK FALSE)
  endif()

  if(WITH_LAPACK)
    if(USE_LAPACK)
      set(WITH_LAPACK OFF CACHE BOOL "Build lapack as built-in library" FORCE)
    endif()
  endif()

  if(NOT USE_LAPACK)
    set(WITH_LAPACK ON CACHE BOOL "Build lapack as built-in library" FORCE)
  endif()
endif()
