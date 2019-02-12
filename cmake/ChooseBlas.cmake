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

set(BLAS/LAPACK "Open" CACHE STRING "Selected BLAS library")
set_property(CACHE BLAS/LAPACK PROPERTY STRINGS "Atlas;Open;MKL;Netlib")

set(USE_LAPACK FALSE)
set(USE_ATLAS FALSE)
set(USE_OPENBLAS FALSE)
set(USE_MKL FALSE)
set(USE_LAPACK_NETLIB FALSE)

if(BLAS/LAPACK STREQUAL "Atlas" OR BLAS/LAPACK STREQUAL "atlas")
  find_package(Atlas)

  if(ATLAS_FOUND)
    set(USE_LAPACK TRUE)
    set(USE_ATLAS TRUE)
  endif()
elseif(BLAS/LAPACK STREQUAL "Open" OR BLAS/LAPACK STREQUAL "open")
  find_package(OpenBLAS)

  if(OpenBLAS_FOUND)
    set(USE_LAPACK TRUE)
    set(USE_OPENBLAS TRUE)
  endif()
elseif(BLAS/LAPACK STREQUAL "MKL" OR BLAS/LAPACK STREQUAL "mkl")
  find_package(MKL)

  if(MKL_FOUND)
    set(USE_LAPACK TRUE)
    set(USE_MKL TRUE)
  endif()
elseif(BLAS/LAPACK STREQUAL "Netlib" OR BLAS/LAPACK STREQUAL "netlib")
  find_package(LAPACK_C)

  if(LAPACK_C_FOUND)
    set(USE_LAPACK TRUE)
    set(USE_LAPACK_NETLIB TRUE)
  endif()
endif()
