

set(OpenBLAS_INCLUDE_SEARCH_PATHS
  /usr/include
  /usr/include/openblas
  /usr/include/openblas-base
  /usr/local/include
  /usr/local/include/openblas
  /usr/local/include/openblas-base
  /usr/local/opt/openblas/include
  /opt/OpenBLAS/include
  $ENV{OpenBLAS_HOME}
  $ENV{OpenBLAS_HOME}/include
)

set(OpenBLAS_LIB_SEARCH_PATHS
  /lib/
  /lib/openblas-base
  /lib64/
  /usr/lib
  /usr/lib/openblas-base
  /usr/lib64
  /usr/local/lib
  /usr/local/lib64
  /usr/local/opt/openblas/lib
  /opt/OpenBLAS/lib
  $ENV{OpenBLAS}cd
  $ENV{OpenBLAS}/lib
  $ENV{OpenBLAS_HOME}
  $ENV{OpenBLAS_HOME}/lib
)

find_path(OpenBLAS_INCLUDE_DIR NAMES cblas.h PATHS ${OpenBLAS_INCLUDE_SEARCH_PATHS})
if(UNIX)
  # Here we are looking for lapack library to be able to swith between OpenBLAS, Atlas and Netlib
  # with update-alternatives --config libblas.so.3-<multiarch>
  find_library(OpenBLAS_LIB NAMES lapack PATHS ${OpenBLAS_LIB_SEARCH_PATHS})
else()
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
  if(NOT OpenBLAS_FIND_QUIETLY)
    message(STATUS "Found OpenBLAS libraries: ${OpenBLAS_LIB}")
    message(STATUS "Found OpenBLAS include: ${OpenBLAS_INCLUDE_DIR}")
  endif()
else()
  if(OpenBLAS_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find OpenBLAS")
  endif()
endif()

mark_as_advanced(
  OpenBLAS_INCLUDE_DIR
  OpenBLAS_LIB
  OpenBLAS
)
