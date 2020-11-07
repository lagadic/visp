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
  /opt/OpenBLAS/lib
  /usr/local/opt/openblas/lib
  /usr/local/lib
  /usr/local/lib64
  /usr/lib
  /usr/lib/openblas-base
  /usr/lib64
  /lib/
  /lib/openblas-base
  /lib64/
)

find_path(OpenBLAS_INCLUDE_DIR NAMES cblas.h PATHS ${OpenBLAS_INCLUDE_SEARCH_PATHS} NO_DEFAULT_PATH)

if(NOT OpenBLAS_INCLUDE_DIR)
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
  vp_get_version_from_pkg2(blas-openblas "${OpenBLAS_LIB_DIR}/pkgconfig" OpenBLAS_VERSION)
  if(NOT OpenBLAS_VERSION)
    vp_get_version_from_pkg(blas-openblas "${OpenBLAS_LIB_DIR}/pkgconfig" OpenBLAS_VERSION)
  endif()

  if(NOT OpenBLAS_VERSION)
    vp_get_version_from_pkg2(openblas "${OpenBLAS_LIB_DIR}/pkgconfig" OpenBLAS_VERSION)
  endif()
  if(NOT OpenBLAS_VERSION)
    vp_get_version_from_pkg(openblas "${OpenBLAS_LIB_DIR}/pkgconfig" OpenBLAS_VERSION)
  endif()
else()
  if(OpenBLAS_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find OpenBLAS")
  endif()
endif()

mark_as_advanced(
  OpenBLAS
  OpenBLAS_INCLUDE_DIR
  OpenBLAS_LAPACK_LIB
  OpenBLAS_LIBRARIES
)
