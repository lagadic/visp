################################################################################
#
# \file      cmake/FindMKL.cmake
# \author    J. Bakosi
# \copyright 2012-2015, Jozsef Bakosi, 2016, Los Alamos National Security, LLC.
# \brief     Find the Math Kernel Library from Intel
# \date      Thu 26 Jan 2017 02:05:50 PM MST
#
# Modified by F. Spindler to make var advanced
################################################################################

# Find the Math Kernel Library from Intel
#
#  MKL_FOUND - System has MKL
#  MKL_INCLUDE_DIRS - MKL include files directories
#  MKL_LIBRARIES - The MKL libraries
#  MKL_INTERFACE_LIBRARY - MKL interface library
#  MKL_SEQUENTIAL_LAYER_LIBRARY - MKL sequential layer library
#  MKL_CORE_LIBRARY - MKL core library
#
#  The MKLROOT (CMake or environment) and INTEL (environment) variables are
#  used to find the library.
#  Everything else is ignored. If MKL is found "-DMKL_ILP64" is added to
#  CMAKE_C_FLAGS and CMAKE_CXX_FLAGS.
#
#  Example usage:
#
#  find_package(MKL)
#  if(MKL_FOUND)
#    target_link_libraries(TARGET ${MKL_LIBRARIES})
#  endif()

# If already in cache, be silent
if(MKL_INCLUDE_DIRS AND MKL_LIBRARIES AND MKL_INTERFACE_LIBRARY AND
   MKL_SEQUENTIAL_LAYER_LIBRARY AND MKL_CORE_LIBRARY)
  set(MKL_FIND_QUIETLY TRUE)
endif()

# Only support of 64 bits architecture
if(CMAKE_CXX_SIZEOF_DATA_PTR EQUAL 8)
  include(CheckTypeSize)
  check_type_size(int _sizeof_int)
  if(_sizeof_int EQUAL 4)
    set(INT_LIB "mkl_intel_lp64")
  else()
    set(INT_LIB "mkl_intel_ilp64")
  endif()

  set(SEQ_LIB "mkl_sequential")
  set(THR_LIB "mkl_intel_thread")
  set(COR_LIB "mkl_core")
endif()

set(MKL_SEARCH_HINTS
  ${MKLROOT}
  $ENV{MKLROOT}
)
if(DEFINED ENV{INTEL})
  list(APPEND MKL_SEARCH_HINTS "$ENV{INTEL}/mkl")
endif()
set(MKL_LIB_SUFFIXES lib lib/intel64)

find_path(MKL_INCLUDE_DIR
  NAMES mkl.h
  HINTS ${MKL_SEARCH_HINTS}
  PATH_SUFFIXES include
)

find_library(MKL_INTERFACE_LIBRARY
  NAMES ${INT_LIB}
  HINTS ${MKL_SEARCH_HINTS}
  PATH_SUFFIXES ${MKL_LIB_SUFFIXES}
  NO_DEFAULT_PATH
)

find_library(MKL_SEQUENTIAL_LAYER_LIBRARY
  NAMES ${SEQ_LIB}
  HINTS ${MKL_SEARCH_HINTS}
  PATH_SUFFIXES ${MKL_LIB_SUFFIXES}
  NO_DEFAULT_PATH
)

find_library(MKL_CORE_LIBRARY
  NAMES ${COR_LIB}
  HINTS ${MKL_SEARCH_HINTS}
  PATH_SUFFIXES ${MKL_LIB_SUFFIXES}
  NO_DEFAULT_PATH
)

if(MKL_INCLUDE_DIR AND MKL_INTERFACE_LIBRARY AND MKL_SEQUENTIAL_LAYER_LIBRARY AND MKL_CORE_LIBRARY)
  set(MKL_INCLUDE_DIRS ${MKL_INCLUDE_DIR})
  set(MKL_LIBRARIES ${MKL_INTERFACE_LIBRARY} ${MKL_SEQUENTIAL_LAYER_LIBRARY} ${MKL_CORE_LIBRARY})
else()
  set(MKL_INCLUDE_DIRS "")
  set(MKL_LIBRARIES "")
  set(MKL_INTERFACE_LIBRARY "")
  set(MKL_SEQUENTIAL_LAYER_LIBRARY "")
  set(MKL_CORE_LIBRARY "")
endif()

# Handle the QUIETLY and REQUIRED arguments and set MKL_FOUND to TRUE if
# all listed variables are TRUE.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MKL DEFAULT_MSG MKL_LIBRARIES MKL_INCLUDE_DIRS MKL_INTERFACE_LIBRARY MKL_SEQUENTIAL_LAYER_LIBRARY MKL_CORE_LIBRARY)

mark_as_advanced(
  MKL_INCLUDE_DIR
  MKL_INTERFACE_LIBRARY
  MKL_SEQUENTIAL_LAYER_LIBRARY
  MKL_CORE_LIBRARY
)
