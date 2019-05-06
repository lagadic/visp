set(AVAILABLE_CXX_STANDARD TRUE)

set(USE_CXX98 FALSE)
set(USE_CXX11 FALSE)
set(USE_CXX14 FALSE)

# Set default c++ standard to 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED FALSE)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++11 instead of -std=gnu++11
if(CMAKE_CXX11_COMPILE_FEATURES)
  set(CXX11_STANDARD_FOUND ON)
endif()

if(CMAKE_CXX98_COMPILE_FEATURES)
  list (FIND CMAKE_CXX98_COMPILE_FEATURES "cxx_std_98" _index)
  if (${_index} GREATER -1)
    set(CXX98_STANDARD_FOUND ON)
  endif()
endif()

if(CMAKE_CXX11_COMPILE_FEATURES)
  list (FIND CMAKE_CXX11_COMPILE_FEATURES "cxx_std_11" _index)
  if (${_index} GREATER -1)
    set(CXX11_STANDARD_FOUND ON)
  endif()
endif()

if(CMAKE_CXX14_COMPILE_FEATURES)
  list (FIND CMAKE_CXX14_COMPILE_FEATURES "cxx_std_14" _index)
  if (${_index} GREATER -1)
    set(CXX14_STANDARD_FOUND ON)
  endif()
endif()

if(CXX11_STANDARD_FOUND)
  set(USE_CXX_STANDARD "11" CACHE STRING "Selected cxx standard")
elseif(CXX98_STANDARD_FOUND)
  set(USE_CXX_STANDARD "98" CACHE STRING "Selected cxx standard")
elseif(CMAKE_CXX14_COMPILE_FEATURES)
  set(USE_CXX_STANDARD "14" CACHE STRING "Selected cxx standard")
else()
  set(AVAILABLE_CXX_STANDARD FALSE)
endif()

if(AVAILABLE_CXX_STANDARD)
  if(CXX98_STANDARD_FOUND)
    set_property(CACHE USE_CXX_STANDARD APPEND_STRING PROPERTY STRINGS ";98")
  endif()
  if(CXX11_STANDARD_FOUND)
    set_property(CACHE USE_CXX_STANDARD APPEND_STRING PROPERTY STRINGS ";11")
  endif()
  if(CXX14_STANDARD_FOUND)
    set_property(CACHE USE_CXX_STANDARD APPEND_STRING PROPERTY STRINGS ";14")
  endif()

  if(USE_CXX_STANDARD STREQUAL "98")
    if(CXX98_STANDARD_FOUND)
      set(CMAKE_CXX_STANDARD 98)
      set(USE_CXX98 TRUE)
    endif()
  elseif(USE_CXX_STANDARD STREQUAL "11")
    if(CXX11_STANDARD_FOUND)
      set(CMAKE_CXX_STANDARD 11)
      set(USE_CXX11 TRUE)
    endif()
  elseif(USE_CXX_STANDARD STREQUAL "14")
    if(CXX14_STANDARD_FOUND)
      set(CMAKE_CXX_STANDARD 14)
      set(USE_CXX14 TRUE)
    endif()
  endif()

endif()

# Setting CMAKE_CXX_STANDARD doesn't affect check_cxx_source_compiles() used to detect isnan() in FindIsNaN.cmake
# or erfc() in FindErfc.cmake.
# That's why we have the following lines that are used to set cxx flags corresponding to the c++ standard
vp_check_compiler_flag(CXX "-std=c++11" HAVE_STD_CXX11_FLAG "${PROJECT_SOURCE_DIR}/cmake/checks/cxx11.cpp")
if(HAVE_STD_CXX11_FLAG)
  set(CXX11_CXX_FLAGS "-std=c++11" CACHE STRING "C++ compiler flags for C++11 support")
  mark_as_advanced(CXX11_CXX_FLAGS)
endif()

vp_check_compiler_flag(CXX "-std=c++14" HAVE_STD_CXX14_FLAG "${PROJECT_SOURCE_DIR}/cmake/checks/cxx14.cpp")
if(HAVE_STD_CXX14_FLAG)
  set(CXX14_CXX_FLAGS "-std=c++14" CACHE STRING "C++ compiler flags for C++14 support")
  mark_as_advanced(CXX14_CXX_FLAGS)
endif()

if(CXX11_STANDARD_FOUND OR CXX11_CXX_FLAGS)
  set(CXX11_FOUND ON)
endif()

if(CXX14_STANDARD_FOUND OR CXX14_CXX_FLAGS)
  set(CXX14_FOUND ON)
endif()

