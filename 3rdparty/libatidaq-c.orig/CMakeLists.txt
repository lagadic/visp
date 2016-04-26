project(ati)

cmake_minimum_required(VERSION 2.8)

########### already in VISP
# remove all matching elements from the list
macro(vp_list_filterout lst regex)
  foreach(item ${${lst}})
    if(item MATCHES "${regex}")
      list(REMOVE_ITEM ${lst} "${item}")
    endif()
  endforeach()
endmacro()

set(VP_COMPILER_FAIL_REGEX
    "command line option .* is valid for .* but not for C\\+\\+" # GNU
    "command line option .* is valid for .* but not for C" # GNU
    "unrecognized .*option"                     # GNU
    "unknown .*option"                          # Clang
    "ignoring unknown option"                   # MSVC
    "warning D9002"                             # MSVC, any lang
    "option .*not supported"                    # Intel
    "[Uu]nknown option"                         # HP
    "[Ww]arning: [Oo]ption"                     # SunPro
    "command option .* is not recognized"       # XL
    "not supported in this configuration; ignored"       # AIX
    "File with unknown suffix passed to linker" # PGI
    "WARNING: unknown flag:"                    # Open64
  )

macro(vp_check_compiler_flag LANG FLAG RESULT)
  if(NOT DEFINED ${RESULT})
    if("_${LANG}_" MATCHES "_CXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.cxx")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_C_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c")
      if("${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_C_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main(void) { return 0; }\n")
      else()
        file(WRITE "${_fname}" "#pragma\nint main(void) { return 0; }\n")
      endif()
    elseif("_${LANG}_" MATCHES "_OBJCXX_")
      set(_fname "${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.mm")
      if("${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror " OR "${CMAKE_CXX_FLAGS} ${FLAG} " MATCHES "-Werror=unknown-pragmas ")
        file(WRITE "${_fname}" "int main() { return 0; }\n")
      else()
        file(WRITE "${_fname}" "#pragma\nint main() { return 0; }\n")
      endif()
    else()
      unset(_fname)
    endif()
    if(_fname)
      message(STATUS "Performing Test ${RESULT}")
      try_compile(${RESULT}
        "${CMAKE_BINARY_DIR}"
        "${_fname}"
        COMPILE_DEFINITIONS "${FLAG}"
        OUTPUT_VARIABLE OUTPUT)

      foreach(_regex ${VP_COMPILER_FAIL_REGEX})
        if("${OUTPUT}" MATCHES "${_regex}")
          set(${RESULT} 0)
          break()
        endif()
      endforeach()

      if(${RESULT})
        set(${RESULT} 1 CACHE INTERNAL "Test ${RESULT}")
        message(STATUS "Performing Test ${RESULT} - Success")
      else()
        message(STATUS "Performing Test ${RESULT} - Failed")
        set(${RESULT} "" CACHE INTERNAL "Test ${RESULT}")
      endif()
    else()
      set(${RESULT} 0)
    endif()
  endif()
endmacro()

macro(vp_check_flag_support lang flag varname)
  if("_${lang}_" MATCHES "_CXX_")
    set(_lang CXX)
  elseif("_${lang}_" MATCHES "_C_")
    set(_lang C)
  else()
    set(_lang ${lang})
  endif()

  string(TOUPPER "${flag}" ${varname})
  string(REGEX REPLACE "^(/|-)" "HAVE_${_lang}_" ${varname} "${${varname}}")
  string(REGEX REPLACE " -|-|=| |\\." "_" ${varname} "${${varname}}")

  vp_check_compiler_flag("${_lang}" "${ARGN} ${flag}" ${${varname}})
endmacro()

# turns off warnings
macro(vp_warnings_disable)
    set(_flag_vars "")
    set(_msvc_warnings "")
    set(_gxx_warnings "")
    foreach(arg ${ARGN})
      if(arg MATCHES "^CMAKE_")
        list(APPEND _flag_vars ${arg})
      elseif(arg MATCHES "^/wd")
        list(APPEND _msvc_warnings ${arg})
      elseif(arg MATCHES "^-W")
        list(APPEND _gxx_warnings ${arg})
      endif()
    endforeach()
    if(MSVC AND _msvc_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_msvc_warnings})
          set(${var} "${${var}} ${warning}")
        endforeach()
      endforeach()
    elseif((CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")) AND _gxx_warnings AND _flag_vars)
      foreach(var ${_flag_vars})
        foreach(warning ${_gxx_warnings})
          if(NOT warning MATCHES "^-Wno-")
            string(REPLACE "${warning}" "" ${var} "${${var}}")
            string(REPLACE "-W" "-Wno-" warning "${warning}")
          endif()
          vp_check_flag_support(${var} "${warning}" _varname)
          if(${_varname})
            set(${var} "${${var}} ${warning}")
          endif()
        endforeach()
      endforeach()
    endif()
    unset(_flag_vars)
    unset(_msvc_warnings)
    unset(_gxx_warnings)
endmacro()


set(ATIDAQ_LIBRARY atidaq)

add_subdirectory(ATIDAQ)

include_directories(${PROJECT_SOURCE_DIR}/ATIDAQ)
#add_executable(ftconvert Samples/ftconvert.c)
#target_link_libraries(ftconvert ${ATIDAQ_LIBRARY})
add_executable(ftconvert test/ftconvert.cpp)
target_link_libraries(ftconvert ${ATIDAQ_LIBRARY})
