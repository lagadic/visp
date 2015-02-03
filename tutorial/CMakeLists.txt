project(ViSP-tutorial)

cmake_minimum_required(VERSION 2.6)

if(MSVC)
  if(NOT VISP_SHARED)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif()
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif()
    endforeach(flag_var)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
  endif()
endif()

add_subdirectory(computer-vision)
add_subdirectory(grabber)
add_subdirectory(detection/barcode)
add_subdirectory(detection/face)
add_subdirectory(detection/matching)
add_subdirectory(detection/object)
add_subdirectory(image)
add_subdirectory(robot/pioneer)
add_subdirectory(simulator/image)
add_subdirectory(trace)
#add_subdirectory(tracking/barcode)
add_subdirectory(tracking/blob)
add_subdirectory(tracking/keypoint)
add_subdirectory(tracking/model-based/edges)
add_subdirectory(tracking/model-based/hybrid)
add_subdirectory(tracking/model-based/keypoint)
add_subdirectory(tracking/template-tracker)
add_subdirectory(tracking/moving-edges)
add_subdirectory(visual-servo/ibvs)
