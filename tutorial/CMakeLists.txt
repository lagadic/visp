project(ViSP-tutorial)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

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

visp_add_subdirectory(bridge/opencv                 REQUIRED_DEPS visp_core visp_io)
visp_add_subdirectory(computer-vision               REQUIRED_DEPS visp_core visp_blob visp_vision visp_io visp_gui)
visp_add_subdirectory(grabber                       REQUIRED_DEPS visp_core visp_sensor visp_io visp_gui)
visp_add_subdirectory(detection/barcode             REQUIRED_DEPS visp_core visp_detection visp_io visp_gui visp_sensor)
visp_add_subdirectory(detection/face                REQUIRED_DEPS visp_core visp_detection visp_io visp_gui visp_sensor)
visp_add_subdirectory(detection/matching            REQUIRED_DEPS visp_core visp_vision visp_io visp_gui)
visp_add_subdirectory(detection/object              REQUIRED_DEPS visp_core visp_vision visp_mbt visp_io visp_gui)
visp_add_subdirectory(detection/tag                 REQUIRED_DEPS visp_core visp_detection visp_io visp_gui visp_sensor)
visp_add_subdirectory(image                         REQUIRED_DEPS visp_core visp_io visp_gui)
visp_add_subdirectory(robot/pioneer                 REQUIRED_DEPS visp_core visp_robot visp_vs visp_gui)
visp_add_subdirectory(simulator/image               REQUIRED_DEPS visp_core visp_robot visp_io visp_gui)
visp_add_subdirectory(trace                         REQUIRED_DEPS visp_core)
visp_add_subdirectory(tracking/blob                 REQUIRED_DEPS visp_core visp_blob visp_io visp_gui visp_sensor)
visp_add_subdirectory(tracking/keypoint             REQUIRED_DEPS visp_core visp_vision visp_klt visp_io visp_gui visp_sensor)
visp_add_subdirectory(tracking/model-based/generic  REQUIRED_DEPS visp_core visp_mbt visp_io visp_gui)
visp_add_subdirectory(tracking/model-based/stereo   REQUIRED_DEPS visp_core visp_mbt visp_io visp_gui)
visp_add_subdirectory(tracking/model-based/edges    REQUIRED_DEPS visp_core visp_mbt visp_io visp_gui)
visp_add_subdirectory(tracking/model-based/hybrid   REQUIRED_DEPS visp_core visp_mbt visp_klt visp_io visp_gui)
visp_add_subdirectory(tracking/model-based/keypoint REQUIRED_DEPS visp_core visp_mbt visp_klt visp_io visp_gui)
visp_add_subdirectory(tracking/template-tracker     REQUIRED_DEPS visp_core visp_tt visp_io visp_gui)
visp_add_subdirectory(tracking/moving-edges         REQUIRED_DEPS visp_core visp_me visp_io visp_gui visp_sensor)
visp_add_subdirectory(visual-servo/ibvs             REQUIRED_DEPS visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui visp_ar)
