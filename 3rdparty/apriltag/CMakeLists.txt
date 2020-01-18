project(${APRILTAG_LIBRARY})

set(APRILTAG_MAJOR_VERSION 3 PARENT_SCOPE)
set(APRILTAG_MINOR_VERSION 1 PARENT_SCOPE)
set(APRILTAG_PATCH_VERSION 1 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB tag_srcs tag*.cpp)
file(GLOB lib_srcs *.cpp common/*.cpp)
file(GLOB lib_hdrs *.h common/*.h common/sys/*.h)

vp_list_filterout(lib_srcs common/unionfind.cpp) # has no symbols
vp_list_filterout(lib_srcs common/pjpeg.cpp)     # useless
vp_list_filterout(lib_srcs common/getopt.cpp)    # useless
vp_list_filterout(lib_hdrs common/getopt.h)      # useless

if(NOT WITH_APRILTAG_BIG_FAMILY)
  vp_list_filterout(lib_srcs tagCircle49h12.cpp)
  vp_list_filterout(lib_hdrs tagCircle49h12.h)
  vp_list_filterout(lib_srcs tagCustom48h12.cpp)
  vp_list_filterout(lib_hdrs tagCustom48h12.h)
  vp_list_filterout(lib_srcs tagStandard41h12.cpp)
  vp_list_filterout(lib_hdrs tagStandard41h12.h)
  vp_list_filterout(lib_srcs tagStandard52h13.cpp)
  vp_list_filterout(lib_hdrs tagStandard52h13.h)
endif()

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  endif()
endif()

add_library(${APRILTAG_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})
if(WIN32 AND MSVC)
  target_link_libraries (${APRILTAG_LIBRARY} ${PTHREADS_LIBRARY})
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${APRILTAG_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${APRILTAG_LIBRARY}
  PROPERTIES OUTPUT_NAME ${APRILTAG_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${APRILTAG_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${APRILTAG_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${APRILTAG_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

if(MSVC)
  if(BUILD_SHARED_LIBS)
    vp_warnings_disable(CMAKE_CXX_FLAGS /wd4018 /wd4098 /wd4244 /wd4267 /wd4305 /wd4334 /wd4244 /wd4838 /wd4996 /wd6011 /wd6385 /wd6386 /wd6387 /wd6011 /wd26451)
    vp_set_source_file_compile_flag(common/zmaxheap.cpp /wd4098 /wd4244)
    vp_set_source_file_compile_flag(common/workerpool.cpp /wd4018 /wd4244)
    vp_set_source_file_compile_flag(common/pam.cpp /wd4018)
    vp_set_source_file_compile_flag(common/string_util.cpp /wd4018 /wd4267 /wd4996)
    vp_set_source_file_compile_flag(tag16h5.cpp /wd4996)
    vp_set_source_file_compile_flag(tag25h7.cpp /wd4996)
    vp_set_source_file_compile_flag(tag25h9.cpp /wd4996)
    vp_set_source_file_compile_flag(tag36h10.cpp /wd4996)
    vp_set_source_file_compile_flag(tag36h11.cpp /wd4996)
    # disable optimization
    foreach(f ${tag_srcs})
      vp_set_source_file_compile_flag(${f} /O0)
    endforeach()
  else()
    vp_warnings_disable(CMAKE_CXX_FLAGS /wd4018 /wd4098 /wd4244 /wd4267 /wd4305 /wd4334 /wd4244 /wd4838 /wd4996)
  endif()
elseif(MINGW)
  foreach(f ${lib_srcs})
    vp_set_source_file_compile_flag(${f} -Wno-deprecated -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-float-equal -Wno-strict-overflow -Wno-type-limits -Wno-shift-negative-value -Wno-class-memaccess)
  endforeach()
  # disable optimization
  foreach(f ${tag_srcs})
    vp_set_source_file_compile_flag(${f} -O0)
  endforeach()
else()
  foreach(f ${lib_srcs})
    vp_set_source_file_compile_flag(${f} -Wno-deprecated -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-float-equal -Wno-strict-overflow -Wno-type-limits -Wno-shift-negative-value -Wno-shorten-64-to-32 -Wno-attributes)
  endforeach()
  # disable optimization
  foreach(f ${tag_srcs})
    vp_set_source_file_compile_flag(${f} -O0)
  endforeach()
endif()
