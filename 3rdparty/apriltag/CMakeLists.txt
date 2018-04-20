project(${APRILTAG_LIBRARY})

set(APRILTAG_MAJOR_VERSION 0 PARENT_SCOPE)
set(APRILTAG_MINOR_VERSION 9 PARENT_SCOPE)
set(APRILTAG_PATCH_VERSION 8 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.c common/*.c)
file(GLOB lib_hdrs *.h common/*.h common/sys/*.h)

vp_list_filterout(lib_srcs common/unionfind.c) # has no symbols
vp_list_filterout(lib_srcs common/getopt.c)    # useless and produces errors on uwp
vp_list_filterout(lib_hdrs common/getopt.h)    # useless and produces errors on uwp

foreach(f ${lib_srcs})
  set_source_files_properties(${f} PROPERTIES LANGUAGE CXX )
endforeach()

add_library(${APRILTAG_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})
if(WIN32 AND MSVC)
  target_link_libraries (${APRILTAG_LIBRARY} ${PTHREADS_LIBRARY})
endif()

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
      set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fPIC")
  endif()
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
  vp_set_source_file_compile_flag(apriltag.c /wd"4018" /wd"4305" /wd"4334" /wd"4244")
  vp_set_source_file_compile_flag(apriltag_quad_thresh.c /wd"4018" /wd"4244")
  vp_set_source_file_compile_flag(common/g2d.c /wd"4244")
  vp_set_source_file_compile_flag(common/getopt.c /wd"4018" /wd"4244" /wd"4996")
  vp_set_source_file_compile_flag(common/homography.c /wd"4244")
  vp_set_source_file_compile_flag(common/image_f32.c /wd"4244")
  vp_set_source_file_compile_flag(common/image_u8.c /wd"4244")
  vp_set_source_file_compile_flag(common/image_u8x3.c /wd"4244")
  vp_set_source_file_compile_flag(common/matd.c /wd"4018" /wd"4244")
  vp_set_source_file_compile_flag(common/pjpeg-idct.c /wd"4244")
  vp_set_source_file_compile_flag(common/pjpeg.c /wd4018 /wd"4267")
  vp_set_source_file_compile_flag(common/string_util.c /wd"4996" /wd"4267")
  vp_set_source_file_compile_flag(common/time_util.c /wd"4244")
  vp_set_source_file_compile_flag(common/times.c /wd"4244")
  vp_set_source_file_compile_flag(common/zhash.c /wd"4267")
  vp_set_source_file_compile_flag(common/zmaxheap.c /wd"4098" /wd"4244")
  vp_set_source_file_compile_flag(common/workerpool.c /wd"4018" /wd"4244")
  vp_set_source_file_compile_flag(common/pam.c /wd"4018")
  vp_set_source_file_compile_flag(common/string_util.c /wd"4018" /wd"4267" /wd"4996")
  vp_set_source_file_compile_flag(tag16h5.c /wd"4996")
  vp_set_source_file_compile_flag(tag25h7.c /wd"4996")
  vp_set_source_file_compile_flag(tag25h9.c /wd"4996")
  vp_set_source_file_compile_flag(tag36artoolkit.c /wd"4996")
  vp_set_source_file_compile_flag(tag36h10.c /wd"4996")
  vp_set_source_file_compile_flag(tag36h11.c /wd"4996")
elseif(CMAKE_COMPILER_IS_CLANGCXX)
  foreach(f ${lib_srcs})
    vp_set_source_file_compile_flag(${f} -Wno-deprecated -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable)
  endforeach()
endif()
