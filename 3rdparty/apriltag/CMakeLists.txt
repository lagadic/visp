project(${APRILTAG_LIBRARY})

set(APRILTAG_MAJOR_VERSION 0 PARENT_SCOPE)
set(APRILTAG_MINOR_VERSION 9 PARENT_SCOPE)
set(APRILTAG_PATCH_VERSION 8 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.c common/*.c)
file(GLOB lib_hdrs *.h common/*.h common/sys/*.h)

vp_list_filterout(lib_srcs common/unionfind.c) # has no symbols

add_library(${APRILTAG_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})
if(WIN32 AND MSVC)
  target_link_libraries (${APRILTAG_LIBRARY} ${PTHREADS_LIBRARY})
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C99_C_FLAGS}")
if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
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
  vp_set_source_file_compile_flag(apriltag.c /wd4018 /wd4305 /wd4334 /wd4244)
  vp_set_source_file_compile_flag(apriltag_quad_thresh.c /wd4018 /wd4244)
  vp_set_source_file_compile_flag(common/g2d.c /wd4244)
  vp_set_source_file_compile_flag(common/getopt.c /wd4018 /wd4244 /wd4996)
  vp_set_source_file_compile_flag(common/homography.c /wd4244)
  vp_set_source_file_compile_flag(common/image_f32.c /wd4244)
  vp_set_source_file_compile_flag(common/image_u8.c /wd4244)
  vp_set_source_file_compile_flag(common/image_u8x3.c /wd4244)
  vp_set_source_file_compile_flag(common/matd.c /wd4018 /wd4244)
  vp_set_source_file_compile_flag(common/pjpeg-idct.c /wd4244)
  vp_set_source_file_compile_flag(common/pjpeg.c /wd4018 /wd4267)
  vp_set_source_file_compile_flag(common/string_util.c /wd4996 /wd4267)
  vp_set_source_file_compile_flag(common/time_util.c /wd4244)
  vp_set_source_file_compile_flag(common/times.c /wd4244)
  vp_set_source_file_compile_flag(common/zhash.c /wd4267)
  vp_set_source_file_compile_flag(common/zmaxheap.c /wd4098 /wd4244)
  vp_set_source_file_compile_flag(common/workerpool.c /wd4018 /wd4244)
  vp_set_source_file_compile_flag(tag16h5.c /wd4996)
  vp_set_source_file_compile_flag(tag25h7.c /wd4996)
  vp_set_source_file_compile_flag(tag25h9.c /wd4996)
  vp_set_source_file_compile_flag(tag36artoolkit.c /wd4996)
  vp_set_source_file_compile_flag(tag36h10.c /wd4996)
  vp_set_source_file_compile_flag(tag36h11.c /wd4996)
endif()
