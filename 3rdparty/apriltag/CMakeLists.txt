project(${APRILTAG_LIBRARY})

set(APRILTAG_MAJOR_VERSION 3 PARENT_SCOPE)
set(APRILTAG_MINOR_VERSION 0 PARENT_SCOPE)
set(APRILTAG_PATCH_VERSION 0 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB lib_srcs *.c common/*.c)
file(GLOB lib_hdrs *.h common/*.h common/sys/*.h)

vp_list_filterout(lib_srcs common/unionfind.c) # has no symbols
vp_list_filterout(lib_srcs common/getopt.c)    # useless and produces errors on uwp
vp_list_filterout(lib_hdrs common/getopt.h)    # useless and produces errors on uwp

#vp_list_filterout(lib_srcs tagCircle49h12.c)     # build take too much time with RelWithDebInfo on GCC 5.4.0
#vp_list_filterout(lib_hdrs tagCircle49h12.h)     # build take too much time with RelWithDebInfo on GCC 5.4.0
#vp_list_filterout(lib_srcs tagCustom48h12.c)     # build take too much time with RelWithDebInfo on GCC 5.4.0
#vp_list_filterout(lib_hdrs tagCustom48h12.h)     # build take too much time with RelWithDebInfo on GCC 5.4.0
#vp_list_filterout(lib_srcs tagStandard52h13.c)   # build take too much time with RelWithDebInfo on GCC 5.4.0
#vp_list_filterout(lib_hdrs tagStandard52h13.h)   # build take too much time with RelWithDebInfo on GCC 5.4.0

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  endif()
endif()

foreach(f ${lib_srcs})
  set_source_files_properties(${f} PROPERTIES LANGUAGE CXX )
endforeach()

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
    vp_warnings_disable(CMAKE_CXX_FLAGS /wd4018 /wd4098 /wd4244 /wd4267 /wd4305 /wd4334 /wd4244 /wd4838 /wd4996)
    vp_set_source_file_compile_flag(common/zmaxheap.c /wd4098 /wd4244)
    vp_set_source_file_compile_flag(common/workerpool.c /wd4018 /wd4244)
    vp_set_source_file_compile_flag(common/pam.c /wd4018)
    vp_set_source_file_compile_flag(common/string_util.c /wd4018 /wd4267 /wd4996)
    vp_set_source_file_compile_flag(tag16h5.c /wd4996)
    vp_set_source_file_compile_flag(tag25h7.c /wd4996)
    vp_set_source_file_compile_flag(tag25h9.c /wd4996)
    vp_set_source_file_compile_flag(tag36h10.c /wd4996)
    vp_set_source_file_compile_flag(tag36h11.c /wd4996)
  else()
    vp_warnings_disable(CMAKE_CXX_FLAGS /wd4018 /wd4098 /wd4244 /wd4267 /wd4305 /wd4334 /wd4244 /wd4838 /wd4996)
  endif()
else()
  foreach(f ${lib_srcs})
    vp_set_source_file_compile_flag(${f} -Wno-deprecated -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-float-equal -Wno-strict-overflow -Wno-type-limits -Wno-shift-negative-value -Wno-shorten-64-to-32)
  endforeach()
endif()
