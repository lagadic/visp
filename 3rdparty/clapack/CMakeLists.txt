project(${LAPACK_LIBRARY} C)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

set(LAPACK_MAJOR_VERSION 3 PARENT_SCOPE)
set(LAPACK_MINOR_VERSION 2 PARENT_SCOPE)
set(LAPACK_PATCH_VERSION 1 PARENT_SCOPE)

file(GLOB lib_srcs src/*.c blas/*.c install/*.c f2c/*.c)
file(GLOB lib_incs include/*.h)

add_library(${LAPACK_LIBRARY} STATIC ${lib_srcs} ${lib_incs})
target_include_directories(${LAPACK_LIBRARY} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${LAPACK_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${LAPACK_LIBRARY}
  PROPERTIES OUTPUT_NAME ${LAPACK_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${LAPACK_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${LAPACK_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${LAPACK_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

foreach(f ${lib_srcs})
  vp_set_source_file_compile_flag(${f} -Wno-logical-op-parentheses -Wno-shift-op-parentheses -Wno-parentheses -Wno-format-extra-args -Wno-format)
endforeach()
