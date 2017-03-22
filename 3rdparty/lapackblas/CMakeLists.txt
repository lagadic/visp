project(${LAPACK_LIBRARY} Fortran)

if(CMAKE_Fortran_COMPILER)

  if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -fPIC")
  endif()

  set(LAPACK_MAJOR_VERSION 3 PARENT_SCOPE)
  set(LAPACK_MINOR_VERSION 7 PARENT_SCOPE)
  set(LAPACK_PATCH_VERSION 0 PARENT_SCOPE)
  set(LAPACK_IS_BUILD TRUE PARENT_SCOPE)

  file(GLOB lib_srcs lapack/*.f lapack/*.F blas/*.f install/*.f)

  set(Fortran_LIBRARIES)
  foreach(lib ${CMAKE_Fortran_IMPLICIT_LINK_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES ${lib}
      HINTS ${CMAKE_Fortran_IMPLICIT_LINK_DIRECTORIES})
    if(${lib}_LIBRARY)
      list(APPEND Fortran_LIBRARIES ${${lib}_LIBRARY})
    endif()
  endforeach()
  add_library(${LAPACK_LIBRARY} STATIC ${lib_srcs})
  if(Fortran_LIBRARIES)
    target_link_libraries(${LAPACK_LIBRARY} ${Fortran_LIBRARIES})
  endif()

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

  # It seems that gfortran doesn't support -Wno-missing-include-dirs on osx
  foreach(f ${lib_srcs})
    vp_set_source_file_compile_flag(${f} -Wno-missing-include-dirs)
  endforeach()

else()
  set(LAPACK_IS_BUILD FALSE PARENT_SCOPE)
  message(STATUS "Fortran compiler not found. Cannot build lapack/blas in 3rdparty folder")
endif()
