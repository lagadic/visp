#----------------------------------------------------------------------
# Add 3rd-party libraries build as static libs
#----------------------------------------------------------------------
include(CheckCSourceCompiles)
include(CMakePushCheckState)

if(WITH_ATIDAQ)
  set(ATIDAQ_LIBRARY visp_atidaq)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/atidaq")
  set(ATIDAQ_INCLUDE_DIRS "${${ATIDAQ_LIBRARY}_SOURCE_DIR}" "${${ATIDAQ_LIBRARY}_BINARY_DIR}")
  set(ATIDAQ_LIBRARIES ${ATIDAQ_LIBRARY})
  set(ATIDAQ_VERSION ${ATIDAQ_MAJOR_VERSION}.${ATIDAQ_MINOR_VERSION}.${ATIDAQ_PATCH_VERSION})
endif()

if(WITH_CLIPPER)
  set(CLIPPER_LIBRARY visp_clipper)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clipper")
  set(CLIPPER_INCLUDE_DIRS "${${CLIPPER_LIBRARY}_SOURCE_DIR}" "${${CLIPPER_LIBRARY}_BINARY_DIR}")
  set(CLIPPER_LIBRARIES ${CLIPPER_LIBRARY})
  set(CLIPPER_VERSION ${CLIPPER_MAJOR_VERSION}.${CLIPPER_MINOR_VERSION}.${CLIPPER_PATCH_VERSION})
endif()

if(WITH_LAPACK)
  set(LAPACK_LIBRARY visp_lapack)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/clapack")
  set(LAPACK_LIBRARIES ${LAPACK_LIBRARY})
  set(LAPACK_VERSION ${LAPACK_MAJOR_VERSION}.${LAPACK_MINOR_VERSION}.${LAPACK_PATCH_VERSION})
endif()

if(USE_APRILTAG)
  if(BUILD_APRILTAG)
    vp_clear_vars(apriltag_FOUND MyApriltag_FOUND)
  else()
    find_package(MyApriltag QUIET) # Trick to overcome issue in apriltagTargets.cmake occuring with version 3.3.0
    if((NOT MyApriltag_FOUND) OR (MyApriltag_VERSION VERSION_GREATER "3.3.0"))
      find_package(apriltag QUIET)
    endif()
  endif()

  if(apriltag_FOUND)
    set(VISP_HAVE_APRILTAG YES)
    set(APRILTAG_VERSION "${apriltag_VERSION_MAJOR}.${apriltag_VERSION_MINOR}.${apriltag_VERSION_PATCH}")
    set(APRILTAG_LIBRARIES "apriltag::apriltag")
    set(VISP_HAVE_APRILTAG_VERSION "(${apriltag_VERSION_MAJOR}<<16 | ${apriltag_VERSION_MINOR}<<8 | ${apriltag_VERSION_PATCH})") # for vpConfig.h

    message(STATUS "Found system apriltag: ${APRILTAG_LIBRARIES} "
            "(found version \"${APRILTAG_VERSION}\")")
  elseif(MyApriltag_FOUND)
    set(VISP_HAVE_APRILTAG YES)
    set(APRILTAG_VERSION "${MyApriltag_VERSION}")
    set(APRILTAG_INCLUDE_DIRS ${MyApriltag_INCLUDE_DIRS})
    set(APRILTAG_LIBRARIES "apriltag::apriltag")
    set(VISP_HAVE_APRILTAG_VERSION "(${MyApriltag_VERSION_MAJOR}<<16 | ${MyApriltag_VERSION_MINOR}<<8 | ${MyApriltag_VERSION_PATCH})") # for vpConfig.h

    message(STATUS "Found system apriltag: ${APRILTAG_LIBRARIES} "
            "(found version \"${APRILTAG_VERSION}\")")
  else()
    vp_clear_vars(APRILTAG_MAJOR_VERSION APRILTAG_MINOR_VERSION APRILTAG_PATCH_VERSION APRILTAG_LIBRARIES APRILTAG_INCLUDE_DIRS)
    message(STATUS "Could NOT find libapriltag. It will be built from sources")
    add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/apriltag")
    set(BUILD_APRILTAG TRUE)
    set(VISP_HAVE_APRILTAG YES)
    set(VISP_HAVE_APRILTAG_EXTENDED_API YES)
    set(VISP_HAVE_APRILTAG_ARUCO YES)
    message(STATUS "apriltag library will be built from sources: ${APRILTAG_LIBRARIES} "
            "(version \"${APRILTAG_VERSION}\")")

    set(VISP_HAVE_APRILTAG_VERSION "(${APRILTAG_MAJOR_VERSION}<<16 | ${APRILTAG_MINOR_VERSION}<<8 | ${APRILTAG_PATCH_VERSION})") # for vpConfig.h
  endif()

  # Detect if additional functionalities are available
  # - void apriltag_detection_copy(apriltag_detection_t* src, apriltag_detection_t* dst);
  # - zarray_t* apriltag_detections_copy(zarray_t* detections);
  # - apriltag_detector_t *apriltag_detector_copy(apriltag_detector_t *td);
  # - void get_second_solution(matd_t* v[4], matd_t* p[4], apriltag_pose_t* solution1, apriltag_pose_t* solution2, int nIters, double* err2);
  if(apriltag_FOUND OR MyApriltag_FOUND)
    cmake_push_check_state()
    if(APRILTAG_INCLUDE_DIRS)
      set(CMAKE_REQUIRED_INCLUDES ${APRILTAG_INCLUDE_DIRS})
    elseif(TARGET apriltag::apriltag)
      get_target_property(_inc_dirs apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)
      set(CMAKE_REQUIRED_INCLUDES ${_inc_dirs})
    endif()

    set(CMAKE_REQUIRED_LIBRARIES ${APRILTAG_LIBRARIES})

    set(CHECK_APRILTAG_EXTENDED_API_SOURCE "
      #include <apriltag.h>
      #include <apriltag_pose.h>

      int main() {
        // Test des fonctions de apriltag.h
        apriltag_detection_t *det_src = 0;
        apriltag_detection_t *det_dst = 0;
        apriltag_detection_copy(det_src, det_dst);

        zarray_t* detections = 0;
        zarray_t* detections_copy = apriltag_detections_copy(detections);

        apriltag_detector_t *td = 0;
        apriltag_detector_t *td_copy = apriltag_detector_copy(td);

        // Test de la fonction de apriltag_pose.h
        matd_t* v[4];
        matd_t* p[4];
        apriltag_pose_t *s1 = 0;
        apriltag_pose_t *s2 = 0;
        double err2 = 0;
        get_second_solution(v, p, s1, s2, 10, &err2);

        return 0;
      }
    ")

    check_c_source_compiles("${CHECK_APRILTAG_EXTENDED_API_SOURCE}" APRILTAG_HAVE_EXTENDED_API)

    set(ARUCO_HEADERS
      tagAruco4x4_50.h  tagAruco4x4_100.h  tagAruco4x4_250.h  tagAruco4x4_1000.h
      tagAruco5x5_50.h  tagAruco5x5_100.h  tagAruco5x5_250.h  tagAruco5x5_1000.h
      tagAruco6x6_50.h  tagAruco6x6_100.h  tagAruco6x6_250.h  tagAruco6x6_1000.h
      tagAruco7x7_50.h  tagAruco7x7_100.h  tagAruco7x7_250.h  tagAruco7x7_1000.h
      tagArucoMIP36h12.h
    )

    set(CHECK_ARUCO_HEADERS_SOURCE "#include <apriltag.h>\n")
    foreach(h ${ARUCO_HEADERS})
      string(APPEND CHECK_ARUCO_HEADERS_SOURCE "#include <${h}>\n")
    endforeach()
    string(APPEND CHECK_ARUCO_HEADERS_SOURCE "int main() { return 0; }")

    check_c_source_compiles("${CHECK_ARUCO_HEADERS_SOURCE}" APRILTAG_HAVE_ARUCO_HEADERS)

    cmake_pop_check_state()

    if(APRILTAG_HAVE_EXTENDED_API)
      message(STATUS "Apriltag: extended api is available")
      set(VISP_HAVE_APRILTAG_EXTENDED_API YES)
    else()
      message(STATUS "Apriltag: extended api is NOT available")
      set(VISP_HAVE_APRILTAG_EXTENDED_API NO)
    endif()

    if(APRILTAG_HAVE_ARUCO_HEADERS)
      message(STATUS "Apriltag: ArUco headers found")
      set(VISP_HAVE_APRILTAG_ARUCO YES)
    else()
      message(STATUS "Apriltag: ArUco headers NOT found")
      set(VISP_HAVE_APRILTAG_ARUCO NO)
    endif()
  endif()

  mark_as_advanced(apriltag_DIR)
endif()

if(WITH_QBDEVICE)
  set(QBDEVICE_LIBRARY visp_qbdevice)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/qbdevice")
  set(QBDEVICE_INCLUDE_DIRS "${${QBDEVICE_LIBRARY}_SOURCE_DIR}" "${${QBDEVICE_LIBRARY}_BINARY_DIR}")
  set(QBDEVICE_LIBRARIES ${QBDEVICE_LIBRARY})
  set(QBDEVICE_VERSION ${QBDEVICE_MAJOR_VERSION}.${QBDEVICE_MINOR_VERSION}.${QBDEVICE_PATCH_VERSION})
endif()

if(WITH_TAKKTILE2)
  set(TAKKTILE2_LIBRARY visp_reflex_takktile2)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/reflex-takktile2")
  set(TAKKTILE2_INCLUDE_DIRS "${${TAKKTILE2_LIBRARY}_SOURCE_DIR}" "${${TAKKTILE2_LIBRARY}_BINARY_DIR}")
  set(TAKKTILE2_LIBRARIES ${TAKKTILE2_LIBRARY})
  set(TAKKTILE2_VERSION ${TAKKTILE2_MAJOR_VERSION}.${TAKKTILE2_MINOR_VERSION}.${TAKKTILE2_PATCH_VERSION})
endif()

if(WITH_PUGIXML)
  set(PUGIXML_LIBRARY visp_pugixml)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/pugixml-1.14")
  set(PUGIXML_INCLUDE_DIRS "${${PUGIXML_LIBRARY}_SOURCE_DIR}" "${${PUGIXML_LIBRARY}_BINARY_DIR}")
  set(PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
  set(PUGIXML_VERSION ${PUGIXML_MAJOR_VERSION}.${PUGIXML_MINOR_VERSION}.${PUGIXML_PATCH_VERSION})
endif()

if(WITH_SIMDLIB)
  set(SIMD_LIBRARY visp_simdlib)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/simdlib")
  set(SIMDLIB_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/simdlib")
  set(SIMDLIB_LIBRARIES ${SIMD_LIBRARY})
endif()

if(WITH_STBIMAGE)
  set(STBIMAGE_LIBRARY visp_stbimage)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/stb_image")
  set(STBIMAGE_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/stb_image")
  set(STBIMAGE_VERSION ${STBIMAGE_MAJOR_VERSION}.${STBIMAGE_MINOR_VERSION}.${STBIMAGE_PATCH_VERSION})
endif()

if(WITH_TINYEXR)
  set(TINYEXR_LIBRARY visp_tinyexr)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/tinyexr")
  set(TINYEXR_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/tinyexr")
  set(TINYEXR_VERSION ${TINYEXR_MAJOR_VERSION}.${TINYEXR_MINOR_VERSION}.${TINYEXR_PATCH_VERSION})
endif()

if(WITH_CATCH2)
  set(CATCH2_LIBRARY visp_catch2)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/catch2")
  set(CATCH2_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/catch2")
  set(CATCH2_LIBRARIES ${CATCH2_LIBRARY})
  set(CATCH2_VERSION ${CATCH2_MAJOR_VERSION}.${CATCH2_MINOR_VERSION}.${CATCH2_PATCH_VERSION})
endif()

if(WITH_POLOLU)
  set(POLOLU_LIBRARY visp_pololu)
  add_subdirectory("${VISP_SOURCE_DIR}/3rdparty/pololu")
  set(POLOLU_INCLUDE_DIRS "${VISP_SOURCE_DIR}/3rdparty/pololu/include")
  set(POLOLU_LIBRARIES ${POLOLU_LIBRARY})
  set(POLOLU_VERSION ${POLOLU_MAJOR_VERSION}.${POLOLU_MINOR_VERSION}.${POLOLU_PATCH_VERSION})
endif()
