# --------------------------------------------------------------------------------------------
#  Installation for CMake Module:  VISPConfig.cmake
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use with "make install"
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
# -------------------------------------------------------------------------------------------

if(INSTALL_TO_MANGLED_PATHS)
  set(VISP_USE_MANGLED_PATHS_CONFIGCMAKE TRUE)
else()
  set(VISP_USE_MANGLED_PATHS_CONFIGCMAKE FALSE)
endif()

if(ANDROID)
  if(NOT ANDROID_NATIVE_API_LEVEL)
    set(VISP_ANDROID_NATIVE_API_LEVEL_CONFIGCMAKE 0)
  else()
    set(VISP_ANDROID_NATIVE_API_LEVEL_CONFIGCMAKE "${ANDROID_NATIVE_API_LEVEL}")
  endif()
  vp_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-ANDROID.cmake.in" ANDROID_CONFIGCMAKE @ONLY)
endif()

set(VISP_MODULES_CONFIGCMAKE ${VISP_MODULES_PUBLIC})

if(BUILD_FAT_JAVA_LIB AND HAVE_VISP_java)
  list(APPEND VISP_MODULES_CONFIGCMAKE VISP_java)
endif()

# -------------------------------------------------------------------------------------------
#  Part 1/3: ${BIN_DIR}/VISPConfig.cmake              -> For use *without* "make install"
# -------------------------------------------------------------------------------------------
set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\"${VISP_CONFIG_FILE_INCLUDE_DIR}\" \"${VISP_SOURCE_DIR}/include\" \"${VISP_SOURCE_DIR}/include/VISP\"")

foreach(m ${VISP_MODULES_BUILD})
  if(EXISTS "${VISP_MODULE_${m}_LOCATION}/include")
    set(VISP_INCLUDE_DIRS_CONFIGCMAKE "${VISP_INCLUDE_DIRS_CONFIGCMAKE} \"${VISP_MODULE_${m}_LOCATION}/include\"")
  endif()
endforeach()

export(TARGETS ${VISPModules_TARGETS} FILE "${CMAKE_BINARY_DIR}/VISPModule.cmake")

# TODO: not using IPPICV
#if(TARGET ippicv AND NOT BUILD_SHARED_LIBS)
#  set(USE_IPPICV TRUE)
#  file(RELATIVE_PATH IPPICV_INSTALL_PATH_RELATIVE_CONFIGCMAKE "${CMAKE_BINARY_DIR}" "${IPPICV_LOCATION_PATH}")
#  ocv_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-IPPICV.cmake.in" IPPICV_CONFIGCMAKE @ONLY)
#else()
#  set(USE_IPPICV FALSE)
#endif()
#
#if(TARGET ippiw AND NOT BUILD_SHARED_LIBS AND IPPIW_INSTALL_PATH)
#  set(USE_IPPIW TRUE)
#  file(RELATIVE_PATH IPPIW_INSTALL_PATH_RELATIVE_CONFIGCMAKE "${CMAKE_BINARY_DIR}" "${IPPIW_LOCATION_PATH}")
#  ocv_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-IPPIW.cmake.in" IPPIW_CONFIGCMAKE @ONLY)
#else()
#  set(USE_IPPIW FALSE)
#endif()

configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISPConfig.cmake.in" "${CMAKE_BINARY_DIR}/VISPConfig.cmake" @ONLY)
#support for version checking when finding opencv. find_package(OpenCV 2.3.1 EXACT) should now work.
configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISPConfig-version.cmake.in" "${CMAKE_BINARY_DIR}/VISPConfig-version.cmake" @ONLY)

# --------------------------------------------------------------------------------------------
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use *with* "make install"
# -------------------------------------------------------------------------------------------
file(RELATIVE_PATH VISP_INSTALL_PATH_RELATIVE_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}/${VISP_CONFIG_INSTALL_PATH}/" ${CMAKE_INSTALL_PREFIX})
set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\"\${VISP_INSTALL_PATH}/${VISP_INCLUDE_INSTALL_PATH}\" \"\${VISP_INSTALL_PATH}/${VISP_INCLUDE_INSTALL_PATH}/VISP\"")


# TODO: not using IPPICV
#if(USE_IPPICV)
#  file(RELATIVE_PATH IPPICV_INSTALL_PATH_RELATIVE_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}" "${IPPICV_INSTALL_PATH}")
#  ocv_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-IPPICV.cmake.in" IPPICV_CONFIGCMAKE @ONLY)
#endif()
#if(USE_IPPIW)
#  file(RELATIVE_PATH IPPIW_INSTALL_PATH_RELATIVE_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}" "${IPPIW_INSTALL_PATH}")
#  ocv_cmake_configure("${CMAKE_CURRENT_LIST_DIR}/templates/VISPConfig-IPPIW.cmake.in" IPPIW_CONFIGCMAKE @ONLY)
#endif()

if((CMAKE_HOST_SYSTEM_NAME MATCHES "Linux" OR UNIX) AND NOT ANDROID)
  vp_gen_config("${CMAKE_BINARY_DIR}/unix-install" "" "")
endif()

if(ANDROID)
  vp_gen_config("${CMAKE_BINARY_DIR}/unix-install" "abi-${ANDROID_NDK_ABI_NAME}" "VISPConfig.root-ANDROID.cmake.in")
  install(FILES "${VISP_SOURCE_DIR}/platforms/android/android.toolchain.cmake" DESTINATION "${VISP_CONFIG_INSTALL_PATH}" COMPONENT dev)
endif()

# --------------------------------------------------------------------------------------------
#  Part 3/3: ${BIN_DIR}/win-install/VISPConfig.cmake  -> For use within binary installers/packages
# --------------------------------------------------------------------------------------------
if(WIN32)
  if(CMAKE_HOST_SYSTEM_NAME MATCHES Windows)
    if(BUILD_SHARED_LIBS)
      set(_lib_suffix "lib")
    else()
      set(_lib_suffix "staticlib")
    endif()
    vp_gen_config("${CMAKE_BINARY_DIR}/win-install" "${VISP_INSTALL_BINARIES_PREFIX}${_lib_suffix}" "VISPConfig.root-WIN32.cmake.in")
  else()
    vp_gen_config("${CMAKE_BINARY_DIR}/win-install" "" "")
  endif()
endif()
