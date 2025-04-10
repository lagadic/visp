
project(${the_module}_android)

if(ANDROID_EXECUTABLE)
  set(VISP_JAVA_DIR "${ViSP_BINARY_DIR}/android_sdk" CACHE INTERNAL "")
else(ANDROID_EXECUTABLE)  # gradle
  set(VISP_JAVA_DIR "${ANDROID_BUILD_BASE_DIR}/visp" CACHE INTERNAL "")
endif(ANDROID_EXECUTABLE)
set(VISP_ANDROID_LIB_DIR "${VISP_JAVA_DIR}" CACHE INTERNAL "")  # for ViSP samples

file(REMOVE_RECURSE "${VISP_JAVA_DIR}")
file(MAKE_DIRECTORY "${VISP_JAVA_DIR}/bin")
set(java_src_dir "${VISP_JAVA_DIR}/src")
file(MAKE_DIRECTORY "${java_src_dir}")

vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/java" "${java_src_dir}")

if(ANDROID_EXECUTABLE)

  vp_assert(ANDROID_TOOLS_Pkg_Revision GREATER 13)

  vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/android/java" "${java_src_dir}")

  # calc default SDK Target
  android_get_compatible_target(ANDROID_SDK_COMPATIBLE_TARGET ${ANDROID_NATIVE_API_LEVEL} ${ANDROID_SDK_TARGET} 14)
  if(ANDROID_SDK_COMPATIBLE_TARGET)
    set(ANDROID_SDK_COMPATIBLE_TARGET "${ANDROID_SDK_COMPATIBLE_TARGET}" CACHE INTERNAL "")
  endif()
  string(REGEX REPLACE "android-" "" android_sdk_target_num ${ANDROID_SDK_COMPATIBLE_TARGET})
  if( (ANDROID_SDK_TARGET AND ANDROID_SDK_TARGET LESS 21) OR (android_sdk_target_num LESS 21) )
    message(STATUS "[ViSP for Android SDK]: A new OpenGL Camera Bridge (CameraGLSurfaceView, CameraGLRendererBase, CameraRenderer, Camera2Renderer) is disabled, because ANDROID_SDK_TARGET (${android_sdk_target_num}) < 21")
  else()
    vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/android-21/java" "${java_src_dir}")
  endif()

  # copy boilerplate
  file(GLOB_RECURSE seed_project_files_rel RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/android_lib/" "${CMAKE_CURRENT_SOURCE_DIR}/android_lib/*")
  list(REMOVE_ITEM seed_project_files_rel "${ANDROID_MANIFEST_FILE}")
  foreach(file ${seed_project_files_rel})
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/android_lib/${file}" "${VISP_JAVA_DIR}/${file}" @ONLY)
    list(APPEND depends "${CMAKE_CURRENT_SOURCE_DIR}/android_lib/${file}")
    get_filename_component(install_subdir "${file}" PATH)
    install(FILES "${VISP_JAVA_DIR}/${file}" DESTINATION "${JAVA_INSTALL_ROOT}/${install_subdir}" COMPONENT java)
  endforeach()

  list(APPEND depends gen_visp_java_source "${VISP_DEPHELPER}/gen_visp_java_source")
  vp_copyfiles_add_target(${the_module}_android_source_copy JAVA_SRC_COPY "Copy Java(Android SDK) source files" ${depends})
  file(REMOVE "${VISP_DEPHELPER}/${the_module}_android_source_copy")  # force rebuild after CMake run

  set(depends ${the_module}_android_source_copy "${VISP_DEPHELPER}/${the_module}_android_source_copy")

  # generate Android library project
  set(android_sdk_project_files ${ANDROID_LIB_PROJECT_FILES})  # build.xml;local.properties;proguard-project.txt;project.properties
  vp_list_add_prefix(android_sdk_project_files "${VISP_JAVA_DIR}/")

  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/android_lib/${ANDROID_MANIFEST_FILE}" "${CMAKE_CURRENT_BINARY_DIR}/${ANDROID_MANIFEST_FILE}" @ONLY)

  add_custom_command(
      OUTPUT ${android_sdk_project_files} "${VISP_JAVA_DIR}/${ANDROID_MANIFEST_FILE}"
      COMMAND ${CMAKE_COMMAND} -E remove ${android_sdk_project_files}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/${ANDROID_MANIFEST_FILE}" "${VISP_JAVA_DIR}/${ANDROID_MANIFEST_FILE}"
      COMMAND ${ANDROID_EXECUTABLE} --silent create lib-project --path \"${VISP_JAVA_DIR}\" --target \"${ANDROID_SDK_COMPATIBLE_TARGET}\" --name ViSP --package org.visp 2>\"${CMAKE_CURRENT_BINARY_DIR}/create_lib_project.log\"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/${ANDROID_MANIFEST_FILE}" "${VISP_JAVA_DIR}/${ANDROID_MANIFEST_FILE}"
      WORKING_DIRECTORY "${VISP_JAVA_DIR}"
      MAIN_DEPENDENCY "${CMAKE_CURRENT_BINARY_DIR}/${ANDROID_MANIFEST_FILE}"
      DEPENDS ${depends}
      COMMENT "Generating ViSP Android library project. SDK target: ${lib_target_sdk_target}"
  )
  list(APPEND depends ${android_sdk_project_files} "${VISP_JAVA_DIR}/${ANDROID_MANIFEST_FILE}")

  install(DIRECTORY "${VISP_JAVA_DIR}/src" DESTINATION "${JAVA_INSTALL_ROOT}" COMPONENT java)
  install(FILES "${VISP_JAVA_DIR}/${ANDROID_PROJECT_PROPERTIES_FILE}" DESTINATION ${JAVA_INSTALL_ROOT} COMPONENT java)
  install(FILES "${VISP_JAVA_DIR}/${ANDROID_MANIFEST_FILE}" DESTINATION ${JAVA_INSTALL_ROOT} COMPONENT java)

  # build jar
  set(JAR_FILE "${ViSP_BINARY_DIR}/bin/classes.jar")
  # build the library project
  # normally we should do this after a native part, but for a library project we can build the java part first
  add_custom_command(
      OUTPUT "${JAR_FILE}" "${VISP_DEPHELPER}/${the_module}_android"
      COMMAND ${ANT_EXECUTABLE} -q -noinput -k debug -Djava.target=1.6 -Djava.source=1.6
      COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/${the_module}_android"
      WORKING_DIRECTORY "${VISP_JAVA_DIR}"
      DEPENDS ${depends}
      COMMENT "Building ViSP Android library project"
  )

  add_custom_target(${the_module}_android DEPENDS "${VISP_DEPHELPER}/${the_module}_android" SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/android_lib/${ANDROID_MANIFEST_FILE}")
  add_dependencies(${the_module} ${the_module}_android)

  # empty 'gen' and 'res' folders
  install(CODE "
  FILE(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${JAVA_INSTALL_ROOT}/gen\")
  FILE(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${JAVA_INSTALL_ROOT}/res\")
  " COMPONENT java)

  vp_update(ANDROID_COMPILE_SDK_VERSION "27")
  vp_update(ANDROID_MIN_SDK_VERSION "14")
  vp_update(ANDROID_TARGET_SDK_VERSION "21")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/build.gradle.in" "${CMAKE_CURRENT_BINARY_DIR}/build.gradle" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/build.gradle" DESTINATION ${JAVA_INSTALL_ROOT}/.. COMPONENT java)


else(ANDROID_EXECUTABLE)  # gradle build
  #
  # Android Gradle-based project
  #

  #TODO: INSTALL ONLY
  vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/android/java" "${java_src_dir}")
  vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/android-21/java" "${java_src_dir}")

  # copy boilerplate
  set(__base_dir "${CMAKE_CURRENT_SOURCE_DIR}/android_gradle_lib/")
  file(GLOB_RECURSE seed_project_files_rel RELATIVE "${__base_dir}/" "${__base_dir}/*")
  list(REMOVE_ITEM seed_project_files_rel "${ANDROID_MANIFEST_FILE}")
  string(REGEX REPLACE "-[a-zA-Z]*[0-9]*$" "" ANDROID_NDK_VERSION ${ANDROID_NDK_REVISION})
  foreach(file ${seed_project_files_rel})
    string(REGEX REPLACE "\.in$" "" stripped_name ${file})
    list(APPEND depends "${__base_dir}/${file}")
    if(NOT file MATCHES "settings.gradle")
      configure_file("${__base_dir}/${file}" "${VISP_JAVA_DIR}/${stripped_name}" @ONLY)
      get_filename_component(install_subdir "${file}" PATH)
      if(NOT stripped_name MATCHES "build.gradle")
        install(FILES "${VISP_JAVA_DIR}/${stripped_name}" DESTINATION "${JAVA_INSTALL_ROOT}/${install_subdir}" COMPONENT java)
      endif()
    else()
      file(COPY "${__base_dir}/${file}" DESTINATION ${ANDROID_BUILD_BASE_DIR})
    endif()
  endforeach()

  # copy libcxx_helper
  set(__base_dir "${CMAKE_CURRENT_SOURCE_DIR}/")
  file(GLOB_RECURSE __files_rel RELATIVE "${__base_dir}/" "${__base_dir}/libcxx_helper/*")
  foreach(file ${__files_rel})
    configure_file("${__base_dir}/${file}" "${VISP_JAVA_DIR}/${file}" @ONLY)
    list(APPEND depends "${__base_dir}/${file}")
    get_filename_component(install_subdir "${file}" PATH)
    install(FILES "${VISP_JAVA_DIR}/${file}" DESTINATION "${JAVA_INSTALL_ROOT}/../${install_subdir}" COMPONENT java)
  endforeach()

  list(APPEND depends gen_visp_java_source "${VISP_DEPHELPER}/gen_visp_java_source")
  vp_copyfiles_add_target(${the_module}_android_source_copy JAVA_SRC_COPY "Copy Java(Android SDK) source files" ${depends})
  file(REMOVE "${VISP_DEPHELPER}/${the_module}_android_source_copy")  # force rebuild after CMake run

  set(depends ${the_module}_android_source_copy "${VISP_DEPHELPER}/${the_module}_android_source_copy")

  # build jar
  set(AAR_FILE "${VISP_JAVA_DIR}/build/outputs/aar/visp-release.aar")
  vp_update(VISP_GRADLE_VERBOSE_OPTIONS "-i")
  add_custom_command(
      OUTPUT "${AAR_FILE}" "${VISP_DEPHELPER}/${the_module}_android"
      COMMAND ./gradlew ${VISP_GRADLE_VERBOSE_OPTIONS} "visp:assemble"
      COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/${the_module}_android"
      WORKING_DIRECTORY "${ANDROID_BUILD_BASE_DIR}"
      DEPENDS ${depends} ${the_module}
      COMMENT "Building ViSP Android library project"
  )
  file(REMOVE "${VISP_DEPHELPER}/${the_module}_android")  # force rebuild after CMake run

  add_custom_target(${the_module}_android ALL DEPENDS "${VISP_DEPHELPER}/${the_module}_android" SOURCES "${__base_dir}/${ANDROID_MANIFEST_FILE}")

  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/build.gradle.in" "${ANDROID_BUILD_BASE_DIR}/build.gradle" @ONLY)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/android_gradle_lib/install/build.gradle.in" "${ANDROID_TMP_INSTALL_BASE_DIR}/visp/build.gradle" @ONLY)
  install(FILES "${ANDROID_TMP_INSTALL_BASE_DIR}/visp/build.gradle" DESTINATION ${JAVA_INSTALL_ROOT}/.. COMPONENT java)

  install(DIRECTORY "${java_src_dir}" DESTINATION "${JAVA_INSTALL_ROOT}" COMPONENT java)

  set(VISP_ANDROID_MODULE_NAME "${the_module}_android" CACHE INTERNAL "Name of the Android module")

endif(ANDROID_EXECUTABLE)  # gradle build
