find_host_program(ANDROID_EXECUTABLE
  NAMES android.bat android
  PATHS "${ANDROID_SDK_TOOLS}"
  DOC "Android 'android' tool location"
)

if(NOT ANDROID_EXECUTABLE)
  message(FATAL_ERROR "Android SDK Tools: Can't find 'android' tool")
elseif(NOT ANDROID_SDK_DETECT_QUIET)
  message(STATUS "Android SDK Tools: Found 'android' tool: ${ANDROID_EXECUTABLE}")
endif()

set(ANDROID_PROJECT_PROPERTIES_FILE project.properties)
set(ANDROID_ANT_PROPERTIES_FILE ant.properties)

set(ANDROID_MANIFEST_FILE AndroidManifest.xml)
set(ANDROID_LIB_PROJECT_FILES build.xml local.properties proguard-project.txt ${ANDROID_PROJECT_PROPERTIES_FILE})
set(ANDROID_PROJECT_FILES ${ANDROID_LIB_PROJECT_FILES})

execute_process(COMMAND ${ANDROID_EXECUTABLE} list target -c
  RESULT_VARIABLE ANDROID_PROCESS
  OUTPUT_VARIABLE ANDROID_SDK_TARGETS
  ERROR_VARIABLE ANDROID_PROCESS_ERRORS
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )
if(NOT ANDROID_PROCESS EQUAL 0)
  set(ANDROID_EXECUTABLE "ANDROID_EXECUTABLE-NOTFOUND" CACHE INTERNAL)
  message(FATAL_ERROR "Android: Failed to get list of installed Android targets.")
endif()
string(REGEX MATCHALL "[^\n]+" ANDROID_SDK_TARGETS "${ANDROID_SDK_TARGETS}")

# clear ANDROID_SDK_TARGET if no target is provided by user
if(NOT ANDROID_SDK_TARGET)
  set(ANDROID_SDK_TARGET "" CACHE STRING "Android SDK target for the ViSP Java API and samples")
endif()
if(ANDROID_SDK_TARGETS)
  message(STATUS "Android SDK Tools: Available targets: ${ANDROID_SDK_TARGETS}")
  set_property(CACHE ANDROID_SDK_TARGET PROPERTY STRINGS ${ANDROID_SDK_TARGETS} )
else()
  message(FATAL_ERROR "Android: List of installed Android targets is empty")
endif()

# finds minimal installed SDK target compatible with provided names or API levels
# usage:
#   get_compatible_android_api_level(VARIABLE [level1] [level2] ...)
macro(android_get_compatible_target VAR)
  set(${VAR} "${VAR}-NOTFOUND")
  if(ANDROID_SDK_TARGETS)
    list(GET ANDROID_SDK_TARGETS 0 __lvl)
    string(REGEX MATCH "[0-9]+$" __lvl "${__lvl}")

    #find minimal level mathing to all provided levels
    foreach(lvl ${ARGN})
      string(REGEX MATCH "[0-9]+$" __level "${lvl}")
      if(__level GREATER __lvl)
        set(__lvl ${__level})
      endif()
    endforeach()

    #search for compatible levels
    foreach(lvl ${ANDROID_SDK_TARGETS})
      string(REGEX MATCH "[0-9]+$" __level "${lvl}")
      if(__level EQUAL __lvl)
        #look for exact match
        foreach(usrlvl ${ARGN})
          if("${usrlvl}" STREQUAL "${lvl}")
            set(${VAR} "${lvl}")
            break()
          endif()
        endforeach()
        if("${${VAR}}" STREQUAL "${lvl}")
          break() #exact match was found
        elseif(NOT ${VAR})
          set(${VAR} "${lvl}")
        endif()
      elseif(__level GREATER __lvl)
        if(NOT ${VAR})
          set(${VAR} "${lvl}")
        endif()
        break()
      endif()
    endforeach()

    unset(__lvl)
    unset(__level)
  endif()
endmacro()

unset(__android_project_chain CACHE)

# add_android_project(target_name ${path} NATIVE_DEPS visp_core LIBRARY_DEPS ${VISP_BINARY_DIR} SDK_TARGET 11)
macro(add_android_project target path)
  # parse arguments
  set(android_proj_arglist NATIVE_DEPS LIBRARY_DEPS SDK_TARGET IGNORE_JAVA IGNORE_MANIFEST COPY_LIBS)
  set(__varname "android_proj_")
  foreach(v ${android_proj_arglist})
    set(${__varname}${v} "")
  endforeach()
  foreach(arg ${ARGN})
    set(__var "${__varname}")
    foreach(v ${android_proj_arglist})
      if("${v}" STREQUAL "${arg}")
        set(__varname "android_proj_${v}")
        break()
      endif()
    endforeach()
    if(__var STREQUAL __varname)
      list(APPEND ${__var} "${arg}")
    endif()
  endforeach()

  # get compatible SDK target
  android_get_compatible_target(android_proj_sdk_target ${ANDROID_NATIVE_API_LEVEL} ${android_proj_SDK_TARGET})

  if(NOT android_proj_sdk_target)
    message(WARNING "Can not find any SDK target compatible with: ${ANDROID_NATIVE_API_LEVEL} ${android_proj_SDK_TARGET}
                     The project ${target} will not be build")
  endif()

  # check native dependencies
  if(android_proj_IGNORE_JAVA)
    vp_check_dependencies(${android_proj_NATIVE_DEPS})
  else()
    vp_check_dependencies(${android_proj_NATIVE_DEPS} visp_java)
  endif()

  if(EXISTS "${path}/jni/Android.mk" )
    # find if native_app_glue is used
    file(STRINGS "${path}/jni/Android.mk" NATIVE_APP_GLUE REGEX ".*(call import-module,android/native_app_glue)" )
    if(NATIVE_APP_GLUE)
      if(ANDROID_NATIVE_API_LEVEL LESS 9 OR NOT EXISTS "${ANDROID_NDK}/sources/android/native_app_glue")
        set(VP_DEPENDENCIES_FOUND FALSE)
      endif()
    endif()
  endif()

  if(VP_DEPENDENCIES_FOUND AND android_proj_sdk_target AND ANDROID_EXECUTABLE AND ANT_EXECUTABLE AND ANDROID_TOOLS_Pkg_Revision GREATER 13 AND EXISTS "${path}/${ANDROID_MANIFEST_FILE}")

    project(${target})
    set(android_proj_bin_dir "${CMAKE_CURRENT_BINARY_DIR}/.build")

    # project sources
    set(__src_glob "${path}/res/*" "${path}/src/*")
    if(NOT android_proj_IGNORE_MANIFEST)
      list(APPEND __src_glob "${path}/${ANDROID_MANIFEST_FILE}")
    endif()

    vp_copyfiles_append_dir(SRC_COPY "${path}" "${android_proj_bin_dir}" ${__src_glob})
    vp_copyfiles_add_forced_target(${target}_copy_src SRC_COPY "Copy project sources: ${target}")

    set(android_proj_file_deps ${target}_copy_src "${VISP_DEPHELPER}/${target}_copy_src")

    set(android_proj_lib_deps_commands "")
    set(android_proj_target_files ${ANDROID_PROJECT_FILES})
    vp_list_add_prefix(android_proj_target_files "${android_proj_bin_dir}/")

    # process Android library dependencies
    foreach(dep ${android_proj_LIBRARY_DEPS})
      file(RELATIVE_PATH __dep "${android_proj_bin_dir}" "${dep}")
      list(APPEND android_proj_lib_deps_commands
        COMMAND ${ANDROID_EXECUTABLE} --silent update project --path "${android_proj_bin_dir}" --library "${__dep}")
    endforeach()

    # fix Android project
    add_custom_command(
        OUTPUT ${android_proj_target_files}
        COMMAND ${CMAKE_COMMAND} -E remove ${android_proj_target_files}
                                           "${android_proj_bin_dir}/bin/${ANDROID_MANIFEST_FILE}"  # avoid warning about sub-projects
        COMMAND ${ANDROID_EXECUTABLE} --silent update project --path "${android_proj_bin_dir}" --target "${android_proj_sdk_target}" --name "${target}"
        ${android_proj_lib_deps_commands}
        WORKING_DIRECTORY "${android_proj_bin_dir}"
        DEPENDS ${android_proj_file_deps}
        COMMENT "Updating Android project at ${path}. SDK target: ${android_proj_sdk_target}"
        )

    list(APPEND android_proj_file_deps ${android_proj_target_files})

    # build native part
    file(GLOB_RECURSE android_proj_jni_files "${path}/jni/*.c" "${path}/jni/*.h" "${path}/jni/*.cpp" "${path}/jni/*.hpp")

    if(android_proj_jni_files AND EXISTS ${path}/jni/Android.mk AND NOT DEFINED JNI_LIB_NAME)
      # find local module name in Android.mk file to build native lib
      file(STRINGS "${path}/jni/Android.mk" JNI_LIB_NAME REGEX "LOCAL_MODULE[ ]*:=[ ]*.*" )
      string(REGEX REPLACE "LOCAL_MODULE[ ]*:=[ ]*([a-zA-Z_][a-zA-Z_0-9]*)[ ]*" "\\1" JNI_LIB_NAME "${JNI_LIB_NAME}")

      if(JNI_LIB_NAME)
        if(NATIVE_APP_GLUE)
          include_directories(${ANDROID_NDK}/sources/android/native_app_glue)
          list(APPEND android_proj_jni_files ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)
          vp_warnings_disable(CMAKE_C_FLAGS -Wstrict-prototypes -Wunused-parameter -Wmissing-prototypes)
          set(android_proj_NATIVE_DEPS ${android_proj_NATIVE_DEPS} android)
        endif()

        add_library(${JNI_LIB_NAME} SHARED ${android_proj_jni_files})
        vp_target_include_modules_recurse(${JNI_LIB_NAME} ${android_proj_NATIVE_DEPS})
        vp_target_include_directories(${JNI_LIB_NAME} "${path}/jni")
        vp_target_link_libraries(${JNI_LIB_NAME} LINK_PRIVATE ${VISP_LINKER_LIBS} ${android_proj_NATIVE_DEPS})

        set_target_properties(${JNI_LIB_NAME} PROPERTIES
            OUTPUT_NAME "${JNI_LIB_NAME}"
            LIBRARY_OUTPUT_DIRECTORY "${android_proj_bin_dir}/libs/${ANDROID_NDK_ABI_NAME}"
            )

        if (NOT (CMAKE_BUILD_TYPE MATCHES "debug"))
            add_custom_command(TARGET ${JNI_LIB_NAME} POST_BUILD COMMAND ${CMAKE_STRIP} --strip-unneeded "$<TARGET_FILE:${JNI_LIB_NAME}>")
        endif()
      endif()
    endif()

    # build java part
    if(android_proj_IGNORE_JAVA)
      set(android_proj_extra_deps "")
    else()
      list(APPEND android_proj_extra_deps visp_java_android)
    endif()

    set(_native_deps "")
    if(NOT android_proj_IGNORE_JAVA)
      set(_native_deps visp_java)
    endif()
    if(android_proj_native_deps)
      list(APPEND _native_deps ${android_proj_native_deps})
    endif()
    if(_native_deps)
      list(APPEND android_proj_extra_deps ${_native_deps})
    endif()

    if((android_proj_COPY_LIBS OR ANDROID_EXAMPLES_WITH_LIBS) AND _native_deps)
      message(STATUS "Android project with native libs: " ${target} " (" ${_native_deps} ")")

      vp_copyfiles_append_dir(NATIVE_COPY "${VISP_BINARY_DIR}/lib" "${android_proj_bin_dir}/libs" "${VISP_BINARY_DIR}/lib/*.so")
      vp_copyfiles_add_target(${target}_copy_libs NATIVE_COPY "Copy native libs for project: ${target}" ${_native_deps})

      list(APPEND android_proj_extra_deps ${target}_copy_libs)
    endif()

    add_custom_command(
        OUTPUT "${android_proj_bin_dir}/bin/${target}-debug.apk"
        COMMAND ${ANT_EXECUTABLE} -q -noinput -k debug -Djava.target=1.6 -Djava.source=1.6
        COMMAND ${CMAKE_COMMAND} -E touch "${android_proj_bin_dir}/bin/${target}-debug.apk" # needed because ant does not update the timestamp of updated apk
        WORKING_DIRECTORY "${android_proj_bin_dir}"
        DEPENDS ${android_proj_extra_deps} ${android_proj_file_deps} ${JNI_LIB_NAME}
        COMMENT "Generating ${target}-debug.apk"
    )

    unset(JNI_LIB_NAME)

    add_custom_target(${target} ALL SOURCES "${android_proj_bin_dir}/bin/${target}-debug.apk" )

    # There is some strange problem with concurrent Android .APK builds:
    # <android-sdk>/tools/ant/build.xml:781: Problem reading <build_dir>/bin/classes.jar'
    if(__android_project_chain)
      add_dependencies(${target} ${__android_project_chain})
    endif()
    set(__android_project_chain ${target} CACHE INTERNAL "auxiliary variable used for Android progects chaining")

    # put the final .apk to the ViSP's bin folder
    add_custom_command(TARGET ${target} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${android_proj_bin_dir}/bin/${target}-debug.apk" "${VISP_BINARY_DIR}/bin/${target}.apk")
    if(INSTALL_ANDROID_EXAMPLES AND "${target}" MATCHES "^example-")
      get_filename_component(sample_dir "${path}" NAME)
      # apk
      install(FILES "${VISP_BINARY_DIR}/bin/${target}.apk" DESTINATION "samples" COMPONENT samples)
      # clear "external" project files (need to remove files generated by 'android' tool)
      set(external_target_files ${ANDROID_PROJECT_FILES})
      vp_list_add_prefix(external_target_files "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/samples/${sample_dir}/")
      install(CODE "
MESSAGE(STATUS \"Cleaning generated project files from Android sample install directory: ${sample_dir}\")
FOREACH(f ${ANDROID_PROJECT_FILES})
  FILE(REMOVE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/samples/${sample_dir}/\${f}\")
ENDFOREACH()
" COMPONENT samples)
      #java part
      install(DIRECTORY "${android_proj_bin_dir}/res" DESTINATION "samples/${sample_dir}" COMPONENT samples)
      install(DIRECTORY "${android_proj_bin_dir}/src" DESTINATION "samples/${sample_dir}" COMPONENT samples)
      install(FILES "${android_proj_bin_dir}/${ANDROID_MANIFEST_FILE}" DESTINATION "samples/${sample_dir}" COMPONENT samples)
      #jni part + eclipse files
      file(GLOB_RECURSE jni_files RELATIVE "${path}" "${path}/jni/*" "${path}/.cproject")
      vp_list_filterout(jni_files "\\\\.svn")
      foreach(f ${jni_files} ".classpath" ".project" ".settings/org.eclipse.jdt.core.prefs")
        get_filename_component(install_subdir "${f}" PATH)
        install(FILES "${path}/${f}" DESTINATION "samples/${sample_dir}/${install_subdir}" COMPONENT samples)
      endforeach()
      #update proj
      if(android_proj_lib_deps_commands)
        set(inst_lib_opt " --library ../../sdk/java")
      endif()
      install(CODE "
MESSAGE(STATUS \"Fixing Android library reference for sample: ${sample_dir}\")
EXECUTE_PROCESS(
    COMMAND ${ANDROID_EXECUTABLE} --silent update project --path . --target \"${android_proj_sdk_target}\" --name \"${target}\" ${inst_lib_opt}
    WORKING_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/samples/${sample_dir}\"
)
" COMPONENT samples)
      # empty 'gen'
      install(CODE "FILE(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/samples/${sample_dir}/gen\")" COMPONENT samples)
    endif()
  endif()
endmacro()
