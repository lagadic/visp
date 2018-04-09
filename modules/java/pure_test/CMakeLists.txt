if(NOT ANT_EXECUTABLE)
  return()
endif()

project(visp_test_java)

set(visp_test_java_bin_dir "${CMAKE_CURRENT_BINARY_DIR}/.build")
set(test_dir ${CMAKE_CURRENT_SOURCE_DIR})
set(test_common_dir "${CMAKE_CURRENT_SOURCE_DIR}/../common_test")

set(visp_test_java_file_deps "")

# make sure the build directory exists
file(MAKE_DIRECTORY "${visp_test_java_bin_dir}")

# 1. gather and copy common test files (resources, utils, etc.)
copy_common_tests(test_common_dir visp_test_java_bin_dir visp_test_java_file_deps)

# 2. gather and copy tests from each module
copy_modules_tests(VISP_JAVA_MODULES visp_test_java_bin_dir visp_test_java_file_deps)

# 3. gather and copy specific files for pure java
file(GLOB_RECURSE test_files RELATIVE "${test_dir}" "${test_dir}/src/*")
file(GLOB_RECURSE test_lib_files RELATIVE "${test_dir}" "${test_dir}/lib/*.jar")
foreach(f ${test_files} ${test_lib_files})
  add_custom_command(OUTPUT "${visp_test_java_bin_dir}/${f}"
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different "${test_dir}/${f}" "${visp_test_java_bin_dir}/${f}"
                     DEPENDS "${test_dir}/${f}"
                     COMMENT "Copying ${f}"
                    )
  list(APPEND visp_test_java_file_deps "${test_dir}/${f}" "${visp_test_java_bin_dir}/${f}")
endforeach()

# Copy the ViSP jar after it has been generated.
add_custom_command(OUTPUT "${visp_test_java_bin_dir}/bin/${JAR_NAME}"
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different "${JAR_FILE}" "${visp_test_java_bin_dir}/bin/${JAR_NAME}"
                   DEPENDS "${JAR_FILE}"
                   COMMENT "Copying the ViSP jar"
                  )

add_custom_command(OUTPUT "${visp_test_java_bin_dir}/build.xml"
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/build.xml" "${visp_test_java_bin_dir}/build.xml"
                   DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build.xml"
                   COMMENT "Copying build.xml"
                  )

add_custom_command(OUTPUT "${visp_test_java_bin_dir}/build/jar/visp-test.jar"
                   COMMAND "${ANT_EXECUTABLE}" build
                   WORKING_DIRECTORY "${visp_test_java_bin_dir}"
                   DEPENDS ${visp_test_java_file_deps} "${visp_test_java_bin_dir}/build.xml" "${CMAKE_CURRENT_SOURCE_DIR}/build.xml" "${JAR_FILE}" "${visp_test_java_bin_dir}/bin/${JAR_NAME}"
                   COMMENT "Build Java tests"
                  )

# Not add_custom_command because generator expressions aren't supported in
# OUTPUT file names, and we need to generate different files for different
# configurations.
add_custom_target(${PROJECT_NAME}_properties
                  COMMAND "${CMAKE_COMMAND}" -E echo "visp.lib.path = $<TARGET_FILE_DIR:${the_module}>"
                    > "${visp_test_java_bin_dir}/ant-$<CONFIGURATION>.properties"
                  VERBATIM
                 )

add_custom_target(${PROJECT_NAME} ALL
                  DEPENDS ${the_module} ${PROJECT_NAME}_properties
                  SOURCES "${visp_test_java_bin_dir}/build/jar/visp-test.jar"
                 )

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "tests accuracy")
endif()
