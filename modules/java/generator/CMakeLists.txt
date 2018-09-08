set(MODULE_NAME "java_bindings_generator")
set(VISP_MODULE_IS_PART_OF_WORLD FALSE)
vp_add_module(${MODULE_NAME} INTERNAL)

set(VISP_JAVA_SIGNATURES_FILE "${CMAKE_CURRENT_BINARY_DIR}/visp_java_signatures.json" CACHE INTERNAL "")
set(VISP_JAVA_BINDINGS_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "")

file(REMOVE_RECURSE "${VISP_JAVA_BINDINGS_DIR}/gen")
file(REMOVE "${VISP_DEPHELPER}/gen_visp_java_source")  # force re-run after CMake

# This file is included from a subdirectory
set(JAVA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
include(${JAVA_SOURCE_DIR}/common.cmake)

set(__remap_config "") # list of remapped ".in" files (configure_file)
set(__remap_targets "")

macro(vp_remap_files files_list_var)
  set(target_dir "${VISP_BINARY_DIR}/configured")
  foreach(f ${${files_list_var}})
    if(NOT "${f}" MATCHES "^(.*)\\.in$")
      #continue()  # since CMake 3.2+
    else()
    set(f_ "${CMAKE_MATCH_1}")
    file(RELATIVE_PATH rel_path0 "${VISP_SOURCE_DIR}" "${f}")
    file(RELATIVE_PATH rel_path1 "${VISP_SOURCE_DIR}" "${f_}")
    set(__target_file "${target_dir}/${rel_path1}")
    configure_file("${f}" "${__target_file}" @ONLY)
    if(__remap_config)
      set(__remap_config "${__remap_config},\n")
    endif()
    set(__remap_config "${__remap_config}    { \"src\": \"${rel_path0}\", \"target\": \"${__target_file}\" }")
    list(APPEND __remap_targets "${__target_file}")
    endif()
  endforeach()
endmacro()

# common files
file(GLOB_RECURSE deps "${CMAKE_CURRENT_SOURCE_DIR}/src/*" "${CMAKE_CURRENT_SOURCE_DIR}/android*/*" "${CMAKE_CURRENT_SOURCE_DIR}/templates/*")
vp_remap_files(deps)

set(__modules_config "") # list of ViSP modules
foreach(m ${VISP_JAVA_MODULES})
  string(REGEX REPLACE "^visp_" "" m_ "${m}")
  set(module_java_dir "${CMAKE_CURRENT_SOURCE_DIR}/../misc/${m_}")
  list(APPEND deps ${VISP_MODULE_${m}_HEADERS})
  file(GLOB_RECURSE misc_files "${module_java_dir}/*")
  list(APPEND deps ${misc_files})

  if(__modules_config)
    set(__modules_config "${__modules_config},\n")
  endif()
  file(RELATIVE_PATH rel_path "${VISP_SOURCE_DIR}" "${VISP_MODULE_${m}_LOCATION}")
  set(__modules_config "${__modules_config}    { \"name\": \"${m_}\", \"location\": \"${rel_path}\" }")

  vp_remap_files(misc_files)
endforeach(m)

set(CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/gen_java.json")
set(__config_str
"{
  \"rootdir\": \"${VISP_SOURCE_DIR}\",
  \"modules\": [
${__modules_config}
  ],
  \"files_remap\": [
${__remap_config}
  ]
}
")
if(EXISTS "${CONFIG_FILE}")
  file(READ "${CONFIG_FILE}" __content)
else()
  set(__content "")
endif()
if(NOT "${__content}" STREQUAL "${__config_str}")
  file(WRITE "${CONFIG_FILE}" "${__config_str}")
  file(REMOVE "${VISP_DEPHELPER}/gen_visp_java_source")
endif()
unset(__config_str)

set(java_generated_files
    # "${VISP_JAVA_SIGNATURES_FILE}"
    "${VISP_DEPHELPER}/gen_visp_java_source"
)

add_custom_command(
    OUTPUT ${java_generated_files}
    COMMAND ${PYTHON_DEFAULT_EXECUTABLE} "${JAVA_SOURCE_DIR}/generator/gen_java.py" -p "${JAVA_SOURCE_DIR}/generator/gen2.py" -c "${CONFIG_FILE}"
    COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/gen_visp_java_source"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS "${JAVA_SOURCE_DIR}/generator/gen_java.py"
            "${JAVA_SOURCE_DIR}/generator/gen2.py"
            "${JAVA_SOURCE_DIR}/generator/hdr_parser.py"
            # don't, result of file(WRITE): "${CMAKE_CURRENT_BINARY_DIR}/gen_java.json"
            ${deps} ${__remap_targets}
            # not allowed (file(WRITE) result): "${CONFIG_FILE}"
    COMMENT "Generate files for Java bindings"
)

add_custom_target(gen_visp_java_source DEPENDS ${java_generated_files}
    SOURCES "${JAVA_SOURCE_DIR}/generator/gen_java.py"
            "${CMAKE_CURRENT_BINARY_DIR}/gen_java.json"
)
