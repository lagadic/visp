
if(BUILD_PYTHON)

# this avoids non-active conda from getting picked anyway on Windows
set(Python_FIND_REGISTRY LAST)
message("CMAKE INCLUDE DIRS ${VISP}")
# Use environment variable PATH to decide preference for Python
set(Python_FIND_VIRTUALENV FIRST)
set(Python_FIND_STRATEGY LOCATION)
find_package(Python 3.7 COMPONENTS Interpreter Development) # TODO: use visp function to find python?


#todo : Run generator in an initial pass, depends on ConfigFile and vpConfig.h

if(Python_FOUND) # We have Python, we can install module

  # Set pip args
  if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
    set(_pip_args)
  else()
    set(_pip_args "--user")
  endif()

  # Step 1: Generate configuration file
  # Define modules for which to generate python bindings
  set(python_ignored_modules "visp_python" "visp_java_bindings_generator" "visp_java")
  set(python_bound_modules ${VISP_MODULES_BUILD})
  list(REMOVE_ITEM python_bound_modules ${python_ignored_modules})

  # Define ViSP module
  vp_add_module(python BINDINGS OPTIONAL ${python_bound_modules})


  # Configure
  set(bindgen_package_location "${CMAKE_CURRENT_SOURCE_DIR}/generator")
  set(bindings_package_location "${CMAKE_CURRENT_SOURCE_DIR}/bindings")
  set(bindings_gen_location "${CMAKE_CURRENT_BINARY_DIR}/bindings")
  file(MAKE_DIRECTORY "${bindings_gen_location}/src")
  #file(TOUCH "${bindings_gen_location}/src/main.cpp")
  set(python_bindings_cpp_src "${bindings_gen_location}/src/main.cpp")

  foreach(module ${python_bound_modules})
    # get_target_property(dirs "${module}" INCLUDE_DIRECTORIES)
    # message("Dirs = ${dirs}")
    string(REPLACE "visp_" "" clean_module_name ${module})
    set(cpp_src "${bindings_gen_location}/src/${clean_module_name}.cpp")
    list(APPEND python_bindings_cpp_src "${cpp_src}")
  endforeach()


  set(json_config_file "{}")
  set(json_include_dirs "[]")
  set(include_dirs_count 0)
  foreach(include_dir ${VISP_INCLUDE_DIRS})
    message("Include dir: ${include_dir}")
    string(JSON json_include_dirs SET ${json_include_dirs} "${include_dirs_count}" "\"${include_dir}\"")
    MATH(EXPR include_dir${CMAKE_BINARY_DIR}/VISPGenerateConfigScript.info.cmakes_count "${include_dirs_count}+1")
  endforeach()
  string(JSON json_config_file SET ${json_config_file} "include_dirs" "${json_include_dirs}")
  foreach(module ${python_bound_modules})
    message("${python_bound_modules}")
    string(REPLACE "visp_" "" clean_module_name ${module})

    string(JSON json_config_file SET ${json_config_file} ${clean_module_name} "{}")

    set(json_header_list "[]")
    set(header_count 0)
    foreach(module_header ${VISP_MODULE_${module}_HEADERS})
      string(JSON json_header_list SET ${json_header_list} "${header_count}" "\"${module_header}\"")
      MATH(EXPR header_count "${header_count}+1")
    endforeach()
    string(JSON json_config_file SET ${json_config_file} ${clean_module_name} "headers" "${json_header_list}")
    #file(TOUCH "${cpp_src}")
  endforeach()
  message("${json_config_file}")




  # Step 2: Generate bindings




  add_custom_target( visp_python_bindings_generator_run
    COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -v ${bindgen_package_location}
    COMMAND ${Python_EXECUTABLE} -m visp_python_bindgen.generator --config "${CMAKE_CURRENT_SOURCE_DIR}/config" --build-folder ${bindings_gen_location}
    BYPRODUCTS ${python_bindings_cpp_src}
    DEPENDS ${python_bound_modules}
  )


  add_subdirectory(bindings "${bindings_gen_location}")


  # Step 3: Compile and install bindings as a python package
  # Cmake needs to be run
  # add_custom_target(visp_python_bindings_compile
  #   COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${bindings_package_location}" -DVISP_DIR="${VISP_BINARY_DIR}"
  #   COMMAND "${CMAKE_COMMAND}" --build . --parallel ${N_PROC}
  #   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  #   DEPENDS visp_python_bindings_generator_run
  # )

  # add_custom_target(visp_python_bindings_install
  #   COMMAND ${Python_EXECUTABLE} -m pip install ${_pip_args} ${bindings_package_location}
  # )

  # Step 4: Copy stubs dir and install stubs for autocompletion

  # Step 5: Build documentation




endif()

endif()