
find_package(Python COMPONENTS Interpreter)
if(Python_FOUND) # We have Python, we can install module

  # Set pip args
  if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
    set(_pip_args)
  else()
    set(_pip_args "--user")
  endif()
  # Step 1: Generate configuration file
  # Define modules for which to generate python bindings
  set(python_ignored_modules "visp_python" "visp_java_bindings_generator")
  set(python_bound_modules ${VISP_MODULES_BUILD})
  list(REMOVE_ITEM python_bound_modules ${python_ignored_modules})

  # Define ViSP module
  vp_add_module(python BINDINGS OPTIONAL ${python_bound_modules})
  message("${VISP_MODULE_${visp_python}_REQ_DEPS}")

  foreach(module ${python_bound_modules})
    message("Module ${module} headers: ${VISP_MODULE_${module}_HEADERS}")
  endforeach()





  # Step 1: Generate bindings
  set(bindgen_package_location "${CMAKE_CURRENT_SOURCE_DIR}/generator")
  set(bindings_gen_location "${CMAKE_CURRENT_BINARY_DIR}/bindings")
  add_custom_target(visp_python_bindings_generator_run
    COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -e ${bindgen_package_location}
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/bindings" ${bindings_gen_location}
    COMMAND visp_pybindgen --config "${CMAKE_CURRENT_SOURCE_DIR}/config" --build-folder ${bindings_gen_location}
    DEPENDS visp_doc
  )




  # Step 3: Compile and install bindings as a python package
  add_custom_target(visp_python_bindings_install
    COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -e ${bindings_gen_location}
  )

  # Step 4: Copy stubs dir and install stubs for autocompletion

  # Step 5: Build documentation




endif()
