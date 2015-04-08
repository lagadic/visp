# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file(
  "${VISP_CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(uninstall PROPERTIES FOLDER "CMakeTargets")
endif()

# ----------------------------------------------------------------------------
#   Doxygen documentation target, for "make visp_doc" and "make html-doc" (to keep compat with previous versions)
# ----------------------------------------------------------------------------
if(DOXYGEN_FOUND)
  add_custom_target(html-doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen) # for compat with previous versions
  add_custom_target(visp_doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_doc PROPERTIES FOLDER "extra")
    set_target_properties(html-doc PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_library
# ----------------------------------------------------------------------------
add_custom_target(visp_library)
if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(visp_library PROPERTIES FOLDER "extra")
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_tests
# ----------------------------------------------------------------------------
if(BUILD_TESTS)
  add_custom_target(visp_tests)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_tests PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_examples
# ----------------------------------------------------------------------------
if(BUILD_EXAMPLES)
  add_custom_target(visp_examples)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_examples PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_demos
# ----------------------------------------------------------------------------
if(BUILD_DEMOS)
  add_custom_target(visp_demos)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_demos PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_tutorials
# ----------------------------------------------------------------------------
if(BUILD_TUTORIALS)
  add_custom_target(visp_tutorials)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_tutorials PROPERTIES FOLDER "extra")
  endif()
endif()
