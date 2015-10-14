if(NOT VISP_MODULES_PATH)
  set(VISP_MODULES_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# extra modules are outside ViSP. Useful for contrib or specific additional modules
vp_glob_modules(${VISP_MODULES_PATH} EXTRA ${VISP_CONTRIB_MODULES_PATH})

