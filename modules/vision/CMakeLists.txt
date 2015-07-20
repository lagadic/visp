vp_add_module(vision visp_core)
vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

if (CMAKE_VERSION VERSION_LESS 2.8.12)
  vp_add_tests(
    SOURCES_EXCLUDE
      key-point/testKeyPoint-2.cpp
      key-point/testKeyPoint-4.cpp
    DEPENDS_ON visp_mbt)
else()
  vp_add_tests(DEPENDS_ON visp_mbt)
endif()
