# visp_robot is optional to run testFeatureSegment.cpp
vp_add_module(vs visp_core OPTIONAL visp_robot)
vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()
vp_add_tests()
