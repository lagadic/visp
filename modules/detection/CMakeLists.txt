# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

if(USE_ZBAR)
  list(APPEND opt_incs ${ZBAR_INCLUDE_DIRS})
  list(APPEND opt_libs ${ZBAR_LIBRARIES})
endif()
if(USE_DMTX)
  list(APPEND opt_incs ${DMTX_INCLUDE_DIRS})
  list(APPEND opt_libs ${DMTX_LIBRARIES})
endif()

vp_add_module(detection visp_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

