# platform-specific config file
configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpConfig.h.in" "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp3/core
  COMPONENT dev
)

#----------------------------------------------------------------------
# information file
#----------------------------------------------------------------------
configure_file(${VISP_SOURCE_DIR}/cmake/templates/ViSP-third-party.txt.in "${VISP_BINARY_DIR}/ViSP-third-party.txt")

# ----------------------------------------------------------------------------
#  visp_modules.h based on actual modules list
# ----------------------------------------------------------------------------
set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "#ifndef __visp_modules_h__\n#define __visp_modules_h__\n\n")

set(VISP_MOD_LIST ${VISP_MODULES_PUBLIC})
vp_list_sort(VISP_MOD_LIST)
foreach(m ${VISP_MOD_LIST})
  if(m MATCHES "^visp_")
    string(REGEX REPLACE "^visp_" "" m "${m}")
  endif()
  string(TOUPPER "${m}" m)
  set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}#define VISP_HAVE_MODULE_${m}\n")
endforeach()

set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}\n#endif\n")

configure_file("${VISP_SOURCE_DIR}/cmake/templates/visp_modules.h.in" "${VISP_INCLUDE_DIR}/visp3/visp_modules.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/visp_modules.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp3
  COMPONENT dev
)

# ----------------------------------------------------------------------------
#  install old headers
# ----------------------------------------------------------------------------
file(GLOB old_hdrs "${VISP_INCLUDE_DIR}/visp/*.h")
install(FILES ${old_hdrs}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp
  COMPONENT dev
)
