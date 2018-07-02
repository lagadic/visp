if(ANDROID)
  vp_update(VISP_JAVA_LIB_NAME_SUFFIX "${VISP_VERSION_MAJOR}")
  vp_update(JAVA_INSTALL_ROOT "sdk/java")
else()
  vp_update(VISP_JAVA_LIB_NAME_SUFFIX "${VISP_VERSION_MAJOR}${VISP_VERSION_MINOR}${VISP_VERSION_PATCH}")
endif()

# set the list of modules to wrap.
# To add Java Wrapper for a module, find and change the line given below in <module>/CMakeLists.txt:
# vp_add_module(<mod-name> ....)   -->   vp_add_module(<mod-name> .... WRAP java)
set(VISP_JAVA_MODULES)
foreach(m ${VISP_MODULES_BUILD})
  if (";${VISP_MODULE_${m}_WRAPPERS};" MATCHES ";java;" AND HAVE_${m})
    list(APPEND VISP_JAVA_MODULES ${m})
    #message(STATUS "\t${m}")
  endif()
endforeach()
