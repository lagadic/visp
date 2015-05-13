# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

# If
if(USE_COIN3D AND NOT HAVE_visp_ar)
  if(WIN32)
    add_definitions("-DCOIN_DLL")
  endif()
  list(APPEND opt_incs ${COIN3D_INCLUDE_DIRS})
  # On OSX cmake 2.8 found OpenGL but OPENGL_INCLUDE_DIR was set to NOT_FOUND
  # We add a test to be sure that the OPENGL vars exist.
  if(OPENGL_INCLUDE_DIR)
    list(APPEND opt_incs ${OPENGL_INCLUDE_DIR})
  endif()
  if(OPENGL_LIBRARIES)
    list(APPEND opt_libs ${OPENGL_LIBRARIES})
  endif()

  list(APPEND opt_libs ${COIN3D_LIBRARIES})
  if(USE_SOWIN)
    add_definitions("-DSOWIN_DLL")
    list(APPEND opt_incs ${SOWIN_INCLUDE_DIRS})
    list(APPEND opt_libs ${SOWIN_LIBRARIES})
  endif()

  if(USE_SOQT AND USE_QT)
    list(APPEND opt_incs ${SOQT_INCLUDE_DIRS})
    list(APPEND opt_incs ${QT_INCLUDE_DIR})
    list(APPEND opt_incs ${QT_INCLUDES})
    list(APPEND opt_libs ${SOQT_LIBRARIES})
    if(WIN32)
      add_definitions("-DSOQT_DLL")
    endif()

    # We manage QT libraries
    if(DESIRED_QT_VERSION MATCHES 3)
      #Add Qt3 libraries
      set(VISP_HAVE_QT3_FOUND "yes")       # for ViSP-third-party.txt
      #message("QT_QT_LIBRARY ${QT_QT_LIBRARY}")
      list(APPEND opt_libs ${QT_QT_LIBRARY})
    elseif(DESIRED_QT_VERSION MATCHES 4)
      #Add Qt4 libraries
      set(VISP_HAVE_QT4_FOUND "yes")       # for ViSP-third-party.txt
      #message("QT_QTGUI_LIBRARY ${QT_QTGUI_LIBRARY}")
      #message("QT_QTGUI_LIBRARY_RELEASE ${QT_QTGUI_LIBRARY_RELEASE}")
      #message("QT_QTGUI_LIBRARY_DEBUG ${QT_QTGUI_LIBRARY_DEBUG}")
      if(QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE AND QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
        list(APPEND opt_libs optimized ${QT_QTGUI_LIBRARY_RELEASE})
        list(APPEND opt_libs optimized ${QT_QTCORE_LIBRARY_RELEASE})
        list(APPEND opt_libs debug ${QT_QTGUI_LIBRARY_DEBUG})
        list(APPEND opt_libs debug ${QT_QTCORE_LIBRARY_DEBUG})
      elseif(QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
        list(APPEND opt_libs ${QT_QTGUI_LIBRARY_RELEASE})
        list(APPEND opt_libs ${QT_QTCORE_LIBRARY_RELEASE})
      elseif(QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
        list(APPEND opt_libs ${QT_QTGUI_LIBRARY_DEBUG})
        list(APPEND opt_libs ${QT_QTCORE_LIBRARY_DEBUG})
      endif()
    endif()

    # Because in QT_DEFINITIONS defs are separated by ";", parse the
    # QT_DEFINITIONS in order to build a space separated string
    vp_list_remove_separator(QT_DEFINITIONS)
    add_definitions(${QT_DEFINITIONS})
    add_definitions("-DQT_DLL")
  endif(USE_SOQT AND USE_QT)

  if(USE_SOXT)
    # OpenGL and SoXt are found
    list(APPEND opt_libs ${SOXT_LIBRARIES})
  endif()
endif()

vp_add_module(mbt visp_vision visp_core OPTIONAL visp_ar)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
