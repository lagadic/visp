if (ANDROID)
    # --------------------------------------------------------------------------------------------
    #  Installation for Android ndk-build makefile:  VISP.mk
    #  Part 1/2: ${BIN_DIR}/VISP.mk              -> For use *without* "make install"
    #  Part 2/2: ${BIN_DIR}/unix-install/VISP.mk -> For use with "make install"
    # -------------------------------------------------------------------------------------------

    # build type
    if (BUILD_SHARED_LIBS)
        set(VISP_LIBTYPE_CONFIGMAKE "SHARED")
    else ()
        set(VISP_LIBTYPE_CONFIGMAKE "STATIC")
    endif ()

    if (BUILD_FAT_JAVA_LIB)
        set(VISP_LIBTYPE_CONFIGMAKE "SHARED")
        set(VISP_STATIC_LIBTYPE_CONFIGMAKE "STATIC")
    else ()
        set(VISP_STATIC_LIBTYPE_CONFIGMAKE ${VISP_LIBTYPE_CONFIGMAKE})
    endif ()

    # todo: i guess we can exclude 3rd party libraries here, so that i can show a quick demo
    # initially was : ocv_get_all_libs(VISP_MODULES_BUILD OPENCV_EXTRA_COMPONENTS OPENCV_3RDPARTY_COMPONENTS)
    # initially wanted to build the list of visp libs and dependencies for all modules/
    # todo : for now i'm including only core module. To add all libs, u've to define a fn like ocv_get_all_libs
    # That function should have VISP_MODULES_PUBLIC, not *_BUILD. Refer ocv_get_all_libs in OpenCVUtils.cmake
    list(APPEND VISP_MODULES_BUILD "visp_core")

    # list -> string
    # initially was : foreach (_var VISP_MODULES_BUILD OPENCV_EXTRA_COMPONENTS OPENCV_3RDPARTY_COMPONENTS)
    foreach (_var VISP_MODULES_BUILD)
        set(var "${_var}_CONFIGMAKE")
        set(${var} "")
        foreach (lib ${${_var}})
            set(lib_name "${lib}")
            if (TARGET ${lib})
                get_target_property(_output ${lib} IMPORTED_LOCATION)
                if (NOT _output)
                    get_target_property(output_name ${lib} OUTPUT_NAME)
                    if (output_name)
                        set(lib_name "${output_name}")
                    endif ()
                else ()
                    vp_get_libname(lib_name "${_output}")
                endif ()
            endif ()
            set(${var} "${${var}} ${lib_name}")
        endforeach ()
        string(STRIP "${${var}}" ${var})
    endforeach ()

    # replace 'visp_<module>' -> '<module>''
    string(REPLACE "visp_" "" VISP_MODULES_BUILD_CONFIGMAKE "${VISP_MODULES_BUILD_CONFIGMAKE}")

    if (BUILD_FAT_JAVA_LIB)
        set(VISP_LIBS_CONFIGMAKE java3)
    else ()
        set(VISP_LIBS_CONFIGMAKE "${VISP_MODULES_BUILD_CONFIGMAKE}")
    endif ()

    # -------------------------------------------------------------------------------------------
    #  Part 1/2: ${BIN_DIR}/VISP.mk              -> For use *without* "make install"
    # -------------------------------------------------------------------------------------------
    set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\"${VISP_CONFIG_FILE_INCLUDE_DIR}\"")
    set(VISP_BASE_INCLUDE_DIR_CONFIGCMAKE "\"${VISP_SOURCE_DIR}\"")
    set(VISP_LIBS_DIR_CONFIGCMAKE "\$(VISP_THIS_DIR)/lib/\$(VISP_TARGET_ARCH_ABI)")

    # 3rd party later
    # set(OPENCV_3RDPARTY_LIBS_DIR_CONFIGCMAKE "\$(OPENCV_THIS_DIR)/3rdparty/lib/\$(OPENCV_TARGET_ARCH_ABI)")

    configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISP.mk.in" "${CMAKE_BINARY_DIR}/VISP.mk" @ONLY)
    configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISP-abi.mk.in" "${CMAKE_BINARY_DIR}/VISP-${ANDROID_NDK_ABI_NAME}.mk" @ONLY)

    # -------------------------------------------------------------------------------------------
    #  Part 2/2: ${BIN_DIR}/unix-install/VISP.mk -> For use with "make install"
    # -------------------------------------------------------------------------------------------

    # TODO : no need for includes in VISP
    # set(VISP_INCLUDE_DIRS_CONFIGCMAKE "\"\$(LOCAL_PATH)/\$(VISP_THIS_DIR)/include/opencv\" \"\$(LOCAL_PATH)/\$(VISP_THIS_DIR)/include\"")
    set(VISP_INCLUDE_DIRS_CONFIGCMAKE  "")

    set(VISP_BASE_INCLUDE_DIR_CONFIGCMAKE "")
    set(VISP_LIBS_DIR_CONFIGCMAKE "\$(VISP_THIS_DIR)/../libs/\$(VISP_TARGET_ARCH_ABI)")

    # 3rd party later
    # set(OPENCV_3RDPARTY_LIBS_DIR_CONFIGCMAKE "\$(OPENCV_THIS_DIR)/../3rdparty/libs/\$(OPENCV_TARGET_ARCH_ABI)")

    configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISP.mk.in" "${CMAKE_BINARY_DIR}/unix-install/VISP.mk" @ONLY)
    configure_file("${VISP_SOURCE_DIR}/cmake/templates/VISP-abi.mk.in" "${CMAKE_BINARY_DIR}/unix-install/VISP-${ANDROID_NDK_ABI_NAME}.mk" @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/unix-install/VISP.mk DESTINATION ${VISP_CONFIG_INSTALL_PATH} COMPONENT dev)
    install(FILES ${CMAKE_BINARY_DIR}/unix-install/VISP-${ANDROID_NDK_ABI_NAME}.mk DESTINATION ${VISP_CONFIG_INSTALL_PATH} COMPONENT dev)
endif (ANDROID)
