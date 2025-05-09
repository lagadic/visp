project(${SIMD_LIBRARY})

# SIMD_VERSION should math the version set in Simd/SimdVersion.h
set(SIMD_VERSION "4.9.109"  PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(COMMON_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

if(UNIX)
    if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -fPIC")
    endif()
endif()

# on android, we have -march=armv7-a that need to be removed
string(REGEX REPLACE "(-march=[a-z0-9-]*)|(-mtune=[a-z]*)|(-msse[0-9,\\.]*)|(-mavx[0-9]*)|(-mfma)" "" COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS}")

file(GLOB_RECURSE SIMD_BASE_HDR ${CMAKE_CURRENT_SOURCE_DIR}/Simd/*.h ${CMAKE_CURRENT_SOURCE_DIR}/Simd/*.hpp)

# Inspired from https://github.com/ermig1979/Simd/blob/master/prj/cmake/CMakeLists.txt
# but here we use vars set in VISPDetectPlatform.cmake
if(X86 OR X86_64)

    # Flags check
    set(SSE2_FLAG   "")
    set(SSE4_2_FLAG "")
    set(AVX_FLAG    "")
    set(AVX2_FLAG   "")

    if(MSVC AND NOT ((CMAKE_CXX_COMPILER_ID MATCHES "AppleClang") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang")))
        if(NOT MSVC64)
            vp_check_compiler_flag(CXX "/arch:SSE2"   HAVE_SSE2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        endif()

        vp_check_compiler_flag(CXX "/arch:AVX"    HAVE_AVX_FLAG     "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX2"   HAVE_AVX2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")

        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "/arch:SSE2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG    "/arch:AVX")
            set(SSE4_2_FLAG "/arch:AVX")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "/arch:AVX2")
        endif()

        vp_check_compiler_flag(CXX "/wd4244"  HAVE_W4244_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "/wd4101"  HAVE_W4101_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "/wd4267"  HAVE_W4267_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        if(HAVE_W4244_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4244")
        endif()
        if(HAVE_W4101_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4101")
        endif()
        if(HAVE_W4267_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} /wd4267")
        endif()
    else()
        vp_check_compiler_flag(CXX "-msse2"   HAVE_SSE2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        vp_check_compiler_flag(CXX "-msse4.2" HAVE_SSE4_2_FLAG  "${VISP_SOURCE_DIR}/cmake/checks/cpu_sse42.cpp")
        vp_check_compiler_flag(CXX "-mavx"    HAVE_AVX_FLAG     "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "-mavx2"   HAVE_AVX2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")
        vp_check_compiler_flag(CXX "-mbmi"    HAVE_MBMI_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-mbmi2"   HAVE_MBMI2_FLAG   "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-mlzcnt"  HAVE_MLZCNT_FLAG  "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-mfma"    HAVE_MFMA_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-mno-avx256-split-unaligned-load"    HAVE_AVX256_SUL_FLAG            "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-mno-avx256-split-unaligned-store"   HAVE_AVX256_SUS_FLAG            "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-fabi-version=4"                     HAVE_ABIV4_FLAG                 "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-unused-variable"                HAVE_NO_UNUSED_VARIABLE_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-unused-parameter"               HAVE_NO_UNUSED_PARAMETER_FLAG   "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-unused-function"                HAVE_NO_UNUSED_FUNCTION_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-sign-compare"                   HAVE_NO_SIGN_COMPARE_FLAG       "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-ignored-qualifiers"             HAVE_NO_IGNORED_QUALIFIERS_FLAG "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-reorder"                        HAVE_NO_REORDER_FLAG            "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-strict-aliasing"                HAVE_NO_STRICT_ALIASING_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-missing-field-initializers"     HAVE_NO_MISSING_FIELD_FLAG      "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-stringop-overflow"              HAVE_NO_STRINGOP_OVERFLOW_EQUAL_FLAG "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")

        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "-msse -msse2")
        endif()
        if(HAVE_SSE4_2_FLAG)
            set(SSE4_2_FLAG "-msse3 -mssse3 -msse4.1 -msse4.2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG "-mavx")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -mavx2")
        endif()
        if(HAVE_MBMI_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -mbmi")
        endif()
        if(HAVE_MBMI2_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -mbmi2")
        endif()
        if(HAVE_MLZCNT_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -mlzcnt")
        endif()
        if(HAVE_MFMA_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -mfma")
        endif()
        if(HAVE_AVX256_SUL_FLAG)
          set(AVX_FLAG  "${AVX_FLAG}  -mno-avx256-split-unaligned-load")
          set(AVX2_FLAG "${AVX2_FLAG} -mno-avx256-split-unaligned-load")
        endif()
        if(HAVE_AVX256_SUS_FLAG)
          set(AVX_FLAG  "${AVX_FLAG}  -mno-avx256-split-unaligned-store")
          set(AVX2_FLAG "${AVX2_FLAG} -mno-avx256-split-unaligned-store")
        endif()
        if(HAVE_ABIV4_FLAG)
            set(AVX2_FLAG "${AVX2_FLAG} -fabi-version=4")
        endif()
        if(HAVE_NO_UNUSED_VARIABLE_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-variable")
        endif()
        if(HAVE_NO_UNUSED_PARAMETER_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-parameter")
        endif()
        if(HAVE_NO_UNUSED_FUNCTION_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-function")
        endif()
        if(HAVE_NO_SIGN_COMPARE_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-sign-compare")
        endif()
        if(HAVE_NO_IGNORED_QUALIFIERS_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-ignored-qualifiers")
        endif()
        if(HAVE_NO_REORDER_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-reorder")
        endif()
        if(HAVE_NO_STRICT_ALIASING_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-strict-aliasing")
        endif()
        if(HAVE_NO_MISSING_FIELD_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-missing-field-initializers")
        endif()
        if(HAVE_NO_STRINGOP_OVERFLOW_EQUAL_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-stringop-overflow")
        endif()
        if(MSVC AND ((CMAKE_CXX_COMPILER_ID MATCHES "AppleClang") OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang")))
            # Clang under windows needs AVX flags
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} ${AVX_FLAG} ${AVX2_FLAG}")
        endif()
    endif()

    file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
    set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

    file(GLOB_RECURSE SIMD_SSE2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse2*.cpp)
    set_source_files_properties(${SIMD_SSE2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE2_FLAG}")

    file(GLOB_RECURSE SIMD_SSE41_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse41*.cpp)
    set_source_files_properties(${SIMD_SSE41_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_2_FLAG}")

    file(GLOB_RECURSE SIMD_AVX1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx1*.cpp)
    set_source_files_properties(${SIMD_AVX1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX_FLAG}")

    file(GLOB_RECURSE SIMD_AVX2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx2*.cpp)
    set_source_files_properties(${SIMD_AVX2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")

    set(SIMD_LIB_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")
    set(SIMD_ALG_SRC ${SIMD_BASE_SRC} ${SIMD_SSE2_SRC} ${SIMD_SSE41_SRC} ${SIMD_AVX1_SRC} ${SIMD_AVX2_SRC})

    file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
    set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${SIMD_LIB_FLAGS}")
    add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_ALG_SRC} ${SIMD_BASE_HDR})

elseif(ARM OR AARCH64)
    if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") AND (NOT(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0.0")))
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-psabi")
    endif()

    vp_check_compiler_flag(CXX "-Wno-unused-command-line-argument" HAVE_NO_UNUSED_CMD_LINE_FLAG "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
    vp_check_compiler_flag(CXX "-Wno-asm-operand-widths"           HAVE_NO_ASM_WIDTHS_FLAG      "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
    vp_check_compiler_flag(CXX "-Wno-switch"                       HAVE_NO_SWITCH_FLAG          "${VISP_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        if( NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang") OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")))
            set(CXX_NEON_FLAG "-mfpu=neon -mfpu=neon-fp16")
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
            set(CXX_NEON_FLAG "${CXX_NEON_FLAG} -mfp16-format=ieee")
        endif()
    else()
        set(CXX_NEON_FLAG "")
    endif()

    if(HAVE_NO_ASM_WIDTHS_FLAG)
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-asm-operand-widths")
    endif()
    if(HAVE_NO_SWITCH_FLAG)
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-switch")
    endif()
    if(HAVE_NO_UNUSED_CMD_LINE_FLAG)
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-command-line-argument")
    endif()

    if((CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang") OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
        add_definitions(-DSIMD_NEON_FP16_DISABLE)
    endif()

    file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
    set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

    file(GLOB_RECURSE SIMD_NEON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdNeon*.cpp)
    set_source_files_properties(${SIMD_NEON_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${CXX_NEON_FLAG}")

    file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
    set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${CXX_NEON_FLAG}")
    add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_BASE_SRC} ${SIMD_NEON_SRC} ${SIMD_BASE_HDR})

elseif(WINRT)
    if(CMAKE_GENERATOR_PLATFORM MATCHES "ARM")
        add_definitions(-DSIMD_NEON_FP16_DISABLE)
        add_definitions(-DSIMD_NEON_DISABLE)
        set(CXX_NEON_FLAG "")

        file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
        set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

        file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
        set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${CXX_NEON_FLAG}")
       add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_BASE_SRC} ${SIMD_NEON_SRC} ${SIMD_BASE_HDR})
    else()
        # Flags check
        set(SSE2_FLAG   "")
        set(SSE4_2_FLAG "")
        set(AVX_FLAG    "")
        set(AVX2_FLAG   "")

        vp_check_compiler_flag(CXX "/arch:SSE2"   HAVE_SSE2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX"    HAVE_AVX_FLAG     "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX2"   HAVE_AVX2_FLAG    "${VISP_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")

        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "/arch:SSE2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG    "/arch:AVX")
            set(SSE4_2_FLAG "/arch:AVX")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "/arch:AVX2")
        endif()

        file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
        set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

        file(GLOB_RECURSE SIMD_SSE2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse2*.cpp)
        set_source_files_properties(${SIMD_SSE2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE2_FLAG}")

        file(GLOB_RECURSE SIMD_SSE41_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse41*.cpp)
        set_source_files_properties(${SIMD_SSE41_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_2_FLAG}")

        file(GLOB_RECURSE SIMD_AVX1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx1*.cpp)
        set_source_files_properties(${SIMD_AVX1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX_FLAG}")

        file(GLOB_RECURSE SIMD_AVX2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx2*.cpp)
        set_source_files_properties(${SIMD_AVX2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")

        set(SIMD_LIB_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")
        set(SIMD_ALG_SRC ${SIMD_BASE_SRC} ${SIMD_SSE2_SRC} ${SIMD_SSE41_SRC} ${SIMD_AVX1_SRC} ${SIMD_AVX2_SRC})

        file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
        set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${SIMD_LIB_FLAGS}")
        add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_ALG_SRC} ${SIMD_BASE_HDR})
    endif()
else()
    file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
    set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

    file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
    set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")
    add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_BASE_SRC} ${SIMD_BASE_HDR})
endif()

if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(${SIMD_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${SIMD_LIBRARY}
    PROPERTIES OUTPUT_NAME ${SIMD_LIBRARY}
    DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
    COMPILE_PDB_NAME ${SIMD_LIBRARY}
    COMPILE_PDB_NAME_DEBUG "${SIMD_LIBRARY}${VISP_DEBUG_POSTFIX}"
    ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
)

if(NOT BUILD_SHARED_LIBS)
    vp_install_target(${SIMD_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

vp_parse_header3(SIMD "${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdVersion.h" "SIMD_VERSION" SIMD_VERSION PARENT_SCOPE)
