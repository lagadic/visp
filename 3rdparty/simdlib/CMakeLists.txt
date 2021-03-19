project(${SIMD_LIBRARY})

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
    set(SSE_FLAG    "")
    set(SSE2_FLAG   "")
    set(SSE3_FLAG   "")
    set(SSSE3_FLAG  "")
    set(SSE4_1_FLAG "")
    set(SSE4_2_FLAG "")
    set(AVX_FLAG    "")
    set(AVX2_FLAG   "")

    if(MSVC)
        if(NOT MSVC64)
            vp_check_compiler_flag(CXX "/arch:SSE"    HAVE_SSE_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse.cpp")
            vp_check_compiler_flag(CXX "/arch:SSE2"   HAVE_SSE2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        endif()

        vp_check_compiler_flag(CXX "/arch:AVX"    HAVE_AVX_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX2"   HAVE_AVX2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")

        if(HAVE_SSE_FLAG)
            set(SSE_FLAG "/arch:SSE")
        endif()
        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "/arch:SSE2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG    "/arch:AVX")
            set(SSE4_2_FLAG "/arch:AVX")
            set(SSE4_1_FLAG "/arch:AVX")
            set(SSSE3_FLAG  "/arch:AVX")
            set(SSE3_FLAG   "/arch:AVX")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "/arch:AVX2")
        endif()
    else()
        vp_check_compiler_flag(CXX "-msse"    HAVE_SSE_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse.cpp")
        vp_check_compiler_flag(CXX "-msse2"   HAVE_SSE2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        vp_check_compiler_flag(CXX "-msse3"   HAVE_SSE3_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse3.cpp")
        vp_check_compiler_flag(CXX "-mssse3"  HAVE_SSSE3_FLAG   "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_ssse3.cpp")
        vp_check_compiler_flag(CXX "-msse4.1" HAVE_SSE4_1_FLAG  "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse41.cpp")
        vp_check_compiler_flag(CXX "-msse4.2" HAVE_SSE4_2_FLAG  "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse42.cpp")
        vp_check_compiler_flag(CXX "-mavx"    HAVE_AVX_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "-mavx2"   HAVE_AVX2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")
        vp_check_compiler_flag(CXX "-Wno-unused-variable"    HAVE_NO_UNUSED_VARIABLE_FLAG  "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-unused-parameter"   HAVE_NO_UNUSED_PARAMETER_FLAG "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-sign-compare"       HAVE_NO_SIGN_COMPARE_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")
        vp_check_compiler_flag(CXX "-Wno-ignored-qualifiers" HAVE_NO_IGNORED_QUALIFIERS    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_warning.cpp")

        if(HAVE_SSE_FLAG)
            set(SSE_FLAG "-msse")
        endif()
        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "-msse2")
        endif()
        if(HAVE_SSE3_FLAG)
            set(SSE3_FLAG "-msse3")
        endif()
        if(HAVE_SSSE3_FLAG)
            set(SSSE3_FLAG "-mssse3")
        endif()
        if(HAVE_SSE4_1_FLAG)
            set(SSE4_1_FLAG "-msse4.1")
        endif()
        if(HAVE_SSE4_2_FLAG)
            set(SSE4_2_FLAG "-msse4.2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG "-mavx")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "-mavx2")
        endif()
        if(HAVE_NO_UNUSED_VARIABLE_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-variable")
        endif()
        if(HAVE_NO_UNUSED_PARAMETER_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-unused-parameter")
        endif()
        if(HAVE_NO_SIGN_COMPARE_FLAG)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-sign-compare")
        endif()
        if(HAVE_NO_IGNORED_QUALIFIERS)
            set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-ignored-qualifiers")
        endif()
    endif()

    file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
    set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

    file(GLOB_RECURSE SIMD_SSE1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse1*.cpp)
    set_source_files_properties(${SIMD_SSE1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE_FLAG}")

    file(GLOB_RECURSE SIMD_SSE2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse2*.cpp)
    set_source_files_properties(${SIMD_SSE2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE2_FLAG}")

    file(GLOB_RECURSE SIMD_SSE3_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse3*.cpp)
    set_source_files_properties(${SIMD_SSE3_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE3_FLAG}")

    file(GLOB_RECURSE SIMD_SSSE3_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSsse3*.cpp)
    set_source_files_properties(${SIMD_SSSE3_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSSE3_FLAG}")

    file(GLOB_RECURSE SIMD_SSE41_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse41*.cpp)
    set_source_files_properties(${SIMD_SSE41_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_1_FLAG}")

    file(GLOB_RECURSE SIMD_SSE42_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse42*.cpp)
    set_source_files_properties(${SIMD_SSE42_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_2_FLAG}")

    file(GLOB_RECURSE SIMD_AVX1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx1*.cpp)
    set_source_files_properties(${SIMD_AVX1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX_FLAG}")

    file(GLOB_RECURSE SIMD_AVX2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx2*.cpp)
    if(MSVC)
        set_source_files_properties(${SIMD_AVX2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")
    else()
        set_source_files_properties(${SIMD_AVX2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG} -mfma")
    endif()

    set(SIMD_LIB_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")
    set(SIMD_ALG_SRC ${SIMD_BASE_SRC} ${SIMD_SSE1_SRC} ${SIMD_SSE2_SRC} ${SIMD_SSE3_SRC} ${SIMD_SSSE3_SRC} ${SIMD_SSE41_SRC} ${SIMD_SSE42_SRC} ${SIMD_AVX1_SRC} ${SIMD_AVX2_SRC})

    file(GLOB_RECURSE SIMD_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdLib.cpp)
    set_source_files_properties(${SIMD_LIB_SRC} PROPERTIES COMPILE_FLAGS "${SIMD_LIB_FLAGS}")
    add_library(${SIMD_LIBRARY} STATIC ${SIMD_LIB_SRC} ${SIMD_ALG_SRC} ${SIMD_BASE_HDR})

elseif(ARM OR AARCH64)
    if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") AND (NOT(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0.0")))
        set(COMMON_CXX_FLAGS "${COMMON_CXX_FLAGS} -Wno-psabi")
    endif()

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        set(CXX_NEON_FLAG "-mfpu=neon")
    else()
        set(CXX_NEON_FLAG "")
    endif()

    if((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") OR (CMAKE_CXX_COMPILER MATCHES "clang"))
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
        set(SSE_FLAG    "")
        set(SSE2_FLAG   "")
        set(SSE3_FLAG   "")
        set(SSSE3_FLAG  "")
        set(SSE4_1_FLAG "")
        set(SSE4_2_FLAG "")
        set(AVX_FLAG    "")
        set(AVX2_FLAG   "")

        vp_check_compiler_flag(CXX "/arch:SSE"    HAVE_SSE_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse.cpp")
        vp_check_compiler_flag(CXX "/arch:SSE2"   HAVE_SSE2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_sse2.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX"    HAVE_AVX_FLAG     "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx.cpp")
        vp_check_compiler_flag(CXX "/arch:AVX2"   HAVE_AVX2_FLAG    "${PROJECT_SOURCE_DIR}/cmake/checks/cpu_avx2.cpp")

        if(HAVE_SSE_FLAG)
            set(SSE_FLAG "/arch:SSE")
        endif()
        if(HAVE_SSE2_FLAG)
            set(SSE2_FLAG "/arch:SSE2")
        endif()
        if(HAVE_AVX_FLAG)
            set(AVX_FLAG    "/arch:AVX")
            set(SSE4_2_FLAG "/arch:AVX")
            set(SSE4_1_FLAG "/arch:AVX")
            set(SSSE3_FLAG  "/arch:AVX")
            set(SSE3_FLAG   "/arch:AVX")
        endif()
        if(HAVE_AVX2_FLAG)
            set(AVX2_FLAG "/arch:AVX2")
        endif()

        file(GLOB_RECURSE SIMD_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdBase*.cpp)
        set_source_files_properties(${SIMD_BASE_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS}")

        file(GLOB_RECURSE SIMD_SSE1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse1*.cpp)
        set_source_files_properties(${SIMD_SSE1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE_FLAG}")

        file(GLOB_RECURSE SIMD_SSE2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse2*.cpp)
        set_source_files_properties(${SIMD_SSE2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE2_FLAG}")

        file(GLOB_RECURSE SIMD_SSE3_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse3*.cpp)
        set_source_files_properties(${SIMD_SSE3_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE3_FLAG}")

        file(GLOB_RECURSE SIMD_SSSE3_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSsse3*.cpp)
        set_source_files_properties(${SIMD_SSSE3_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSSE3_FLAG}")

        file(GLOB_RECURSE SIMD_SSE41_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse41*.cpp)
        set_source_files_properties(${SIMD_SSE41_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_1_FLAG}")

        file(GLOB_RECURSE SIMD_SSE42_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdSse42*.cpp)
        set_source_files_properties(${SIMD_SSE42_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${SSE4_2_FLAG}")

        file(GLOB_RECURSE SIMD_AVX1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx1*.cpp)
        set_source_files_properties(${SIMD_AVX1_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX_FLAG}")

        file(GLOB_RECURSE SIMD_AVX2_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Simd/SimdAvx2*.cpp)
        set_source_files_properties(${SIMD_AVX2_SRC} PROPERTIES COMPILE_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")

        set(SIMD_LIB_FLAGS "${COMMON_CXX_FLAGS} ${AVX2_FLAG}")
        set(SIMD_ALG_SRC ${SIMD_BASE_SRC} ${SIMD_SSE1_SRC} ${SIMD_SSE2_SRC} ${SIMD_SSE3_SRC} ${SIMD_SSSE3_SRC} ${SIMD_SSE41_SRC} ${SIMD_SSE42_SRC} ${SIMD_AVX1_SRC} ${SIMD_AVX2_SRC})

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
