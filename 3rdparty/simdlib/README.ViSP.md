Here we give the list of the main changes introduced in Simdlib for ViSP

- 2022.01.18: Fix memory leak in BgraToRGBa conversion (see PR #990)

  Modified files:

    3rdparty/simdlib/Simd/SimdAvx2BgraToBgr.cpp  (search "Modif FS Fix memleak")
    3rdparty/simdlib/Simd/SimdSse41BgraToBgr.cpp (search "Modif FS Fix memleak")
    3rdparty/simdlib/Simd/SimdLib.cpp            (avx2 support in SimdBgraToBgr)

- 2022.01.13: Fix compat with iOS (see PR #983)

  Modified files (search "Modified for iOS" and "Modified for c++98"):

    3rdparty/simdlib/Simd/SimdBaseCpu.cpp
    3rdparty/simdlib/Simd/SimdNeonCpu.cpp
    ...

- 2022.01.12: Update Simd to v4.9.109 (see PR #982)

    See PR #982

- 2021.02.16: Fix compat with UWP

  Modified files:

    3rdparty/simdli/CMakeLists.txt

- 2020.12.07: introduce BgraToRgba conversion optimized for Avx2, Ssse3, Neon (see PR #862)

  New files:

	3rdparty/simdlib/Simd/SimdAvx2BgraToRgba.cpp
	3rdparty/simdlib/Simd/SimdBaseBgraToRgba.cpp
	3rdparty/simdlib/Simd/SimdNeonBgraToRgba.cpp
	3rdparty/simdlib/Simd/SimdSsse3BgraToRGBa.cpp

  Modified files:

	3rdparty/simdlib/Simd/SimdAvx2.h
	3rdparty/simdlib/Simd/SimdBase.h
	3rdparty/simdlib/Simd/SimdConst.h
	3rdparty/simdlib/Simd/SimdConversion.h
	3rdparty/simdlib/Simd/SimdLib.cpp
	3rdparty/simdlib/Simd/SimdLib.h
	3rdparty/simdlib/Simd/SimdNeon.h
	3rdparty/simdlib/Simd/SimdSsse3.h

