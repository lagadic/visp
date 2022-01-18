Here we give the list of the main changes introduced in Simdlib for ViSP

- 2022.01.18: (see PR #990)

  - Fix memory leak in BgraToRGBa conversion
    Modified files:
      3rdparty/simdlib/Simd/SimdAvx2BgraToBgr.cpp  (search "Modif FS Fix memleak")
      3rdparty/simdlib/Simd/SimdSse41BgraToBgr.cpp (search "Modif FS Fix memleak")

  - Introduce avx2 support in SimdBgraToBgr()
    Modified files:
      3rdparty/simdlib/Simd/SimdLib.cpp

  - Remove CpuSocketNumber() and CpuCoreNumber() that are using lscpu that doesn't exist on macOS
    When examples are run on macOS they produce extra printings like:
    ```
    $ ./tutorial-ibvs-4pts-plotter
    sh: lscpu: command not found
    sh: lscpu: command not found
    sh: lscpu: command not found
    sh: lscpu: command not found
    ```
    One solution is to modify `CpuCoreNumber()` to detect macOS and run 
    `sysctl -a | grep machdep.cpu.thread_count | grep -o -E '[0-9]+'`
    instead of
    `lscpu -b -p=Core | grep -v '^#' | sort -u | wc -l"`.
    But then how to modify `CpuSocketNumber()`?
    Theses functions could be removed since they also introduce a non significant time when
    launching a binary.
    Modified files:
      3rdparty/simdlib/Simd/SimdBaseCpu.cpp
      3rdparty/simdlib/Simd/SimdCpu.h
    to remove all reference to
      CpuSocketNumber() 
      CpuCoreNumber(
      AlgCacheL2()
      AlgCacheL3()
      SOCKET_NUMBER
      CORE_NUMBER
      L2_CACHE_SIZE
      L3_CACHE_SIZE

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

