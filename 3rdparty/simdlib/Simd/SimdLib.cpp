/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2019 Yermalayeu Ihar,
*               2014-2018 Antonenka Mikhail,
*               2018-2018 Radchenko Andrey,
*               2019-2019 Facundo Galan.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "Simd/SimdConfig.h"

#ifndef SIMD_LIB_CPP
#define SIMD_LIB_CPP
#endif

#if defined(WIN32) && !defined(SIMD_STATIC)

#define SIMD_EXPORTS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hModule, DWORD dwReasonForCall, LPVOID lpReserved)
{
    switch (dwReasonForCall)
    {
    case DLL_PROCESS_DETACH:
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
        return TRUE;
    }
    return TRUE;
}
#endif//WIN32

#include "Simd/SimdLib.h"

#include "Simd/SimdMemory.h"
#include "Simd/SimdEnable.h"
#include "Simd/SimdConst.h"
#include "Simd/SimdCpu.h"
#include "Simd/SimdLog.h"

#include "Simd/SimdResizer.h"

#include "Simd/SimdBase.h"
#include "Simd/SimdSse1.h"
#include "Simd/SimdSse2.h"
#include "Simd/SimdSsse3.h"
#include "Simd/SimdAvx2.h"
#include "Simd/SimdNeon.h"

#if !defined(SIMD_VERSION)
#include "Simd/SimdVersion.h"
#endif

SIMD_API const char * SimdVersion()
{
    return SIMD_VERSION;
}

using namespace Simd;

SIMD_API size_t SimdCpuInfo(SimdCpuInfoType type)
{
    switch (type)
    {
    case SimdCpuInfoSockets: return Cpu::SOCKET_NUMBER;
    case SimdCpuInfoCores: return Cpu::CORE_NUMBER;
    case SimdCpuInfoThreads: return Cpu::THREAD_NUMBER;
    case SimdCpuInfoCacheL1: return Cpu::L1_CACHE_SIZE;
    case SimdCpuInfoCacheL2: return Cpu::L2_CACHE_SIZE;
    case SimdCpuInfoCacheL3: return Cpu::L3_CACHE_SIZE;
#ifdef SIMD_SSE_ENABLE
    case SimdCpuInfoSse: return Sse::Enable ? 1 : 0;
#endif
#ifdef SIMD_SSE2_ENABLE
    case SimdCpuInfoSse2: return Sse2::Enable ? 1 : 0;
#endif
#ifdef SIMD_SSE3_ENABLE
    case SimdCpuInfoSse3: return Sse3::Enable ? 1 : 0;
#endif
#ifdef SIMD_SSSE3_ENABLE
    case SimdCpuInfoSsse3: return Ssse3::Enable ? 1 : 0;
#endif
#ifdef SIMD_SSE41_ENABLE
    case SimdCpuInfoSse41: return Sse41::Enable ? 1 : 0;
#endif
#ifdef SIMD_SSE42_ENABLE
    case SimdCpuInfoSse42: return Sse42::Enable ? 1 : 0;
#endif
#ifdef SIMD_AVX_ENABLE
    case SimdCpuInfoAvx: return Avx::Enable ? 1 : 0;
#endif
#ifdef SIMD_AVX2_ENABLE
    case SimdCpuInfoAvx2: return Avx2::Enable ? 1 : 0;
#endif
#ifdef SIMD_NEON_ENABLE
    case SimdCpuInfoNeon: return Neon::Enable ? 1 : 0;
#endif
    default:
        return 0;
    }
}

SIMD_API const char * SimdPerformanceStatistic()
{
#if defined(SIMD_PERFORMANCE_STATISTIC) && defined(NDEBUG)
    return Base::PerformanceMeasurerStorage::s_storage.PerformanceStatistic();
#else
    return "";
#endif
}

SIMD_API void * SimdAllocate(size_t size, size_t align)
{
    return Allocate(size, align);
}

SIMD_API void SimdFree(void * ptr)
{
    Free(ptr);
}

SIMD_API size_t SimdAlign(size_t size, size_t align)
{
    return AlignHi(size, align);
}

SIMD_API size_t SimdAlignment()
{
    return Simd::ALIGNMENT;
}

SIMD_API void SimdRelease(void * context)
{
    delete (Deletable*)context;
}

SIMD_API SimdBool SimdGetFastMode()
{
#ifdef SIMD_SSE_ENABLE
    if (Sse::Enable)
        return Sse::GetFastMode();
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable)
        return Neon::GetFastMode();
    else
#endif
        return SimdFalse;
}

SIMD_API void SimdSetFastMode(SimdBool value)
{
#ifdef SIMD_SSE_ENABLE
    if (Sse::Enable)
        Sse::SetFastMode(value);
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable)
        Neon::SetFastMode(value);
#endif
}

SIMD_API void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
    else
#endif
        Base::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
}

SIMD_API void SimdBgraToGray(const uint8_t *bgra, size_t width, size_t height, size_t bgraStride, uint8_t *gray, size_t grayStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::HA)
        Neon::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
        Base::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
}

SIMD_API void SimdRgbaToGray(const uint8_t *rgba, size_t width, size_t height, size_t rgbaStride, uint8_t *gray, size_t grayStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::HA)
        Neon::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
        Base::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
}

SIMD_API void SimdBgrToBgra(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *bgra, size_t bgraStride, uint8_t alpha)
{
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
        Base::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdBgrToRgba(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *rgba, size_t rgbaStride, uint8_t alpha)
{
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToRgba(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToRgba(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToRgba(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
    else
#endif
        Base::BgrToRgba(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
}

SIMD_API void SimdBgr48pToBgra32(const uint8_t * blue, size_t blueStride, size_t width, size_t height,
    const uint8_t * green, size_t greenStride, const uint8_t * red, size_t redStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::HA)
        Avx2::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::HA)
        Sse2::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
    else
#endif
        Base::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdBgrToGray(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *gray, size_t grayStride)
{
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
        Base::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
}

SIMD_API void SimdRgbToGray(const uint8_t *rgb, size_t width, size_t height, size_t rgbStride, uint8_t *gray, size_t grayStride)
{
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
      Neon::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
    Base::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
}

SIMD_API void SimdBgrToRgb(const uint8_t *bgr, size_t bgrStride, size_t width, size_t height, uint8_t * rgb, size_t rgbStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToRgb(bgr, bgrStride, width, height, rgb, rgbStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToRgb(bgr, bgrStride, width, height, rgb, rgbStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToRgb(bgr, bgrStride, width, height, rgb, rgbStride);
    else
#endif
        Base::BgrToRgb(bgr, bgrStride, width, height, rgb, rgbStride);
}

SIMD_API void SimdCopy(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize, uint8_t * dst, size_t dstStride)
{
    Base::Copy(src, srcStride, width, height, pixelSize, dst, dstStride);
}

SIMD_API void SimdCopyFrame(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize,
                           size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t * dst, size_t dstStride)
{
    Base::CopyFrame(src, srcStride, width, height, pixelSize, frameLeft, frameTop, frameRight, frameBottom, dst, dstStride);
}

SIMD_API void SimdDeinterleaveBgr(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height,
    uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
    else
#endif
        Base::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
}

SIMD_API void SimdDeinterleaveBgra(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height,
    uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride, uint8_t * a, size_t aStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
    else
#endif
        Base::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
}

SIMD_API void SimdGaussianBlur3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
                     size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && (width - 1)*channelCount >= Avx2::A)
        Avx2::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && (width - 1)*channelCount >= Ssse3::A)
        Ssse3::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && (width - 1)*channelCount >= Sse2::A)
        Sse2::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 1)*channelCount >= Neon::A)
        Neon::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdGrayToBgr(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
    else
#endif
        Base::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
}

SIMD_API void SimdGrayToBgra(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
        Base::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdInterleaveBgr(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride,
    size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
    else
#endif
        Base::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
}

SIMD_API void SimdInterleaveBgra(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride, const uint8_t * a, size_t aStride,
    size_t width, size_t height, uint8_t * bgra, size_t bgraStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
        Base::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
}

SIMD_API void SimdOperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
               size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width*channelCount >= Avx2::A)
        Avx2::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width*channelCount >= Sse2::A)
        Sse2::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width*channelCount >= Neon::A)
        Neon::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
        Base::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
}

SIMD_API void SimdReduceColor2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && srcWidth >= Ssse3::DA)
        Ssse3::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && srcWidth >= Sse2::DA)
        Sse2::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::DA)
        Neon::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
        Base::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
}

SIMD_API void SimdReduceGray2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                   uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && srcWidth >= Ssse3::DA)
        Ssse3::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && srcWidth >= Sse2::DA)
        Sse2::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::DA)
        Neon::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
        Base::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
}

SIMD_API void SimdReduceGray3x3(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                   uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && srcWidth >= Sse2::A)
        Sse2::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::DA)
        Neon::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
        Base::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
}

SIMD_API void SimdReduceGray4x4(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                   uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth > Avx2::DA)
        Avx2::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && srcWidth > Ssse3::A)
        Ssse3::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && srcWidth > Sse2::A)
        Sse2::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth > Neon::DA)
        Neon::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
        Base::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
}

SIMD_API void SimdReduceGray5x5(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                   uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && srcWidth >= Sse2::A)
        Sse2::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::DA)
        Neon::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
        Base::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
}

SIMD_API void SimdResizeBilinear(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && dstWidth >= Avx2::A)
        Avx2::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && dstWidth >= Ssse3::A)
        Ssse3::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && dstWidth >= Sse2::A)
        Sse2::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && dstWidth >= Neon::A)
        Neon::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
        Base::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
}

SIMD_API void * SimdResizerInit(size_t srcX, size_t srcY, size_t dstX, size_t dstY, size_t channels, SimdResizeChannelType type, SimdResizeMethodType method)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable)
        return Avx2::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_AVX_ENABLE
    if (Avx::Enable)
        return Avx::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable)
        return Sse41::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable)
        return Ssse3::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable)
        return Sse2::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_SSE_ENABLE
    if (Sse::Enable)
        return Sse::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable)
        return Neon::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
        return Base::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
}

SIMD_API void SimdResizerRun(const void * resizer, const uint8_t * src, size_t srcStride, uint8_t * dst, size_t dstStride)
{
    ((Resizer*)resizer)->Run(src, srcStride, dst, dstStride);
}

SIMD_API void SimdStretchGray2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::A)
        Avx2::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && srcWidth >= Sse2::A)
        Sse2::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::A)
        Neon::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
        Base::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
}

// ViSP custom SIMD code
SIMD_API void SimdImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::ImageErosion(img, buff, width, height, connexityType);
    else
#endif
        Base::ImageErosion(img, buff, width, height, connexityType);
}

SIMD_API void SimdImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::ImageDilatation(img, buff, width, height, connexityType);
    else
#endif
        Base::ImageDilatation(img, buff, width, height, connexityType);
}
