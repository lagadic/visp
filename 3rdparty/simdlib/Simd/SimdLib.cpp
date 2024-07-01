/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2022 Yermalayeu Ihar,
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

#if defined(_WIN32) && !defined(SIMD_STATIC)

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
#endif//_WIN32

#include "Simd/SimdLib.h"

#include "Simd/SimdMemory.h"
#include "Simd/SimdCpu.h"
#include "Simd/SimdEnable.h"
#include "Simd/SimdAlignment.h"
#include "Simd/SimdConst.h"
#include "Simd/SimdLog.h"
#include "Simd/SimdEmpty.h"

#include "Simd/SimdGaussianBlur.h"
#include "Simd/SimdImageLoad.h"
#include "Simd/SimdImageSave.h"
#include "Simd/SimdResizer.h"

#include "Simd/SimdBase.h"
#include "Simd/SimdSse41.h"
#include "Simd/SimdAvx1.h"
#include "Simd/SimdAvx2.h"
#include "Simd/SimdNeon.h"

#if !defined(SIMD_VERSION)
#include "Simd/SimdVersion.h"
#endif

namespace Simd
{
    const size_t ALIGNMENT = GetAlignment();
}

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
#ifdef SIMD_CPP_2011_ENABLE
    case SimdCpuInfoThreads: return Cpu::THREAD_NUMBER;
#endif
    case SimdCpuInfoCacheL1: return Cpu::L1_CACHE_SIZE;
    case SimdCpuInfoCacheL2: return Cpu::L2_CACHE_SIZE;
    case SimdCpuInfoCacheL3: return Cpu::L3_CACHE_SIZE;
#ifdef SIMD_SSE41_ENABLE
    case SimdCpuInfoSse41: return Sse41::Enable ? 1 : 0;
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
#if defined(SIMD_PERFORMANCE_STATISTIC) && (defined(NDEBUG) || defined(SIMD_PERF_STAT_IN_DEBUG))
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
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable)
        return Sse41::GetFastMode();
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
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable)
        Sse41::SetFastMode(value);
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable)
        Neon::SetFastMode(value);
#endif
}

SIMD_API void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::F)
        Avx2::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
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
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::HA)
        Neon::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
        Base::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
}

SIMD_API void SimdBgraToRgb(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgb, size_t rgbStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::F)
        Avx2::BgraToRgb(bgra, width, height, bgraStride, rgb, rgbStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::BgraToRgb(bgra, width, height, bgraStride, rgb, rgbStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgraToRgb(bgra, width, height, bgraStride, rgb, rgbStride);
    else
#endif
        Base::BgraToRgb(bgra, width, height, bgraStride, rgb, rgbStride);
}

SIMD_API void SimdBgraToRgba(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgba, size_t rgbaStride)
{
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::BgraToRgba(bgra, width, height, bgraStride, rgba, rgbaStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::BgraToRgba(bgra, width, height, bgraStride, rgba, rgbaStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgraToRgba(bgra, width, height, bgraStride, rgba, rgbaStride);
    else
#endif
        Base::BgraToRgba(bgra, width, height, bgraStride, rgba, rgbaStride);
}

SIMD_API void SimdBgrToBgra(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *bgra, size_t bgraStride, uint8_t alpha)
{
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
        Base::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdBgr48pToBgra32(const uint8_t * blue, size_t blueStride, size_t width, size_t height,
    const uint8_t * green, size_t greenStride, const uint8_t * red, size_t redStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::HA)
        Avx2::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::HA)
        Sse41::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
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
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
        Base::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
}

SIMD_API void SimdBgrToRgb(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgb, size_t rgbStride)
{
    SIMD_EMPTY();
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToRgb(bgr, width, height, bgrStride, rgb, rgbStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToRgb(bgr, width, height, bgrStride, rgb, rgbStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::BgrToRgb(bgr, width, height, bgrStride, rgb, rgbStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToRgb(bgr, width, height, bgrStride, rgb, rgbStride);
    else
#endif
        Base::BgrToRgb(bgr, width, height, bgrStride, rgb, rgbStride);
}

SIMD_API void SimdCopy(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize, uint8_t * dst, size_t dstStride)
{
    SIMD_EMPTY();
    Base::Copy(src, srcStride, width, height, pixelSize, dst, dstStride);
}

SIMD_API void SimdCopyFrame(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize,
                           size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t * dst, size_t dstStride)
{
    SIMD_EMPTY();
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
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
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
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && (width - 1)*channelCount >= Sse41::A)
        Sse41::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 1)*channelCount >= Neon::A)
        Neon::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void* SimdGaussianBlurInit(size_t width, size_t height, size_t channels, const float* sigma, const float* epsilon)
{
    SIMD_EMPTY();
    typedef void* (*SimdGaussianBlurInitPtr) (size_t width, size_t height, size_t channels, const float* sigma, const float* epsilon);
    const static SimdGaussianBlurInitPtr simdGaussianBlurInit = SIMD_FUNC3(GaussianBlurInit, SIMD_AVX2_FUNC, SIMD_SSE41_FUNC, SIMD_NEON_FUNC);

    return simdGaussianBlurInit(width, height, channels, sigma, epsilon);
}

SIMD_API void SimdGaussianBlurRun(const void* filter, const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride)
{
    SIMD_EMPTY();
    ((GaussianBlur*)filter)->Run(src, srcStride, dst, dstStride);
}

SIMD_API void SimdGrayToBgr(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgr, size_t bgrStride)
{
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A)
        Sse41::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
        Base::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
}

SIMD_API uint8_t* SimdImageSaveToMemory(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, size_t* size)
{
    SIMD_EMPTY();
    const static Simd::ImageSaveToMemoryPtr imageSaveToMemory = SIMD_FUNC3(ImageSaveToMemory, SIMD_AVX2_FUNC, SIMD_SSE41_FUNC, SIMD_NEON_FUNC);

    return imageSaveToMemory(src, stride, width, height, format, file, quality, size);
}

SIMD_API SimdBool SimdImageSaveToFile(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, const char* path)
{
    SIMD_EMPTY();
    const static Simd::ImageSaveToMemoryPtr imageSaveToMemory = SIMD_FUNC3(ImageSaveToMemory, SIMD_AVX2_FUNC, SIMD_SSE41_FUNC, SIMD_NEON_FUNC);

    return ImageSaveToFile(imageSaveToMemory, src, stride, width, height, format, file, quality, path);
}

SIMD_API uint8_t* SimdImageLoadFromMemory(const uint8_t* data, size_t size, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType* format)
{
    SIMD_EMPTY();
    const static Simd::ImageLoadFromMemoryPtr imageLoadFromMemory = SIMD_FUNC3(ImageLoadFromMemory, SIMD_AVX2_FUNC, SIMD_SSE41_FUNC, SIMD_NEON_FUNC);

    return imageLoadFromMemory(data, size, stride, width, height, format);
}

SIMD_API uint8_t* SimdImageLoadFromFile(const char* path, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType* format)
{
    SIMD_EMPTY();
    const static Simd::ImageLoadFromMemoryPtr imageLoadFromMemory = SIMD_FUNC3(ImageLoadFromMemory, SIMD_AVX2_FUNC, SIMD_SSE41_FUNC, SIMD_NEON_FUNC);

    return ImageLoadFromFile(imageLoadFromMemory, path, stride, width, height, format);
}

SIMD_API void SimdInterleaveBgr(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride,
    size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
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
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width*channelCount >= Sse41::A)
        Sse41::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && srcWidth >= Sse41::DA)
        Sse41::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && srcWidth >= Sse41::DA)
        Sse41::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && srcWidth >= Sse41::A)
        Sse41::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth > Avx2::DA)
        Avx2::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && srcWidth > Sse41::A)
        Sse41::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
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
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::DA)
        Avx2::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && srcWidth >= Sse41::A)
        Sse41::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
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
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && dstWidth >= Sse41::A)
        Sse41::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
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
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable)
        return Sse2::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
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
    SIMD_EMPTY();
    ((Resizer*)resizer)->Run(src, srcStride, dst, dstStride);
}

SIMD_API void SimdRgbToBgra(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha)
{
    SIMD_EMPTY();
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::RgbToBgra(rgb, width, height, rgbStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::RgbToBgra(rgb, width, height, rgbStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::RgbToBgra(rgb, width, height, rgbStride, bgra, bgraStride, alpha);
    else
#endif
        Base::RgbToBgra(rgb, width, height, rgbStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdRgbToGray(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* gray, size_t grayStride)
{
    SIMD_EMPTY();
#if defined(SIMD_AVX2_ENABLE) && !defined(SIMD_CLANG_AVX2_BGR_TO_BGRA_ERROR)
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
    else
#endif
        Base::RgbToGray(rgb, width, height, rgbStride, gray, grayStride);
}

SIMD_API void SimdRgbaToGray(const uint8_t* rgba, size_t width, size_t height, size_t rgbaStride, uint8_t* gray, size_t grayStride)
{
    SIMD_EMPTY();
#if defined(SIMD_AVX2_ENABLE)
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
    else
#endif
        Base::RgbaToGray(rgba, width, height, rgbaStride, gray, grayStride);
}

SIMD_API void SimdStretchGray2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
{
    SIMD_EMPTY();
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && srcWidth >= Avx2::A)
        Avx2::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && srcWidth >= Sse41::A)
        Sse41::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::A)
        Neon::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
        Base::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
}

/* ViSP custom SIMD code */
SIMD_API void SimdImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::ImageErosion(img, buff, width, height, connexityType);
    else
#endif
        Base::ImageErosion(img, buff, width, height, connexityType);
}

SIMD_API void SimdImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::ImageDilatation(img, buff, width, height, connexityType);
    else
#endif
        Base::ImageDilatation(img, buff, width, height, connexityType);
}

SIMD_API double SimdVectorSum(const double * vec, size_t size)
{
#ifdef SIMD_SSE41_ENABLE
    const int unrollLoopSize = 2;
    if (Sse41::Enable && size*sizeof(double) >= unrollLoopSize*Sse41::A)
        return Sse41::SimdVectorSum(vec, size);
    else
#endif
        return Base::SimdVectorSum(vec, size);
}

SIMD_API double SimdVectorSumSquare(const double * vec, size_t size)
{
#ifdef SIMD_SSE41_ENABLE
    const int unrollLoopSize = 2;
    if (Sse41::Enable && size*sizeof(double) >= unrollLoopSize*Sse41::A)
        return Sse41::SimdVectorSumSquare(vec, size);
    else
#endif
        return Base::SimdVectorSumSquare(vec, size);
}

SIMD_API double SimdVectorStdev(const double * vec, size_t size, bool useBesselCorrection)
{
#ifdef SIMD_SSE41_ENABLE
    const int unrollLoopSize = 2;
    if (Sse41::Enable && size*sizeof(double) >= unrollLoopSize*Sse41::A)
        return Sse41::SimdVectorStdev(vec, size, useBesselCorrection);
    else
#endif
        return Base::SimdVectorStdev(vec, size, useBesselCorrection);
}

SIMD_API void SimdVectorHadamard(const double * src1, const double * src2, size_t size, double * dst)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && size*sizeof(double) >= Sse41::A)
        Sse41::SimdVectorHadamard(src1, src2, size, dst);
    else
#endif
        Base::SimdVectorHadamard(src1, src2, size, dst);
}

SIMD_API void SimdMatMulTwist(const double * mat, size_t rows, const double * twist, double * dst)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable)
        Sse41::SimdMatMulTwist(mat, rows, twist, dst);
    else
#endif
#if defined(SIMD_NEON_ENABLE) && defined(SIMD_ARM64_ENABLE)
    if (Neon::Enable)
        Neon::SimdMatMulTwist(mat, rows, twist, dst);
    else
#endif
        Base::SimdMatMulTwist(mat, rows, twist, dst);
}

SIMD_API void SimdMatTranspose(const double * mat, size_t rows, size_t cols, double * dst)
{
#ifdef SIMD_AVX_ENABLE
    if (Avx::Enable)
        Avx::SimdMatTranspose(mat, rows, cols, dst);
    else
#endif
        Base::SimdMatTranspose(mat, rows, cols, dst);
}

SIMD_API void SimdImageDifference(const unsigned char * img1, const unsigned char * img2, size_t size, unsigned char * imgDiff)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && size >= Sse41::A)
        Sse41::SimdImageDifference(img1,img2, size, imgDiff);
    else
#endif
        Base::SimdImageDifference(img1, img2, size, imgDiff);
}

SIMD_API void SimdNormalizedCorrelation(const double * img1, double mean1, const double * img2, double mean2, size_t size,
                                        double& a2, double& b2, double& ab, bool useOptimized)
{
    if (useOptimized) {
#ifdef SIMD_SSE41_ENABLE
        if (Sse41::Enable && size*sizeof(double) >= Sse41::A)
            Sse41::SimdNormalizedCorrelation(img1, mean1, img2, mean2, size, a2, b2, ab);
        else
#endif
            Base::SimdNormalizedCorrelation(img1, mean1, img2, mean2, size, a2, b2, ab);
    } else {
        Base::SimdNormalizedCorrelation(img1, mean1, img2, mean2, size, a2, b2, ab);
    }
}

SIMD_API void SimdNormalizedCorrelation2(const double * img1, size_t width1, const double * img2,
                                         size_t width2, size_t height2, size_t i0, size_t j0, double& ab)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width2*sizeof(double) >= Sse41::A)
        Sse41::SimdNormalizedCorrelation2(img1, width1, img2, width2, height2, i0, j0, ab);
    else
#endif
        Base::SimdNormalizedCorrelation2(img1, width1, img2, width2, height2, i0, j0, ab);
}

SIMD_API void SimdRemap(const unsigned char * src, size_t channels, size_t width, size_t height, size_t offset,
                        const int * mapU, const int * mapV, const float * mapDu, const float * mapDv, unsigned char * dst)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && channels >= 4)
        Sse41::SimdRemap(src, channels, width, height, offset, mapU, mapV, mapDu, mapDv, dst);
    else
#endif
        Base::SimdRemap(src, channels, width, height, offset, mapU, mapV, mapDu, mapDv, dst);
}

SIMD_API void SimdComputeJtR(const double * J, size_t rows, const double * R, double * dst)
{
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable)
        Sse41::SimdComputeJtR(J, rows, R, dst);
    else
#endif
#if defined(SIMD_NEON_ENABLE) && defined(SIMD_ARM64_ENABLE)
    if (Neon::Enable)
        Neon::SimdComputeJtR(J, rows, R, dst);
    else
#endif
        Base::SimdComputeJtR(J, rows, R, dst);
}
