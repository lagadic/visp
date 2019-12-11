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
#include "Simd/SimdPerformance.h"

#include "Simd/SimdResizer.h"

#include "Simd/SimdBase.h"
#include "Simd/SimdSse1.h"
#include "Simd/SimdSse2.h"
#include "Simd/SimdSse3.h"
#include "Simd/SimdSsse3.h"
#include "Simd/SimdSse41.h"
#include "Simd/SimdSse42.h"
#include "Simd/SimdAvx1.h"
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
#ifdef SIMD_AVX512F_ENABLE
    case SimdCpuInfoAvx512f: return Avx512f::Enable ? 1 : 0;
#endif
#ifdef SIMD_AVX512BW_ENABLE
    case SimdCpuInfoAvx512bw: return Avx512bw::Enable ? 1 : 0;
#endif
#ifdef SIMD_VMX_ENABLE
    case SimdCpuInfoVmx: return Vmx::Enable ? 1 : 0;
#endif
#ifdef SIMD_VSX_ENABLE
    case SimdCpuInfoVsx: return Vsx::Enable ? 1 : 0;
#endif
#ifdef SIMD_NEON_ENABLE
    case SimdCpuInfoNeon: return Neon::Enable ? 1 : 0;
#endif
#ifdef SIMD_MSA_ENABLE
    case SimdCpuInfoMsa: return Msa::Enable ? 1 : 0;
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

SIMD_API size_t SimdGetThreadNumber()
{
    return Base::GetThreadNumber();
}

SIMD_API void SimdSetThreadNumber(size_t threadNumber)
{
    Base::SetThreadNumber(threadNumber);
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

SIMD_API void SimdAlphaBlending(const uint8_t *src, size_t srcStride, size_t width, size_t height, size_t channelCount,
                   const uint8_t *alpha, size_t alphaStride, uint8_t *dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
    else
#endif
        Base::AlphaBlending(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
}

SIMD_API void SimdAlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8_t * channel, size_t channelCount, const uint8_t * alpha, size_t alphaStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A)
        Ssse3::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
    else
#endif
        Base::AlphaFilling(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
}

SIMD_API void SimdBayerToBgr(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width >= Avx512bw::A + 2)
        Avx512bw::BayerToBgr(bayer, width, height, bayerStride, bayerFormat, bgr, bgrStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A + 2)
        Avx2::BayerToBgr(bayer, width, height, bayerStride, bayerFormat, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::A + 2)
        Ssse3::BayerToBgr(bayer, width, height, bayerStride, bayerFormat, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A + 2)
        Neon::BayerToBgr(bayer, width, height, bayerStride, bayerFormat, bgr, bgrStride);
    else
#endif
        Base::BayerToBgr(bayer, width, height, bayerStride, bayerFormat, bgr, bgrStride);
}

SIMD_API void SimdBayerToBgra(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width >= Avx512bw::A + 2)
        Avx512bw::BayerToBgra(bayer, width, height, bayerStride, bayerFormat, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A + 2)
        Avx2::BayerToBgra(bayer, width, height, bayerStride, bayerFormat, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A + 2)
        Sse2::BayerToBgra(bayer, width, height, bayerStride, bayerFormat, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A + 2)
        Neon::BayerToBgra(bayer, width, height, bayerStride, bayerFormat, bgra, bgraStride, alpha);
    else
#endif
        Base::BayerToBgra(bayer, width, height, bayerStride, bayerFormat, bgra, bgraStride, alpha);
}

SIMD_API void SimdBgraToBayer(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToBayer(bgra, width, height, bgraStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgraToBayer(bgra, width, height, bgraStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgraToBayer(bgra, width, height, bgraStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgraToBayer(bgra, width, height, bgraStride, bayer, bayerStride, bayerFormat);
    else
#endif
        Base::BgraToBayer(bgra, width, height, bgraStride, bayer, bayerStride, bayerFormat);
}

SIMD_API void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgraToBgr(bgra, width, height, bgraStride, bgr, bgrStride);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgraToGray(bgra, width, height, bgraStride, gray, grayStride);
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

SIMD_API void SimdBgraToYuv420p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::DA)
        Sse2::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgraToYuv420p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBgraToYuv422p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::DA)
        Sse2::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgraToYuv422p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBgraToYuv444p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgraToYuv444p(bgra, width, height, bgraStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBgraToYuva420p(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height,
    uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride, uint8_t * a, size_t aStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::DA)
        Avx2::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::DA)
        Sse2::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
    else
#endif
        Base::BgraToYuva420p(bgra, bgraStride, width, height, y, yStride, u, uStride, v, vStride, a, aStride);
}

SIMD_API void SimdBgrToBayer(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToBayer(bgr, width, height, bgrStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToBayer(bgr, width, height, bgrStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgrToBayer(bgr, width, height, bgrStride, bayer, bayerStride, bayerFormat);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToBayer(bgr, width, height, bgrStride, bayer, bayerStride, bayerFormat);
    else
#endif
        Base::BgrToBayer(bgr, width, height, bgrStride, bayer, bayerStride, bayerFormat);
}

SIMD_API void SimdBgrToBgra(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToBgra(bgr, width, height, bgrStride, bgra, bgraStride, alpha);
    else
#endif
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::HA)
        Vmx::Bgr48pToBgra32(blue, blueStride, width, height, green, greenStride, red, redStride, bgra, bgraStride, alpha);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgrToGray(bgr, width, height, bgrStride, gray, grayStride);
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

SIMD_API void SimdBgrToHsl(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsl, size_t hslStride)
{
    Base::BgrToHsl(bgr, width, height, bgrStride, hsl, hslStride);
}

SIMD_API void SimdBgrToHsv(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsv, size_t hsvStride)
{
    Base::BgrToHsv(bgr, width, height, bgrStride, hsv, hsvStride);
}

SIMD_API void SimdBgrToRgb(const uint8_t *bgr, size_t bgrStride, size_t width, size_t height, uint8_t * rgb, size_t rgbStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToRgb(bgr, bgrStride, width, height, rgb, rgbStride);
    else
#endif
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

SIMD_API void SimdBgrToYuv420p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgrToYuv420p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBgrToYuv422p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgrToYuv422p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBgrToYuv444p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
    else
#endif
        Base::BgrToYuv444p(bgr, width, height, bgrStride, y, yStride, u, uStride, v, vStride);
}

SIMD_API void SimdBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height,
                  uint8_t value, uint8_t positive, uint8_t negative, uint8_t * dst, size_t dstStride, SimdCompareType compareType)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
    else
#endif
        Base::Binarization(src, srcStride, width, height, value, positive, negative, dst, dstStride, compareType);
}

SIMD_API void SimdAveragingBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height,
                           uint8_t value, size_t neighborhood, uint8_t threshold, uint8_t positive, uint8_t negative,
                           uint8_t * dst, size_t dstStride, SimdCompareType compareType)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
    else
#endif
        Base::AveragingBinarization(src, srcStride, width, height, value, neighborhood, threshold, positive, negative, dst, dstStride, compareType);
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

SIMD_API void SimdDeinterleaveUv(const uint8_t * uv, size_t uvStride, size_t width, size_t height,
                    uint8_t * u, size_t uStride, uint8_t * v, size_t vStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
    else
#endif
        Base::DeinterleaveUv(uv, uvStride, width, height, u, uStride, v, vStride);
}

SIMD_API void SimdDeinterleaveBgr(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height,
    uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DeinterleaveBgr(bgr, bgrStride, width, height, b, bStride, g, gStride, r, rStride);
    else
#endif
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DeinterleaveBgra(bgra, bgraStride, width, height, b, bStride, g, gStride, r, rStride, a, aStride);
    else
#endif
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

SIMD_API void * SimdDetectionLoadStringXml(char * xml)
{
    return Base::DetectionLoadStringXml(xml);
}

SIMD_API void * SimdDetectionLoadA(const char * path)
{
    return Base::DetectionLoadA(path);
}

SIMD_API void SimdDetectionInfo(const void * data, size_t * width, size_t * height, SimdDetectionInfoFlags * flags)
{
    Base::DetectionInfo(data, width, height, flags);
}

SIMD_API void * SimdDetectionInit(const void * data, uint8_t * sum, size_t sumStride, size_t width, size_t height,
    uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride, int throughColumn, int int16)
{
    return Base::DetectionInit(data, sum, sumStride, width, height, sqsum, sqsumStride, tilted, tiltedStride, throughColumn, int16);
}

SIMD_API void SimdDetectionPrepare(void * hid)
{
    Base::DetectionPrepare(hid);
}

SIMD_API void SimdDetectionHaarDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionHaarDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionHaarDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionHaarDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionHaarDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionHaarDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdDetectionHaarDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionHaarDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionHaarDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionHaarDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionHaarDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionHaarDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdDetectionLbpDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionLbpDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionLbpDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionLbpDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionLbpDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionLbpDetect32fp(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdDetectionLbpDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionLbpDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionLbpDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionLbpDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionLbpDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionLbpDetect32fi(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdDetectionLbpDetect16ip(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionLbpDetect16ip(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionLbpDetect16ip(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionLbpDetect16ip(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionLbpDetect16ip(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionLbpDetect16ip(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdDetectionLbpDetect16ii(const void * hid, const uint8_t * mask, size_t maskStride,
    ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride)
{
    size_t width = right - left;
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::DetectionLbpDetect16ii(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::DetectionLbpDetect16ii(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if (Sse41::Enable && width >= Sse41::A)
        Sse41::DetectionLbpDetect16ii(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::DetectionLbpDetect16ii(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
    else
#endif
        Base::DetectionLbpDetect16ii(hid, mask, maskStride, left, top, right, bottom, dst, dstStride);
}

SIMD_API void SimdFill(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize, uint8_t value)
{
    Base::Fill(dst, stride, width, height, pixelSize, value);
}

SIMD_API void SimdFillFrame(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize,
                           size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t value)
{
    Base::FillFrame(dst, stride, width, height, pixelSize, frameLeft, frameTop, frameRight, frameBottom, value);
}

SIMD_API void SimdFillBgr(uint8_t * dst, size_t stride, size_t width, size_t height, uint8_t blue, uint8_t green, uint8_t red)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::FillBgr(dst, stride, width, height, blue, green, red);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::FillBgr(dst, stride, width, height, blue, green, red);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::FillBgr(dst, stride, width, height, blue, green, red);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::FillBgr(dst, stride, width, height, blue, green, red);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::FillBgr(dst, stride, width, height, blue, green, red);
    else
#endif
        Base::FillBgr(dst, stride, width, height, blue, green, red);
}

SIMD_API void SimdFillBgra(uint8_t * dst, size_t stride, size_t width, size_t height, uint8_t blue, uint8_t green, uint8_t red, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::FillBgra(dst, stride, width, height, blue, green, red, alpha);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::F)
        Avx2::FillBgra(dst, stride, width, height, blue, green, red, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::F)
        Sse2::FillBgra(dst, stride, width, height, blue, green, red, alpha);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::F)
        Vmx::FillBgra(dst, stride, width, height, blue, green, red, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::F)
        Neon::FillBgra(dst, stride, width, height, blue, green, red, alpha);
    else
#endif
        Base::FillBgra(dst, stride, width, height, blue, green, red, alpha);
}

SIMD_API void SimdFillPixel(uint8_t * dst, size_t stride, size_t width, size_t height, const uint8_t * pixel, size_t pixelSize)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::FillPixel(dst, stride, width, height, pixel, pixelSize);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::FillPixel(dst, stride, width, height, pixel, pixelSize);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::FillPixel(dst, stride, width, height, pixel, pixelSize);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::FillPixel(dst, stride, width, height, pixel, pixelSize);
    else
#endif
        Base::FillPixel(dst, stride, width, height, pixel, pixelSize);
}

typedef void(*SimdFill32fPtr) (float * dst, size_t size, const float * value);
SimdFill32fPtr simdFill32f = SIMD_FUNC4(Fill32f, SIMD_AVX512F_FUNC, SIMD_AVX_FUNC, SIMD_SSE_FUNC, SIMD_NEON_FUNC);

SIMD_API void SimdFill32f(float * dst, size_t size, const float * value)
{
    simdFill32f(dst, size, value);
}

SIMD_API void SimdGaussianBlur3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
                     size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 1)*channelCount >= Avx512bw::A)
        Avx512bw::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && (width - 1)*channelCount >= Vmx::A)
        Vmx::GaussianBlur3x3(src, srcStride, width, height, channelCount, dst, dstStride);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GrayToBgr(gray, width, height, grayStride, bgr, bgrStride);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
    else
#endif
        Base::GrayToBgra(gray, width, height, grayStride, bgra, bgraStride, alpha);
}

SIMD_API void SimdAbsSecondDerivativeHistogram(const uint8_t *src, size_t width, size_t height, size_t stride, size_t step, size_t indent, uint32_t * histogram)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width >= Avx512bw::A + 2 * indent)
        Avx512bw::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A + 2*indent)
        Avx2::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A + 2*indent)
        Sse2::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A + 2*indent)
        Vmx::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A + 2 * indent)
        Neon::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
    else
#endif
        Base::AbsSecondDerivativeHistogram(src, width, height, stride, step, indent, histogram);
}

SIMD_API void SimdHistogram(const uint8_t *src, size_t width, size_t height, size_t stride, uint32_t * histogram)
{
    Base::Histogram(src, width, height, stride, histogram);
}

SIMD_API void SimdHistogramMasked(const uint8_t *src, size_t srcStride, size_t width, size_t height,
                                  const uint8_t * mask, size_t maskStride, uint8_t index, uint32_t * histogram)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
    else
#endif
        Base::HistogramMasked(src, srcStride, width, height, mask, maskStride, index, histogram);
}

SIMD_API void SimdHistogramConditional(const uint8_t * src, size_t srcStride, size_t width, size_t height,
    const uint8_t * mask, size_t maskStride, uint8_t value, SimdCompareType compareType, uint32_t * histogram)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::HistogramConditional(src, srcStride, width, height, mask, maskStride, value, compareType, histogram);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::HistogramConditional(src, srcStride, width, height, mask, maskStride, value, compareType, histogram);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::HistogramConditional(src, srcStride, width, height, mask, maskStride, value, compareType, histogram);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::HistogramConditional(src, srcStride, width, height, mask, maskStride, value, compareType, histogram);
    else
#endif
        Base::HistogramConditional(src, srcStride, width, height, mask, maskStride, value, compareType, histogram);
}

SIMD_API void SimdNormalizedColors(const uint32_t * histogram, uint8_t * colors)
{
    Base::NormalizedColors(histogram, colors);
}

SIMD_API void SimdChangeColors(const uint8_t * src, size_t srcStride, size_t width, size_t height, const uint8_t * colors, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width >= Avx512bw::HA)
        Avx512bw::ChangeColors(src, srcStride, width, height, colors, dst, dstStride);
    else
#endif
        Base::ChangeColors(src, srcStride, width, height, colors, dst, dstStride);
}

SIMD_API void SimdNormalizeHistogram(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width >= Avx512bw::HA)
        Avx512bw::NormalizeHistogram(src, srcStride, width, height, dst, dstStride);
    else
#endif
        Base::NormalizeHistogram(src, srcStride, width, height, dst, dstStride);
}

SIMD_API void SimdIntegral(const uint8_t * src, size_t srcStride, size_t width, size_t height,
                      uint8_t * sum, size_t sumStride, uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride,
                      SimdPixelFormatType sumFormat, SimdPixelFormatType sqsumFormat)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Integral(src, srcStride, width, height, sum, sumStride, sqsum, sqsumStride, tilted, tiltedStride, sumFormat, sqsumFormat);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable)
        Avx2::Integral(src, srcStride, width, height, sum, sumStride, sqsum, sqsumStride, tilted, tiltedStride, sumFormat, sqsumFormat);
    else
#endif
        Base::Integral(src, srcStride, width, height, sum, sumStride, sqsum, sqsumStride, tilted, tiltedStride, sumFormat, sqsumFormat);
}

SIMD_API void SimdInterleaveUv(const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * uv, size_t uvStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::A)
        Avx2::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::A)
        Sse2::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if (Vmx::Enable && width >= Vmx::A)
        Vmx::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
    else
#endif
        Base::InterleaveUv(u, uStride, v, vStride, width, height, uv, uvStride);
}

SIMD_API void SimdInterleaveBgr(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride,
    size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::InterleaveBgr(b, bStride, g, gStride, r, rStride, width, height, bgr, bgrStride);
    else
#endif
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::InterleaveBgra(b, bStride, g, gStride, r, rStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
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

SIMD_API void SimdMeanFilter3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 1)*channelCount >= Avx512bw::A)
        Avx512bw::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && (width - 1)*channelCount >= Avx2::A)
        Avx2::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if (Ssse3::Enable && (width - 1)*channelCount >= Ssse3::A)
        Ssse3::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && (width - 1)*channelCount >= Sse2::A)
        Sse2::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if (Vmx::Enable && (width - 1)*channelCount >= Vmx::A)
        Vmx::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 1)*channelCount >= Neon::A)
        Neon::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::MeanFilter3x3(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdMedianFilterRhomb3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 1)*channelCount >= Avx512bw::A)
        Avx512bw::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && (width - 1)*channelCount >= Avx2::A)
        Avx2::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && (width - 1)*channelCount >= Sse2::A)
        Sse2::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && (width - 1)*channelCount >= Vmx::A)
        Vmx::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 1)*channelCount >= Neon::A)
        Neon::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::MedianFilterRhomb3x3(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdMedianFilterRhomb5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 2)*channelCount >= Avx512bw::A)
        Avx512bw::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && (width - 2)*channelCount >= Avx2::A)
        Avx2::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && (width - 2)*channelCount >= Sse2::A)
        Sse2::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && (width - 2)*channelCount >= Vmx::A)
        Vmx::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 2)*channelCount >= Neon::A)
        Neon::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::MedianFilterRhomb5x5(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdMedianFilterSquare3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 1)*channelCount >= Avx512bw::A)
        Avx512bw::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && (width - 1)*channelCount >= Avx2::A)
        Avx2::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && (width - 1)*channelCount >= Sse2::A)
        Sse2::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && (width - 1)*channelCount >= Vmx::A)
        Vmx::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 1)*channelCount >= Neon::A)
        Neon::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::MedianFilterSquare3x3(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdMedianFilterSquare5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && (width - 2)*channelCount >= Avx512bw::A)
        Avx512bw::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && (width - 2)*channelCount >= Avx2::A)
        Avx2::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && (width - 2)*channelCount >= Sse2::A)
        Sse2::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && (width - 2)*channelCount >= Vmx::A)
        Vmx::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && (width - 2)*channelCount >= Neon::A)
        Neon::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
    else
#endif
        Base::MedianFilterSquare5x5(src, srcStride, width, height, channelCount, dst, dstStride);
}

SIMD_API void SimdOperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
               size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width*channelCount >= Vmx::A)
        Vmx::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width*channelCount >= Neon::A)
        Neon::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
#ifdef SIMD_MSA_ENABLE
    if (Msa::Enable && width*channelCount >= Msa::A)
        Msa::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
    else
#endif
        Base::OperationBinary8u(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
}

SIMD_API void SimdReduceColor2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::ReduceColor2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && srcWidth >= Vmx::DA)
        Vmx::ReduceGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && srcWidth >= Avx512bw::DA)
        Avx512bw::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && srcWidth >= Vmx::DA)
        Vmx::ReduceGray3x3(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && srcWidth > Avx512bw::DA)
        Avx512bw::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && srcWidth > Vmx::DA)
        Vmx::ReduceGray4x4(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && srcWidth >= Avx512bw::DA)
        Avx512bw::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && srcWidth >= Vmx::DA)
        Vmx::ReduceGray5x5(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, compensation);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && dstWidth >= Avx512bw::A)
        Avx512bw::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && dstWidth >= Vmx::A)
        Vmx::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
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
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        return Avx512bw::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
#ifdef SIMD_AVX512F_ENABLE
    if (Avx512f::Enable)
        return Avx512f::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
    else
#endif
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

SIMD_API void SimdSegmentationChangeIndex(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t oldIndex, uint8_t newIndex)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
    else
#endif
        Base::SegmentationChangeIndex(mask, stride, width, height, oldIndex, newIndex);
}

SIMD_API void SimdSegmentationFillSingleHoles(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SegmentationFillSingleHoles(mask, stride, width, height, index);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A + 2)
        Avx2::SegmentationFillSingleHoles(mask, stride, width, height, index);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width > Sse2::A + 2)
        Sse2::SegmentationFillSingleHoles(mask, stride, width, height, index);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A + 2)
        Vmx::SegmentationFillSingleHoles(mask, stride, width, height, index);
    else
#endif
#ifdef SIMD_NEON_ENABLE
        if (Neon::Enable && width > Neon::A + 2)
            Neon::SegmentationFillSingleHoles(mask, stride, width, height, index);
        else
#endif
        Base::SegmentationFillSingleHoles(mask, stride, width, height, index);
}

SIMD_API void SimdSegmentationPropagate2x2(const uint8_t * parent, size_t parentStride, size_t width, size_t height,
                                           uint8_t * child, size_t childStride, const uint8_t * difference, size_t differenceStride,
                                           uint8_t currentIndex, uint8_t invalidIndex, uint8_t emptyIndex, uint8_t differenceThreshold)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
        difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A + 1)
        Avx2::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
        difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A + 1)
        Sse2::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
        difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A + 1)
        Vmx::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
        difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A + 1)
        Neon::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
            difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
    else
#endif
        Base::SegmentationPropagate2x2(parent, parentStride, width, height, child, childStride,
        difference, differenceStride, currentIndex, invalidIndex, emptyIndex, differenceThreshold);
}

SIMD_API void SimdSegmentationShrinkRegion(const uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index,
                                           ptrdiff_t * left, ptrdiff_t * top, ptrdiff_t * right, ptrdiff_t * bottom)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A && *right - *left >= (ptrdiff_t)Avx2::A)
        Avx2::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
    else
#endif
#ifdef SIMD_SSE41_ENABLE
    if(Sse41::Enable && width >= Sse41::A && *right - *left >= (ptrdiff_t)Sse41::A)
        Sse41::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A && *right - *left >= (ptrdiff_t)Vmx::A)
        Vmx::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A && *right - *left >= (ptrdiff_t)Neon::A)
        Neon::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
    else
#endif
        Base::SegmentationShrinkRegion(mask, stride, width, height, index, left, top, right, bottom);
}

SIMD_API void SimdSobelDx(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width > Sse2::A)
        Sse2::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDx(src, srcStride, width, height, dst, dstStride);
    else
#endif
        Base::SobelDx(src, srcStride, width, height, dst, dstStride);
}

SIMD_API void SimdSobelDxAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
        Base::SobelDxAbs(src, srcStride, width, height, dst, dstStride);
}

SIMD_API void SimdSobelDxAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDxAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDxAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDxAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDxAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDxAbsSum(src, stride, width, height, sum);
    else
#endif
        Base::SobelDxAbsSum(src, stride, width, height, sum);
}

SIMD_API void SimdSobelDy(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width > Sse2::A)
        Sse2::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDy(src, srcStride, width, height, dst, dstStride);
    else
#endif
        Base::SobelDy(src, srcStride, width, height, dst, dstStride);
}

SIMD_API void SimdSobelDyAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
    else
#endif
        Base::SobelDyAbs(src, srcStride, width, height, dst, dstStride);
}

SIMD_API void SimdSobelDyAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable && width > Avx512bw::A)
        Avx512bw::SobelDyAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width > Avx2::A)
        Avx2::SobelDyAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width > Ssse3::A)
        Ssse3::SobelDyAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width > Vmx::A)
        Vmx::SobelDyAbsSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width > Neon::A)
        Neon::SobelDyAbsSum(src, stride, width, height, sum);
    else
#endif
        Base::SobelDyAbsSum(src, stride, width, height, sum);
}

SIMD_API void SimdSquaredDifferenceSum(const uint8_t *a, size_t aStride, const uint8_t *b, size_t bStride,
                          size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
        Base::SquaredDifferenceSum(a, aStride, b, bStride, width, height, sum);
}

SIMD_API void SimdSquaredDifferenceSumMasked(const uint8_t *a, size_t aStride, const uint8_t *b, size_t bStride,
                          const uint8_t *mask, size_t maskStride, uint8_t index, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
    else
#endif
        Base::SquaredDifferenceSumMasked(a, aStride, b, bStride, mask, maskStride, index, width, height, sum);
}

typedef void (* SimdSquaredDifferenceSum32fPtr) (const float * a, const float * b, size_t size, float * sum);
SimdSquaredDifferenceSum32fPtr simdSquaredDifferenceSum32f = SIMD_FUNC5(SquaredDifferenceSum32f, SIMD_AVX512F_FUNC, SIMD_AVX_FUNC, SIMD_SSE_FUNC, SIMD_VSX_FUNC, SIMD_NEON_FUNC);

SIMD_API void SimdSquaredDifferenceSum32f(const float * a, const float * b, size_t size, float * sum)
{
    simdSquaredDifferenceSum32f(a, b, size, sum);
}

typedef void (* SimdSquaredDifferenceKahanSum32fPtr) (const float * a, const float * b, size_t size, float * sum);
SimdSquaredDifferenceKahanSum32fPtr simdSquaredDifferenceKahanSum32f = SIMD_FUNC5(SquaredDifferenceKahanSum32f, SIMD_AVX512F_FUNC, SIMD_AVX_FUNC, SIMD_SSE_FUNC, SIMD_VSX_FUNC, SIMD_NEON_FUNC);

SIMD_API void SimdSquaredDifferenceKahanSum32f(const float * a, const float * b, size_t size, float * sum)
{
    simdSquaredDifferenceKahanSum32f(a, b, size, sum);
}

SIMD_API void SimdGetStatistic(const uint8_t * src, size_t stride, size_t width, size_t height,
                  uint8_t * min, uint8_t * max, uint8_t * average)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GetStatistic(src, stride, width, height, min, max, average);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GetStatistic(src, stride, width, height, min, max, average);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GetStatistic(src, stride, width, height, min, max, average);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GetStatistic(src, stride, width, height, min, max, average);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GetStatistic(src, stride, width, height, min, max, average);
    else
#endif
        Base::GetStatistic(src, stride, width, height, min, max, average);
}

SIMD_API void SimdGetRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GetRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GetRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GetRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GetRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GetRowSums(src, stride, width, height, sums);
    else
#endif
        Base::GetRowSums(src, stride, width, height, sums);
}

SIMD_API void SimdGetColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GetColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GetColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GetColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GetColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GetColSums(src, stride, width, height, sums);
    else
#endif
        Base::GetColSums(src, stride, width, height, sums);
}

SIMD_API void SimdGetAbsDyRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GetAbsDyRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GetAbsDyRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GetAbsDyRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GetAbsDyRowSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GetAbsDyRowSums(src, stride, width, height, sums);
    else
#endif
        Base::GetAbsDyRowSums(src, stride, width, height, sums);
}

SIMD_API void SimdGetAbsDxColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::GetAbsDxColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::GetAbsDxColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::GetAbsDxColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::GetAbsDxColSums(src, stride, width, height, sums);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::GetAbsDxColSums(src, stride, width, height, sums);
    else
#endif
        Base::GetAbsDxColSums(src, stride, width, height, sums);
}

SIMD_API void SimdValueSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::ValueSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::ValueSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::ValueSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::ValueSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::ValueSum(src, stride, width, height, sum);
    else
#endif
        Base::ValueSum(src, stride, width, height, sum);
}

SIMD_API void SimdSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::SquareSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::SquareSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::SquareSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::SquareSum(src, stride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::SquareSum(src, stride, width, height, sum);
    else
#endif
        Base::SquareSum(src, stride, width, height, sum);
}

SIMD_API void SimdValueSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * valueSum, uint64_t * squareSum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::ValueSquareSum(src, stride, width, height, valueSum, squareSum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::ValueSquareSum(src, stride, width, height, valueSum, squareSum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::ValueSquareSum(src, stride, width, height, valueSum, squareSum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::ValueSquareSum(src, stride, width, height, valueSum, squareSum);
    else
#endif
        Base::ValueSquareSum(src, stride, width, height, valueSum, squareSum);
}

SIMD_API void SimdCorrelationSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::CorrelationSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::CorrelationSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::CorrelationSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::CorrelationSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::CorrelationSum(a, aStride, b, bStride, width, height, sum);
    else
#endif
        Base::CorrelationSum(a, aStride, b, bStride, width, height, sum);
}

SIMD_API void SimdStretchGray2x2(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
                    uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
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
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && srcWidth >= Vmx::A)
        Vmx::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && srcWidth >= Neon::A)
        Neon::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
    else
#endif
        Base::StretchGray2x2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
}

SIMD_API void SimdYuva420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
    const uint8_t * a, size_t aStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuva420pToBgra(y, yStride, u, uStride, v, vStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if (Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuva420pToBgra(y, yStride, u, uStride, v, vStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if (Sse2::Enable && width >= Sse2::DA)
        Sse2::Yuva420pToBgra(y, yStride, u, uStride, v, vStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuva420pToBgra(y, yStride, u, uStride, v, vStride, a, aStride, width, height, bgra, bgraStride);
    else
#endif
        Base::Yuva420pToBgra(y, yStride, u, uStride, v, vStride, a, aStride, width, height, bgra, bgraStride);
}

SIMD_API void SimdYuv420pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                 size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
        Base::Yuv420pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
}

SIMD_API void SimdYuv422pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                 size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::DA)
        Ssse3::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
        Base::Yuv422pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
}

SIMD_API void SimdYuv444pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                               size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_SSSE3_ENABLE
    if(Ssse3::Enable && width >= Ssse3::A)
        Ssse3::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
    else
#endif
        Base::Yuv444pToBgr(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
}

SIMD_API void SimdYuv420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                  size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::DA)
        Sse2::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
        Base::Yuv420pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
}

SIMD_API void SimdYuv422pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                                size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::DA)
        Sse2::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::DA)
        Vmx::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
        Base::Yuv422pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
}

SIMD_API void SimdYuv444pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                  size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_VMX_ENABLE
    if(Vmx::Enable && width >= Vmx::A)
        Vmx::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
    else
#endif
        Base::Yuv444pToBgra(y, yStride, u, uStride, v, vStride, width, height, bgra, bgraStride, alpha);
}

SIMD_API void SimdYuv444pToHsl(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                               size_t width, size_t height, uint8_t * hsl, size_t hslStride)
{
    Base::Yuv444pToHsl(y, yStride, u, uStride, v, vStride, width, height, hsl, hslStride);
}

SIMD_API void SimdYuv444pToHsv(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                               size_t width, size_t height, uint8_t * hsv, size_t hsvStride)
{
    Base::Yuv444pToHsv(y, yStride, u, uStride, v, vStride, width, height, hsv, hsvStride);
}

SIMD_API void SimdYuv420pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                 size_t width, size_t height, uint8_t * hue, size_t hueStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::DA)
        Avx2::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::DA)
        Sse2::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_VSX_ENABLE
    if(Vsx::Enable && width >= Vsx::DA)
        Vsx::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::DA)
        Neon::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
        Base::Yuv420pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
}

SIMD_API void SimdYuv444pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
                 size_t width, size_t height, uint8_t * hue, size_t hueStride)
{
#ifdef SIMD_AVX512BW_ENABLE
    if (Avx512bw::Enable)
        Avx512bw::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_AVX2_ENABLE
    if(Avx2::Enable && width >= Avx2::A)
        Avx2::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_SSE2_ENABLE
    if(Sse2::Enable && width >= Sse2::A)
        Sse2::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_VSX_ENABLE
    if(Vsx::Enable && width >= Vsx::A)
        Vsx::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
#ifdef SIMD_NEON_ENABLE
    if (Neon::Enable && width >= Neon::A)
        Neon::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
    else
#endif
        Base::Yuv444pToHue(y, yStride, u, uStride, v, vStride, width, height, hue, hueStride);
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
