/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2019 Yermalayeu Ihar,
*               2014-2016 Antonenka Mikhail,
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

#ifndef __SimdLib_h__
#define __SimdLib_h__

#include "Simd/SimdConfig.h"

#include <stddef.h>

#if defined(_MSC_VER) || defined(__CODEGEARC__)

#define SIMD_INLINE __forceinline

#elif defined(__GNUC__)

#define SIMD_INLINE inline __attribute__ ((always_inline))

#else

#error This platform is unsupported!

#endif

#if defined(__GNUC__) || (defined(_MSC_VER) && (_MSC_VER >= 1600)) || (defined(__CODEGEARC__) && (__CODEGEARC__ >= 1840))
#include <stdint.h>
#else
#  if (_MSC_VER < 1300)
typedef signed char       int8_t;
typedef signed short      int16_t;
typedef signed int        int32_t;
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;
#  else
typedef signed __int8     int8_t;
typedef signed __int16    int16_t;
typedef signed __int32    int32_t;
typedef unsigned __int8   uint8_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int32  uint32_t;
#  endif
typedef signed __int64    int64_t;
typedef unsigned __int64  uint64_t;
#endif

/*! @ingroup c_types
    Describes boolean type.
*/
typedef enum
{
    SimdFalse = 0, /*!< False value. */
    SimdTrue = 1, /*!< True value. */
} SimdBool;

/*! @ingroup c_types
    Describes types of compare operation.
    Operation compare(a, b) is
*/
typedef enum
{
    /*! equal to: a == b */
    SimdCompareEqual,
    /*! equal to: a != b */
    SimdCompareNotEqual,
    /*! equal to: a > b */
    SimdCompareGreater,
    /*! equal to: a >= b */
    SimdCompareGreaterOrEqual,
    /*! equal to: a < b */
    SimdCompareLesser,
    /*! equal to: a <= b */
    SimdCompareLesserOrEqual,
} SimdCompareType;

/*! @ingroup c_types
    Describes type of information which can return function ::SimdCpuInfo.
*/
typedef enum
{
    SimdCpuInfoSockets,/*!< A number of sockets. */
    SimdCpuInfoCores, /*!< A number of psysical CPU cores. */
    SimdCpuInfoThreads, /*!< A number of logical CPU cores. */
    SimdCpuInfoCacheL1, /*!< A size of level 1 data cache. */
    SimdCpuInfoCacheL2, /*!< A size of level 2 cache. */
    SimdCpuInfoCacheL3, /*!< A size of level 3 cache. */
    SimdCpuInfoSse, /*!< Availability of SSE (x86). */
    SimdCpuInfoSse2, /*!< Availability of SSE2 (x86). */
    SimdCpuInfoSse3, /*!< Availability of SSE3 (x86). */
    SimdCpuInfoSsse3, /*!< Availability of SSSE3 (x86). */
    SimdCpuInfoSse41, /*!< Availability of SSE4.1 (x86). */
    SimdCpuInfoSse42, /*!< Availability of SSE4.2 (x86). */
    SimdCpuInfoAvx, /*!< Availability of AVX (x86). */
    SimdCpuInfoAvx2, /*!< Availability of AVX2 (x86). */
    SimdCpuInfoAvx512f, /*!< Availability of AVX-512F (x86). */
    SimdCpuInfoAvx512bw, /*!< Availability of AVX-512BW (x86). */
    SimdCpuInfoVmx, /*!< Availability of VMX or Altivec (PowerPC). */
    SimdCpuInfoVsx, /*!< Availability of VSX (PowerPC). */
    SimdCpuInfoNeon, /*!< Availability of NEON (ARM). */
    SimdCpuInfoMsa, /*!< Availability of MSA (MIPS). */
} SimdCpuInfoType;

/*! @ingroup c_types
    Describes types and flags to get information about classifier cascade with using function ::SimdDetectionInfo.
    \note This type is used for implementation of Simd::Detection.
*/
typedef enum
{
    /*! A HAAR cascade classifier type. */
    SimdDetectionInfoFeatureHaar = 0,
    /*! A LBP cascade classifier type. */
    SimdDetectionInfoFeatureLbp,
    /*! A mask to select cascade classifier type. */
    SimdDetectionInfoFeatureMask = 3,
    /*! A flag which defines existence of tilted features in the HAAR cascade. */
    SimdDetectionInfoHasTilted = 4,
    /*! A flag which defines possibility to use 16-bit integers for calculation. */
    SimdDetectionInfoCanInt16 = 8,
} SimdDetectionInfoFlags;

/*! @ingroup c_types
    Describes types of binary operation between two images performed by function ::SimdOperationBinary8u.
    Images must have the same format (unsigned 8-bit integer for every channel).
*/
typedef enum
{
    /*! Computes the average value for every channel of every point of two images. \n Average(a, b) = (a + b + 1)/2. */
    SimdOperationBinary8uAverage,
    /*! Computes the bitwise AND between two images. */
    SimdOperationBinary8uAnd,
    /*! Computes the bitwise OR between two images. */
    SimdOperationBinary8uOr,
    /*! Computes maximal value for every channel of every point of two images. */
    SimdOperationBinary8uMaximum,
    /*! Computes minimal value for every channel of every point of two images. */
    SimdOperationBinary8uMinimum,
    /*! Subtracts unsigned 8-bit integer b from unsigned 8-bit integer a and saturates (for every channel of every point of the images). */
    SimdOperationBinary8uSaturatedSubtraction,
    /*! Adds unsigned 8-bit integer b from unsigned 8-bit integer a and saturates (for every channel of every point of the images). */
    SimdOperationBinary8uSaturatedAddition,
    /*! Subtracts unsigned 8-bit integer b from unsigned 8-bit integer a (for every channel of every point of the images). */
    SimdOperationBinary8uSubtraction,
    /*! Adds unsigned 8-bit integer b from unsigned 8-bit integer a (for every channel of every point of the images). */
    SimdOperationBinary8uAddition,
} SimdOperationBinary8uType;

/*! @ingroup c_types
    Describes pixel format types of an image.
    In particular this type is used in functions ::SimdBayerToBgr, ::SimdBayerToBgra, ::SimdBgraToBayer and ::SimdBgrToBayer.
    \note This type is corresponds to C++ type Simd::View::Format.
*/
typedef enum
{
    /*! An undefined pixel format. */
    SimdPixelFormatNone = 0,
    /*! A 8-bit gray pixel format. */
    SimdPixelFormatGray8,
    /*! A 16-bit (2 8-bit channels) pixel format (UV plane of NV12 pixel format). */
    SimdPixelFormatUv16,
    /*! A 24-bit (3 8-bit channels) BGR (Blue, Green, Red) pixel format. */
    SimdPixelFormatBgr24,
    /*! A 32-bit (4 8-bit channels) BGRA (Blue, Green, Red, Alpha) pixel format. */
    SimdPixelFormatBgra32,
    /*! A single channel 16-bit integer pixel format. */
    SimdPixelFormatInt16,
    /*! A single channel 32-bit integer pixel format. */
    SimdPixelFormatInt32,
    /*! A single channel 64-bit integer pixel format. */
    SimdPixelFormatInt64,
    /*! A single channel 32-bit float point pixel format. */
    SimdPixelFormatFloat,
    /*! A single channel 64-bit float point pixel format. */
    SimdPixelFormatDouble,
    /*! A 8-bit Bayer pixel format (GRBG). */
    SimdPixelFormatBayerGrbg,
    /*! A 8-bit Bayer pixel format (GBRG). */
    SimdPixelFormatBayerGbrg,
    /*! A 8-bit Bayer pixel format (RGGB). */
    SimdPixelFormatBayerRggb,
    /*! A 8-bit Bayer pixel format (BGGR). */
    SimdPixelFormatBayerBggr,
    /*! A 24-bit (3 8-bit channels) HSV (Hue, Saturation, Value) pixel format. */
    SimdPixelFormatHsv24,
    /*! A 24-bit (3 8-bit channels) HSL (Hue, Saturation, Lightness) pixel format. */
    SimdPixelFormatHsl24,
    /*! A 24-bit (3 8-bit channels) RGB (Red, Green, Blue) pixel format. */
    SimdPixelFormatRgb24,
} SimdPixelFormatType;

/*! @ingroup c_types
    Describes type of algorithm used for image reducing (downscale in 2 times) (see function Simd::ReduceGray).
*/
enum SimdReduceType
{
    SimdReduce2x2, /*!< Using of function ::SimdReduceGray2x2 for image reducing. */
    SimdReduce3x3, /*!< Using of function ::SimdReduceGray3x3 for image reducing. */
    SimdReduce4x4, /*!< Using of function ::SimdReduceGray4x4 for image reducing. */
    SimdReduce5x5, /*!< Using of function ::SimdReduceGray5x5 for image reducing. */
};

/*! @ingroup resizing
    Describes resized image channel types.
*/
typedef enum
{
    /*! 8-bit integer channel type.  */
    SimdResizeChannelByte,
    /*! 32-bit float channel type.  */
    SimdResizeChannelFloat,
} SimdResizeChannelType;

/*! @ingroup resizing
    Describes methods used in oreder to resize image.
*/
typedef enum
{
    /*! Bilinear method. */
    SimdResizeMethodBilinear,
    /*! caffe::interp compatible method. */
    SimdResizeMethodCaffeInterp,
    /*! Area method. */
    SimdResizeMethodArea,
} SimdResizeMethodType;

// ViSP custom SIMD code
typedef enum
{
    /*! 4-connexity. */
    SimdImageConnexity4,
    /*! 8-connexity. */
    SimdImageConnexity8
} SimdImageConnexityType;

//#if defined(WIN32) && !defined(SIMD_STATIC)
//#  ifdef SIMD_EXPORTS
//#    define SIMD_API __declspec(dllexport)
//#  else//SIMD_EXPORTS
//#    define SIMD_API __declspec(dllimport)
//#  endif//SIMD_EXPORTS
//#else //WIN32
//#    define SIMD_API
//#endif//WIN32

#define SIMD_API

#ifdef __cplusplus
extern "C"
{
#endif//__cplusplus

    /*! @ingroup info

        \fn const char * SimdVersion();

        \short Gets version of %Simd Library.

        \return string with version of %Simd Library (major version number, minor version number, release number, number of SVN's commits).
    */
    SIMD_API const char * SimdVersion();

    /*! @ingroup info

        \fn size_t SimdCpuInfo(SimdCpuInfoType type);

        \short Gets info about CPU and %Simd Library.

        \note See enumeration ::SimdCpuInfoType.

        Using example:
        \verbatim
        #include "Simd/SimdLib.h"
        #include <iostream>

        int main()
        {
            std::cout << "Sockets : " << SimdCpuInfo(SimdCpuInfoSockets) << std::endl;
            std::cout << "Cores : " << SimdCpuInfo(SimdCpuInfoCores) << std::endl;
            std::cout << "Threads : " << SimdCpuInfo(SimdCpuInfoThreads) << std::endl;
            std::cout << "L1D Cache : " << SimdCpuInfo(SimdCpuInfoCacheL1) / 1024  << " KB" << std::endl;
            std::cout << "L2 Cache : " << SimdCpuInfo(SimdCpuInfoCacheL2) / 1024  << " KB" << std::endl;
            std::cout << "L3 Cache : " << SimdCpuInfo(SimdCpuInfoCacheL3) / 1024  << " KB" << std::endl;
            std::cout << "SSE: " << (SimdCpuInfo(SimdCpuInfoSse) ? "Yes" : "No") << std::endl;
            std::cout << "SSE2: " << (SimdCpuInfo(SimdCpuInfoSse2) ? "Yes" : "No") << std::endl;
            std::cout << "SSE3: " << (SimdCpuInfo(SimdCpuInfoSse3) ? "Yes" : "No") << std::endl;
            std::cout << "SSSE3: " << (SimdCpuInfo(SimdCpuInfoSsse3) ? "Yes" : "No") << std::endl;
            std::cout << "SSE4.1: " << (SimdCpuInfo(SimdCpuInfoSse41) ? "Yes" : "No") << std::endl;
            std::cout << "SSE4.2: " << (SimdCpuInfo(SimdCpuInfoSse42) ? "Yes" : "No") << std::endl;
            std::cout << "AVX: " << (SimdCpuInfo(SimdCpuInfoAvx) ? "Yes" : "No") << std::endl;
            std::cout << "AVX2: " << (SimdCpuInfo(SimdCpuInfoAvx2) ? "Yes" : "No") << std::endl;
            std::cout << "AVX-512F: " << (SimdCpuInfo(SimdCpuInfoAvx512f) ? "Yes" : "No") << std::endl;
            std::cout << "AVX-512BW: " << (SimdCpuInfo(SimdCpuInfoAvx512bw) ? "Yes" : "No") << std::endl;
            std::cout << "PowerPC-Altivec: " << (SimdCpuInfo(SimdCpuInfoVmx) ? "Yes" : "No") << std::endl;
            std::cout << "PowerPC-VSX: " << (SimdCpuInfo(SimdCpuInfoVsx) ? "Yes" : "No") << std::endl;
            std::cout << "ARM-NEON: " << (SimdCpuInfo(SimdCpuInfoNeon) ? "Yes" : "No") << std::endl;
            std::cout << "MIPS-MSA: " << (SimdCpuInfo(SimdCpuInfoMsa) ? "Yes" : "No") << std::endl;
            return 0;
        }
        \endverbatim

        \param [in] type - a type of required information.
        \return a value which contains information about CPU and %Simd Library.
    */
    SIMD_API size_t SimdCpuInfo(SimdCpuInfoType type);

    /*! @ingroup info

        \fn const char *SimdPerformanceStatistic();

        \short Gets internal performance statistics of %Simd Library.

        \note %Simd Library have to be build with defined SIMD_PERFORMANCE_STATISTIC macro.

        \return string with internal performance statistics of %Simd Library.
    */
    SIMD_API const char * SimdPerformanceStatistic();

    /*! @ingroup memory

        \fn void * SimdAllocate(size_t size, size_t align);

        \short Allocates aligned memory block.

        \note The memory allocated by this function is must be deleted by function ::SimdFree.

        \param [in] size - a size of memory block.
        \param [in] align - a required alignment of memory block.

        \return a pointer to allocated memory.
    */
    SIMD_API void * SimdAllocate(size_t size, size_t align);

    /*! @ingroup memory

        \fn void SimdFree(void * ptr);

        \short Frees aligned memory block.

        \note This function frees a memory allocated by function ::SimdAllocate.

        \param [in] ptr - a pointer to the memory to be deleted.
    */
    SIMD_API void SimdFree(void * ptr);

    /*! @ingroup memory

        \fn size_t SimdAlign(size_t size, size_t align);

        \short Gets aligned size.

        \param [in] size - an original size.
        \param [in] align - a required alignment.

        \return an aligned size.
    */
    SIMD_API size_t SimdAlign(size_t size, size_t align);

    /*! @ingroup memory

        \fn size_t SimdAlignment();

        \short Gets alignment required for the most productive work of the Simd Library.

        \return a required alignment.
    */
    SIMD_API size_t SimdAlignment();

    /*! @ingroup memory

        \fn void SimdRelease(void * context);

        \short Releases context created with using of Simd Library API.

        \note This function releases a context created by functions ::SimdDetectionLoadA and ::SimdDetectionInit.

        \param [in] context - a context to be released.
    */
    SIMD_API void SimdRelease(void * context);

    /*! @ingroup thread

        \fn size_t SimdGetThreadNumber();

        \short Gets number of threads used by Simd Library to parallelize some algorithms.

        \return current thread number.
    */
    SIMD_API size_t SimdGetThreadNumber();

    /*! @ingroup thread

        \fn void SimdSetThreadNumber(size_t threadNumber);

        \short Sets number of threads used by Simd Library to parallelize some algorithms.

        \param [in] threadNumber - a number of threads.
    */
    SIMD_API void SimdSetThreadNumber(size_t threadNumber);

    /*! @ingroup cpu_flags

        \fn SimdBool SimdGetFastMode();

        \short Gets current CPU Flush-To-Zero (FTZ) and Denormals-Are-Zero (DAZ) flags. It is used in order to process subnormal numbers.

        \return current 'fast' mode.
    */
    SIMD_API SimdBool SimdGetFastMode();

    /*! @ingroup cpu_flags

        \fn void SimdSetFastMode(SimdBool value);

        \short Sets current CPU Flush-To-Zero (FTZ) and Denormals-Are-Zero (DAZ) flags. It is used in order to process subnormal numbers.

        \param [in] value - a value of 'fast' mode.
    */
    SIMD_API void SimdSetFastMode(SimdBool value);

    /*! @ingroup drawing

        \fn void SimdAlphaBlending(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, const uint8_t * alpha, size_t alphaStride, uint8_t * dst, size_t dstStride);

        \short Performs alpha blending operation.

        All images must have the same width and height. Source and destination images must have the same format (8 bit per channel, for example GRAY8, BGR24 or BGRA32). Alpha must be 8-bit gray image.

        For every point:
        \verbatim
        dst[x, y, c] = (src[x, y, c]*alpha[x, y] + dst[x, y, c]*(255 - alpha[x, y]))/255;
        \endverbatim

        This function is used for image drawing.

        \note This function has a C++ wrapper Simd::AlphaBlending(const View<A>& src, const View<A>& alpha, View<A>& dst).

        \param [in] src - a pointer to pixels data of foreground image.
        \param [in] srcStride - a row size of the foreground image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count for foreground and background images (1 <= channelCount <= 4).
        \param [in] alpha - a pointer to pixels data of image with alpha channel.
        \param [in] alphaStride - a row size of the alpha image.
        \param [in, out] dst - a pointer to pixels data of background image.
        \param [in] dstStride - a row size of the background image.
    */
    SIMD_API void SimdAlphaBlending(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount,
        const uint8_t * alpha, size_t alphaStride, uint8_t * dst, size_t dstStride);

    /*! @ingroup drawing

        \fn void SimdAlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8_t * channel, size_t channelCount, const uint8_t * alpha, size_t alphaStride);

        \short Performs alpha filling operation.

        All images must have the same width and height. Destination images must have 8 bit per channel (for example GRAY8, BGR24 or BGRA32). Alpha must be 8-bit gray image.

        For every point:
        \verbatim
        dst[x, y, c] = (channel[c]*alpha[x, y] + dst[x, y, c]*(255 - alpha[x, y]))/255;
        \endverbatim

        This function is used for image drawing.

        \note This function has a C++ wrapper Simd::AlphaFilling(View<A> & dst, const Pixel & pixel, const View<A> & alpha).

        \param [in, out] dst - a pointer to pixels data of background image.
        \param [in] dstStride - a row size of the background image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channel - a pointer to pixel with foreground color.
        \param [in] channelCount - a channel count for foreground color and background images (1 <= channelCount <= 4).
        \param [in] alpha - a pointer to pixels data of image with alpha channel.
        \param [in] alphaStride - a row size of the alpha image.
    */
    SIMD_API void SimdAlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8_t * channel, size_t channelCount, const uint8_t * alpha, size_t alphaStride);

    /*! @ingroup bayer_conversion

        \fn void SimdBayerToBgr(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgr, size_t bgrStride);

        \short Converts 8-bit Bayer image to 24-bit BGR.

        All images must have the same width and height. The width and the height must be even.

        \note This function has a C++ wrapper Simd::BayerToBgr(const View<A>& bayer, View<A>& bgr).

        \param [in] bayer - a pointer to pixels data of input 8-bit Bayer image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bayerStride - a row size of the bayer image.
        \param [in] bayerFormat - a format of the input bayer image. It can be ::SimdPixelFormatBayerGrbg, ::SimdPixelFormatBayerGbrg, ::SimdPixelFormatBayerRggb or ::SimdPixelFormatBayerBggr.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdBayerToBgr(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup bayer_conversion

        \fn void SimdBayerToBgra(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 8-bit Bayer image to 32-bit BGRA.

        All images must have the same width and height. The width and the height must be even.

        \note This function has a C++ wrapper Simd::BayerToBgra(const View<A>& bayer, View<A>& bgra, uint8_t alpha).

        \param [in] bayer - a pointer to pixels data of input 8-bit Bayer image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bayerStride - a row size of the bayer image.
        \param [in] bayerFormat - a format of the input bayer image. It can be ::SimdPixelFormatBayerGrbg, ::SimdPixelFormatBayerGbrg, ::SimdPixelFormatBayerRggb or ::SimdPixelFormatBayerBggr.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdBayerToBgra(const uint8_t * bayer, size_t width, size_t height, size_t bayerStride, SimdPixelFormatType bayerFormat, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToBayer(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat);

        \short Converts 32-bit BGRA image to 8-bit Bayer image.

        All images must have the same width and height. The width and the height must be even.

        \note This function has a C++ wrapper Simd::BgraToBayer(const View<A>& bgra, View<A>& bayer).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] bayer - a pointer to pixels data of output 8-bit Bayer image.
        \param [in] bayerStride - a row size of the bayer image.
        \param [in] bayerFormat - a format of the output bayer image. It can be ::SimdPixelFormatBayerGrbg, ::SimdPixelFormatBayerGbrg, ::SimdPixelFormatBayerRggb or ::SimdPixelFormatBayerBggr.
    */
    SIMD_API void SimdBgraToBayer(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride);

        \short Converts 32-bit BGRA image to 24-bit BGR image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgraToBgr(const View<A>& bgra, View<A>& bgr).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToGray(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * gray, size_t grayStride);

        \short Converts 32-bit BGRA image to 8-bit gray image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgraToGray(const View<A>& bgra, View<A>& gray).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdBgraToGray(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * gray, size_t grayStride);

    /*! @ingroup bgra_conversion

        \fn void SimdRgbaToGray(const uint8_t * rgba, size_t width, size_t height, size_t rgbaStride, uint8_t * gray, size_t grayStride);

        \short Converts 32-bit RGBA image to 8-bit gray image.

        All images must have the same width and height.

        \param [in] rgba - a pointer to pixels data of input 32-bit RGBA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] rgbaStride - a row size of the rgba image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdRgbaToGray(const uint8_t * rgba, size_t width, size_t height, size_t rgbaStride, uint8_t * gray, size_t grayStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToYuv420p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 32-bit BGRA image to YUV420P.

        The input BGRA and output Y images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrapper Simd::BgraToYuv420p(const View<A>& bgra, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the BGRA image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgraToYuv420p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToYuv422p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 32-bit BGRA image to YUV422P.

        The input BGRA and output Y images must have the same width and height.
        The input U and V images must have the same width and height (their width is equal to half width of Y component).

        \note This function has a C++ wrapper Simd::BgraToYuv422p(const View<A>& bgra, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the BGRA image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgraToYuv422p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToYuv444p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 32-bit BGRA image to YUV444P.

        The input BGRA and output Y, U and V images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgraToYuv444p(const View<A>& bgra, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the BGRA image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgraToYuv444p(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToYuva420p(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride, uint8_t * a, size_t aStride);

        \short Converts 32-bit BGRA image to YUVA420P.

        The input BGRA and output Y and A images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrapper Simd::BgraToYuva420p(const View<A> & bgra, View<A> & y, View<A> & u, View<A> & v, View<A> & a).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA image.
        \param [in] bgraStride - a row size of the BGRA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [out] a - a pointer to pixels data of output 8-bit image with alpha plane.
        \param [in] aStride - a row size of the a image.
    */
    SIMD_API void SimdBgraToYuva420p(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height,
        uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride, uint8_t * a, size_t aStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToBayer(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat);

        \short Converts 24-bit BGR image to 8-bit Bayer image.

        All images must have the same width and height. The width and the height must be even.

        \note This function has a C++ wrapper Simd::BgrToBayer(const View<A>& bgr, View<A>& bayer).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] bayer - a pointer to pixels data of output 8-bit Bayer image.
        \param [in] bayerStride - a row size of the bayer image.
        \param [in] bayerFormat - a format of the output bayer image. It can be ::SimdPixelFormatBayerGrbg, ::SimdPixelFormatBayerGbrg, ::SimdPixelFormatBayerRggb or ::SimdPixelFormatBayerBggr.
    */
    SIMD_API void SimdBgrToBayer(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * bayer, size_t bayerStride, SimdPixelFormatType bayerFormat);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToBgra(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 24-bit BGR image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToBgra(const View<A>& bgr, View<A>& bgra, uint8_t alpha).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdBgrToBgra(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha);

        \short Converts 24-bit BGR image to 32-bit RGBA image.

        All images must have the same width and height.

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] rgba - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] rgbaStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdBgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha);

    /*! @ingroup other_conversion

        \fn void SimdBgr48pToBgra32(const uint8_t * blue, size_t blueStride, size_t width, size_t height, const uint8_t * green, size_t greenStride, const uint8_t * red, size_t redStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 48-bit planar BGR image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::Bgr48pToBgra32(const View<A>& blue, const View<A>& green, const View<A>& red, View<A>& bgra, uint8_t alpha).

        \param [in] blue - a pointer to pixels data of input 16-bit image with blue color plane.
        \param [in] blueStride - a row size of the blue image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] green - a pointer to pixels data of input 16-bit image with green color plane.
        \param [in] greenStride - a row size of the blue image.
        \param [in] red - a pointer to pixels data of input 16-bit image with red color plane.
        \param [in] redStride - a row size of the red image.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdBgr48pToBgra32(const uint8_t * blue, size_t blueStride, size_t width, size_t height,
        const uint8_t * green, size_t greenStride, const uint8_t * red, size_t redStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToGray(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * gray, size_t grayStride);

        \short Converts 24-bit BGR image to 8-bit gray image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToGray(const View<A>& bgr, View<A>& gray).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdBgrToGray(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * gray, size_t grayStride);

    /*! @ingroup bgr_conversion

        \fn void SimdRgbToGray(const uint8_t * rgb, size_t width, size_t height, size_t rgbStride, uint8_t * gray, size_t grayStride);

        \short Converts 24-bit RGB image to 8-bit gray image.

        All images must have the same width and height.

        \param [in] rgb - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] rgbStride - a row size of the bgr image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdRgbToGray(const uint8_t * rgb, size_t width, size_t height, size_t rgbStride, uint8_t * gray, size_t grayStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToHsl(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsl, size_t hslStride);

        \short Converts 24-bit BGR image to 24-bit HSL(Hue, Saturation, Lightness) image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToHsl(const View<A>& bgr, View<A>& hsl).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] hsl - a pointer to pixels data of output 24-bit HSL image.
        \param [in] hslStride - a row size of the hsl image.
    */
    SIMD_API void SimdBgrToHsl(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsl, size_t hslStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToHsv(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsv, size_t hsvStride);

        \short Converts 24-bit BGR image to 24-bit HSV(Hue, Saturation, Value) image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToHsv(const View<A>& bgr, View<A>& hsv).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] hsv - a pointer to pixels data of output 24-bit HSV image.
        \param [in] hsvStride - a row size of the hsv image.
    */
    SIMD_API void SimdBgrToHsv(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * hsv, size_t hsvStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToRgb(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height, uint8_t * rgb, size_t rgbStride);

        \short Converts 24-bit BGR image to 24-bit RGB image (also it performs backward conversion).

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToRgb(const View<A> & bgr, View<A> & rgb).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] rgb - a pointer to pixels data of output 24-bit RGB image.
        \param [in] rgbStride - a row size of the rgb image.
    */
    SIMD_API void SimdBgrToRgb(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height, uint8_t * rgb, size_t rgbStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToYuv420p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 24-bit BGR image to YUV420P.

        The input BGR and output Y images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrapper Simd::BgrToYuv420p(const View<A>& bgr, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the BGR image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgrToYuv420p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToYuv422p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 24-bit BGR image to YUV422P.

        The input BGR and output Y images must have the same width and height.
        The input U and V images must have the same width and height (their width is equal to half width of Y component).

        \note This function has a C++ wrapper Simd::BgrToYuv422p(const View<A>& bgr, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the BGR image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgrToYuv422p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup bgr_conversion

        \fn void SimdBgrToYuv444p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Converts 24-bit BGR image to YUV444P.

        The input BGR and output Y, U and V images must have the same width and height.

        \note This function has a C++ wrapper Simd::BgrToYuv444p(const View<A>& bgr, View<A>& y, View<A>& u, View<A>& v).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the BGR image.
        \param [out] y - a pointer to pixels data of output 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [out] u - a pointer to pixels data of output 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of output 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdBgrToYuv444p(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * y, size_t yStride, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup binarization

        \fn void SimdBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t value, uint8_t positive, uint8_t negative, uint8_t * dst, size_t dstStride, SimdCompareType compareType);

        \short Performs binarization of 8-bit gray image.

        All images must have 8-bit gray format and must have the same width and height.

        For every point:
        \verbatim
        dst[i] = compare(src[i], value) ? positive : negative;
        \endverbatim
        where compare(a, b) depends from compareType (see ::SimdCompareType).

        \note This function has a C++ wrapper Simd::Binarization(const View<A>& src, uint8_t value, uint8_t positive, uint8_t negative, View<A>& dst, SimdCompareType compareType).

        \param [in] src - a pointer to pixels data of input 8-bit gray image (first value for compare operation).
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] value - a second value for compare operation.
        \param [in] positive - a destination value if comparison operation has a positive result.
        \param [in] negative - a destination value if comparison operation has a negative result.
        \param [out] dst - a pointer to pixels data of output 8-bit gray binarized image.
        \param [in] dstStride - a row size of the dst image.
        \param [in] compareType - a compare operation type (see ::SimdCompareType).
    */
    SIMD_API void SimdBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        uint8_t value, uint8_t positive, uint8_t negative, uint8_t * dst, size_t dstStride, SimdCompareType compareType);

    /*! @ingroup binarization

        \fn void SimdAveragingBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t value, size_t neighborhood, uint8_t threshold, uint8_t positive, uint8_t negative, uint8_t * dst, size_t dstStride, SimdCompareType compareType);

        \short Performs averaging binarization of 8-bit gray image.

        All images must have 8-bit gray format and must have the same width and height.

        For every point:
        \verbatim
        sum = 0; area = 0;
        for(dy = -neighborhood; dy <= neighborhood; ++dy)
        {
            for(dx = -neighborhood; dx <= neighborhood; ++dx)
            {
                if(x + dx >= 0 && x + dx < width && y + dy >= 0 && y + dy < height)
                {
                    area++;
                    if(compare(src[x + dx, x + dy], value))
                        sum++;
                }
            }
        }
        dst[x, y] = sum*255 > area*threshold ? positive : negative;
        \endverbatim
        where compare(a, b) depends from compareType (see ::SimdCompareType).

        \note This function has a C++ wrapper Simd::AveragingBinarization(const View<A>& src, uint8_t value, size_t neighborhood, uint8_t threshold, uint8_t positive, uint8_t negative, View<A>& dst, SimdCompareType compareType).

        \param [in] src - a pointer to pixels data of input 8-bit gray image (first value for compare operation).
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] value - a second value for compare operation.
        \param [in] neighborhood - an averaging neighborhood.
        \param [in] threshold - a threshold value for binarization. It can range from 0 to 255.
        \param [in] positive - a destination value if for neighborhood of this point number of positive comparison is greater then threshold.
        \param [in] negative - a destination value if for neighborhood of this point number of positive comparison is lesser or equal then threshold.
        \param [out] dst - a pointer to pixels data of output 8-bit gray binarized image.
        \param [in] dstStride - a row size of the dst image.
        \param [in] compareType - a compare operation type (see ::SimdCompareType).
    */
    SIMD_API void SimdAveragingBinarization(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        uint8_t value, size_t neighborhood, uint8_t threshold, uint8_t positive, uint8_t negative,
        uint8_t * dst, size_t dstStride, SimdCompareType compareType);

    /*! @ingroup copying

        \fn void SimdCopy(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize, uint8_t * dst, size_t dstStride);

        \short Copies pixels data of image from source to destination.

        All images must have the same width, height and format.

        \note This function has a C++ wrapper Simd::Copy(const View<A> & src, View<B> & dst).

        \param [in] src - a pointer to pixels data of source image.
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] pixelSize - a size of the image pixel.
        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdCopy(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize, uint8_t * dst, size_t dstStride);

    /*! @ingroup copying

        \fn void SimdCopyFrame(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize, size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t * dst, size_t dstStride);

        \short Copies pixels data of image from source to destination except for the portion bounded frame.

        All images must have the same width, height and format.

        \note This function has a C++ wrapper Simd::CopyFrame(const View<A>& src, const Rectangle<ptrdiff_t> & frame, View<A>& dst).

        \param [in] src - a pointer to pixels data of source image.
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] pixelSize - a size of the image pixel.
        \param [in] frameLeft - a frame left side.
        \param [in] frameTop - a frame top side.
        \param [in] frameRight - a frame right side.
        \param [in] frameBottom - a frame bottom side.
        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdCopyFrame(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t pixelSize,
        size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup other_conversion

        \fn void SimdDeinterleaveUv(const uint8_t * uv, size_t uvStride, size_t width, size_t height, uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

        \short Deinterleaves 16-bit UV interleaved image into separated 8-bit U and V planar images.

        All images must have the same width and height.
        This function used for NV12 to YUV420P conversion.

        \note This function has a C++ wrapper Simd::DeinterleaveUv(const View<A>& uv, View<A>& u, View<A>& v).

        \param [in] uv - a pointer to pixels data of input 16-bit UV interleaved image.
        \param [in] uvStride - a row size of the uv image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] u - a pointer to pixels data of 8-bit U planar image.
        \param [in] uStride - a row size of the u image.
        \param [out] v - a pointer to pixels data of 8-bit V planar image.
        \param [in] vStride - a row size of the v image.
    */
    SIMD_API void SimdDeinterleaveUv(const uint8_t * uv, size_t uvStride, size_t width, size_t height,
        uint8_t * u, size_t uStride, uint8_t * v, size_t vStride);

    /*! @ingroup other_conversion

        \fn void SimdDeinterleaveBgr(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height, uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride);

        \short Deinterleaves 24-bit BGR interleaved image into separated 8-bit Blue, Green and Red planar images.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::DeinterleaveBgr(const View<A>& bgr, View<A>& b, View<A>& g, View<A>& r).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR interleaved image.
        \param [in] bgrStride - a row size of the bgr image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] b - a pointer to pixels data of 8-bit Blue planar image.
        \param [in] bStride - a row size of the b image.
        \param [out] g - a pointer to pixels data of 8-bit Green planar image.
        \param [in] gStride - a row size of the g image.
        \param [out] r - a pointer to pixels data of 8-bit Red planar image.
        \param [in] rStride - a row size of the r image.
    */
    SIMD_API void SimdDeinterleaveBgr(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height,
        uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride);

    /*! @ingroup other_conversion

        \fn void SimdDeinterleaveBgra(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height, uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride, uint8_t * a, size_t aStride);

        \short Deinterleaves 32-bit BGRA interleaved image into separated 8-bit Blue, Green, Red and Alpha planar images.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::DeinterleaveBgra(const View<A>& bgra, View<A>& b, View<A>& g, View<A>& r, View<A>& a).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA interleaved image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] b - a pointer to pixels data of 8-bit Blue planar image.
        \param [in] bStride - a row size of the b image.
        \param [out] g - a pointer to pixels data of 8-bit Green planar image.
        \param [in] gStride - a row size of the g image.
        \param [out] r - a pointer to pixels data of 8-bit Red planar image.
        \param [in] rStride - a row size of the r image.
        \param [out] a - a pointer to pixels data of 8-bit Alpha planar image.
        \param [in] aStride - a row size of the a image.
    */
    SIMD_API void SimdDeinterleaveBgra(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height,
        uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride, uint8_t * a, size_t aStride);

    /*! @ingroup object_detection

        \fn void * SimdDetectionLoadA(const char * path);

        \short Loads a classifier cascade from file.

        This function supports OpenCV HAAR and LBP cascades type.
        Tree based cascades and old cascade formats are not supported.

        \note This function is used for implementation of Simd::Detection.

        \param [in] path - a path to cascade.
        \return a pointer to loaded cascade. On error it returns NULL.
                This pointer is used in functions ::SimdDetectionInfo and ::SimdDetectionInit, and must be released with using of function ::SimdRelease.
    */
    SIMD_API void * SimdDetectionLoadA(const char * path);

    /*! @ingroup object_detection

        \fn void * SimdDetectionLoadStringXml(char * xml);

        \short Loads a classifier cascade from a string.

        This function supports OpenCV HAAR and LBP cascades type.
        Tree based cascades and old cascade formats are not supported.

        \note This function is used for implementation of Simd::Detection.

        \param [in,out] xml - A string with the xml of a classifier cascade.
        \return a pointer to loaded cascade. On error it returns NULL.
                This pointer is used in functions ::SimdDetectionInfo and ::SimdDetectionInit, and must be released with using of function ::SimdRelease.
    */
    SIMD_API void * SimdDetectionLoadStringXml(char * xml);

    /*! @ingroup object_detection

        \fn void SimdDetectionInfo(const void * data, size_t * width, size_t * height, SimdDetectionInfoFlags * flags);

        \short Gets information about the classifier cascade.

        \note This function is used for implementation of Simd::Detection.

        \param [in] data - a pointer to cascade which was received with using of function ::SimdDetectionLoadA.
        \param [out] width - a pointer to returned width of cascade window.
        \param [out] height - a pointer to returned height of cascade window.
        \param [out] flags - a pointer to flags with other information (See ::SimdDetectionInfoFlags).
    */
    SIMD_API void SimdDetectionInfo(const void * data, size_t * width, size_t * height, SimdDetectionInfoFlags * flags);

    /*! @ingroup object_detection

        \fn void * SimdDetectionInit(const void * data, uint8_t * sum, size_t sumStride, size_t width, size_t height, uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride, int throughColumn, int int16);

        \short Initializes hidden classifier cascade structure to work with given size of input 8-bit gray image.

        \note This function is used for implementation of Simd::Detection.

        \param [in] data - a pointer to cascade which was received with using of function ::SimdDetectionLoadA.
        \param [in] sum - a pointer to pixels data of 32-bit integer image with integral sum of given input 8-bit gray image.
                          See function ::SimdIntegral in order to estimate this integral sum.
        \param [in] sumStride - a row size of the sum image.
        \param [in] width - a width of the sum image. It must be per unit greater than width of input 8-bit gray image.
        \param [in] height - a height of the sum image. It must be per unit greater than height of input 8-bit gray image.
        \param [in] sqsum - a pointer to pixels data of 32-bit integer image with squared integral sum of given input 8-bit gray image.
                            Its size must be equal to sum image. See function ::SimdIntegral in order to estimate this squared integral sum. Its
        \param [in] sqsumStride - a row size of the sqsum image.
        \param [in] tilted - a pointer to pixels data of 32-bit integer image with tilted integral sum of given input 8-bit gray image.
                             Its size must be equal to sum image. See function ::SimdIntegral in order to estimate this tilted integral sum.
        \param [in] tiltedStride - a row size of the tilted image.
        \param [in] throughColumn - a flag to detect objects only in even columns and rows (to increase performance).
        \param [in] int16 - a flag use for 16-bit integer version of detection algorithm. (See ::SimdDetectionInfo).
        \return a pointer to hidden cascade. On error it returns NULL.
                This pointer is used in functions ::SimdDetectionPrepare, ::SimdDetectionHaarDetect32fp, ::SimdDetectionHaarDetect32fi,
                ::SimdDetectionLbpDetect32fp, ::SimdDetectionLbpDetect32fi, ::SimdDetectionLbpDetect16ip and ::SimdDetectionLbpDetect16ii.
                It must be released with using of function ::SimdRelease.
    */
    SIMD_API void * SimdDetectionInit(const void * data, uint8_t * sum, size_t sumStride, size_t width, size_t height,
        uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride, int throughColumn, int int16);

    /*! @ingroup object_detection

        \fn void SimdDetectionPrepare(void * hid);

        \short Prepares hidden classifier cascade structure to work with given input 8-bit gray image.

        You must call this function before calling of functions ::SimdDetectionHaarDetect32fp, ::SimdDetectionHaarDetect32fi,
         ::SimdDetectionLbpDetect32fp, ::SimdDetectionLbpDetect32fi, ::SimdDetectionLbpDetect16ip and ::SimdDetectionLbpDetect16ii.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
    */
    SIMD_API void SimdDetectionPrepare(void * hid);

    /*! @ingroup object_detection

        \fn void SimdDetectionHaarDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of HAAR cascade classifier (uses 32-bit float numbers, processes all points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionHaarDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup object_detection

        \fn void SimdDetectionHaarDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of HAAR cascade classifier (uses 32-bit float numbers, processes only even points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionHaarDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup object_detection

        \fn void SimdDetectionLbpDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of LBP cascade classifier (uses 32-bit float numbers, processes all points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionLbpDetect32fp(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup object_detection

        \fn void SimdDetectionLbpDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of LBP cascade classifier (uses 32-bit float numbers, processes only even points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionLbpDetect32fi(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup object_detection

        \fn void SimdDetectionLbpDetect16ip(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of LBP cascade classifier (uses 16-bit integer numbers, processes all points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionLbpDetect16ip(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup object_detection

        \fn void SimdDetectionLbpDetect16ii(const void * hid, const uint8_t * mask, size_t maskStride, ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

        \short Performs object detection with using of LBP cascade classifier (uses 16-bit integer numbers, processes only even points).

        You must call function ::SimdDetectionPrepare before calling of this functions.
        All restriction (input mask and bounding box) affects to left-top corner of scanning window.

        \note This function is used for implementation of Simd::Detection.

        \param [in] hid - a pointer to hidden cascade which was received with using of function ::SimdDetectionInit.
        \param [in] mask - a pointer to pixels data of 8-bit image with mask. The mask restricts detection region.
        \param [in] maskStride - a row size of the mask image.
        \param [in] left - a left side of bounding box which restricts detection region.
        \param [in] top - a top side of bounding box which restricts detection region.
        \param [in] right - a right side of bounding box which restricts detection region.
        \param [in] bottom - a bottom side of bounding box which restricts detection region.
        \param [out] dst - a pointer to pixels data of 8-bit image with output result. None zero points refer to left-top corner of detected objects.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdDetectionLbpDetect16ii(const void * hid, const uint8_t * mask, size_t maskStride,
        ptrdiff_t left, ptrdiff_t top, ptrdiff_t right, ptrdiff_t bottom, uint8_t * dst, size_t dstStride);

    /*! @ingroup filling

        \fn void SimdFill(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize, uint8_t value);

        \short Fills pixels data of image by given value.

        \note This function has a C++ wrapper Simd::Fill(View<A>& dst, uint8_t value).

        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] stride - a row size of the dst image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] pixelSize - a size of the image pixel.
        \param [in] value - a value to fill image.
    */
    SIMD_API void SimdFill(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize, uint8_t value);

    /*! @ingroup filling

        \fn void SimdFillFrame(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize, size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t value);

        \short Fills pixels data of image except for the portion bounded frame by given value.

        \note This function has a C++ wrapper Simd::FillFrame(View<A>& dst, const Rectangle<ptrdiff_t> & frame, uint8_t value).

        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] stride - a row size of the dst image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] pixelSize - a size of the image pixel.
        \param [in] frameLeft - a frame left side.
        \param [in] frameTop - a frame top side.
        \param [in] frameRight - a frame right side.
        \param [in] frameBottom - a frame bottom side.
        \param [in] value - a value to fill image.
    */
    SIMD_API void SimdFillFrame(uint8_t * dst, size_t stride, size_t width, size_t height, size_t pixelSize,
        size_t frameLeft, size_t frameTop, size_t frameRight, size_t frameBottom, uint8_t value);

    /*! @ingroup filling

        \fn void SimdFillBgr(uint8_t * dst, size_t stride, size_t width, size_t height, uint8_t blue, uint8_t green, uint8_t red);

        \short Fills pixels data of 24-bit BGR image by given color(blue, green, red).

        \note This function has a C++ wrapper Simd::FillBgr(View<A>& dst, uint8_t blue, uint8_t green, uint8_t red).

        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] stride - a row size of the dst image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] blue - a blue channel of BGR to fill image.
        \param [in] green - a green channel of BGR to fill image.
        \param [in] red - a red channel of BGR to fill image.
    */
    SIMD_API void SimdFillBgr(uint8_t * dst, size_t stride, size_t width, size_t height, uint8_t blue, uint8_t green, uint8_t red);

    /*! @ingroup filling

        \fn void SimdFillBgra(uint8_t * dst, size_t stride, size_t width, size_t height, uint8_t blue, uint8_t green, uint8_t red, uint8_t alpha);

        \short Fills pixels data of 32-bit BGRA image by given color(blue, green, red, alpha).

        \note This function has a C++ wrapper Simd::FillBgra(View<A>& dst, uint8_t blue, uint8_t green, uint8_t red, uint8_t alpha).

        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] stride - a row size of the dst image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] blue - a blue channel of BGRA to fill image.
        \param [in] green - a green channel of BGRA to fill image.
        \param [in] red - a red channel of BGRA to fill image.
        \param [in] alpha - a alpha channel of BGRA to fill image.
    */
    SIMD_API void SimdFillBgra(uint8_t * dst, size_t stride, size_t width, size_t height,
        uint8_t blue, uint8_t green, uint8_t red, uint8_t alpha);

    /*! @ingroup filling

        \fn void SimdFillPixel(uint8_t * dst, size_t stride, size_t width, size_t height, const uint8_t * pixel, size_t pixelSize);

        \short Fills image by value of given pixel.

        \note This function has a C++ wrapper Simd::FillPixel(View<A> & dst, const Pixel & pixel).

        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] stride - a row size of the dst image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] pixel - a pointer to pixel to fill.
        \param [in] pixelSize - a size of the image pixel. Parameter is restricted by range [1, 4].
    */
    SIMD_API void SimdFillPixel(uint8_t * dst, size_t stride, size_t width, size_t height, const uint8_t * pixel, size_t pixelSize);

    /*! @ingroup filling

        \fn void SimdFill32f(float * dst, size_t size, const float * value);

        \short Fills 32-bit float array by given value.

        \param [out] dst - a pointer to 32-bit float array.
        \param [in] size - a size of the array.
        \param [in] value - a pointer to value to fill. Can be NULL (filling value is assumed to be equal to zero).
    */
    SIMD_API void SimdFill32f(float * dst, size_t size, const float * value);

    /*! @ingroup other_filter

        \fn void SimdGaussianBlur3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs Gaussian blur filtration with window 3x3.

        For every point:
        \verbatim
        dst[x, y] = (src[x-1, y-1] + 2*src[x, y-1] + src[x+1, y-1] +
                    2*(src[x-1, y] + 2*src[x, y] + src[x+1, y]) +
                    src[x-1, y+1] + 2*src[x, y+1] + src[x+1, y+1] + 8) / 16;
        \endverbatim

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrapper Simd::GaussianBlur3x3(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of source image.
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdGaussianBlur3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup gray_conversion

        \fn void SimdGrayToBgr(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgr, size_t bgrStride);

        \short Converts 8-bit gray image to 24-bit BGR image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::GrayToBgr(const View<A>& gray, View<A>& bgr).

        \param [in] gray - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] grayStride - a row size of the gray image.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdGrayToBgr(const uint8_t *gray, size_t width, size_t height, size_t grayStride, uint8_t *bgr, size_t bgrStride);

    /*! @ingroup gray_conversion

        \fn void SimdGrayToBgra(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 8-bit gray image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::GrayToBgra(const View<A>& gray, View<A>& bgra, uint8_t alpha).

        \param [in] gray - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] grayStride - a row size of the gray image.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdGrayToBgra(const uint8_t *gray, size_t width, size_t height, size_t grayStride,
        uint8_t *bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup histogram

        \fn void SimdAbsSecondDerivativeHistogram(const uint8_t * src, size_t width, size_t height, size_t stride, size_t step, size_t indent, uint32_t * histogram);

        \short Calculates histogram of second derivative for 8-bit gray image.

        For all points except the boundary (defined by parameter indent):
        \verbatim
        dx = abs(src[x, y] - average(src[x+step, y], src[x-step, y]));
        dy = abs(src[x, y] - average(src[x, y+step], src[x, y-step]));
        histogram[max(dx, dy)]++;
        \endverbatim

        \note This function has a C++ wrapper Simd::AbsSecondDerivativeHistogram(const View<A>& src, size_t step, size_t indent, uint32_t * histogram).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] stride - a row size of the image.
        \param [in] step - a step for second derivative calculation.
        \param [in] indent - a indent from image boundary.
        \param [out] histogram - a pointer to histogram (array of 256 unsigned 32-bit values).
    */
    SIMD_API void SimdAbsSecondDerivativeHistogram(const uint8_t * src, size_t width, size_t height, size_t stride,
        size_t step, size_t indent, uint32_t * histogram);

    /*! @ingroup histogram

        \fn void SimdHistogram(const uint8_t * src, size_t width, size_t height, size_t stride, uint32_t * histogram);

        \short Calculates histogram for 8-bit gray image.

        For all points:
        \verbatim
        histogram[src[i]]++.
        \endverbatim

        \note This function has a C++ wrapper Simd::Histogram(const View<A>& src, uint32_t * histogram).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] stride - a row size of the image.
        \param [out] histogram - a pointer to histogram (array of 256 unsigned 32-bit values).
    */
    SIMD_API void SimdHistogram(const uint8_t * src, size_t width, size_t height, size_t stride, uint32_t * histogram);

    /*! @ingroup histogram

        \fn void SimdHistogramMasked(const uint8_t * src, size_t srcStride, size_t width, size_t height, const uint8_t * mask, size_t maskStride, uint8_t index, uint32_t * histogram);

        \short Calculates histogram for 8-bit gray image with using mask.

        For every point:
        \verbatim
        if(mask[i] == index)
            histogram[src[i]]++.
        \endverbatim

        \note This function has a C++ wrapper Simd::HistogramMasked(const View<A> & src, const View<A> & mask, uint8_t index, uint32_t * histogram).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] srcStride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] mask - a pointer to pixels data of the mask 8-bit image.
        \param [in] maskStride - a row size of the mask image.
        \param [in] index - a mask index.
        \param [out] histogram - a pointer to histogram (array of 256 unsigned 32-bit values).
    */
    SIMD_API void SimdHistogramMasked(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        const uint8_t * mask, size_t maskStride, uint8_t index, uint32_t * histogram);

    /*! @ingroup histogram

        \fn void SimdHistogramConditional(const uint8_t * src, size_t srcStride, size_t width, size_t height, const uint8_t * mask, size_t maskStride, uint8_t value, SimdCompareType compareType, uint32_t * histogram);

        \short Calculates histogram of 8-bit gray image for those points when mask points satisfying certain condition.

        For every point:
        \verbatim
        if(compare(mask[x, y], value))
            histogram[src[x, y]]++.
        \endverbatim

        \note This function has a C++ wrapper Simd::HistogramConditional(const View<A>& src, const View<A>& mask, uint8_t value, SimdCompareType compareType, uint32_t * histogram).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] srcStride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] mask - a pointer to pixels data of the mask 8-bit image.
        \param [in] maskStride - a row size of the mask image.
        \param [in] value - a second value for compare operation.
        \param [in] compareType - a compare operation type (see ::SimdCompareType).
        \param [out] histogram - a pointer to histogram (array of 256 unsigned 32-bit values).
    */
    SIMD_API void SimdHistogramConditional(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        const uint8_t * mask, size_t maskStride, uint8_t value, SimdCompareType compareType, uint32_t * histogram);

    /*! @ingroup histogram

        \fn void SimdNormalizedColors(const uint32_t * histogram, uint8_t * colors);

        \short Gets normalized color map for given histogram.

        \param [in] histogram - a pointer to histogram (array of 256 unsigned 32-bit values).
        \param [out] colors - a pointer to the color map (array of 256 unsigned 8-bit values).
    */
    SIMD_API void SimdNormalizedColors(const uint32_t * histogram, uint8_t * colors);

    /*! @ingroup histogram

        \fn void SimdChangeColors(const uint8_t * src, size_t srcStride, size_t width, size_t height, const uint8_t * colors, uint8_t * dst, size_t dstStride);

        \short Changes colors for 8-bit gray image with using of color map.

        The input and output 8-bit gray images must have the same size.
        Algorithm description:
        \verbatim
        for(y = 0; y < height; ++y)
            for(x = 0; x < width; ++x)
                dst[x, y] = colors[src[x, y]];
        \endverbatim

        \note This function has a C++ wrapper Simd::ChangeColors(const View<A> & src, const uint8_t * colors, View<A> & dst).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] srcStride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] colors - a pointer to the color map (array of 256 unsigned 8-bit values).
        \param [out] dst - a pointer to pixels data of output 8-bit gray image.
        \param [in] dstStride - a row size of the output gray image.
    */
    SIMD_API void SimdChangeColors(const uint8_t * src, size_t srcStride, size_t width, size_t height, const uint8_t * colors, uint8_t * dst, size_t dstStride);

    /*! @ingroup histogram

        \fn void SimdNormalizeHistogram(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

        \short Normalizes histogram for 8-bit gray image.

        The input and output 8-bit gray images must have the same size.

        \note This function has a C++ wrapper Simd::NormalizeHistogram(const View<A> & src, View<A> & dst).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] srcStride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] dst - a pointer to pixels data of output 8-bit image with normalized histogram.
        \param [in] dstStride - a row size of the output image.
    */
    SIMD_API void SimdNormalizeHistogram(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

    /*! @ingroup integral

        \fn void SimdIntegral(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * sum, size_t sumStride, uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride, SimdPixelFormatType sumFormat, SimdPixelFormatType sqsumFormat);

        \short Calculates integral images for input 8-bit gray image.

        The function can calculates sum integral image, square sum integral image (optionally) and tilted sum integral image (optionally).
        A integral images must have width and height per unit greater than that of the input image.

        \note This function has a C++ wrappers:
        \n Simd::Integral(const View<A>& src, View<A>& sum),
        \n Simd::Integral(const View<A>& src, View<A>& sum, View<A>& sqsum),
        \n Simd::Integral(const View<A>& src, View<A>& sum, View<A>& sqsum, View<A>& tilted).

        \param [in] src - a pointer to pixels data of input 8-bit gray image.
        \param [in] srcStride - a row size of src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - a pointer to pixels data of 32-bit integer sum image.
        \param [in] sumStride - a row size of sum image (in bytes).
        \param [out] sqsum - a pointer to pixels data of 32-bit integer or 64-bit float point square sum image. It can be NULL.
        \param [in] sqsumStride - a row size of sqsum image (in bytes).
        \param [out] tilted - a pointer to pixels data of 32-bit integer tilted sum image. It can be NULL.
        \param [in] tiltedStride - a row size of tilted image (in bytes).
        \param [in] sumFormat - a format of sum image and tilted image. It can be equal to ::SimdPixelFormatInt32.
        \param [in] sqsumFormat - a format of sqsum image. It can be equal to ::SimdPixelFormatInt32 or ::SimdPixelFormatDouble.
    */
    SIMD_API void SimdIntegral(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        uint8_t * sum, size_t sumStride, uint8_t * sqsum, size_t sqsumStride, uint8_t * tilted, size_t tiltedStride,
        SimdPixelFormatType sumFormat, SimdPixelFormatType sqsumFormat);

    /*! @ingroup other_conversion

        \fn void SimdInterleaveUv(const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * uv, size_t uvStride);

        \short Interleaves 8-bit U and V planar images into one 16-bit UV interleaved image.

        All images must have the same width and height.
        This function used for YUV420P to NV12 conversion.

        \note This function has a C++ wrapper Simd::InterleaveUv(const View<A>& u, const View<A>& v, View<A>& uv).

        \param [in] u - a pointer to pixels data of input 8-bit U planar image.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit V planar image.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] uv - a pointer to pixels data of output 16-bit UV interleaved image.
        \param [in] uvStride - a row size of the uv image.
    */
    SIMD_API void SimdInterleaveUv(const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * uv, size_t uvStride);

    /*! @ingroup other_conversion

        \fn void SimdInterleaveBgr(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride, size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

        \short Interleaves 8-bit Blue, Green and Red planar images into one 24-bit BGR interleaved image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::InterleaveBgr(const View<A>& b, const View<A>& g, const View<A>& r, View<A>& bgr).

        \param [in] b - a pointer to pixels data of input 8-bit Blue planar image.
        \param [in] bStride - a row size of the b image.
        \param [in] g - a pointer to pixels data of input 8-bit Green planar image.
        \param [in] gStride - a row size of the g image.
        \param [in] r - a pointer to pixels data of input 8-bit Red planar image.
        \param [in] rStride - a row size of the r image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR interleaved image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdInterleaveBgr(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride,
        size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup other_conversion

        \fn void SimdInterleaveBgra(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride, const uint8_t * a, size_t aStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride);

        \short Interleaves 8-bit Blue, Green, Red and Alpha planar images into one 32-bit BGRA interleaved image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::InterleaveBgra(const View<A>& b, const View<A>& g, const View<A>& r, const View<A>& a, View<A>& bgra).

        \param [in] b - a pointer to pixels data of input 8-bit Blue planar image.
        \param [in] bStride - a row size of the b image.
        \param [in] g - a pointer to pixels data of input 8-bit Green planar image.
        \param [in] gStride - a row size of the g image.
        \param [in] r - a pointer to pixels data of input 8-bit Red planar image.
        \param [in] rStride - a row size of the r image.
        \param [in] a - a pointer to pixels data of input 8-bit Alpha planar image.
        \param [in] aStride - a row size of the a image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA interleaved image.
        \param [in] bgraStride - a row size of the bgr image.
    */
    SIMD_API void SimdInterleaveBgra(const uint8_t * b, size_t bStride, const uint8_t * g, size_t gStride, const uint8_t * r, size_t rStride, const uint8_t * a, size_t aStride,
        size_t width, size_t height, uint8_t * bgra, size_t bgraStride);

    /*! @ingroup other_filter

        \fn void SimdMeanFilter3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs an averaging with window 3x3.

        For every point:
        \verbatim
        dst[x, y] = (src[x-1, y-1] + src[x, y-1] + src[x+1, y-1] +
                     src[x-1, y] + src[x, y] + src[x+1, y] +
                     src[x-1, y+1] + src[x, y+1] + src[x+1, y+1] + 4) / 9;
        \endverbatim

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrapper Simd::MeanFilter3x3(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of source image.
        \param [in] srcStride - a row size of the src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of destination image.
        \param [in] dstStride - a row size of the dst image.
    */
    SIMD_API void SimdMeanFilter3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup median_filter

        \fn void SimdMedianFilterRhomb3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs median filtration of input image (filter window is a rhomb 3x3).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::MedianFilterRhomb3x3(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of original input image.
        \param [in] srcStride - a row size of src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of filtered output image.
        \param [in] dstStride - a row size of dst image.
    */
    SIMD_API void SimdMedianFilterRhomb3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup median_filter

        \fn void SimdMedianFilterRhomb5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs median filtration of input image (filter window is a rhomb 5x5).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::MedianFilterRhomb5x5(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of original input image.
        \param [in] srcStride - a row size of src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of filtered output image.
        \param [in] dstStride - a row size of dst image.
    */
    SIMD_API void SimdMedianFilterRhomb5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup median_filter

        \fn void SimdMedianFilterSquare3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs median filtration of input image (filter window is a square 3x3).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::MedianFilterSquare3x3(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of original input image.
        \param [in] srcStride - a row size of src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of filtered output image.
        \param [in] dstStride - a row size of dst image.
    */
    SIMD_API void SimdMedianFilterSquare3x3(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup median_filter

        \fn void SimdMedianFilterSquare5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride);

        \short Performs median filtration of input image (filter window is a square 5x5).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::MedianFilterSquare5x5(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of original input image.
        \param [in] srcStride - a row size of src image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of filtered output image.
        \param [in] dstStride - a row size of dst image.
    */
    SIMD_API void SimdMedianFilterSquare5x5(const uint8_t * src, size_t srcStride, size_t width, size_t height,
        size_t channelCount, uint8_t * dst, size_t dstStride);

    /*! @ingroup operation

        \fn void SimdOperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type);

        \short Performs given operation between two images.

        All images must have the same width, height and format (8-bit gray, 16-bit UV (UV plane of NV12 pixel format), 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::OperationBinary8u(const View<A>& a, const View<A>& b, View<A>& dst, SimdOperationBinary8uType type).

        \param [in] a - a pointer to pixels data of the first input image.
        \param [in] aStride - a row size of the first image.
        \param [in] b - a pointer to pixels data of the second input image.
        \param [in] bStride - a row size of the second image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] channelCount - a channel count.
        \param [out] dst - a pointer to pixels data of output image.
        \param [in] dstStride - a row size of dst image.
        \param [in] type - a type of operation (see ::SimdOperationBinary8uType).
    */
    SIMD_API void SimdOperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
        size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type);

    /*! @ingroup resizing

        \fn void SimdReduceColor2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

        \short Performs reducing and Gaussian blurring (in two time) a 8-bit channel color image with using window 2x2.

        For input and output image must be performed: dstWidth = (srcWidth + 1)/2,  dstHeight = (srcHeight + 1)/2.

        For all points:
        \verbatim
        dst[x, y, c] = (src[2*x, 2*y, c] + src[2*x, 2*y + 1, c] + src[2*x + 1, 2*y, c] + src[2*x + 1, 2*y + 1, c] + 2)/4;
        \endverbatim

        \note This function has a C++ wrappers: Simd::Reduce2x2(const View<A> & src, View<A> & dst).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
        \param [in] channelCount - a nmber of channels for input and output images.
    */
    SIMD_API void SimdReduceColor2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

    /*! @ingroup resizing

        \fn void SimdReduceGray2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        \short Performs reducing and Gaussian blurring (in two time) a 8-bit gray image with using window 2x2.

        For input and output image must be performed: dstWidth = (srcWidth + 1)/2,  dstHeight = (srcHeight + 1)/2.

        For all points:
        \verbatim
        dst[x, y] = (src[2*x, 2*y] + src[2*x, 2*y + 1] + src[2*x + 1, 2*y] + src[2*x + 1, 2*y + 1] + 2)/4;
        \endverbatim

        \note This function has a C++ wrappers: Simd::ReduceGray2x2(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
    */
    SIMD_API void SimdReduceGray2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

    /*! @ingroup resizing

        \fn void SimdReduceGray3x3(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

        \short Performs reducing and Gaussian blurring (in two time) a 8-bit gray image with using window 3x3.

        For input and output image must be performed: dstWidth = (srcWidth + 1)/2,  dstHeight = (srcHeight + 1)/2.

        For every point:
        \verbatim
        dst[x, y] = (src[2*x-1, 2*y-1] + 2*src[2*x, 2*y-1] + src[2*x+1, 2*y-1] +
                  2*(src[2*x-1, 2*y]   + 2*src[2*x, 2*y]   + src[2*x+1, 2*y]) +
                     src[2*x-1, 2*y+1] + 2*src[2*x, 2*y+1] + src[2*x+1, 2*y+1] + compensation ? 8 : 0) / 16;
        \endverbatim

        \note This function has a C++ wrappers: Simd::ReduceGray3x3(const View<A>& src, View<A>& dst, bool compensation).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
        \param [in] compensation - a flag of compensation of rounding.
    */
    SIMD_API void SimdReduceGray3x3(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

    /*! @ingroup resizing

        \fn void SimdReduceGray4x4(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        \short Performs reducing and Gaussian blurring (in two time) a 8-bit gray image with using window 4x4.

        For input and output image must be performed: dstWidth = (srcWidth + 1)/2,  dstHeight = (srcHeight + 1)/2.

        For every point:
        \verbatim
        dst[x, y] = (src[2*x-1, 2*y-1] + 3*src[2*x, 2*y-1] + 3*src[2*x+1, 2*y-1] + src[2*x+2, 2*y-1]
                  3*(src[2*x-1, 2*y]   + 3*src[2*x, 2*y]   + 3*src[2*x+1, 2*y]   + src[2*x+2, 2*y]) +
                  3*(src[2*x-1, 2*y+1] + 3*src[2*x, 2*y+1] + 3*src[2*x+1, 2*y+1] + src[2*x+2, 2*y+1]) +
                     src[2*x-1, 2*y+2] + 3*src[2*x, 2*y+2] + 3*src[2*x+1, 2*y+2] + src[2*x+2, 2*y+2] + 32) / 64;
        \endverbatim

        \note This function has a C++ wrappers: Simd::ReduceGray4x4(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
    */
    SIMD_API void SimdReduceGray4x4(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

    /*! @ingroup resizing

        \fn void SimdReduceGray5x5(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

        \short Performs reducing and Gaussian blurring (in two time) a 8-bit gray image with using window 5x5.

        For input and output image must be performed: dstWidth = (srcWidth + 1)/2,  dstHeight = (srcHeight + 1)/2.

        For every point:
        \verbatim
        dst[x, y] = (
               src[2*x-2, 2*y-2] + 4*src[2*x-1, 2*y-2] + 6*src[2*x, 2*y-2] + 4*src[2*x+1, 2*y-2] + src[2*x+2, 2*y-2] +
            4*(src[2*x-2, 2*y-1] + 4*src[2*x-1, 2*y-1] + 6*src[2*x, 2*y-1] + 4*src[2*x+1, 2*y-1] + src[2*x+2, 2*y-1]) +
            6*(src[2*x-2, 2*y]   + 4*src[2*x-1, 2*y]   + 6*src[2*x, 2*y]   + 4*src[2*x+1, 2*y]   + src[2*x+2, 2*y]) +
            4*(src[2*x-2, 2*y+1] + 4*src[2*x-1, 2*y+1] + 6*src[2*x, 2*y+1] + 4*src[2*x+1, 2*y+1] + src[2*x+2, 2*y+1]) +
               src[2*x-2, 2*y+2] + 4*src[2*x-1, 2*y+2] + 6*src[2*x, 2*y+2] + 4*src[2*x+1, 2*y+2] + src[2*x+2, 2*y+2] +
            compensation ? 128 : 0) / 256;
        \endverbatim

        \note This function has a C++ wrappers: Simd::ReduceGray5x5(const Viewc<A>& src, View<A>& dst, bool compensation).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
        \param [in] compensation - a flag of compensation of rounding.
    */
    SIMD_API void SimdReduceGray5x5(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

    /*! @ingroup resizing

        \fn void SimdResizeBilinear(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

        \short Performs resizing of input image with using bilinear interpolation.

        All images must have the same format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function has a C++ wrappers: Simd::ResizeBilinear(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the reduced output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
        \param [in] channelCount - a channel count.
    */
    SIMD_API void SimdResizeBilinear(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

    /*! @ingroup resizing

        \fn void * SimdResizerInit(size_t srcX, size_t srcY, size_t dstX, size_t dstY, size_t channels, SimdResizeChannelType type, SimdResizeMethodType method);

        \short Creates resize context.

        \param [in] srcX - a width of the input image.
        \param [in] srcY - a height of the input image.
        \param [in] dstX - a width of the output image.
        \param [in] dstY - a height of the output image.
        \param [in] channels - a channel number of input and output image.
        \param [in] type - a type of input and output image channel.
        \param [in] method - a method used in order to resize image.
        \return a pointer to resize context. On error it returns NULL.
                This pointer is used in functions ::SimdResizerRun.
                It must be released with using of function ::SimdRelease.
    */
    SIMD_API void * SimdResizerInit(size_t srcX, size_t srcY, size_t dstX, size_t dstY, size_t channels, SimdResizeChannelType type, SimdResizeMethodType method);

    /*! @ingroup resizing

        \fn void SimdResizerRun(const void * resizer, const uint8_t * src, size_t srcStride, uint8_t * dst, size_t dstStride);

        \short Performs image resizing.

        \param [in] resizer - a resize context. It must be created by function ::SimdResizerInit and released by function ::SimdRelease.
        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcStride - a row size (in bytes) of the input image.
        \param [out] dst - a pointer to pixels data of the resized output image.
        \param [in] dstStride - a row size (in bytes) of the output image.
    */
    SIMD_API void SimdResizerRun(const void * resizer, const uint8_t * src, size_t srcStride, uint8_t * dst, size_t dstStride);

    /*! @ingroup segmentation

        \fn void SimdSegmentationChangeIndex(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t oldIndex, uint8_t newIndex);

        \short Changes certain index in mask.

        Mask must has 8-bit gray pixel format.

        For every point:
        \verbatim
        if(mask[i] == oldIndex)
            mask[i] = newIndex;
        \endverbatim

        \note This function has a C++ wrappers: Simd::SegmentationChangeIndex(View<A> & mask, uint8_t oldIndex, uint8_t newIndex).

        \param [in, out] mask - a pointer to pixels data of 8-bit gray mask image.
        \param [in] stride - a row size of the mask image.
        \param [in] width - a mask width.
        \param [in] height - a mask height.
        \param [in] oldIndex - a mask old index.
        \param [in] newIndex - a mask new index.
    */
    SIMD_API void SimdSegmentationChangeIndex(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t oldIndex, uint8_t newIndex);

    /*! @ingroup segmentation

        \fn void SimdSegmentationFillSingleHoles(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index);

        \short Fill single holes in mask.

        Mask must has 8-bit gray pixel format.

        \note This function has a C++ wrappers: Simd::SegmentationFillSingleHoles(View<A> & mask, uint8_t index).

        \param [in, out] mask - a pointer to pixels data of 8-bit gray mask image.
        \param [in] stride - a row size of the mask image.
        \param [in] width - an mask width.
        \param [in] height - an mask height.
        \param [in] index - a mask index.
    */
    SIMD_API void SimdSegmentationFillSingleHoles(uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index);

    /*! @ingroup segmentation

        \fn void SimdSegmentationPropagate2x2(const uint8_t * parent, size_t parentStride, size_t width, size_t height, uint8_t * child, size_t childStride, const uint8_t * difference, size_t differenceStride, uint8_t currentIndex, uint8_t invalidIndex, uint8_t emptyIndex, uint8_t differenceThreshold);

        \short Propagates mask index from parent (upper) to child (lower) level of mask pyramid with using 2x2 scan window.

        For parent and child image must be performed: parentWidth = (childWidth + 1)/2, parentHeight = (childHeight + 1)/2.
        All images must have 8-bit gray pixel format. Size of different image is equal to child image.

        \note This function has a C++ wrappers: Simd::SegmentationPropagate2x2(const View<A> & parent, View<A> & child, const View<A> & difference, uint8_t currentIndex, uint8_t invalidIndex, uint8_t emptyIndex, uint8_t thresholdDifference).

        \param [in] parent - a pointer to pixels data of 8-bit gray parent mask image.
        \param [in] parentStride - a row size of the parent mask image.
        \param [in] width - a parent mask width.
        \param [in] height - a parent mask height.
        \param [in, out] child - a pointer to pixels data of 8-bit gray child mask image.
        \param [in] childStride - a row size of the child mask image.
        \param [in] difference - a pointer to pixels data of 8-bit gray difference image.
        \param [in] differenceStride - a row size of the difference image.
        \param [in] currentIndex - propagated mask index.
        \param [in] invalidIndex - invalid mask index.
        \param [in] emptyIndex - empty mask index.
        \param [in] differenceThreshold - a difference threshold for conditional index propagating.
    */
    SIMD_API void SimdSegmentationPropagate2x2(const uint8_t * parent, size_t parentStride, size_t width, size_t height,
        uint8_t * child, size_t childStride, const uint8_t * difference, size_t differenceStride,
        uint8_t currentIndex, uint8_t invalidIndex, uint8_t emptyIndex, uint8_t differenceThreshold);

    /*! @ingroup segmentation

        \fn void SimdSegmentationShrinkRegion(const uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index, ptrdiff_t * left, ptrdiff_t * top, ptrdiff_t * right, ptrdiff_t * bottom);

        \short Finds actual region of mask index location.

        Mask must has 8-bit gray pixel format.

        \note This function has a C++ wrappers: Simd::SegmentationShrinkRegion(const View<A> & mask, uint8_t index, Rectangle<ptrdiff_t> & rect).

        \param [in] mask - a pointer to pixels data of 8-bit gray mask image.
        \param [in] stride - a row size of the mask image.
        \param [in] width - an mask width.
        \param [in] height - an mask height.
        \param [in] index - a mask index.
        \param [in, out] left - a pointer to left side.
        \param [in, out] top - a pointer to top side.
        \param [in, out] right - a pointer to right side.
        \param [in, out] bottom - a pointer to bottom side.
    */
    SIMD_API void SimdSegmentationShrinkRegion(const uint8_t * mask, size_t stride, size_t width, size_t height, uint8_t index,
        ptrdiff_t * left, ptrdiff_t * top, ptrdiff_t * right, ptrdiff_t * bottom);

    /*! @ingroup sobel_filter

        \fn void SimdSobelDx(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

        \short Calculates Sobel's filter along x axis.

        All images must have the same width and height. Input image must has 8-bit gray format, output image must has 16-bit integer format.

        For every point:
        \n dst[x, y] = (src[x+1,y-1] + 2*src[x+1, y] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x-1, y] + src[x-1, y+1]).

        \note This function has a C++ wrappers: Simd::SobelDx(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] dst - a pointer to pixels data of the output image.
        \param [in] dstStride - a row size of the output image (in bytes).
    */
    SIMD_API void SimdSobelDx(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

    /*! @ingroup sobel_filter

        \fn void SimdSobelDxAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

        \short Calculates absolute value of Sobel's filter along x axis.

        All images must have the same width and height. Input image must has 8-bit gray format, output image must has 16-bit integer format.

        For every point:
        \verbatim
        dst[x, y] = (src[x+1,y-1] + 2*src[x+1, y] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x-1, y] + src[x-1, y+1]).
        \endverbatim

        \note This function has a C++ wrappers: Simd::SobelDxAbs(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] dst - a pointer to pixels data of the output image.
        \param [in] dstStride - a row size of the output image (in bytes).
    */
    SIMD_API void SimdSobelDxAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

    /*! @ingroup sobel_statistic

        \fn void SimdSobelDxAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

        \short Calculates sum of absolute value of Sobel's filter along x axis.

        Input image must has 8-bit gray format.

        For every point:
        \verbatim
        dst[x, y] = abs((src[x+1,y-1] + 2*src[x+1, y] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x-1, y] + src[x-1, y+1])).
        \endverbatim

        \note This function has a C++ wrappers: Simd::SobelDxAbsSum(const View<A>& src, uint64_t & sum).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - a pointer to unsigned 64-bit integer value with result sum.
    */
    SIMD_API void SimdSobelDxAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

    /*! @ingroup sobel_filter

        \fn void SimdSobelDy(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

        \short Calculates Sobel's filter along y axis.

        All images must have the same width and height. Input image must has 8-bit gray format, output image must has 16-bit integer format.

        For every point:
        \verbatim
        dst[x, y] = (src[x-1,y+1] + 2*src[x, y+1] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x, y-1] + src[x+1, y-1]);
        \endverbatim

        \note This function has a C++ wrappers: Simd::SobelDy(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] dst - a pointer to pixels data of the output image.
        \param [in] dstStride - a row size of the output image (in bytes).
    */
    SIMD_API void SimdSobelDy(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

    /*! @ingroup sobel_filter

        \fn void SimdSobelDyAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

        \short Calculates absolute value of Sobel's filter along y axis.

        All images must have the same width and height. Input image must has 8-bit gray format, output image must has 16-bit integer format.

        For every point:
        \verbatim
        dst[x, y] = abs((src[x-1,y+1] + 2*src[x, y+1] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x, y-1] + src[x+1, y-1]));
        \endverbatim

        \note This function has a C++ wrappers: Simd::SobelDyAbs(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] dst - a pointer to pixels data of the output image.
        \param [in] dstStride - a row size of the output image (in bytes).
    */
    SIMD_API void SimdSobelDyAbs(const uint8_t * src, size_t srcStride, size_t width, size_t height, uint8_t * dst, size_t dstStride);

    /*! @ingroup sobel_statistic

        \fn void SimdSobelDyAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

        \short Calculates sum of absolute value of Sobel's filter along y axis.

        Input image must has 8-bit gray format.

        For every point:
        \verbatim
        sum += abs((src[x-1,y+1] + 2*src[x, y+1] + src[x+1, y+1]) - (src[x-1,y-1] + 2*src[x, y-1] + src[x+1, y-1]));
        \endverbatim

        \note This function has a C++ wrappers: Simd::SobelDyAbsSum(const View<A>& src, uint64_t & sum).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - a pointer to unsigned 64-bit integer value with result sum.
    */
    SIMD_API void SimdSobelDyAbsSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

    /*! @ingroup correlation

        \fn void SimdSquaredDifferenceSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum);

        \short Calculates sum of squared differences for two 8-bit gray images.

        All images must have the same width and height.

        For every point:
        \verbatim
        sum += (a[i] - b[i])*(a[i] - b[i]);
        \endverbatim

        \note This function has a C++ wrappers: Simd::SquaredDifferenceSum(const View<A>& a, const View<A>& b, uint64_t & sum).

        \param [in] a - a pointer to pixels data of the first image.
        \param [in] aStride - a row size of the first image.
        \param [in] b - a pointer to pixels data of the second image.
        \param [in] bStride - a row size of the second image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - a pointer to unsigned 64-bit integer value with result sum.
    */
    SIMD_API void SimdSquaredDifferenceSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
        size_t width, size_t height, uint64_t * sum);

    /*! @ingroup correlation

        \fn void SimdSquaredDifferenceSumMasked(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, const uint8_t * mask, size_t maskStride, uint8_t index, size_t width, size_t height, uint64_t * sum);

        \short Calculates sum of squared differences for two images with using mask.

        All images must have the same width, height and format (8-bit gray).

        For every point:
        \verbatim
        if(mask[i] == index)
            sum += (a[i] - b[i])*(a[i] - b[i]);
        \endverbatim

        \note This function has a C++ wrappers: Simd::SquaredDifferenceSum(const View<A>& a, const View<A>& b, const View<A>& mask, uint8_t index, uint64_t & sum).

        \param [in] a - a pointer to pixels data of the first image.
        \param [in] aStride - a row size of the first image.
        \param [in] b - a pointer to pixels data of the second image.
        \param [in] bStride - a row size of the second image.
        \param [in] mask - a pointer to pixels data of the mask image.
        \param [in] maskStride - a row size of the mask image.
        \param [in] index - a mask index.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - a pointer to unsigned 64-bit integer value with result sum.
    */
    SIMD_API void SimdSquaredDifferenceSumMasked(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
        const uint8_t * mask, size_t maskStride, uint8_t index, size_t width, size_t height, uint64_t * sum);

    /*! @ingroup correlation

        \fn void SimdSquaredDifferenceSum32f(const float * a, const float * b, size_t size, float * sum);

        \short Calculates sum of squared differences for two 32-bit float arrays.

        All arrays must have the same size.

        For every element:
        \verbatim
        sum += (a[i] - b[i])*(a[i] - b[i]);
        \endverbatim

        \param [in] a - a pointer to the first array.
        \param [in] b - a pointer to the second array.
        \param [in] size - a size of arrays.
        \param [out] sum - a sum of squared differences.
    */
    SIMD_API void SimdSquaredDifferenceSum32f(const float * a, const float * b, size_t size, float * sum);

    /*! @ingroup correlation

        \fn void SimdSquaredDifferenceKahanSum32f(const float * a, const float * b, size_t size, float * sum);

        \short Calculates sum of squared differences for two 32-bit float arrays with using Kahan summation algorithm.

        All arrays must have the same size.

        Algorithm pseudo code:
        \verbatim
        sum = 0; corr = 0;
        for(i = 0; i < size; ++i)
        {
            diff = (a[i] - b[i])*(a[i] - b[i]) - corr;
            temp = sum + diff;
            corr = (temp - sum) - diff;
            sum = temp;
        }
        \endverbatim

        \param [in] a - a pointer to the first array.
        \param [in] b - a pointer to the second array.
        \param [in] size - a size of arrays.
        \param [out] sum - a sum of squared differences.
    */
    SIMD_API void SimdSquaredDifferenceKahanSum32f(const float * a, const float * b, size_t size, float * sum);

    /*! @ingroup other_statistic

        \fn void SimdGetStatistic(const uint8_t * src, size_t stride, size_t width, size_t height, uint8_t * min, uint8_t * max, uint8_t * average);

        \short Finds minimal, maximal and average pixel values for given image.

        The image must has 8-bit gray format.

        \note This function has a C++ wrappers: Simd::GetStatistic(const View<A>& src, uint8_t & min, uint8_t & max, uint8_t & average).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] min - a pointer to unsigned 8-bit integer value with found minimal pixel value.
        \param [out] max - a pointer to unsigned 8-bit integer value with found maximal pixel value.
        \param [out] average - a pointer to unsigned 8-bit integer value with found average pixel value.
    */
    SIMD_API void SimdGetStatistic(const uint8_t * src, size_t stride, size_t width, size_t height,
        uint8_t * min, uint8_t * max, uint8_t * average);

    /*! @ingroup row_statistic

        \fn void SimdGetRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

        \short Calculate sums of rows for given 8-bit gray image.

        For all rows:
        \verbatim
        for(x = 0; x < width; ++x)
            sums[y] += src[x, y];
        \endverbatim

        \note This function has a C++ wrappers: Simd::GetRowSums(const View<A>& src, uint32_t * sums).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sums - a pointer to array of unsigned 32-bit integers result sums of rows. It length must be equal to image height.
    */
    SIMD_API void SimdGetRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

    /*! @ingroup col_statistic

        \fn void SimdGetColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

        \short Calculate sums of columns for given 8-bit gray image.

        For all columns:
        \verbatim
        for(y = 0; y < height; ++y)
            sums[x] += src[x, y];
        \endverbatim

        \note This function has a C++ wrappers: Simd::GetColSums(const View<A>& src, uint32_t * sums).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sums - a pointer to array of unsigned 32-bit integers result sums of columns. It length must be equal to image width.
    */
    SIMD_API void SimdGetColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

    /*! @ingroup row_statistic

        \fn void SimdGetAbsDyRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

        \short Calculate sums of absolute derivate along y axis for rows for given 8-bit gray image.

        For all rows except the last:
        \verbatim
        for(x = 0; x < width; ++x)
            sums[y] += abs(src[x, y+1] - src[x, y]);
        \endverbatim
        For the last row:
        \verbatim
        sums[height-1] = 0;
        \endverbatim

        \note This function has a C++ wrappers: Simd::GetAbsDyRowSums(const View<A>& src, uint32_t * sums).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sums - a pointer to array of unsigned 32-bit integers result sums. It length must be equal to image height.
    */
    SIMD_API void SimdGetAbsDyRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

    /*! @ingroup col_statistic

        \fn void SimdGetAbsDxColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

        \short Calculate sums of absolute derivate along x axis for columns for given 8-bit gray image.

        For all columns except the last:
        \verbatim
        for(y = 0; y < height; ++y)
            sums[y] += abs(src[x+1, y] - src[x, y]);
        \endverbatim
        For the last column:
        \verbatim
        sums[width-1] = 0;
        \endverbatim

        \note This function has a C++ wrappers: Simd::GetAbsDxColSums(const View<A>& src, uint32_t * sums).

        \param [in] src - a pointer to pixels data of the input image.
        \param [in] stride - a row size of the input image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sums - a pointer to array of unsigned 32-bit integers result columns. It length must be equal to image width.
    */
    SIMD_API void SimdGetAbsDxColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums);

    /*! @ingroup other_statistic

        \fn void SimdValueSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

        \short Gets sum of value of pixels for gray 8-bit image.

        \note This function has a C++ wrappers: Simd::ValueSum(const View<A>& src, uint64_t & sum).

        \param [in] src - a pointer to pixels data of the image.
        \param [in] stride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - the result sum.
    */
    SIMD_API void SimdValueSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

    /*! @ingroup other_statistic

        \fn void SimdSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

        \short Gets sum of squared value of pixels for gray 8-bit image .

        \note This function has a C++ wrappers: Simd::SquareSum(const View<A>& src, uint64_t & sum).

        \param [in] src - a pointer to pixels data of the image.
        \param [in] stride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] sum - the result sum.
    */
    SIMD_API void SimdSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum);

        /*! @ingroup other_statistic

        \fn void SimdValueSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * valueSum, uint64_t * squareSum);

        \short Gets sum and squared sum of value of pixels for gray 8-bit image.

        \note This function has a C++ wrappers: Simd::ValueSquareSum(const View<A>& src, uint64_t & valueSum, uint64_t & squareSum).

        \param [in] src - a pointer to pixels data of the image.
        \param [in] stride - a row size of the image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] valueSum - the result value sum.
        \param [out] squareSum - the result square sum.
    */
    SIMD_API void SimdValueSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * valueSum, uint64_t * squareSum);

    /*! @ingroup other_statistic

        \fn void SimdCorrelationSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum);

        \short Gets sum of pixel correlation for two gray 8-bit images.

        For all points:
        \verbatim
        sum += a[i]*b[i];
        \endverbatim

        All images must have the same width and height and 8-bit gray pixel format.

        \note This function has a C++ wrappers: Simd::CorrelationSum(const View<A> & a, const View<A> & b, uint64_t & sum).

        \param [in] a - a pointer to pixels data of the first image.
        \param [in] aStride - a row size of the first image.
        \param [in] b - a pointer to pixels data of the second image.
        \param [in] bStride - a row size of the second image.
        \param [in] width - an images width.
        \param [in] height - an images height.
        \param [out] sum - a pointer to result sum.
    */
    SIMD_API void SimdCorrelationSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum);

    /*! @ingroup resizing

        \fn void SimdStretchGray2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        \short Stretches input 8-bit gray image in two times.

        \note This function has a C++ wrappers: Simd::StretchGray2x2(const View<A>& src, View<A>& dst).

        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcWidth - a width of the input image.
        \param [in] srcHeight - a height of the input image.
        \param [in] srcStride - a row size of the input image.
        \param [out] dst - a pointer to pixels data of the stretched output image.
        \param [in] dstWidth - a width of the output image.
        \param [in] dstHeight - a height of the output image.
        \param [in] dstStride - a row size of the output image.
    */
    SIMD_API void SimdStretchGray2x2(const uint8_t * src, size_t srcWidth, size_t srcHeight, size_t srcStride,
        uint8_t * dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuva420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, const uint8_t * a, size_t aStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride);

        \short Converts YUVA420P image to 32-bit BGRA image.

        The input Y, A and output BGRA images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrappers: Simd::Yuva420pToBgra(const View<A>& y, const View<A>& u, const View<A>& v, const View<A> & a, View<A>& bgra).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] a - a pointer to pixels data of input 8-bit image with alpha channel.
        \param [in] aStride - a row size of the a image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
    */
    SIMD_API void SimdYuva420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        const uint8_t * a, size_t aStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv420pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

        \short Converts YUV420P image to 24-bit BGR image.

        The input Y and output BGR images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrappers: Simd::Yuv420pToBgr(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgr);

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdYuv420pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv422pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

        \short Converts YUV422P image to 24-bit BGR image.

        The input Y and output BGR images must have the same width and height.
        The input U and V images must have the same width and height (their width is equal to half width of Y component).

        \note This function has a C++ wrappers: Simd::Yuv422pToBgr(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgr);

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdYuv422pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv444pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

        \short Converts YUV444P image to 24-bit BGR image.

        The input Y, U, V and output BGR images must have the same width and height.

        \note This function has a C++ wrappers: Simd::Yuv444pToBgr(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgr);

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdYuv444pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgr, size_t bgrStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts YUV420P image to 32-bit BGRA image.

        The input Y and output BGRA images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrappers: Simd::Yuv420pToBgra(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgra, uint8_t alpha).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdYuv420pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv422pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts YUV422P image to 32-bit BGRA image.

        The input Y and output BGRA images must have the same width and height.
        The input U and V images must have the same width and height (their width is equal to half width of Y component).

        \note This function has a C++ wrappers: Simd::Yuv422pToBgra(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgra, uint8_t alpha).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdYuv422pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv444pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts YUV444P image to 32-bit BGRA image.

        The input Y, U, V and output BGRA images must have the same width and height.

        \note This function has a C++ wrappers: Simd::Yuv444pToBgra(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& bgra, uint8_t alpha).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdYuv444pToBgra(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv444pToHsl(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * hsl, size_t hslStride);

        \short Converts YUV444P image to 24-bit HSL(Hue, Saturation, Lightness) image.

        The input Y, U, V and output HSL images must have the same width and height.

        \note This function has a C++ wrappers: Simd::Yuv444pToHsl(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& hsl).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] hsl - a pointer to pixels data of output 24-bit HSL image.
        \param [in] hslStride - a row size of the hsl image.
    */
    SIMD_API void SimdYuv444pToHsl(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * hsl, size_t hslStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv444pToHsv(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * hsv, size_t hsvStride);

        \short Converts YUV444P image to 24-bit HSV(Hue, Saturation, Value) image.

        The input Y, U, V and output HSV images must have the same width and height.

        \note This function has a C++ wrappers: Simd::Yuv444pToHsv(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& hsv).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] hsv - a pointer to pixels data of output 24-bit HSV image.
        \param [in] hsvStride - a row size of the hsv image.
    */
    SIMD_API void SimdYuv444pToHsv(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * hsv, size_t hsvStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv420pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * hue, size_t hueStride);

        \short Converts YUV420P image to 8-bit image with Hue component of HSV or HSL color space.

        The input Y and output Hue images must have the same width and height.
        The input U and V images must have the same width and height (half size relative to Y component).

        \note This function has a C++ wrappers: Simd::Yuv420pToHue(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& hue).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] hue - a pointer to pixels data of output 8-bit Hue image.
        \param [in] hueStride - a row size of the hue image.
    */
    SIMD_API void SimdYuv420pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * hue, size_t hueStride);

    /*! @ingroup yuv_conversion

        \fn void SimdYuv444pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride, size_t width, size_t height, uint8_t * hue, size_t hueStride);

        \short Converts YUV444P image to 8-bit image with Hue component of HSV or HSL color space.

        The input Y, U, V and output Hue images must have the same width and height.

        \note This function has a C++ wrappers: Simd::Yuv444pToHue(const View<A>& y, const View<A>& u, const View<A>& v, View<A>& hue).

        \param [in] y - a pointer to pixels data of input 8-bit image with Y color plane.
        \param [in] yStride - a row size of the y image.
        \param [in] u - a pointer to pixels data of input 8-bit image with U color plane.
        \param [in] uStride - a row size of the u image.
        \param [in] v - a pointer to pixels data of input 8-bit image with V color plane.
        \param [in] vStride - a row size of the v image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [out] hue - a pointer to pixels data of output 8-bit Hue image.
        \param [in] hueStride - a row size of the hue image.
    */
    SIMD_API void SimdYuv444pToHue(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
        size_t width, size_t height, uint8_t * hue, size_t hueStride);

    // ViSP custom SIMD code
    SIMD_API void SimdImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

    SIMD_API void SimdImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif//__SimdLib_h__
