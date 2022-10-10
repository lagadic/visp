/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2021 Yermalayeu Ihar,
*               2014-2019 Antonenka Mikhail,
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
    SimdCpuInfoSse2, /*!< Availability of SSE2 (x86). */
    SimdCpuInfoSse41, /*!< Availability of SSE4.1 (x86). */
    SimdCpuInfoAvx, /*!< Availability of AVX (x86). */
    SimdCpuInfoAvx2, /*!< Availability of AVX2 (x86). */
    SimdCpuInfoAvx512f, /*!< Availability of AVX-512F (x86). */
    SimdCpuInfoAvx512bw, /*!< Availability of AVX-512BW (x86). */
    SimdCpuInfoVmx, /*!< Availability of VMX or Altivec (PowerPC). */
    SimdCpuInfoVsx, /*!< Availability of VSX (PowerPC). */
    SimdCpuInfoNeon, /*!< Availability of NEON (ARM). */
} SimdCpuInfoType;

/*! @ingroup c_types
    Describes formats of image file. It is used in functions ::SimdImageSaveToMemory and ::SimdImageSaveToFile.
*/
typedef enum
{
    /*! An undefined image file format (format auto choice). */
    SimdImageFileUndefined = 0,
    /*! A PGM (Portable Gray Map) text (P2) image file format. */
    SimdImageFilePgmTxt,
    /*! A PGM (Portable Gray Map) binary (P5) image file format. */
    SimdImageFilePgmBin,
    /*! A PGM (Portable Pixel Map) text (P3) image file format. */
    SimdImageFilePpmTxt,
    /*! A PGM (Portable Pixel Map) binary (P6) image file format. */
    SimdImageFilePpmBin,
    /*! A PNG (Portable Network Graphics) image file format. */
    SimdImageFilePng,
    /*! A JPEG (Joint Photographic Experts Group) image file format. */
    SimdImageFileJpeg,
} SimdImageFileType;

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
    /*! A 24-bit (3 8-bit channels) RGB (Red, Green, Blue) pixel format. */
    SimdPixelFormatRgb24,
    /*! A 32-bit (4 8-bit channels) RGBA (Red, Green, Blue, Alpha) pixel format. */
    SimdPixelFormatRgba32,
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
    /*! 16-bit integer channel type.  */
    SimdResizeChannelShort,
    /*! 32-bit float channel type.  */
    SimdResizeChannelFloat,
} SimdResizeChannelType;

/*! @ingroup resizing
    Describes methods used in order to resize image.
*/
typedef enum
{
    /*! Nearest method. */
    SimdResizeMethodNearest,
    /*! Nearest Pytorch compatible method. */
    SimdResizeMethodNearestPytorch,
    /*! Bilinear method. */
    SimdResizeMethodBilinear,
    /*! Bilinear Caffe compatible method. It is relevant only for ::SimdResizeChannelFloat (32-bit float channel type).*/
    SimdResizeMethodBilinearCaffe,
    /*! Bilinear Pytorch compatible method. It is relevant only for ::SimdResizeChannelFloat (32-bit float channel type).*/
    SimdResizeMethodBilinearPytorch,
    /*! Bicubic method. */
    SimdResizeMethodBicubic,
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
            std::cout << "SSE4.1: " << (SimdCpuInfo(SimdCpuInfoSse41) ? "Yes" : "No") << std::endl;
            std::cout << "AVX: " << (SimdCpuInfo(SimdCpuInfoAvx) ? "Yes" : "No") << std::endl;
            std::cout << "AVX2: " << (SimdCpuInfo(SimdCpuInfoAvx2) ? "Yes" : "No") << std::endl;
            std::cout << "AVX-512BW: " << (SimdCpuInfo(SimdCpuInfoAvx512bw) ? "Yes" : "No") << std::endl;
            std::cout << "AVX-512VNNI: " << (SimdCpuInfo(SimdCpuInfoAvx512vnni) ? "Yes" : "No") << std::endl;
            std::cout << "AVX-512BF16: " << (SimdCpuInfo(SimdCpuInfoAvx512bf16) ? "Yes" : "No") << std::endl;
            std::cout << "AMX: " << (SimdCpuInfo(SimdCpuInfoAmx) ? "Yes" : "No") << std::endl;
            std::cout << "PowerPC-Altivec: " << (SimdCpuInfo(SimdCpuInfoVmx) ? "Yes" : "No") << std::endl;
            std::cout << "PowerPC-VSX: " << (SimdCpuInfo(SimdCpuInfoVsx) ? "Yes" : "No") << std::endl;
            std::cout << "ARM-NEON: " << (SimdCpuInfo(SimdCpuInfoNeon) ? "Yes" : "No") << std::endl;
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

        \short Gets alignment required for the most productive work of Simd Library.

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

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToBgr(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * bgr, size_t bgrStride);

        \short Converts 32-bit BGRA image to 24-bit BGR image. Also it can be used for 32-bit RGBA to 24-bit RGB conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::BgraToBgr(const View<A>& bgra, View<A>& bgr)
            and Simd::RgbaToRgb(const View<A>& rgba, View<A>& rgb).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA (or 32-bit RGBA) image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR (or 24-bit RGB) image.
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

        \fn void SimdBgraToRgb(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgb, size_t rgbStride);

        \short Converts 32-bit BGRA image to 24-bit RGB image. Also it can be used for 32-bit RGBA to 24-bit BGR conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::BgraToRgb(const View<A>& bgra, View<A>& rgb)
            and Simd::RgbaToBgr(const View<A>& rgba, View<A>& bgr).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA (or 32-bit RGBA) image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] rgb - a pointer to pixels data of output 24-bit RGB (or 24-bit BGR) image.
        \param [in] rgbStride - a row size of the rgb image.
    */
    SIMD_API void SimdBgraToRgb(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgb, size_t rgbStride);

    /*! @ingroup bgra_conversion

        \fn void SimdBgraToRgba(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgba, size_t rgbaStride);

        \short Converts 32-bit BGRA image to 32-bit RGBA image. Also it can be used for 32-bit RGBA to 32-bit BGRA conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::BgraToRgba(const View<A>& bgra, View<A>& rgba)
            and Simd::RgbaToBgra(const View<A>& rgba, View<A>& bgra).

        \param [in] bgra - a pointer to pixels data of input 32-bit BGRA (or 32-bit RGBA) image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgraStride - a row size of the bgra image.
        \param [out] rgba - a pointer to pixels data of output 32-bit RGBA (or 32-bit BGRA) image.
        \param [in] rgbaStride - a row size of the rgb image.
    */
    SIMD_API void SimdBgraToRgba(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgba, size_t rgbaStride);

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

        \fn void SimdBgrToRgb(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgb, size_t rgbStride);

        \short Converts 24-bit BGR image to 24-bit RGB image. Also it can be used for 24-bit RGB to 24-bit BGR conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::BgrToRgb(const View<A> & bgr, View<A> & rgb)
            and Simd::RgbToBgr(const View<A>& rgb, View<A>& bgr).

        \param [in] bgr - a pointer to pixels data of input 24-bit BGR image (or 24-bit RGB image).
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] bgrStride - a row size of the bgr image.
        \param [out] rgb - a pointer to pixels data of output 24-bit RGB image (or 24-bit BGR image).
        \param [in] rgbStride - a row size of the rgb image.
    */
    SIMD_API void SimdBgrToRgb(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgb, size_t rgbStride);

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

    /*! @ingroup deinterleave_conversion

        \fn void SimdDeinterleaveBgr(const uint8_t * bgr, size_t bgrStride, size_t width, size_t height, uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride);

        \short Deinterleaves 24-bit BGR interleaved image into separated 8-bit Blue, Green and Red planar images.

        All images must have the same width and height.

        \note This function has C++ wrappers:
            Simd::DeinterleaveBgr(const View<A>& bgr, View<A>& b, View<A>& g, View<A>& r),
            Simd::DeinterleaveRgb(const View<A>& rgb, View<A>& r, View<A>& g, View<A>& b).

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

    /*! @ingroup deinterleave_conversion

        \fn void SimdDeinterleaveBgra(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height, uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride, uint8_t * a, size_t aStride);

        \short Deinterleaves 32-bit BGRA interleaved image into separated 8-bit Blue, Green, Red and Alpha planar images.

        All images must have the same width and height.

        \note This function has C++ wrappers:
            Simd::DeinterleaveBgra(const View<A>& bgra, View<A>& b, View<A>& g, View<A>& r, View<A>& a),
            Simd::DeinterleaveBgra(const View<A>& bgra, View<A>& b, View<A>& g, View<A>& r),
            Simd::DeinterleaveRgba(const View<A>& rgba, View<A>& r, View<A>& g, View<A>& b, View<A>& a),
            Simd::DeinterleaveRgba(const View<A>& rgba, View<A>& r, View<A>& g, View<A>& b).

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
        \param [out] a - a pointer to pixels data of 8-bit Alpha planar image. It can be NULL.
        \param [in] aStride - a row size of the a image.
    */
    SIMD_API void SimdDeinterleaveBgra(const uint8_t * bgra, size_t bgraStride, size_t width, size_t height,
        uint8_t * b, size_t bStride, uint8_t * g, size_t gStride, uint8_t * r, size_t rStride, uint8_t * a, size_t aStride);

    /*! @ingroup gaussian_filter

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

    /*! @ingroup gaussian_filter

        \fn void * SimdGaussianBlurInit(size_t width, size_t height, size_t channels, const float * sigma, const float* epsilon);

        \short Creates Gaussian blur filter context.

        In particular calculates Gaussian blur coefficients:
        \verbatim
        half = floor(sqrt(log(1/epsilon)) * sigma);
        weight[2*half + 1];

        for(x = -half; x <= half; ++x)
            weight[x + half] = exp(-sqr(x / sigma) / 2);

        sum = 0;
        for (x = -half; x <= half; ++x)
            sum += weight[x + half];

        for (x = -half; x <= half; ++x)
            weight[x + half] /= sum;
        \endverbatim

        \param [in] width - a width of input and output image.
        \param [in] height - a height of input and output image.
        \param [in] channels - a channel number of input and output image. Its value must be in range [1..4].
        \param [in] sigma - a pointer to sigma parameter (blur radius). MIts value must be greater than 0.000001.
        \param [in] epsilon - a pointer to epsilon parameter (permissible relative error).
                              Its value must be greater than 0.000001. Pointer can be NULL and by default value 0.001 is used.
        \return a pointer to filter context. On error it returns NULL.
                This pointer is used in functions ::SimdGaussianBlurRun.
                It must be released with using of function ::SimdRelease.
    */
    SIMD_API void* SimdGaussianBlurInit(size_t width, size_t height, size_t channels, const float * sigma, const float* epsilon);

    /*! @ingroup gaussian_filter

        \fn void SimdGaussianBlurRun(const void* filter, const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride);

        \short Performs image Gaussian bluring.

        Bluring algorithm for every point:
        \verbatim
        sum = 0;
        for(x = -half; x <= half; ++x)
        {
            sx = min(max(0, dx + x), width - 1);
            for(y = -half; y <= half; ++y)
            {
                sy = min(max(0, dy + y), height - 1);
                sum += src[sx, sy]*weight[x + half]*weight[y + half];
            }
        }
        dst[dx, dy] = sum;
        \endverbatim

        \param [in] filter - a filter context. It must be created by function ::SimdGaussianBlurInit and released by function ::SimdRelease.
        \param [in] src - a pointer to pixels data of the original input image.
        \param [in] srcStride - a row size (in bytes) of the input image.
        \param [out] dst - a pointer to pixels data of the filtered output image.
        \param [in] dstStride - a row size (in bytes) of the output image.
    */
    SIMD_API void SimdGaussianBlurRun(const void* filter, const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride);

    /*! @ingroup gray_conversion

        \fn void SimdGrayToBgr(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgr, size_t bgrStride);

        \short Converts 8-bit gray image to 24-bit BGR image. Also it can be used for 8-bit gray to 24-bit RGB conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::GrayToBgr(const View<A>& gray, View<A>& bgr)
            and Simd::GrayToRgb(const View<A>& gray, View<A>& rgb).

        \param [in] gray - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] grayStride - a row size of the gray image.
        \param [out] bgr - a pointer to pixels data of output 24-bit BGR (or 24-bit RGB) image.
        \param [in] bgrStride - a row size of the bgr image.
    */
    SIMD_API void SimdGrayToBgr(const uint8_t *gray, size_t width, size_t height, size_t grayStride, uint8_t *bgr, size_t bgrStride);

    /*! @ingroup gray_conversion

        \fn void SimdGrayToBgra(const uint8_t * gray, size_t width, size_t height, size_t grayStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 8-bit gray image to 32-bit BGRA image. Also it can be used for 8-bit gray to 32-bit RGBA conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::GrayToBgra(const View<A>& gray, View<A>& bgra, uint8_t alpha)
            and Simd::GrayToRgba(const View<A>& gray, View<A>& rgba, uint8_t alpha).

        \param [in] gray - a pointer to pixels data of input 8-bit gray image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] grayStride - a row size of the gray image.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA (or 32-bit RGBA) image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdGrayToBgra(const uint8_t *gray, size_t width, size_t height, size_t grayStride,
        uint8_t *bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup image_io

        \fn uint8_t* SimdImageSaveToMemory(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, size_t * size);

        \short Saves an image to memory in given image file format.

        \param [in] src - a pointer to pixels data of input image.
        \param [in] stride - a row size of input image in bytes.
        \param [in] width - a width of input image.
        \param [in] height - a height of input image.
        \param [in] format - a pixel format of input image.
            Supported pixel formats: ::SimdPixelFormatGray8, ::SimdPixelFormatBgr24, ::SimdPixelFormatBgra32, ::SimdPixelFormatRgb24, ::SimdPixelFormatRgba32.
        \param [in] file - a format of output image file. To auto choise format of output file set this parameter to ::SimdImageFileUndefined.
        \param [in] quality - a parameter of compression quality (if file format supports it).
        \param [out] size - a pointer to the size of output image file in bytes.
        \return a pointer to memory buffer with output image file.
            It has to be deleted after use by function ::SimdFree. On error it returns NULL.
    */
    SIMD_API uint8_t* SimdImageSaveToMemory(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, size_t * size);

    /*! @ingroup image_io

        \fn SimdBool SimdImageSaveToFile(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, const char * path);

        \short Saves an image to memory in given image file format.

        \param [in] src - a pointer to pixels data of input image.
        \param [in] stride - a row size of input image in bytes.
        \param [in] width - a width of input image.
        \param [in] height - a height of input image.
        \param [in] format - a pixel format of input image.
            Supported pixel formats: ::SimdPixelFormatGray8, ::SimdPixelFormatBgr24, ::SimdPixelFormatBgra32, ::SimdPixelFormatRgb24, ::SimdPixelFormatRgba32.
        \param [in] file - a format of output image file. To auto choise format of output file set this parameter to ::SimdImageFileUndefined.
        \param [in] quality - a parameter of compression quality (if file format supports it).
        \param [in] path - a path to output image file.
        \return result of the operation.
    */
    SIMD_API SimdBool SimdImageSaveToFile(const uint8_t* src, size_t stride, size_t width, size_t height, SimdPixelFormatType format, SimdImageFileType file, int quality, const char * path);

    /*! @ingroup image_io

        \fn uint8_t* SimdImageLoadFromMemory(const uint8_t* data, size_t size, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType * format);

        \short Loads an image from memory buffer.

        \param [in] data - a pointer to memory buffer with input image file.
        \param [in] size - a size of input image file in bytes.
        \param [out] stride - a pointer to row size of output image in bytes.
        \param [out] width - a pointer to width of output image.
        \param [out] height - a pointer to height of output image.
        \param [in, out] format - a pointer to pixel format of output image.
            Here you can set desired pixel format (it can be ::SimdPixelFormatGray8, ::SimdPixelFormatBgr24, ::SimdPixelFormatBgra32, ::SimdPixelFormatRgb24, ::SimdPixelFormatRgba32).
            Or set ::SimdPixelFormatNone and use pixel format of input image file.
        \return a pointer to pixels data of output image.
            It has to be deleted after use by function ::SimdFree. On error it returns NULL.
    */
    SIMD_API uint8_t* SimdImageLoadFromMemory(const uint8_t* data, size_t size, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType * format);

    /*! @ingroup image_io

        \fn uint8_t* SimdImageLoadFromFile(const char* path, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType * format);

        \short Loads an image from file.

        \param [in] path - a path to input image file.
        \param [out] stride - a pointer to row size of output image in bytes.
        \param [out] width - a pointer to width of output image.
        \param [out] height - a pointer to height of output image.
        \param [in, out] format - a pointer to pixel format of output image.
            Here you can set desired pixel format (it can be ::SimdPixelFormatGray8, ::SimdPixelFormatBgr24, ::SimdPixelFormatBgra32, ::SimdPixelFormatRgb24, ::SimdPixelFormatRgba32).
            Or set ::SimdPixelFormatNone and use pixel format of input image file.
        \return a pointer to pixels data of output image.
            It has to be deleted after use by function ::SimdFree. On error it returns NULL.
    */
    SIMD_API uint8_t* SimdImageLoadFromFile(const char* path, size_t* stride, size_t* width, size_t* height, SimdPixelFormatType * format);

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

    /*! @ingroup interleave_conversion

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

        An using example (resize of RGBA64 image):
        \verbatim
        void * resizer = SimdResizerInit(srcX, srcY, dstX, dstY, 4, SimdResizeChannelShort, SimdResizeMethodBilinear);
        if (resizer)
        {
             SimdResizerRun(resizer, (uint8_t*)src, srcStride, (uint8_t*)dst, dstStride);
             SimdRelease(resizer);
        }
        \endverbatim

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

    /*! @ingroup rgb_conversion

        \fn void SimdRgbToBgra(const uint8_t * rgb, size_t width, size_t height, size_t rgbStride, uint8_t * bgra, size_t bgraStride, uint8_t alpha);

        \short Converts 24-bit RGB image to 32-bit BGRA image. Also it can be used for 24-bit BGR to 32-bit RGBA conversion.

        All images must have the same width and height.

        \note This function has C++ wrappers: Simd::RgbToBgra(const View<A>& rgb, View<A>& bgra, uint8_t alpha)
            and Simd::BgrToRgba(const View<A>& bgr, View<A>& rgba, uint8_t alpha).

        \param [in] rgb - a pointer to pixels data of input 24-bit RGB (or 24-bit BGR) image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] rgbStride - a row size of the rgb image.
        \param [out] bgra - a pointer to pixels data of output 32-bit BGRA (or 32-bit RGBA) image.
        \param [in] bgraStride - a row size of the bgra image.
        \param [in] alpha - a value of alpha channel.
    */
    SIMD_API void SimdRgbToBgra(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha);

    /*! @ingroup rgb_conversion

        \fn void SimdRgbToGray(const uint8_t * rgb, size_t width, size_t height, size_t rgbStride, uint8_t * gray, size_t grayStride);

        \short Converts 24-bit RGB image to 8-bit gray image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::RgbToGray(const View<A>& rgb, View<A>& gray).

        \param [in] rgb - a pointer to pixels data of input 24-bit RGB image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] rgbStride - a row size of the rgb image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdRgbToGray(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* gray, size_t grayStride);

    /*! @ingroup rgba_conversion

        \fn void SimdRgbaToGray(const uint8_t * rgba, size_t width, size_t height, size_t rgbaStride, uint8_t * gray, size_t grayStride);

        \short Converts 32-bit RGBA image to 8-bit gray image.

        All images must have the same width and height.

        \note This function has a C++ wrapper Simd::RgbaToGray(const View<A>& rgba, View<A>& gray).

        \param [in] rgba - a pointer to pixels data of input 32-bit RGBA image.
        \param [in] width - an image width.
        \param [in] height - an image height.
        \param [in] rgbaStride - a row size of the rgba image.
        \param [out] gray - a pointer to pixels data of output 8-bit gray image.
        \param [in] grayStride - a row size of the gray image.
    */
    SIMD_API void SimdRgbaToGray(const uint8_t* rgba, size_t width, size_t height, size_t rgbaStride, uint8_t* gray, size_t grayStride);

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

    /* ViSP custom SIMD code */
    // vpImageMorphology::erosion()
    SIMD_API void SimdImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

    // vpImageMorphology::dilatation
    SIMD_API void SimdImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

    // vpColVector::sum()
    SIMD_API double SimdVectorSum(const double * vec, size_t size);

    // vpColVector::sumSquare()
    SIMD_API double SimdVectorSumSquare(const double * vec, size_t size);

    // vpColVector::stdev()
    SIMD_API double SimdVectorStdev(const double * vec, size_t size, bool useBesselCorrection);

    // vpColVector::hadamard(const vpColVector &v)
    SIMD_API void SimdVectorHadamard(const double * src1, const double * src2, size_t size, double * dst);

    // vpMatrix::operator*(const vpVelocityTwistMatrix &V)
    SIMD_API void SimdMatMulTwist(const double * mat, size_t rows, const double * twist, double * dst);

    // vpMatrix::transpose(vpMatrix &At)
    SIMD_API void SimdMatTranspose(const double * mat, size_t rows, size_t cols, double * dst);

    // vpImageTools::imageDifference
    SIMD_API void SimdImageDifference(const unsigned char * img1, const unsigned char * img2, size_t size, unsigned char * imgDiff);

    // vpImageTools::normalizedCorrelation
    SIMD_API void SimdNormalizedCorrelation(const double * img1, double mean1, const double * img2, double mean2, size_t size,
                                            double& a2, double& b2, double& ab, bool useOptimized);

    // vpImageTools::normalizedCorrelation
    SIMD_API void SimdNormalizedCorrelation2(const double * img1, size_t width1, const double * img2,
                                             size_t width2, size_t height2, size_t i0, size_t j0, double& ab);

    // vpImageTools::remap()
    SIMD_API void SimdRemap(const unsigned char * src, size_t channels, size_t width, size_t height, size_t offset,
                            const int * mapU, const int * mapV, const float * mapDu, const float * mapDv, unsigned char * dst);

    // vpMbTracker::computeJTR(const vpMatrix &interaction, const vpColVector &error, vpColVector &JTR)
    SIMD_API void SimdComputeJtR(const double * J, size_t rows, const double * R, double * dst);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif//__SimdLib_h__
