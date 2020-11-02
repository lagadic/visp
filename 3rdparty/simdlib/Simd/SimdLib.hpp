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

#include "Simd/SimdView.hpp"
#include "Simd/SimdPixel.hpp"
#include "Simd/SimdPyramid.hpp"

#ifndef __SimdLib_hpp__
#define __SimdLib_hpp__

/*! \namespace Simd */
namespace Simd
{
    /*! @ingroup bgra_conversion

        \fn void BgraToBgr(const View<A>& bgra, View<A>& bgr)

        \short Converts 32-bit BGRA image to 24-bit BGR image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgraToBgr.

        \param [in] bgra - an input 32-bit BGRA image.
        \param [out] bgr - an output 24-bit BGR image.
    */
    template<template<class> class A> SIMD_INLINE void BgraToBgr(const View<A>& bgra, View<A>& bgr)
    {
        assert(EqualSize(bgra, bgr) && bgra.format == View<A>::Bgra32 && bgr.format == View<A>::Bgr24);

        SimdBgraToBgr(bgra.data, bgra.width, bgra.height, bgra.stride, bgr.data, bgr.stride);
    }

    /*! @ingroup bgra_conversion

        \fn void BgraToGray(const View<A>& bgra, View<A>& gray)

        \short Converts 32-bit BGRA image to 8-bit gray image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgraToGray.

        \param [in] bgra - an input 32-bit BGRA image.
        \param [out] gray - an output 8-bit gray image.
    */
    template<template<class> class A> SIMD_INLINE void BgraToGray(const View<A>& bgra, View<A>& gray)
    {
        assert(EqualSize(bgra, gray) && bgra.format == View<A>::Bgra32 && gray.format == View<A>::Gray8);

        SimdBgraToGray(bgra.data, bgra.width, bgra.height, bgra.stride, gray.data, gray.stride);
    }

    /*! @ingroup bgr_conversion

        \fn void BgrToBgra(const View<A>& bgr, View<A>& bgra, uint8_t alpha = 0xFF)

        \short Converts 24-bit BGR image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgrToBgra.

        \param [in] bgr - an input 24-bit BGR image.
        \param [out] bgra - an output 32-bit BGRA image.
        \param [in] alpha - a value of alpha channel. It is equal to 256 by default.
    */
    template<template<class> class A> SIMD_INLINE void BgrToBgra(const View<A>& bgr, View<A>& bgra, uint8_t alpha = 0xFF)
    {
        assert(EqualSize(bgr, bgra) && bgra.format == View<A>::Bgra32 && bgr.format == View<A>::Bgr24);

        SimdBgrToBgra(bgr.data, bgr.width, bgr.height, bgr.stride, bgra.data, bgra.stride, alpha);
    }

    /*! @ingroup other_conversion

        \fn void Bgr48pToBgra32(const View<A>& blue, const View<A>& green, const View<A>& red, View<A>& bgra, uint8_t alpha = 0xFF)

        \short Converts 48-bit planar BGR image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgr48pToBgra32.

        \param [in] blue - an input 16-bit image with blue color plane.
        \param [in] green - an input 16-bit image with green color plane.
        \param [in] red - an input 16-bit image with red color plane.
        \param [out] bgra - an output 32-bit BGRA image.
        \param [in] alpha - a value of alpha channel. It is equal to 256 by default.
    */
    template<template<class> class A> SIMD_INLINE void Bgr48pToBgra32(const View<A>& blue, const View<A>& green, const View<A>& red, View<A>& bgra, uint8_t alpha = 0xFF)
    {
        assert(Compatible(blue, green, red) && EqualSize(blue, bgra) && blue.format == View<A>::Int16 && bgra.format == View<A>::Bgra32);

        SimdBgr48pToBgra32(blue.data, blue.stride, blue.width, blue.height, green.data, green.stride, red.data, red.stride, bgra.data, bgra.stride, alpha);
    }

    /*! @ingroup bgr_conversion

        \fn void BgrToGray(const View<A>& bgr, View<A>& gray)

        \short Converts 24-bit BGR image to 8-bit gray image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgrToGray.

        \param [in] bgr - an input 24-bit BGR image.
        \param [out] gray - an output 8-bit gray image.
    */
    template<template<class> class A> SIMD_INLINE void BgrToGray(const View<A>& bgr, View<A>& gray)
    {
        assert(EqualSize(bgr, gray) && bgr.format == View<A>::Bgr24 && gray.format == View<A>::Gray8);

        SimdBgrToGray(bgr.data, bgr.width, bgr.height, bgr.stride, gray.data, gray.stride);
    }

    /*! @ingroup bgr_conversion

        \fn void BgrToRgb(const View<A> & bgr, View<A> & rgb)

        \short Converts 24-bit BGR image to 24-bit RGB image (also it performs backward conversion).

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdBgrToRgb.

        \param [in] bgr - an input 24-bit BGR image.
        \param [out] rgb - an output 24-bit RGB image.
    */
    template<template<class> class A> SIMD_INLINE void BgrToRgb(const View<A> & bgr, View<A> & rgb)
    {
        assert(EqualSize(bgr, rgb) && bgr.PixelSize() == 3 && rgb.PixelSize() == 3);

        SimdBgrToRgb(bgr.data, bgr.stride, bgr.width, bgr.height, rgb.data, rgb.stride);
    }

    /*! @ingroup copying

        \fn void Copy(const View<A> & src, View<B> & dst)

        \short Copies pixels data of image from source to destination.

        All images must have the same width, height and format.

        \note This function is a C++ wrapper for function ::SimdCopy.

        \param [in] src - a source image.
        \param [out] dst - a destination image.
    */
    template<template<class> class A, template<class> class B> SIMD_INLINE void Copy(const View<A> & src, View<B> & dst)
    {
        assert(Compatible(src, dst));

        if (src.format)
        {
            SimdCopy(src.data, src.stride, src.width, src.height, src.PixelSize(), dst.data, dst.stride);
        }
    }

    /*! @ingroup copying

        \fn void CopyFrame(const View<A>& src, const Rectangle<ptrdiff_t> & frame, View<A>& dst)

        \short Copies pixels data of image from source to destination except for the portion bounded frame.

        All images must have the same width, height and format.

        \note This function is a C++ wrapper for function ::SimdCopyFrame.

        \param [in] src - a source image.
        \param [in] frame - a frame rectangle.
        \param [out] dst - a destination image.
    */
    template<template<class> class A> SIMD_INLINE void CopyFrame(const View<A>& src, const Rectangle<ptrdiff_t> & frame, View<A>& dst)
    {
        assert(Compatible(src, dst) && frame.Width() >= 0 && frame.Height() >= 0);
        assert(frame.left >= 0 && frame.top >= 0 && frame.right <= ptrdiff_t(src.width) && frame.bottom <= ptrdiff_t(src.height));

        SimdCopyFrame(src.data, src.stride, src.width, src.height, src.PixelSize(),
            frame.left, frame.top, frame.right, frame.bottom, dst.data, dst.stride);
    }

    /*! @ingroup other_conversion

        \fn void DeinterleaveBgr(const View<A>& bgr, View<A>& b, View<A>& g, View<A>& r)

        \short Deinterleaves 24-bit BGR interleaved image into separated 8-bit Blue, Green and Red planar images.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdDeinterleaveBgr.

        \param [in] bgr - an input 24-bit BGR interleaved image.
        \param [out] b - an output 8-bit Blue planar image.
        \param [out] g - an output 8-bit Green planar image.
        \param [out] r - an output 8-bit Red planar image.
        */
    template<template<class> class A> SIMD_INLINE void DeinterleaveBgr(const View<A>& bgr, View<A>& b, View<A>& g, View<A>& r)
    {
        assert(EqualSize(bgr, b) && Compatible(b, g, r) && bgr.format == View<A>::Bgr24 && b.format == View<A>::Gray8);

        SimdDeinterleaveBgr(bgr.data, bgr.stride, bgr.width, bgr.height, b.data, b.stride, g.data, g.stride, r.data, r.stride);
    }

    /*! @ingroup other_conversion

        \fn void DeinterleaveBgra(const View<A>& bgra, View<A>& b, View<A>& g, View<A>& r, View<A>& a)

        \short Deinterleaves 32-bit BGRA interleaved image into separated 8-bit Blue, Green, Red and Alpha planar images.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdDeinterleaveBgra.

        \param [in] bgra - an input 32-bit BGRA interleaved image.
        \param [out] b - an output 8-bit Blue planar image.
        \param [out] g - an output 8-bit Green planar image.
        \param [out] r - an output 8-bit Red planar image.
        \param [out] a - an output 8-bit Alpha planar image.
    */
    template<template<class> class A> SIMD_INLINE void DeinterleaveBgra(const View<A>& bgra, View<A>& b, View<A>& g, View<A>& r, View<A>& a)
    {
        assert(EqualSize(bgra, b) && Compatible(b, g, r, a) && bgra.format == View<A>::Bgra32 && b.format == View<A>::Gray8);

        SimdDeinterleaveBgra(bgra.data, bgra.stride, bgra.width, bgra.height, b.data, b.stride, g.data, g.stride, r.data, r.stride, a.data, a.stride);
    }

    /*! @ingroup other_filter

        \fn void GaussianBlur3x3(const View<A>& src, View<A>& dst)

        \short Performs Gaussian blur filtration with window 3x3.

        For every point:
        \verbatim
        dst[x, y] = (src[x-1, y-1] + 2*src[x, y-1] + src[x+1, y-1] +
                    2*(src[x-1, y] + 2*src[x, y] + src[x+1, y]) +
                    src[x-1, y+1] + 2*src[x, y+1] + src[x+1, y+1] + 8) / 16;
        \endverbatim
        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdGaussianBlur3x3.

        \param [in] src - a source image.
        \param [out] dst - a destination image.
    */
    template<template<class> class A> SIMD_INLINE void GaussianBlur3x3(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdGaussianBlur3x3(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup gray_conversion

        \fn void GrayToBgr(const View<A>& gray, View<A>& bgr)

        \short Converts 8-bit gray image to 24-bit BGR image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdGrayToBgr.

        \param [in] gray - an input 8-bit gray image.
        \param [out] bgr - an output 24-bit BGR image.
    */
    template<template<class> class A> SIMD_INLINE void GrayToBgr(const View<A>& gray, View<A>& bgr)
    {
        assert(EqualSize(gray, bgr) && bgr.format == View<A>::Bgr24 && gray.format == View<A>::Gray8);

        SimdGrayToBgr(gray.data, gray.width, gray.height, gray.stride, bgr.data, bgr.stride);
    }

    /*! @ingroup gray_conversion

        \fn void GrayToBgra(const View<A>& gray, View<A>& bgra, uint8_t alpha = 0xFF)

        \short Converts 8-bit gray image to 32-bit BGRA image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdGrayToBgra.

        \param [in] gray - an input 8-bit gray image.
        \param [out] bgra - an output 32-bit BGRA image.
        \param [in] alpha - a value of alpha channel. It is equal to 255 by default.
    */
    template<template<class> class A> SIMD_INLINE void GrayToBgra(const View<A>& gray, View<A>& bgra, uint8_t alpha = 0xFF)
    {
        assert(EqualSize(gray, bgra) && bgra.format == View<A>::Bgra32 && gray.format == View<A>::Gray8);

        SimdGrayToBgra(gray.data, gray.width, gray.height, gray.stride, bgra.data, bgra.stride, alpha);
    }

    /*! @ingroup other_conversion

        \fn void InterleaveBgr(const View<A> & b, const View<A> & g, const View<A> & r, View<A> & bgr)

        \short Interleaves 8-bit Blue, Green and Red planar images into one 24-bit BGR interleaved image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdInterleaveBgr.

        \param [in] b - an input 8-bit Blue planar image.
        \param [in] g - an input 8-bit Green planar image.
        \param [in] r - an input 8-bit Red planar image.
        \param [out] bgr - an output 24-bit BGR interleaved image.
    */
    template<template<class> class A> SIMD_INLINE void InterleaveBgr(const View<A> & b, const View<A> & g, const View<A> & r, View<A> & bgr)
    {
        assert(EqualSize(bgr, b, g, r) && Compatible(b, g, r) && bgr.format == View<A>::Bgr24 && b.format == View<A>::Gray8);

        SimdInterleaveBgr(b.data, b.stride, g.data, g.stride, r.data, r.stride, bgr.width, bgr.height, bgr.data, bgr.stride);
    }

    /*! @ingroup other_conversion

        \fn void InterleaveBgra(const View<A>& b, const View<A>& g, const View<A>& r, const View<A>& a, View<A>& bgra)

        \short Interleaves 8-bit Blue, Green, Red and Alpha planar images into one 32-bit BGRA interleaved image.

        All images must have the same width and height.

        \note This function is a C++ wrapper for function ::SimdInterleaveBgra.

        \param [in] b - an input 8-bit Blue planar image.
        \param [in] g - an input 8-bit Green planar image.
        \param [in] r - an input 8-bit Red planar image.
        \param [in] a - an input 8-bit Alpha planar image.
        \param [out] bgra - an output 32-bit BGRA interleaved image.
    */
    template<template<class> class A> SIMD_INLINE void InterleaveBgra(const View<A>& b, const View<A>& g, const View<A>& r, const View<A>& a, View<A>& bgra)
    {
        assert(EqualSize(bgra, b) && Compatible(b, g, r, a) && bgra.format == View<A>::Bgra32 && b.format == View<A>::Gray8);

        SimdInterleaveBgra(b.data, b.stride, g.data, g.stride, r.data, r.stride, a.data, a.stride, bgra.width, bgra.height, bgra.data, bgra.stride);
    }

    /*! @ingroup other_filter

        \fn void MeanFilter3x3(const View<A>& src, View<A>& dst)

        \short Performs an averaging with window 3x3.

        For every point:
        \verbatim
        dst[x, y] = (src[x-1, y-1] + src[x, y-1] + src[x+1, y-1] +
                     src[x-1, y] + src[x, y] + src[x+1, y] +
                     src[x-1, y+1] + src[x, y+1] + src[x+1, y+1] + 4) / 9;
        \endverbatim

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdMeanFilter3x3.

        \param [in] src - a source image.
        \param [out] dst - a destination image.
    */
    template<template<class> class A> SIMD_INLINE void MeanFilter3x3(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdMeanFilter3x3(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup median_filter

        \fn void MedianFilterRhomb3x3(const View<A>& src, View<A>& dst)

        \short Performs median filtration of input image (filter window is a rhomb 3x3).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdMedianFilterRhomb3x3.

        \param [in] src - an original input image.
        \param [out] dst - a filtered output image.
    */
    template<template<class> class A> SIMD_INLINE void MedianFilterRhomb3x3(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdMedianFilterRhomb3x3(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup median_filter

        \fn void MedianFilterRhomb5x5(const View<A>& src, View<A>& dst)

        \short Performs median filtration of input image (filter window is a rhomb 5x5).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdMedianFilterRhomb5x5.

        \param [in] src - an original input image.
        \param [out] dst - a filtered output image.
    */
    template<template<class> class A> SIMD_INLINE void MedianFilterRhomb5x5(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdMedianFilterRhomb5x5(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup median_filter

        \fn void MedianFilterSquare3x3(const View<A>& src, View<A>& dst)

        \short Performs median filtration of input image (filter window is a square 3x3).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdMedianFilterSquare3x3.

        \param [in] src - an original input image.
        \param [out] dst - a filtered output image.
    */
    template<template<class> class A> SIMD_INLINE void MedianFilterSquare3x3(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdMedianFilterSquare3x3(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup median_filter

        \fn void MedianFilterSquare5x5(const View<A>& src, View<A>& dst)

        \short Performs median filtration of input image (filter window is a square 5x5).

        All images must have the same width, height and format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdMedianFilterSquare5x5.

        \param [in] src - an original input image.
        \param [out] dst - a filtered output image.
    */
    template<template<class> class A> SIMD_INLINE void MedianFilterSquare5x5(const View<A>& src, View<A>& dst)
    {
        assert(Compatible(src, dst) && src.ChannelSize() == 1);

        SimdMedianFilterSquare5x5(src.data, src.stride, src.width, src.height, src.ChannelCount(), dst.data, dst.stride);
    }

    /*! @ingroup operation

        \fn void OperationBinary8u(const View<A>& a, const View<A>& b, View<A>& dst, SimdOperationBinary8uType type)

        \short Performs given operation between two images.

        All images must have the same width, height and format (8-bit gray, 16-bit UV (UV plane of NV12 pixel format), 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdOperationBinary8u.

        \param [in] a - a first input image.
        \param [in] b - a second input image.
        \param [out] dst - an output image.
        \param [in] type - a type of operation (see ::SimdOperationBinary8uType).
    */
    template<template<class> class A> SIMD_INLINE void OperationBinary8u(const View<A>& a, const View<A>& b, View<A>& dst, SimdOperationBinary8uType type)
    {
        assert(Compatible(a, b, dst) && a.ChannelSize() == 1);

        SimdOperationBinary8u(a.data, a.stride, b.data, b.stride, a.width, a.height, a.ChannelCount(), dst.data, dst.stride, type);
    }

    /*! @ingroup resizing

        \fn void ReduceGray2x2(const View<A>& src, View<A>& dst)

        \short Performs reducing (in 2 times) and Gaussian blurring a 8-bit gray image with using window 2x2.

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

        For all points:
        \verbatim
        dst[x, y] = (src[2*x, 2*y] + src[2*x, 2*y + 1] + src[2*x + 1, 2*y] + src[2*x + 1, 2*y + 1] + 2)/4;
        \endverbatim

        \note This function is a C++ wrapper for function ::SimdReduceGray2x2.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
    */
    template<template<class> class A> SIMD_INLINE void ReduceGray2x2(const View<A>& src, View<A>& dst)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8 && Scale(src.Size()) == dst.Size());

        SimdReduceGray2x2(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride);
    }

    /*! @ingroup resizing

        \fn void ReduceGray3x3(const View<A>& src, View<A>& dst, bool compensation = true)

        \short Performs reducing (in 2 times) and Gaussian blurring a 8-bit gray image with using window 3x3.

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

        For every point:
        \verbatim
        dst[x, y] = (src[2*x-1, 2*y-1] + 2*src[2*x, 2*y-1] + src[2*x+1, 2*y-1] +
                  2*(src[2*x-1, 2*y]   + 2*src[2*x, 2*y]   + src[2*x+1, 2*y]) +
                     src[2*x-1, 2*y+1] + 2*src[2*x, 2*y+1] + src[2*x+1, 2*y+1] + compensation ? 8 : 0) / 16;
        \endverbatim

        \note This function is a C++ wrapper for function ::SimdReduceGray3x3.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
        \param [in] compensation - a flag of compensation of rounding. It is equal to 'true' by default.
    */
    template<template<class> class A> SIMD_INLINE void ReduceGray3x3(const View<A>& src, View<A>& dst, bool compensation = true)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8 && Scale(src.Size()) == dst.Size());

        SimdReduceGray3x3(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride, compensation ? 1 : 0);
    }

    /*! @ingroup resizing

        \fn void ReduceGray4x4(const View<A>& src, View<A>& dst)

        \short Performs reducing (in 2 times) and Gaussian blurring a 8-bit gray image with using window 4x4.

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

        For every point:
        \verbatim
        dst[x, y] =   (src[2*x-1, 2*y-1] + 3*src[2*x, 2*y-1] + 3*src[2*x+1, 2*y-1] + src[2*x+2, 2*y-1]
                    3*(src[2*x-1, 2*y]   + 3*src[2*x, 2*y]   + 3*src[2*x+1, 2*y]   + src[2*x+2, 2*y]) +
                    3*(src[2*x-1, 2*y+1] + 3*src[2*x, 2*y+1] + 3*src[2*x+1, 2*y+1] + src[2*x+2, 2*y+1]) +
                       src[2*x-1, 2*y+2] + 3*src[2*x, 2*y+2] + 3*src[2*x+1, 2*y+2] + src[2*x+2, 2*y+2] + 32) / 64;
        \endverbatim

        \note This function is a C++ wrapper for function ::SimdReduceGray4x4.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
    */
    template<template<class> class A> SIMD_INLINE void ReduceGray4x4(const View<A>& src, View<A>& dst)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8 && Scale(src.Size()) == dst.Size());

        SimdReduceGray4x4(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride);
    }

    /*! @ingroup resizing

        \fn void ReduceGray5x5(const View<A>& src, View<A>& dst, bool compensation = true)

        \short Performs reducing (in 2 times) and Gaussian blurring a 8-bit gray image with using window 5x5.

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

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

        \note This function is a C++ wrapper for function ::SimdReduceGray5x5.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
        \param [in] compensation - a flag of compensation of rounding. It is equal to 'true' by default.
    */
    template<template<class> class A> SIMD_INLINE void ReduceGray5x5(const View<A>& src, View<A>& dst, bool compensation = true)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8 && Scale(src.Size()) == dst.Size());

        SimdReduceGray5x5(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride, compensation ? 1 : 0);
    }

    /*! @ingroup resizing

        \fn void ReduceGray(const View<A> & src, View<A> & dst, ::SimdReduceType reduceType, bool compensation = true)

        \short Performs reducing (in 2 times) and Gaussian blurring a 8-bit gray image.

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
        \param [in] reduceType - a type of function used for image reducing.
        \param [in] compensation - a flag of compensation of rounding. It is relevant only for ::SimdReduce3x3 and ::SimdReduce5x5. It is equal to 'true' by default.
    */
    template<template<class> class A> SIMD_INLINE void ReduceGray(const View<A> & src, View<A> & dst, ::SimdReduceType reduceType, bool compensation = true)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8 && Scale(src.Size()) == dst.Size());

        switch (reduceType)
        {
        case SimdReduce2x2:
            Simd::ReduceGray2x2(src, dst);
            break;
        case SimdReduce3x3:
            Simd::ReduceGray3x3(src, dst, compensation);
            break;
        case SimdReduce4x4:
            Simd::ReduceGray4x4(src, dst);
            break;
        case SimdReduce5x5:
            Simd::ReduceGray5x5(src, dst, compensation);
            break;
        default:
            assert(0);
        }
    }

    /*! @ingroup resizing

        \fn void Reduce2x2(const View<A> & src, View<A> & dst)

        \short Performs reducing of image (in 2 times).

        For input and output image must be performed: dst.width = (src.width + 1)/2,  dst.height = (src.height + 1)/2.

        \param [in] src - an original input image.
        \param [out] dst - a reduced output image.
    */
    template<template<class> class A> SIMD_INLINE void Reduce2x2(const View<A> & src, View<A> & dst)
    {
        assert(src.format == dst.format && Scale(src.Size()) == dst.Size() && src.ChannelSize() == 1);

        SimdReduceColor2x2(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride, src.ChannelCount());
    }

    /*! @ingroup resizing

        \fn void ResizeBilinear(const View<A>& src, View<A>& dst)

        \short Performs resizing of input image with using bilinear interpolation.

        All images must have the same format (8-bit gray, 16-bit UV, 24-bit BGR or 32-bit BGRA).

        \note This function is a C++ wrapper for function ::SimdResizeBilinear.

        \param [in] src - an original input image.
        \param [out] dst - a resized output image.
    */
    template<template<class> class A> SIMD_INLINE void ResizeBilinear(const View<A> & src, View<A> & dst)
    {
        assert(src.format == dst.format && src.ChannelSize() == 1);

        if (EqualSize(src, dst))
        {
            Copy(src, dst);
        }
        else
        {
            SimdResizeBilinear(src.data, src.width, src.height, src.stride,
                dst.data, dst.width, dst.height, dst.stride, src.ChannelCount());
        }
    }

    /*! @ingroup resizing

        \fn void ResizeAreaGray(const View<A> & src, View<A> & dst)

        \short Performs resizing of input image with using area interpolation.

        All images must have the same format (8-bit gray).

        \param [in] src - an original input image.
        \param [out] dst - a resized output image.
    */
    template<template<class> class A> SIMD_INLINE void ResizeAreaGray(const View<A> & src, View<A> & dst)
    {
        assert(src.format == dst.format && src.format == View<A>::Gray8);

        if (EqualSize(src, dst))
        {
            Copy(src, dst);
        }
        else
        {
            size_t level = 0;
            for (; (dst.width << (level + 1)) < (size_t)src.width; level++);
            Point<ptrdiff_t> size = src.Size() << level;
            if (level)
            {
                Pyramid<A> pyramid(size, level + 1);
                Simd::ResizeBilinear(src, pyramid[0]);
                for (size_t i = 0; i < level; ++i)
                    Simd::ReduceGray(pyramid.At(i), pyramid.At(i + 1), ::SimdReduce2x2);
                Simd::Copy(pyramid[level], dst);
            }
            else
                Simd::ResizeBilinear(src, dst);
        }
    }

    /*! @ingroup resizing

        \fn void ResizeArea(const View<A> & src, View<A> & dst)

        \short Performs resizing of input image with using area interpolation.

        All images must have the same format.

        \param [in] src - an original input image.
        \param [out] dst - a resized output image.
    */
    template<template<class> class A> SIMD_INLINE void ResizeArea(const View<A> & src, View<A> & dst)
    {
        assert(src.format == dst.format);

        if (EqualSize(src, dst))
        {
            Copy(src, dst);
        }
        else
        {
            size_t level = 0;
            for (; (dst.width << (level + 1)) < (size_t)src.width; level++);
            Point<ptrdiff_t> size = src.Size() << level;
            if (level)
            {
                std::vector<View<A> > pyramid(level);
                pyramid[0].Resize(size, src.format);
                Simd::ResizeBilinear(src, pyramid[0]);
                for (size_t i = 1; i < level; ++i)
                {
                    size = Simd::Scale(size);
                    pyramid[i].Resize(size, src.format);
                    Simd::Reduce2x2(pyramid.At(i - 1), pyramid.At(i));
                }
                Simd::Reduce2x2(pyramid.At(level - 1), dst);
            }
            else
                Simd::ResizeBilinear(src, dst);
        }
    }

    /*! @ingroup resizing

        \fn void Resize(const View<A> & src, View<A> & dst, ::SimdResizeMethodType method = ::SimdResizeMethodBilinear)

        \short Performs resizing of image.

        All images must have the same format.

        \param [in] src - an original input image.
        \param [out] dst - a resized output image.
        \param [in] method - a resizing method. By default it is equal to ::SimdResizeMethodBilinear.
    */
    template<template<class> class A> SIMD_INLINE void Resize(const View<A> & src, View<A> & dst, ::SimdResizeMethodType method = ::SimdResizeMethodBilinear)
    {
        assert(src.format == dst.format && (src.format == View<A>::Float || src.ChannelSize() == 1));

        if (EqualSize(src, dst))
        {
            Copy(src, dst);
        }
        else
        {
            SimdResizeChannelType type = src.format == View<A>::Float ? SimdResizeChannelFloat : SimdResizeChannelByte;
            void * resizer = SimdResizerInit(src.width, src.height, dst.width, dst.height, src.ChannelCount(), type, method);
            if (resizer)
            {
                SimdResizerRun(resizer, src.data, src.stride, dst.data, dst.stride);
                SimdRelease(resizer);
            }
            else
                assert(0);
        }
    }

    /*! @ingroup resizing

        \fn void StretchGray2x2(const View<A>& src, View<A>& dst)

        \short Stretches input 8-bit gray image in two times.

        \note This function is a C++ wrapper for function ::SimdStretchGray2x2.

        \param [in] src - an original input image.
        \param [out] dst - a stretched output image.
    */
    template<template<class> class A> SIMD_INLINE void StretchGray2x2(const View<A> & src, View<A> & dst)
    {
        assert(src.format == View<A>::Gray8 && dst.format == View<A>::Gray8);
        assert(src.width * 2 == dst.width && src.height * 2 == dst.height);

        SimdStretchGray2x2(src.data, src.width, src.height, src.stride, dst.data, dst.width, dst.height, dst.stride);
    }

    /*! @ingroup universal_conversion

        \fn void Convert(const View<A> & src, View<A> & dst)

        \short Converts an image of one format to an image of another format.

        The input and output images must have the same width and height.

        \note This function supports conversion between Gray8, Bgr24 and Bgra32 image formats.

        \param [in] src - an input image.
        \param [out] dst - an output image.
    */
    template<template<class> class A> SIMD_INLINE void Convert(const View<A> & src, View<A> & dst)
    {
        assert(EqualSize(src, dst) && src.format && dst.format);

        if (src.format == dst.format)
        {
            Copy(src, dst);
            return;
        }

        switch (src.format)
        {
        case View<A>::Gray8:
            switch (dst.format)
            {
            case View<A>::Bgra32:
                GrayToBgra(src, dst);
                break;
            case View<A>::Bgr24:
                GrayToBgr(src, dst);
                break;
            default:
                assert(0);
            }
            break;

        case View<A>::Bgr24:
            switch (dst.format)
            {
            case View<A>::Bgra32:
                BgrToBgra(src, dst);
                break;
            case View<A>::Gray8:
                BgrToGray(src, dst);
                break;
            default:
                assert(0);
            }
            break;

        case View<A>::Bgra32:
            switch (dst.format)
            {
            case View<A>::Bgr24:
                BgraToBgr(src, dst);
                break;
            case View<A>::Gray8:
                BgraToGray(src, dst);
                break;
            default:
                assert(0);
            }
            break;

        default:
            assert(0);
        }
    }

    /*! @ingroup cpp_pyramid_functions

        \fn void Fill(Pyramid<A> & pyramid, uint8_t value)

        \short Fills pixels data of images in the pyramid by given value.

        \param [out] pyramid - a pyramid.
        \param [in] value - a value to fill the pyramid.
    */
    template<template<class> class A> SIMD_INLINE void Fill(Pyramid<A> & pyramid, uint8_t value)
    {
        for (size_t level = 0; level < pyramid.Size(); ++level)
            Simd::Fill(pyramid.At(level), value);
    }

    /*! @ingroup cpp_pyramid_functions

        \fn void Copy(const Pyramid<A> & src, Pyramid<A> & dst)

        \short Copies one pyramid to another pyramid.

        \note Input and output pyramids must have the same size.

        \param [in] src - an input pyramid.
        \param [out] dst - an output pyramid.
    */
    template<template<class> class A> SIMD_INLINE void Copy(const Pyramid<A> & src, Pyramid<A> & dst)
    {
        assert(src.Size() == dst.Size());
        for (size_t level = 0; level < src.Size(); ++level)
            Simd::Copy(src.At(level), dst.At(level));
    }

    /*! @ingroup cpp_pyramid_functions

        \fn void Build(Pyramid<A> & pyramid, ::SimdReduceType reduceType, bool compensation = true)

        \short Builds the pyramid (fills upper levels on the base of the lowest level).

        \param [out] pyramid - a built pyramid.
        \param [in] reduceType - a type of function used for image reducing.
        \param [in] compensation - a flag of compensation of rounding. It is relevant only for ::SimdReduce3x3 and ::SimdReduce5x5. It is equal to 'true' by default.
    */
    template<template<class> class A> SIMD_INLINE void Build(Pyramid<A> & pyramid, ::SimdReduceType reduceType, bool compensation = true)
    {
        for (size_t level = 1; level < pyramid.Size(); ++level)
            Simd::ReduceGray(pyramid.At(level - 1), pyramid.At(level), reduceType, compensation);
    }
}

#endif//__SimdLib_hpp__

