/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2017 Yermalayeu Ihar.
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
#include "Simd/SimdMemory.h"
#include "Simd/SimdStore.h"
#include "Simd/SimdConversion.h"

namespace Simd
{
#ifdef SIMD_SSSE3_ENABLE    
    namespace Ssse3
    {
        template <bool align> SIMD_INLINE void YuvToBgr(__m128i y, __m128i u, __m128i v, __m128i * bgr)
        {
            __m128i blue = YuvToBlue(y, u);
            __m128i green = YuvToGreen(y, u, v);
            __m128i red = YuvToRed(y, v);
            Store<align>(bgr + 0, InterleaveBgr<0>(blue, green, red));
            Store<align>(bgr + 1, InterleaveBgr<1>(blue, green, red));
            Store<align>(bgr + 2, InterleaveBgr<2>(blue, green, red));
        }

        template <bool align> SIMD_INLINE void Yuv444pToBgr(const uint8_t * y, const uint8_t * u, const uint8_t * v, uint8_t * bgr)
        {
            YuvToBgr<align>(Load<align>((__m128i*)y), Load<align>((__m128i*)u), Load<align>((__m128i*)v), (__m128i*)bgr);
        }

        template <bool align> void Yuv444pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            assert(width >= A);
            if (align)
            {
                assert(Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride));
                assert(Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride));
            }

            size_t bodyWidth = AlignLo(width, A);
            size_t tail = width - bodyWidth;
            size_t A3 = A * 3;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t colYuv = 0, colBgr = 0; colYuv < bodyWidth; colYuv += A, colBgr += A3)
                {
                    Yuv444pToBgr<align>(y + colYuv, u + colYuv, v + colYuv, bgr + colBgr);
                }
                if (tail)
                {
                    size_t col = width - A;
                    Yuv444pToBgr<false>(y + col, u + col, v + col, bgr + 3 * col);
                }
                y += yStride;
                u += uStride;
                v += vStride;
                bgr += bgrStride;
            }
        }

        template <bool align> SIMD_INLINE void Yuv422pToBgr(const uint8_t * y, const __m128i & u, const __m128i & v, uint8_t * bgr)
        {
            YuvToBgr<align>(Load<align>((__m128i*)y + 0), _mm_unpacklo_epi8(u, u), _mm_unpacklo_epi8(v, v), (__m128i*)bgr + 0);
            YuvToBgr<align>(Load<align>((__m128i*)y + 1), _mm_unpackhi_epi8(u, u), _mm_unpackhi_epi8(v, v), (__m128i*)bgr + 3);
        }

        template <bool align> SIMD_INLINE void Yuv422pToBgr(const uint8_t * y, const uint8_t * u, const uint8_t * v, uint8_t * bgr)
        {
            Yuv422pToBgr<align>(y, Load<align>((__m128i*)u), Load<align>((__m128i*)v), bgr);
        }

        template <bool align> void Yuv422pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            assert((width % 2 == 0) && (width >= DA));
            if (align)
            {
                assert(Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride));
                assert(Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride));
            }

            size_t bodyWidth = AlignLo(width, DA);
            size_t tail = width - bodyWidth;
            size_t A6 = A * 6;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t colUV = 0, colY = 0, colBgr = 0; colY < bodyWidth; colY += DA, colUV += A, colBgr += A6)
                    Yuv422pToBgr<align>(y + colY, u + colUV, v + colUV, bgr + colBgr);
                if (tail)
                {
                    size_t offset = width - DA;
                    Yuv422pToBgr<false>(y + offset, u + offset / 2, v + offset / 2, bgr + 3 * offset);
                }
                y += yStride;
                u += uStride;
                v += vStride;
                bgr += bgrStride;
            }
        }

        void Yuv422pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            if (Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride)
                && Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride))
                Yuv422pToBgr<true>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
            else
                Yuv422pToBgr<false>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
        }

        template <bool align> void Yuv420pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            assert((width % 2 == 0) && (height % 2 == 0) && (width >= DA) && (height >= 2));
            if (align)
            {
                assert(Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride));
                assert(Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride));
            }

            size_t bodyWidth = AlignLo(width, DA);
            size_t tail = width - bodyWidth;
            size_t A6 = A * 6;
            for (size_t row = 0; row < height; row += 2)
            {
                for (size_t colUV = 0, colY = 0, colBgr = 0; colY < bodyWidth; colY += DA, colUV += A, colBgr += A6)
                {
                    __m128i u_ = Load<align>((__m128i*)(u + colUV));
                    __m128i v_ = Load<align>((__m128i*)(v + colUV));
                    Yuv422pToBgr<align>(y + colY, u_, v_, bgr + colBgr);
                    Yuv422pToBgr<align>(y + colY + yStride, u_, v_, bgr + colBgr + bgrStride);
                }
                if (tail)
                {
                    size_t offset = width - DA;
                    __m128i u_ = Load<false>((__m128i*)(u + offset / 2));
                    __m128i v_ = Load<false>((__m128i*)(v + offset / 2));
                    Yuv422pToBgr<false>(y + offset, u_, v_, bgr + 3 * offset);
                    Yuv422pToBgr<false>(y + offset + yStride, u_, v_, bgr + 3 * offset + bgrStride);
                }
                y += 2 * yStride;
                u += uStride;
                v += vStride;
                bgr += 2 * bgrStride;
            }
        }

        void Yuv420pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            if (Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride)
                && Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride))
                Yuv420pToBgr<true>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
            else
                Yuv420pToBgr<false>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
        }

        void Yuv444pToBgr(const uint8_t * y, size_t yStride, const uint8_t * u, size_t uStride, const uint8_t * v, size_t vStride,
            size_t width, size_t height, uint8_t * bgr, size_t bgrStride)
        {
            if (Aligned(y) && Aligned(yStride) && Aligned(u) && Aligned(uStride)
                && Aligned(v) && Aligned(vStride) && Aligned(bgr) && Aligned(bgrStride))
                Yuv444pToBgr<true>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
            else
                Yuv444pToBgr<false>(y, yStride, u, uStride, v, vStride, width, height, bgr, bgrStride);
        }
    }
#endif// SIMD_SSSE3_ENABLE
}
