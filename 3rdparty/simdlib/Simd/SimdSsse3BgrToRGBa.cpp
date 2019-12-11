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
#include "Simd/SimdStore.h"
#include "Simd/SimdMemory.h"

namespace Simd
{
#ifdef SIMD_SSSE3_ENABLE  
    namespace Ssse3
    {
        template <bool align> SIMD_INLINE void BgrToRgba(const uint8_t * bgr, uint8_t * rgba, __m128i alpha, __m128i shuffle)
        {
            Store<align>((__m128i*)rgba + 0, _mm_or_si128(alpha, _mm_shuffle_epi8(Load<align>((__m128i*)(bgr + 0)), shuffle)));
            Store<align>((__m128i*)rgba + 1, _mm_or_si128(alpha, _mm_shuffle_epi8(Load<false>((__m128i*)(bgr + 12)), shuffle)));
            Store<align>((__m128i*)rgba + 2, _mm_or_si128(alpha, _mm_shuffle_epi8(Load<false>((__m128i*)(bgr + 24)), shuffle)));
            Store<align>((__m128i*)rgba + 3, _mm_or_si128(alpha, _mm_shuffle_epi8(_mm_srli_si128(Load<align>((__m128i*)(bgr + 32)), 4), shuffle)));
        }

        template <bool align> void BgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgr) && Aligned(bgrStride));

            size_t alignedWidth = AlignLo(width, A);

            __m128i _alpha = _mm_slli_si128(_mm_set1_epi32(alpha), 3);
            __m128i _shuffle = _mm_setr_epi8(0x2, 0x1, 0x0, -1, 0x5, 0x4, 0x3, -1, 0x8, 0x7, 0x6, -1, 0xB, 0xA, 0x9, -1);

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < alignedWidth; col += A)
                    BgrToRgba<align>(bgr + 3 * col, rgba + 4 * col, _alpha, _shuffle);
                if (width != alignedWidth)
                    BgrToRgba<false>(bgr + 3 * (width - A), rgba + 4 * (width - A), _alpha, _shuffle);
                bgr += bgrStride;
                rgba += rgbaStride;
            }
        }

        void BgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha)
        {
            if (Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgr) && Aligned(bgrStride))
                BgrToRgba<true>(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
            else
                BgrToRgba<false>(bgr, width, height, bgrStride, rgba, rgbaStride, alpha);
        }
    }
#endif// SIMD_SSSE3_ENABLE
}
