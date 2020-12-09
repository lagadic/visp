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
        template <bool align> SIMD_INLINE void BgraToRgba(const uint8_t * bgra, uint8_t * rgba, __m128i shuffle)
        {
            Store<align>((__m128i*)rgba + 0, _mm_shuffle_epi8(Load<align>((__m128i*)(bgra + 0)), shuffle));
            Store<align>((__m128i*)rgba + 1, _mm_shuffle_epi8(Load<align>((__m128i*)(bgra + 16)), shuffle));
            Store<align>((__m128i*)rgba + 2, _mm_shuffle_epi8(Load<align>((__m128i*)(bgra + 32)), shuffle));
            Store<align>((__m128i*)rgba + 3, _mm_shuffle_epi8(Load<align>((__m128i*)(bgra + 48)), shuffle));
        }

        template <bool align> void BgraToRgba(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgba, size_t rgbaStride)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgra) && Aligned(bgraStride));

            size_t alignedWidth = AlignLo(width, A);

            __m128i _shuffle = _mm_setr_epi8(0x2, 0x1, 0x0, 0x3, 0x6, 0x5, 0x4, 0x7, 0xA, 0x9, 0x8, 0xB, 0xE, 0xD, 0xC, 0xF);

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < alignedWidth; col += A)
                    BgraToRgba<align>(bgra + 4 * col, rgba + 4 * col, _shuffle);
                if (width != alignedWidth)
                    BgraToRgba<false>(bgra + 4 * (width - A), rgba + 4 * (width - A), _shuffle);
                bgra += bgraStride;
                rgba += rgbaStride;
            }
        }

        void BgraToRgba(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgba, size_t rgbaStride)
        {
            if (Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgra) && Aligned(bgraStride))
                BgraToRgba<true>(bgra, width, height, bgraStride, rgba, rgbaStride);
            else
                BgraToRgba<false>(bgra, width, height, bgraStride, rgba, rgbaStride);
        }
    }
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdSsse3BgrToRGBa.cpp.o) has no symbols
    void dummy_SimdSsse3BgraToRGBa(){};
#endif// SIMD_SSSE3_ENABLE
}
