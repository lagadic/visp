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
#include "Simd/SimdConversion.h"

namespace Simd
{
#ifdef SIMD_AVX2_ENABLE
    namespace Avx2
    {
        template <bool align> SIMD_INLINE void BgraToRgba(const uint8_t * bgra, uint8_t * rgba)
        {
            Store<align>((__m256i*)rgba + 0, BgraToRgba(Load<align>((__m256i*)(bgra + 0))));
            Store<align>((__m256i*)rgba + 1, BgraToRgba(Load<align>((__m256i*)(bgra + 32))));
            Store<align>((__m256i*)rgba + 2, BgraToRgba(Load<align>((__m256i*)(bgra + 64))));
            Store<align>((__m256i*)rgba + 3, BgraToRgba(Load<align>((__m256i*)(bgra + 96))));
        }

        template <bool align> void BgraToRgba(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgba, size_t rgbaStride)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(bgra) && Aligned(bgraStride) && Aligned(rgba) && Aligned(rgbaStride));

            size_t alignedWidth = AlignLo(width, A);

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < alignedWidth; col += A)
                    BgraToRgba<align>(bgra + 4 * col, rgba + 4 * col);
                if (width != alignedWidth)
                    BgraToRgba<false>(bgra + 4 * (width - A), rgba + 4 * (width - A));
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
    // Work arround to avoid warning: libvisp_simdlib.a(SimdAvx2BgrToRgba.cpp.o) has no symbols
    void dummy_SimdAvx2BgraToRgba(){};
#endif//SIMD_AVX2_ENABLE
}
