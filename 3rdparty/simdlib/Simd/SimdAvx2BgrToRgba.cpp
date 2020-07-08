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
        template <bool align> SIMD_INLINE void BgrToRgba(const uint8_t * bgr, uint8_t * rgba, __m256i alpha)
        {
            Store<align>((__m256i*)rgba + 0, BgrToRgba<false>(Load<align>((__m256i*)(bgr + 0)), alpha));
            Store<align>((__m256i*)rgba + 1, BgrToRgba<false>(Load<false>((__m256i*)(bgr + 24)), alpha));
            Store<align>((__m256i*)rgba + 2, BgrToRgba<false>(Load<false>((__m256i*)(bgr + 48)), alpha));
            Store<align>((__m256i*)rgba + 3, BgrToRgba<true >(Load<align>((__m256i*)(bgr + 64)), alpha));
        }

        template <bool align> void BgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(bgr) && Aligned(bgrStride) && Aligned(rgba) && Aligned(rgbaStride));

            size_t alignedWidth = AlignLo(width, A);

            __m256i _alpha = _mm256_slli_si256(_mm256_set1_epi32(alpha), 3);

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < alignedWidth; col += A)
                    BgrToRgba<align>(bgr + 3 * col, rgba + 4 * col, _alpha);
                if (width != alignedWidth)
                    BgrToRgba<false>(bgr + 3 * (width - A), rgba + 4 * (width - A), _alpha);
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
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdAvx2BgrToRgba.cpp.o) has no symbols
    void dummy_SimdAvx2BgrToRgba(){};
#endif//SIMD_AVX2_ENABLE
}
