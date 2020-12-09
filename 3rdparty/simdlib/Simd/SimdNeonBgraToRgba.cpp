/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2019 Yermalayeu Ihar.
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
#ifdef SIMD_NEON_ENABLE  
    namespace Neon
    {
        const size_t A4 = A * 4;

        union Bgra
        {
            uint8x16x4_t bgra;
        };

        template <bool align> SIMD_INLINE void BgraToRgba(const uint8_t * bgra, uint8_t * rgba, Bgra & _bgra)
        {
            _bgra.bgra = Load4<align>(bgra);
            uint8x16_t tmp = _bgra.bgra.val[0];
            _bgra.bgra.val[0] = _bgra.bgra.val[2];
            _bgra.bgra.val[2] = tmp;
            Store4<align>(rgba, _bgra.bgra);
        }

        template <bool align> void BgraToRgba(const uint8_t * bgra, size_t width, size_t height, size_t bgraStride, uint8_t * rgba, size_t rgbaStride)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgra) && Aligned(bgraStride));

            size_t alignedWidth = AlignLo(width, A);

            Bgra _bgra;

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, colRgba = 0; col < alignedWidth; col += A, colRgba += A4)
                    BgraToRgba<align>(bgra + colRgba, rgba + colRgba, _bgra);
                if (width != alignedWidth)
                    BgraToRgba<false>(bgra + 4 * (width - A), rgba + 4 * (width - A), _bgra);
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
#endif// SIMD_NEON_ENABLE
}
