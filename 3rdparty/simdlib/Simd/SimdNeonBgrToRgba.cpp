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
        const size_t A3 = A * 3;
        const size_t A4 = A * 4;

        union Bgra
        {
            uint8x16x4_t bgra;
            uint8x16x3_t bgr;
        };

        template <bool align> SIMD_INLINE void BgrToRgba(const uint8_t * bgr, uint8_t * rgba, Bgra & _bgra)
        {
            _bgra.bgr = Load3<align>(bgr);
            uint8x16_t tmp = _bgra.bgr.val[0];
            _bgra.bgr.val[0] = _bgra.bgr.val[2];
            _bgra.bgr.val[2] = tmp;
            Store4<align>(rgba, _bgra.bgra);
        }

        template <bool align> void BgrToRgba(const uint8_t * bgr, size_t width, size_t height, size_t bgrStride, uint8_t * rgba, size_t rgbaStride, uint8_t alpha)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(rgba) && Aligned(rgbaStride) && Aligned(bgr) && Aligned(bgrStride));

            size_t alignedWidth = AlignLo(width, A);

            Bgra _bgra;
            _bgra.bgra.val[3] = vdupq_n_u8(alpha);

            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, colRgba = 0, colBgr = 0; col < alignedWidth; col += A, colRgba += A4, colBgr += A3)
                    BgrToRgba<align>(bgr + colBgr, rgba + colRgba, _bgra);
                if (width != alignedWidth)
                    BgrToRgba<false>(bgr + 3 * (width - A), rgba + 4 * (width - A), _bgra);
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
#endif// SIMD_NEON_ENABLE
}
