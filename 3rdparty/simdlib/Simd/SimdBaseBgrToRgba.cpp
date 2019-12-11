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
#include "Simd/SimdDefs.h"
#include <algorithm>

namespace Simd
{
    namespace Base
    {
        void BgrToRgba(const uint8_t *bgr, size_t size, uint8_t *rgba, bool fillAlpha, bool lastRow, uint8_t alpha)
        {
            if (fillAlpha)
            {
#ifdef SIMD_BIG_ENDIAN
                const int32_t alphaMask = alpha;
#else
                const int32_t alphaMask = alpha << 24;
#endif
                for (size_t i = (lastRow ? 1 : 0); i < size; ++i, bgr += 3, rgba += 4)
                {
                    *(int32_t*)rgba = (*(int32_t*)bgr) | alphaMask;
                    std::swap(rgba[0], rgba[2]);
                }
                if (lastRow)
                {
                    rgba[0] = bgr[2];
                    rgba[1] = bgr[1];
                    rgba[2] = bgr[0];
                    rgba[3] = alpha;
                }
            }
            else
            {
                for (size_t i = (lastRow ? 1 : 0); i < size; ++i, bgr += 3, rgba += 4)
                {
                    *(int32_t*)rgba = (*(int32_t*)bgr);
                    std::swap(rgba[0], rgba[2]);
                }
                if (lastRow)
                {
                    rgba[0] = bgr[2];
                    rgba[1] = bgr[1];
                    rgba[2] = bgr[0];
                }
            }
        }

        void BgrToRgba(const uint8_t *bgr, size_t width, size_t height, size_t bgrStride, uint8_t *rgba, size_t bgraStride, uint8_t alpha)
        {
            for (size_t row = 1; row < height; ++row)
            {
                BgrToRgba(bgr, width, rgba, true, false, alpha);
                bgr += bgrStride;
                rgba += bgraStride;
            }
            BgrToRgba(bgr, width, rgba, true, true, alpha);
        }
    }
}
