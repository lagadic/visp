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

namespace Simd
{
#ifdef SIMD_NEON_ENABLE    
    namespace Neon
    {
        namespace
        {
            struct Buffer
            {
                Buffer(size_t width)
                {
                    _p = Allocate(sizeof(uint16_t) * 4 * width);
                    src0 = (uint16_t*)_p;
                    src1 = src0 + width;
                    src2 = src1 + width;
                    src3 = src2 + width;
                }

                ~Buffer()
                {
                    Free(_p);
                }

                uint16_t * src0;
                uint16_t * src1;
                uint16_t * src2;
                uint16_t * src3;
            private:
                void * _p;
            };
        }

        SIMD_INLINE uint16x8_t DivideBy64(uint16x8_t value)
        {
            return vshrq_n_u16(vaddq_u16(value, K16_0020), 6);
        }

        SIMD_INLINE uint16x8_t ReduceColNose(const uint8_t * src)
        {
            const uint8x8x2_t t01 = Deinterleave(LoadBeforeFirst<1>(vld1q_u8(src)));
            const uint8x8x2_t t23 = vld2_u8(src + 1);
            return vaddq_u16(vaddl_u8(t01.val[0], t23.val[1]), vmulq_u16(vaddl_u8(t01.val[1], t23.val[0]), K16_0003));
        }

        SIMD_INLINE uint16x8_t ReduceColBody(const uint8_t *src)
        {
            const uint8x8x2_t t01 = vld2_u8(src - 1);
            const uint8x8x2_t t23 = vld2_u8(src + 1);
            return vaddq_u16(vaddl_u8(t01.val[0], t23.val[1]), vmulq_u16(vaddl_u8(t01.val[1], t23.val[0]), K16_0003));
        }

        template <bool even> SIMD_INLINE uint16x8_t ReduceColTail(const uint8_t *src);

        template <> SIMD_INLINE uint16x8_t ReduceColTail<true>(const uint8_t *src)
        {
            const uint8x8x2_t t01 = vld2_u8(src - 1);
            const uint8x8x2_t t23 = Deinterleave(LoadAfterLast<1>(vld1q_u8(src)));
            return vaddq_u16(vaddl_u8(t01.val[0], t23.val[1]), vmulq_u16(vaddl_u8(t01.val[1], t23.val[0]), K16_0003));
        }

        template <> SIMD_INLINE uint16x8_t ReduceColTail<false>(const uint8_t *src)
        {
            const uint8x8x2_t t01 = vld2_u8(src - 1);
            const uint8x8x2_t t23 = Deinterleave(LoadAfterLast<1>(LoadAfterLast<1>(vld1q_u8(src - 1))));
            return vaddq_u16(vaddl_u8(t01.val[0], t23.val[1]), vmulq_u16(vaddl_u8(t01.val[1], t23.val[0]), K16_0003));
        }

        template <bool align> SIMD_INLINE uint8x8_t ReduceRow(const Buffer & buffer, size_t offset)
        {
            return vmovn_u16(DivideBy64(BinomialSum16(
                Load<align>(buffer.src0 + offset), Load<align>(buffer.src1 + offset),
                Load<align>(buffer.src2 + offset), Load<align>(buffer.src3 + offset))));
        }

        template <bool even> void ReduceGray4x4(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
        {
            assert((srcWidth + 1) / 2 == dstWidth && (srcHeight + 1) / 2 == dstHeight && srcWidth > A);

            size_t alignedDstWidth = Simd::AlignLo(dstWidth, HA);
            size_t srcTail = Simd::AlignHi(srcWidth - A, 2);

            Buffer buffer(Simd::AlignHi(dstWidth, A));

            uint16x8_t tmp = ReduceColNose(src);
            Store<true>(buffer.src0, tmp);
            Store<true>(buffer.src1, tmp);
            size_t srcCol = A, dstCol = HA;
            for (; srcCol < srcWidth - A; srcCol += A, dstCol += HA)
            {
                tmp = ReduceColBody(src + srcCol);
                Store<true>(buffer.src0 + dstCol, tmp);
                Store<true>(buffer.src1 + dstCol, tmp);
            }
            tmp = ReduceColTail<even>(src + srcTail);
            Store<false>(buffer.src0 + dstWidth - HA, tmp);
            Store<false>(buffer.src1 + dstWidth - HA, tmp);

            for (size_t row = 0; row < srcHeight; row += 2, dst += dstStride)
            {
                const uint8_t *src2 = src + srcStride*(row + 1);
                const uint8_t *src3 = src2 + srcStride;
                if (row >= srcHeight - 2)
                {
                    src2 = src + srcStride*(srcHeight - 1);
                    src3 = src2;
                }

                Store<true>(buffer.src2, ReduceColNose(src2));
                Store<true>(buffer.src3, ReduceColNose(src3));
                size_t srcCol = A, dstCol = HA;
                for (; srcCol < srcWidth - A; srcCol += A, dstCol += HA)
                {
                    Store<true>(buffer.src2 + dstCol, ReduceColBody(src2 + srcCol));
                    Store<true>(buffer.src3 + dstCol, ReduceColBody(src3 + srcCol));
                }
                Store<false>(buffer.src2 + dstWidth - HA, ReduceColTail<even>(src2 + srcTail));
                Store<false>(buffer.src3 + dstWidth - HA, ReduceColTail<even>(src3 + srcTail));

                for (size_t col = 0; col < alignedDstWidth; col += HA)
                    vst1_u8(dst + col, ReduceRow<true>(buffer, col));

                if (alignedDstWidth != dstWidth)
                    vst1_u8(dst + dstWidth - HA, ReduceRow<false>(buffer, dstWidth - HA));

                Swap(buffer.src0, buffer.src2);
                Swap(buffer.src1, buffer.src3);
            }
        }

        void ReduceGray4x4(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
        {
            if (Aligned(srcWidth, 2))
                ReduceGray4x4<true>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
            else
                ReduceGray4x4<false>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
        }
    }
#endif// SIMD_NEON_ENABLE
}
