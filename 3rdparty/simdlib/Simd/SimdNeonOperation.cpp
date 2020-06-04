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
#include "Simd/SimdCompare.h"

namespace Simd
{
#ifdef SIMD_NEON_ENABLE  
    namespace Neon
    {
        template <SimdOperationBinary8uType type> SIMD_INLINE uint8x16_t OperationBinary8u(const uint8x16_t & a, const uint8x16_t & b);

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uAverage>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vrhaddq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uAnd>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vandq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uOr>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vorrq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uMaximum>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vmaxq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uMinimum>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vminq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uSaturatedSubtraction>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vqsubq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uSaturatedAddition>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vqaddq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uSubtraction>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vsubq_u8(a, b);
        }

        template <> SIMD_INLINE uint8x16_t OperationBinary8u<SimdOperationBinary8uAddition>(const uint8x16_t & a, const uint8x16_t & b)
        {
            return vaddq_u8(a, b);
        }

        template <bool align, SimdOperationBinary8uType type> void OperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
            size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride)
        {
            assert(width*channelCount >= A);
            if (align)
                assert(Aligned(a) && Aligned(aStride) && Aligned(b) && Aligned(bStride) && Aligned(dst) && Aligned(dstStride));

            size_t size = channelCount*width;
            size_t alignedSize = Simd::AlignLo(size, A);
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t offset = 0; offset < alignedSize; offset += A)
                {
                    const uint8x16_t a_ = Load<align>(a + offset);
                    const uint8x16_t b_ = Load<align>(b + offset);
                    Store<align>(dst + offset, OperationBinary8u<type>(a_, b_));
                }
                if (alignedSize != size)
                {
                    const uint8x16_t a_ = Load<false>(a + size - A);
                    const uint8x16_t b_ = Load<false>(b + size - A);
                    Store<false>(dst + size - A, OperationBinary8u<type>(a_, b_));
                }
                a += aStride;
                b += bStride;
                dst += dstStride;
            }
        }

        template <bool align> void OperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
            size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type)
        {
            switch (type)
            {
            case SimdOperationBinary8uAverage:
                return OperationBinary8u<align, SimdOperationBinary8uAverage>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uAnd:
                return OperationBinary8u<align, SimdOperationBinary8uAnd>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uOr:
                return OperationBinary8u<align, SimdOperationBinary8uOr>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uMaximum:
                return OperationBinary8u<align, SimdOperationBinary8uMaximum>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uMinimum:
                return OperationBinary8u<align, SimdOperationBinary8uMinimum>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uSaturatedSubtraction:
                return OperationBinary8u<align, SimdOperationBinary8uSaturatedSubtraction>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uSaturatedAddition:
                return OperationBinary8u<align, SimdOperationBinary8uSaturatedAddition>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uSubtraction:
                return OperationBinary8u<align, SimdOperationBinary8uSubtraction>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            case SimdOperationBinary8uAddition:
                return OperationBinary8u<align, SimdOperationBinary8uAddition>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride);
            default:
                assert(0);
            }
        }

        void OperationBinary8u(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride,
            size_t width, size_t height, size_t channelCount, uint8_t * dst, size_t dstStride, SimdOperationBinary8uType type)
        {
            if (Aligned(a) && Aligned(aStride) && Aligned(b) && Aligned(bStride) && Aligned(dst) && Aligned(dstStride))
                OperationBinary8u<true>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
            else
                OperationBinary8u<false>(a, aStride, b, bStride, width, height, channelCount, dst, dstStride, type);
        }
    }
#endif// SIMD_NEON_ENABLE
}
