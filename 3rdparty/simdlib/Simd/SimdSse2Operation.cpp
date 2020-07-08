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
#ifdef SIMD_SSE2_ENABLE
    namespace Sse2
    {
        template <SimdOperationBinary8uType type> SIMD_INLINE __m128i OperationBinary8u(const __m128i & a, const __m128i & b);

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uAverage>(const __m128i & a, const __m128i & b)
        {
            return _mm_avg_epu8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uAnd>(const __m128i & a, const __m128i & b)
        {
            return _mm_and_si128(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uOr>(const __m128i & a, const __m128i & b)
        {
            return _mm_or_si128(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uMaximum>(const __m128i & a, const __m128i & b)
        {
            return _mm_max_epu8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uMinimum>(const __m128i & a, const __m128i & b)
        {
            return _mm_min_epu8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uSaturatedSubtraction>(const __m128i & a, const __m128i & b)
        {
            return _mm_subs_epu8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uSaturatedAddition>(const __m128i & a, const __m128i & b)
        {
            return _mm_adds_epu8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uSubtraction>(const __m128i & a, const __m128i & b)
        {
            return _mm_sub_epi8(a, b);
        }

        template <> SIMD_INLINE __m128i OperationBinary8u<SimdOperationBinary8uAddition>(const __m128i & a, const __m128i & b)
        {
            return _mm_add_epi8(a, b);
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
                    const __m128i a_ = Load<align>((__m128i*)(a + offset));
                    const __m128i b_ = Load<align>((__m128i*)(b + offset));
                    Store<align>((__m128i*)(dst + offset), OperationBinary8u<type>(a_, b_));
                }
                if (alignedSize != size)
                {
                    const __m128i a_ = Load<false>((__m128i*)(a + size - A));
                    const __m128i b_ = Load<false>((__m128i*)(b + size - A));
                    Store<false>((__m128i*)(dst + size - A), OperationBinary8u<type>(a_, b_));
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
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdSse2Operation.cpp.o) has no symbols
    void dummy_SimdSse2Operation(){};
#endif// SIMD_SSE2_ENABLE
}
