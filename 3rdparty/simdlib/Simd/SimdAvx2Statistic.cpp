/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2018 Yermalayeu Ihar.
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
#include "Simd/SimdExtract.h"
#include "Simd/SimdSet.h"

namespace Simd
{
#ifdef SIMD_AVX2_ENABLE    
    namespace Avx2
    {
        template <bool align> void GetStatistic(const uint8_t * src, size_t stride, size_t width, size_t height,
            uint8_t * min, uint8_t * max, uint8_t * average)
        {
            assert(width*height && width >= A);
            if (align)
                assert(Aligned(src) && Aligned(stride));

            size_t bodyWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + bodyWidth, 0xFF);
            __m256i sum = _mm256_setzero_si256();
            __m256i min_ = K_INV_ZERO;
            __m256i max_ = K_ZERO;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < bodyWidth; col += A)
                {
                    const __m256i value = Load<align>((__m256i*)(src + col));
                    min_ = _mm256_min_epu8(min_, value);
                    max_ = _mm256_max_epu8(max_, value);
                    sum = _mm256_add_epi64(_mm256_sad_epu8(value, K_ZERO), sum);
                }
                if (width - bodyWidth)
                {
                    const __m256i value = Load<false>((__m256i*)(src + width - A));
                    min_ = _mm256_min_epu8(min_, value);
                    max_ = _mm256_max_epu8(max_, value);
                    sum = _mm256_add_epi64(_mm256_sad_epu8(_mm256_and_si256(tailMask, value), K_ZERO), sum);
                }
                src += stride;
            }

            uint8_t min_buffer[A], max_buffer[A];
            _mm256_storeu_si256((__m256i*)min_buffer, min_);
            _mm256_storeu_si256((__m256i*)max_buffer, max_);
            *min = UCHAR_MAX;
            *max = 0;
            for (size_t i = 0; i < A; ++i)
            {
                *min = Base::MinU8(min_buffer[i], *min);
                *max = Base::MaxU8(max_buffer[i], *max);
            }
            *average = (uint8_t)((ExtractSum<uint64_t>(sum) + width*height / 2) / (width*height));
        }

        void GetStatistic(const uint8_t * src, size_t stride, size_t width, size_t height,
            uint8_t * min, uint8_t * max, uint8_t * average)
        {
            if (Aligned(src) && Aligned(stride))
                GetStatistic<true>(src, stride, width, height, min, max, average);
            else
                GetStatistic<false>(src, stride, width, height, min, max, average);
        }

        template <bool align> void GetRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            size_t alignedWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + alignedWidth, 0xFF);

            memset(sums, 0, sizeof(uint32_t)*height);
            for (size_t row = 0; row < height; ++row)
            {
                __m256i sum = _mm256_setzero_si256();
                for (size_t col = 0; col < alignedWidth; col += A)
                {
                    __m256i _src = Load<align>((__m256i*)(src + col));
                    sum = _mm256_add_epi32(sum, _mm256_sad_epu8(_src, K_ZERO));
                }
                if (alignedWidth != width)
                {
                    __m256i _src = _mm256_and_si256(Load<false>((__m256i*)(src + width - A)), tailMask);
                    sum = _mm256_add_epi32(sum, _mm256_sad_epu8(_src, K_ZERO));
                }
                sums[row] = ExtractSum<uint32_t>(sum);
                src += stride;
            }
        }

        void GetRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            if (Aligned(src) && Aligned(stride))
                GetRowSums<true>(src, stride, width, height, sums);
            else
                GetRowSums<false>(src, stride, width, height, sums);
        }

        namespace
        {
            struct Buffer
            {
                Buffer(size_t width)
                {
                    _p = Allocate(sizeof(uint16_t)*width + sizeof(uint32_t)*width);
                    sums16 = (uint16_t*)_p;
                    sums32 = (uint32_t*)(sums16 + width);
                }

                ~Buffer()
                {
                    Free(_p);
                }

                uint16_t * sums16;
                uint32_t * sums32;
            private:
                void *_p;
            };
        }

        SIMD_INLINE void Sum16(__m256i src8, uint16_t * sums16)
        {
            Store<true>((__m256i*)sums16 + 0, _mm256_add_epi16(Load<true>((__m256i*)sums16 + 0), _mm256_unpacklo_epi8(src8, K_ZERO)));
            Store<true>((__m256i*)sums16 + 1, _mm256_add_epi16(Load<true>((__m256i*)sums16 + 1), _mm256_unpackhi_epi8(src8, K_ZERO)));
        }

        SIMD_INLINE void Sum16To32(const uint16_t * src, uint32_t * dst)
        {
            __m256i lo = LoadPermuted<true>((__m256i*)src + 0);
            __m256i hi = LoadPermuted<true>((__m256i*)src + 1);
            Store<true>((__m256i*)dst + 0, _mm256_add_epi32(Load<true>((__m256i*)dst + 0), _mm256_unpacklo_epi16(lo, K_ZERO)));
            Store<true>((__m256i*)dst + 1, _mm256_add_epi32(Load<true>((__m256i*)dst + 1), _mm256_unpacklo_epi16(hi, K_ZERO)));
            Store<true>((__m256i*)dst + 2, _mm256_add_epi32(Load<true>((__m256i*)dst + 2), _mm256_unpackhi_epi16(lo, K_ZERO)));
            Store<true>((__m256i*)dst + 3, _mm256_add_epi32(Load<true>((__m256i*)dst + 3), _mm256_unpackhi_epi16(hi, K_ZERO)));
        }

        template <bool align> void GetColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            size_t alignedLoWidth = AlignLo(width, A);
            size_t alignedHiWidth = AlignHi(width, A);
            size_t stepSize = SCHAR_MAX + 1;
            size_t stepCount = (height + SCHAR_MAX) / stepSize;

            Buffer buffer(alignedHiWidth);
            memset(buffer.sums32, 0, sizeof(uint32_t)*alignedHiWidth);
            for (size_t step = 0; step < stepCount; ++step)
            {
                size_t rowStart = step*stepSize;
                size_t rowEnd = Min(rowStart + stepSize, height);

                memset(buffer.sums16, 0, sizeof(uint16_t)*alignedHiWidth);
                for (size_t row = rowStart; row < rowEnd; ++row)
                {
                    for (size_t col = 0; col < alignedLoWidth; col += A)
                    {
                        __m256i src8 = Load<align>((__m256i*)(src + col));
                        Sum16(src8, buffer.sums16 + col);
                    }
                    if (alignedLoWidth != width)
                    {
                        __m256i src8 = Load<false>((__m256i*)(src + width - A));
                        Sum16(src8, buffer.sums16 + alignedLoWidth);
                    }
                    src += stride;
                }

                for (size_t col = 0; col < alignedHiWidth; col += A)
                    Sum16To32(buffer.sums16 + col, buffer.sums32 + col);
            }
            memcpy(sums, buffer.sums32, sizeof(uint32_t)*alignedLoWidth);
            if (alignedLoWidth != width)
                memcpy(sums + alignedLoWidth, buffer.sums32 + alignedLoWidth + alignedHiWidth - width, sizeof(uint32_t)*(width - alignedLoWidth));
        }

        void GetColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            if (Aligned(src) && Aligned(stride))
                GetColSums<true>(src, stride, width, height, sums);
            else
                GetColSums<false>(src, stride, width, height, sums);
        }

        template <bool align> void GetAbsDyRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            size_t alignedWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + alignedWidth, 0xFF);

            memset(sums, 0, sizeof(uint32_t)*height);
            const uint8_t * src0 = src;
            const uint8_t * src1 = src + stride;
            height--;
            for (size_t row = 0; row < height; ++row)
            {
                __m256i sum = _mm256_setzero_si256();
                for (size_t col = 0; col < alignedWidth; col += A)
                {
                    __m256i _src0 = Load<align>((__m256i*)(src0 + col));
                    __m256i _src1 = Load<align>((__m256i*)(src1 + col));
                    sum = _mm256_add_epi32(sum, _mm256_sad_epu8(_src0, _src1));
                }
                if (alignedWidth != width)
                {
                    __m256i _src0 = _mm256_and_si256(Load<false>((__m256i*)(src0 + width - A)), tailMask);
                    __m256i _src1 = _mm256_and_si256(Load<false>((__m256i*)(src1 + width - A)), tailMask);
                    sum = _mm256_add_epi32(sum, _mm256_sad_epu8(_src0, _src1));
                }
                sums[row] = ExtractSum<uint32_t>(sum);
                src0 += stride;
                src1 += stride;
            }
        }

        void GetAbsDyRowSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            if (Aligned(src) && Aligned(stride))
                GetAbsDyRowSums<true>(src, stride, width, height, sums);
            else
                GetAbsDyRowSums<false>(src, stride, width, height, sums);
        }

        template <bool align> void GetAbsDxColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            width--;
            size_t alignedLoWidth = AlignLo(width, A);
            size_t alignedHiWidth = AlignHi(width, A);
            size_t stepSize = SCHAR_MAX + 1;
            size_t stepCount = (height + SCHAR_MAX) / stepSize;

            Buffer buffer(alignedHiWidth);
            memset(buffer.sums32, 0, sizeof(uint32_t)*alignedHiWidth);
            for (size_t step = 0; step < stepCount; ++step)
            {
                size_t rowStart = step*stepSize;
                size_t rowEnd = Min(rowStart + stepSize, height);

                memset(buffer.sums16, 0, sizeof(uint16_t)*alignedHiWidth);
                for (size_t row = rowStart; row < rowEnd; ++row)
                {
                    for (size_t col = 0; col < alignedLoWidth; col += A)
                    {
                        __m256i _src0 = Load<align>((__m256i*)(src + col + 0));
                        __m256i _src1 = Load<false>((__m256i*)(src + col + 1));
                        Sum16(AbsDifferenceU8(_src0, _src1), buffer.sums16 + col);
                    }
                    if (alignedLoWidth != width)
                    {
                        __m256i _src0 = Load<false>((__m256i*)(src + width - A + 0));
                        __m256i _src1 = Load<false>((__m256i*)(src + width - A + 1));
                        Sum16(AbsDifferenceU8(_src0, _src1), buffer.sums16 + alignedLoWidth);
                    }
                    src += stride;
                }

                for (size_t col = 0; col < alignedHiWidth; col += A)
                    Sum16To32(buffer.sums16 + col, buffer.sums32 + col);
            }
            memcpy(sums, buffer.sums32, sizeof(uint32_t)*alignedLoWidth);
            if (alignedLoWidth != width)
                memcpy(sums + alignedLoWidth, buffer.sums32 + alignedLoWidth + alignedHiWidth - width, sizeof(uint32_t)*(width - alignedLoWidth));
            sums[width] = 0;
        }

        void GetAbsDxColSums(const uint8_t * src, size_t stride, size_t width, size_t height, uint32_t * sums)
        {
            if (Aligned(src) && Aligned(stride))
                GetAbsDxColSums<true>(src, stride, width, height, sums);
            else
                GetAbsDxColSums<false>(src, stride, width, height, sums);
        }

        template <bool align> void ValueSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(src) && Aligned(stride));

            size_t bodyWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + bodyWidth, 0xFF);
            __m256i fullSum = _mm256_setzero_si256();
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0; col < bodyWidth; col += A)
                {
                    const __m256i src_ = Load<align>((__m256i*)(src + col));
                    fullSum = _mm256_add_epi64(_mm256_sad_epu8(src_, K_ZERO), fullSum);
                }
                if (width - bodyWidth)
                {
                    const __m256i src_ = _mm256_and_si256(tailMask, Load<false>((__m256i*)(src + width - A)));
                    fullSum = _mm256_add_epi64(_mm256_sad_epu8(src_, K_ZERO), fullSum);
                }
                src += stride;
            }
            *sum = ExtractSum<uint64_t>(fullSum);
        }

        void ValueSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
        {
            if (Aligned(src) && Aligned(stride))
                ValueSum<true>(src, stride, width, height, sum);
            else
                ValueSum<false>(src, stride, width, height, sum);
        }

        SIMD_INLINE __m256i Square(__m256i src)
        {
            const __m256i lo = _mm256_unpacklo_epi8(src, _mm256_setzero_si256());
            const __m256i hi = _mm256_unpackhi_epi8(src, _mm256_setzero_si256());
            return _mm256_add_epi32(_mm256_madd_epi16(lo, lo), _mm256_madd_epi16(hi, hi));
        }

        template <bool align> void SquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(src) && Aligned(stride));

            size_t bodyWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + bodyWidth, 0xFF);
            __m256i fullSum = _mm256_setzero_si256();
            for (size_t row = 0; row < height; ++row)
            {
                __m256i rowSum = _mm256_setzero_si256();
                for (size_t col = 0; col < bodyWidth; col += A)
                {
                    const __m256i src_ = Load<align>((__m256i*)(src + col));
                    rowSum = _mm256_add_epi32(rowSum, Square(src_));
                }
                if (width - bodyWidth)
                {
                    const __m256i src_ = _mm256_and_si256(tailMask, Load<false>((__m256i*)(src + width - A)));
                    rowSum = _mm256_add_epi32(rowSum, Square(src_));
                }
                fullSum = _mm256_add_epi64(fullSum, HorizontalSum32(rowSum));
                src += stride;
            }
            *sum = ExtractSum<uint64_t>(fullSum);
        }

        void SquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * sum)
        {
            if (Aligned(src) && Aligned(stride))
                SquareSum<true>(src, stride, width, height, sum);
            else
                SquareSum<false>(src, stride, width, height, sum);
        }

        template <bool align> void ValueSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * valueSum, uint64_t * squareSum)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(src) && Aligned(stride));

            size_t bodyWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + bodyWidth, 0xFF);
            __m256i fullValueSum = _mm256_setzero_si256();
            __m256i fullSquareSum = _mm256_setzero_si256();
            for (size_t row = 0; row < height; ++row)
            {
                __m256i rowSquareSum = _mm256_setzero_si256();
                for (size_t col = 0; col < bodyWidth; col += A)
                {
                    const __m256i value = Load<align>((__m256i*)(src + col));
                    fullValueSum = _mm256_add_epi64(_mm256_sad_epu8(value, K_ZERO), fullValueSum);
                    rowSquareSum = _mm256_add_epi32(rowSquareSum, Square(value));
                }
                if (width - bodyWidth)
                {
                    const __m256i value = _mm256_and_si256(tailMask, Load<false>((__m256i*)(src + width - A)));
                    fullValueSum = _mm256_add_epi64(_mm256_sad_epu8(value, K_ZERO), fullValueSum);
                    rowSquareSum = _mm256_add_epi32(rowSquareSum, Square(value));
                }
                fullSquareSum = _mm256_add_epi64(fullSquareSum, HorizontalSum32(rowSquareSum));
                src += stride;
            }
            *valueSum = ExtractSum<uint64_t>(fullValueSum);
            *squareSum = ExtractSum<uint64_t>(fullSquareSum);
        }

        void ValueSquareSum(const uint8_t * src, size_t stride, size_t width, size_t height, uint64_t * valueSum, uint64_t * squareSum)
        {
            if (Aligned(src) && Aligned(stride))
                ValueSquareSum<true>(src, stride, width, height, valueSum, squareSum);
            else
                ValueSquareSum<false>(src, stride, width, height, valueSum, squareSum);
        }
		
        SIMD_INLINE __m256i Correlation(__m256i a, __m256i b)
        {
            const __m256i lo = _mm256_madd_epi16(_mm256_unpacklo_epi8(a, _mm256_setzero_si256()), _mm256_unpacklo_epi8(b, _mm256_setzero_si256()));
            const __m256i hi = _mm256_madd_epi16(_mm256_unpackhi_epi8(a, _mm256_setzero_si256()), _mm256_unpackhi_epi8(b, _mm256_setzero_si256()));
            return _mm256_add_epi32(lo, hi);
        }

        template <bool align> void CorrelationSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum)
        {
            assert(width >= A);
            if (align)
                assert(Aligned(a) && Aligned(aStride) && Aligned(b) && Aligned(bStride));

            size_t bodyWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + bodyWidth, 0xFF);
            __m256i fullSum = _mm256_setzero_si256();
            for (size_t row = 0; row < height; ++row)
            {
                __m256i rowSum = _mm256_setzero_si256();
                for (size_t col = 0; col < bodyWidth; col += A)
                {
                    const __m256i a_ = Load<align>((__m256i*)(a + col));
                    const __m256i b_ = Load<align>((__m256i*)(b + col));
                    rowSum = _mm256_add_epi32(rowSum, Correlation(a_, b_));
                }
                if (width - bodyWidth)
                {
                    const __m256i a_ = _mm256_and_si256(tailMask, Load<false>((__m256i*)(a + width - A)));
                    const __m256i b_ = _mm256_and_si256(tailMask, Load<false>((__m256i*)(b + width - A)));
                    rowSum = _mm256_add_epi32(rowSum, Correlation(a_, b_));
                }
                fullSum = _mm256_add_epi64(fullSum, HorizontalSum32(rowSum));
                a += aStride;
                b += bStride;
            }
            *sum = ExtractSum<uint64_t>(fullSum);
        }

        void CorrelationSum(const uint8_t * a, size_t aStride, const uint8_t * b, size_t bStride, size_t width, size_t height, uint64_t * sum)
        {
            if (Aligned(a) && Aligned(aStride) && Aligned(b) && Aligned(bStride))
                CorrelationSum<true>(a, aStride, b, bStride, width, height, sum);
            else
                CorrelationSum<false>(a, aStride, b, bStride, width, height, sum);
        }
    }
#endif// SIMD_AVX2_ENABLE
}
