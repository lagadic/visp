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
#include "Simd/SimdSet.h"

namespace Simd
{
#ifdef SIMD_AVX2_ENABLE    
    namespace Avx2
    {
        SIMD_INLINE __m256i AlphaBlendingI16(__m256i src, __m256i dst, __m256i alpha)
        {
            return DivideI16By255(_mm256_add_epi16(_mm256_mullo_epi16(src, alpha), _mm256_mullo_epi16(dst, _mm256_sub_epi16(K16_00FF, alpha))));
        }

        template <bool align> SIMD_INLINE void AlphaBlending(const __m256i * src, __m256i * dst, __m256i alpha)
        {
            __m256i _src = Load<align>(src);
            __m256i _dst = Load<align>(dst);
            __m256i lo = AlphaBlendingI16(_mm256_unpacklo_epi8(_src, K_ZERO), _mm256_unpacklo_epi8(_dst, K_ZERO), _mm256_unpacklo_epi8(alpha, K_ZERO));
            __m256i hi = AlphaBlendingI16(_mm256_unpackhi_epi8(_src, K_ZERO), _mm256_unpackhi_epi8(_dst, K_ZERO), _mm256_unpackhi_epi8(alpha, K_ZERO));
            Store<align>(dst, _mm256_packus_epi16(lo, hi));
        }

        template <bool align, size_t channelCount> struct AlphaBlender
        {
            void operator()(const __m256i * src, __m256i * dst, __m256i alpha);
        };

        template <bool align> struct AlphaBlender<align, 1>
        {
            SIMD_INLINE void operator()(const __m256i * src, __m256i * dst, __m256i alpha)
            {
                AlphaBlending<align>(src, dst, alpha);
            }
        };

        template <bool align> struct AlphaBlender<align, 2>
        {
            SIMD_INLINE void operator()(const __m256i * src, __m256i * dst, __m256i alpha)
            {
                alpha = _mm256_permute4x64_epi64(alpha, 0xD8);
                AlphaBlending<align>(src + 0, dst + 0, _mm256_unpacklo_epi8(alpha, alpha));
                AlphaBlending<align>(src + 1, dst + 1, _mm256_unpackhi_epi8(alpha, alpha));
            }
        };

        template <bool align> struct AlphaBlender<align, 3>
        {
            SIMD_INLINE void operator()(const __m256i * src, __m256i * dst, __m256i alpha)
            {
                AlphaBlending<align>(src + 0, dst + 0, GrayToBgr<0>(alpha));
                AlphaBlending<align>(src + 1, dst + 1, GrayToBgr<1>(alpha));
                AlphaBlending<align>(src + 2, dst + 2, GrayToBgr<2>(alpha));
            }
        };

        template <bool align> struct AlphaBlender<align, 4>
        {
            SIMD_INLINE void operator()(const __m256i * src, __m256i * dst, __m256i alpha)
            {
                alpha = _mm256_permute4x64_epi64(alpha, 0xD8);
                __m256i lo = _mm256_permute4x64_epi64(_mm256_unpacklo_epi8(alpha, alpha), 0xD8);
                AlphaBlending<align>(src + 0, dst + 0, _mm256_unpacklo_epi8(lo, lo));
                AlphaBlending<align>(src + 1, dst + 1, _mm256_unpackhi_epi8(lo, lo));
                __m256i hi = _mm256_permute4x64_epi64(_mm256_unpackhi_epi8(alpha, alpha), 0xD8);
                AlphaBlending<align>(src + 2, dst + 2, _mm256_unpacklo_epi8(hi, hi));
                AlphaBlending<align>(src + 3, dst + 3, _mm256_unpackhi_epi8(hi, hi));
            }
        };

        template <bool align, size_t channelCount> void AlphaBlending(const uint8_t *src, size_t srcStride, size_t width, size_t height,
            const uint8_t *alpha, size_t alphaStride, uint8_t *dst, size_t dstStride)
        {
            size_t alignedWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + alignedWidth, 0xFF);
            size_t step = channelCount * A;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, offset = 0; col < alignedWidth; col += A, offset += step)
                {
                    __m256i _alpha = Load<align>((__m256i*)(alpha + col));
                    AlphaBlender<align, channelCount>()((__m256i*)(src + offset), (__m256i*)(dst + offset), _alpha);
                }
                if (alignedWidth != width)
                {
                    __m256i _alpha = _mm256_and_si256(Load<false>((__m256i*)(alpha + width - A)), tailMask);
                    AlphaBlender<false, channelCount>()((__m256i*)(src + (width - A)*channelCount), (__m256i*)(dst + (width - A)*channelCount), _alpha);
                }
                src += srcStride;
                alpha += alphaStride;
                dst += dstStride;
            }
        }

        template <bool align> void AlphaBlending(const uint8_t *src, size_t srcStride, size_t width, size_t height, size_t channelCount,
            const uint8_t *alpha, size_t alphaStride, uint8_t *dst, size_t dstStride)
        {
            assert(width >= A);
            if (align)
            {
                assert(Aligned(src) && Aligned(srcStride));
                assert(Aligned(alpha) && Aligned(alphaStride));
                assert(Aligned(dst) && Aligned(dstStride));
            }

            switch (channelCount)
            {
            case 1: AlphaBlending<align, 1>(src, srcStride, width, height, alpha, alphaStride, dst, dstStride); break;
            case 2: AlphaBlending<align, 2>(src, srcStride, width, height, alpha, alphaStride, dst, dstStride); break;
            case 3: AlphaBlending<align, 3>(src, srcStride, width, height, alpha, alphaStride, dst, dstStride); break;
            case 4: AlphaBlending<align, 4>(src, srcStride, width, height, alpha, alphaStride, dst, dstStride); break;
            default:
                assert(0);
            }
        }

        void AlphaBlending(const uint8_t *src, size_t srcStride, size_t width, size_t height, size_t channelCount,
            const uint8_t *alpha, size_t alphaStride, uint8_t *dst, size_t dstStride)
        {
            if (Aligned(src) && Aligned(srcStride) && Aligned(alpha) && Aligned(alphaStride) && Aligned(dst) && Aligned(dstStride))
                AlphaBlending<true>(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
            else
                AlphaBlending<false>(src, srcStride, width, height, channelCount, alpha, alphaStride, dst, dstStride);
        }

        template <bool align> SIMD_INLINE void AlphaFilling(__m256i * dst, __m256i channelLo, __m256i channelHi, __m256i alpha)
        {
            __m256i _dst = Load<align>(dst);
            __m256i lo = AlphaBlendingI16(channelLo, _mm256_unpacklo_epi8(_dst, K_ZERO), _mm256_unpacklo_epi8(alpha, K_ZERO));
            __m256i hi = AlphaBlendingI16(channelHi, _mm256_unpackhi_epi8(_dst, K_ZERO), _mm256_unpackhi_epi8(alpha, K_ZERO));
            Store<align>(dst, _mm256_packus_epi16(lo, hi));
        }

        template <bool align, size_t channelCount> struct AlphaFiller
        {
            void operator() (__m256i * dst, const __m256i * channel, __m256i alpha);
        };

        template <bool align> struct AlphaFiller<align, 1>
        {
            SIMD_INLINE void operator()(__m256i * dst, const __m256i * channel, __m256i alpha)
            {
                AlphaFilling<align>(dst, channel[0], channel[0], alpha);
            }
        };

        template <bool align> struct AlphaFiller<align, 2>
        {
            SIMD_INLINE void operator()(__m256i * dst, const __m256i * channel, __m256i alpha)
            {
                alpha = _mm256_permute4x64_epi64(alpha, 0xD8);
                AlphaFilling<align>(dst + 0, channel[0], channel[0], UnpackU8<0>(alpha, alpha));
                AlphaFilling<align>(dst + 1, channel[0], channel[0], UnpackU8<1>(alpha, alpha));
            }
        };

        template <bool align> struct AlphaFiller<align, 3>
        {
            SIMD_INLINE void operator()(__m256i * dst, const __m256i * channel, __m256i alpha)
            {
                AlphaFilling<align>(dst + 0, channel[0], channel[1], GrayToBgr<0>(alpha));
                AlphaFilling<align>(dst + 1, channel[1], channel[2], GrayToBgr<1>(alpha));
                AlphaFilling<align>(dst + 2, channel[2], channel[0], GrayToBgr<2>(alpha));
            }
        };

        template <bool align> struct AlphaFiller<align, 4>
        {
            SIMD_INLINE void operator()(__m256i * dst, const __m256i * channel, __m256i alpha)
            {
                alpha = _mm256_permute4x64_epi64(alpha, 0xD8);
                __m256i lo = _mm256_permute4x64_epi64(_mm256_unpacklo_epi8(alpha, alpha), 0xD8);
                AlphaFilling<align>(dst + 0, channel[0], channel[0], UnpackU8<0>(lo, lo));
                AlphaFilling<align>(dst + 1, channel[0], channel[0], UnpackU8<1>(lo, lo));
                __m256i hi = _mm256_permute4x64_epi64(_mm256_unpackhi_epi8(alpha, alpha), 0xD8);
                AlphaFilling<align>(dst + 2, channel[0], channel[0], UnpackU8<0>(hi, hi));
                AlphaFilling<align>(dst + 3, channel[0], channel[0], UnpackU8<1>(hi, hi));
            }
        };

        template <bool align, size_t channelCount> void AlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const __m256i * channel, const uint8_t * alpha, size_t alphaStride)
        {
            size_t alignedWidth = AlignLo(width, A);
            __m256i tailMask = SetMask<uint8_t>(0, A - width + alignedWidth, 0xFF);
            size_t step = channelCount * A;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, offset = 0; col < alignedWidth; col += A, offset += step)
                {
                    __m256i _alpha = Load<align>((__m256i*)(alpha + col));
                    AlphaFiller<align, channelCount>()((__m256i*)(dst + offset), channel, _alpha);
                }
                if (alignedWidth != width)
                {
                    __m256i _alpha = _mm256_and_si256(Load<false>((__m256i*)(alpha + width - A)), tailMask);
                    AlphaFiller<false, channelCount>()((__m256i*)(dst + (width - A)*channelCount), channel, _alpha);
                }
                alpha += alphaStride;
                dst += dstStride;
            }
        }

        template <bool align> void AlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8_t * channel, size_t channelCount, const uint8_t * alpha, size_t alphaStride)
        {
            assert(width >= A);
            if (align)
            {
                assert(Aligned(dst) && Aligned(dstStride));
                assert(Aligned(alpha) && Aligned(alphaStride));
            }

            __m256i _channel[3];
            switch (channelCount)
            {
            case 1:
                _channel[0] = UnpackU8<0>(_mm256_set1_epi8(*(uint8_t*)channel));
                AlphaFilling<align, 1>(dst, dstStride, width, height, _channel, alpha, alphaStride);
                break;
            case 2:
                _channel[0] = UnpackU8<0>(_mm256_set1_epi16(*(uint16_t*)channel));
                AlphaFilling<align, 2>(dst, dstStride, width, height, _channel, alpha, alphaStride);
                break;
            case 3:
                _channel[0] = _mm256_setr_epi16(
                    channel[0], channel[1], channel[2], channel[0], channel[1], channel[2], channel[0], channel[1],
                    channel[1], channel[2], channel[0], channel[1], channel[2], channel[0], channel[1], channel[2]);
                _channel[1] = _mm256_setr_epi16(
                    channel[2], channel[0], channel[1], channel[2], channel[0], channel[1], channel[2], channel[0],
                    channel[0], channel[1], channel[2], channel[0], channel[1], channel[2], channel[0], channel[1]);
                _channel[2] = _mm256_setr_epi16(
                    channel[1], channel[2], channel[0], channel[1], channel[2], channel[0], channel[1], channel[2],
                    channel[2], channel[0], channel[1], channel[2], channel[0], channel[1], channel[2], channel[0]);
                AlphaFilling<align, 3>(dst, dstStride, width, height, _channel, alpha, alphaStride);
                break;
            case 4:
                _channel[0] = UnpackU8<0>(_mm256_set1_epi32(*(uint32_t*)channel));
                AlphaFilling<align, 4>(dst, dstStride, width, height, _channel, alpha, alphaStride);
                break;
            default:
                assert(0);
            }
        }

        void AlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8_t * channel, size_t channelCount, const uint8_t * alpha, size_t alphaStride)
        {
            if (Aligned(dst) && Aligned(dstStride) && Aligned(alpha) && Aligned(alphaStride))
                AlphaFilling<true>(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
            else
                AlphaFilling<false>(dst, dstStride, width, height, channel, channelCount, alpha, alphaStride);
        }
    }
#endif// SIMD_AVX2_ENABLE
}
