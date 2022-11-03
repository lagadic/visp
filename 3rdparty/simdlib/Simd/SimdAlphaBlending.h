/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2022 Yermalayeu Ihar.
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
#ifndef __SimdAlphaBlending_h__
#define __SimdAlphaBlending_h__

#include "Simd/SimdMath.h"
#include "Simd/SimdStore.h"

namespace Simd
{
    namespace Base
    {
        SIMD_INLINE int DivideBy255(int value)
        {
            return (value + 1 + (value >> 8)) >> 8;
        }

        template<bool argb> void AlphaPremultiply(const uint8_t* src, uint8_t* dst);

        template<> SIMD_INLINE void AlphaPremultiply<false>(const uint8_t* src, uint8_t* dst)
        {
            int alpha = src[3];
            dst[0] = DivideBy255(src[0] * alpha);
            dst[1] = DivideBy255(src[1] * alpha);
            dst[2] = DivideBy255(src[2] * alpha);
            dst[3] = alpha;
        }

        template<> SIMD_INLINE void AlphaPremultiply<true>(const uint8_t* src, uint8_t* dst)
        {
            int alpha = src[0];
            dst[0] = alpha;
            dst[1] = DivideBy255(src[1] * alpha);
            dst[2] = DivideBy255(src[2] * alpha);
            dst[3] = DivideBy255(src[3] * alpha);
        }

        template<bool argb> void AlphaUnpremultiply(const uint8_t* src, uint8_t* dst);

        template<> SIMD_INLINE void AlphaUnpremultiply<false>(const uint8_t* src, uint8_t* dst)
        {
            float alpha = src[3] ? 255.00001f / src[3] : 0.0f;
            dst[0] = RestrictRange(int(src[0] * alpha));
            dst[1] = RestrictRange(int(src[1] * alpha));
            dst[2] = RestrictRange(int(src[2] * alpha));
            dst[3] = src[3];
        }

        template<> SIMD_INLINE void AlphaUnpremultiply<true>(const uint8_t* src, uint8_t* dst)
        {
            float alpha = src[0] ? 255.00001f / src[0] : 0.0f;
            dst[0] = src[0];
            dst[1] = RestrictRange(int(src[1] * alpha));
            dst[2] = RestrictRange(int(src[2] * alpha));
            dst[3] = RestrictRange(int(src[3] * alpha));
        }
    }

#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
        SIMD_INLINE __m128i Divide16iBy255(__m128i value)
        {
            return _mm_mulhi_epi16(_mm_add_epi16(value, K16_0001), K16_0101);
        }

        SIMD_INLINE __m128i Divide16uBy255(__m128i value)
        {
            return _mm_mulhi_epu16(_mm_add_epi16(value, K16_0001), K16_0101);
        }

        SIMD_INLINE __m128i AlphaBlending16i(__m128i src, __m128i dst, __m128i alpha)
        {
            return Divide16uBy255(_mm_add_epi16(_mm_mullo_epi16(src, alpha), _mm_mullo_epi16(dst, _mm_sub_epi16(K16_00FF, alpha))));
        }

        template <bool align> SIMD_INLINE void AlphaBlending(const __m128i* src, __m128i* dst, __m128i alpha)
        {
            __m128i _src = Load<align>(src);
            __m128i _dst = Load<align>(dst);
            __m128i lo = AlphaBlending16i(_mm_unpacklo_epi8(_src, K_ZERO), _mm_unpacklo_epi8(_dst, K_ZERO), _mm_unpacklo_epi8(alpha, K_ZERO));
            __m128i hi = AlphaBlending16i(_mm_unpackhi_epi8(_src, K_ZERO), _mm_unpackhi_epi8(_dst, K_ZERO), _mm_unpackhi_epi8(alpha, K_ZERO));
            Store<align>(dst, _mm_packus_epi16(lo, hi));
        }

        template <bool align> SIMD_INLINE void AlphaBlending2x(const __m128i* src0, __m128i alpha0, const __m128i* src1, __m128i alpha1, __m128i* dst)
        {
            __m128i _dst = Load<align>(dst);
            __m128i lo = _mm_unpacklo_epi8(_dst, K_ZERO);
            __m128i hi = _mm_unpackhi_epi8(_dst, K_ZERO);
            __m128i _src0 = Load<align>(src0);
            lo = AlphaBlending16i(_mm_unpacklo_epi8(_src0, K_ZERO), lo, _mm_unpacklo_epi8(alpha0, K_ZERO));
            hi = AlphaBlending16i(_mm_unpackhi_epi8(_src0, K_ZERO), hi, _mm_unpackhi_epi8(alpha0, K_ZERO));
            __m128i _src1 = Load<align>(src1);
            lo = AlphaBlending16i(_mm_unpacklo_epi8(_src1, K_ZERO), lo, _mm_unpacklo_epi8(alpha1, K_ZERO));
            hi = AlphaBlending16i(_mm_unpackhi_epi8(_src1, K_ZERO), hi, _mm_unpackhi_epi8(alpha1, K_ZERO));
            Store<align>(dst, _mm_packus_epi16(lo, hi));
        }

        template <bool align> SIMD_INLINE void AlphaFilling(__m128i* dst, __m128i channelLo, __m128i channelHi, __m128i alpha)
        {
            __m128i _dst = Load<align>(dst);
            __m128i lo = AlphaBlending16i(channelLo, _mm_unpacklo_epi8(_dst, K_ZERO), _mm_unpacklo_epi8(alpha, K_ZERO));
            __m128i hi = AlphaBlending16i(channelHi, _mm_unpackhi_epi8(_dst, K_ZERO), _mm_unpackhi_epi8(alpha, K_ZERO));
            Store<align>(dst, _mm_packus_epi16(lo, hi));
        }

        SIMD_INLINE __m128i AlphaPremultiply16i(__m128i value, __m128i alpha)
        {
            return Divide16uBy255(_mm_mullo_epi16(value, alpha));
        }
    }
#endif

#ifdef SIMD_AVX2_ENABLE
    namespace Avx2
    {
        SIMD_INLINE __m256i Divide16iBy255(__m256i value)
        {
            return _mm256_mulhi_epi16(_mm256_add_epi16(value, K16_0001), K16_0101);
        }

        SIMD_INLINE __m256i Divide16uBy255(__m256i value)
        {
            return _mm256_mulhi_epu16(_mm256_add_epi16(value, K16_0001), K16_0101);
        }

        SIMD_INLINE __m256i AlphaBlending16i(__m256i src, __m256i dst, __m256i alpha)
        {
            return Divide16uBy255(_mm256_add_epi16(_mm256_mullo_epi16(src, alpha), _mm256_mullo_epi16(dst, _mm256_sub_epi16(K16_00FF, alpha))));
        }

        template <bool align> SIMD_INLINE void AlphaBlending(const __m256i* src, __m256i* dst, __m256i alpha)
        {
            __m256i _src = Load<align>(src);
            __m256i _dst = Load<align>(dst);
            __m256i lo = AlphaBlending16i(_mm256_unpacklo_epi8(_src, K_ZERO), _mm256_unpacklo_epi8(_dst, K_ZERO), _mm256_unpacklo_epi8(alpha, K_ZERO));
            __m256i hi = AlphaBlending16i(_mm256_unpackhi_epi8(_src, K_ZERO), _mm256_unpackhi_epi8(_dst, K_ZERO), _mm256_unpackhi_epi8(alpha, K_ZERO));
            Store<align>(dst, _mm256_packus_epi16(lo, hi));
        }

        template <bool align> SIMD_INLINE void AlphaBlending2x(const __m256i* src0, __m256i alpha0, const __m256i* src1, __m256i alpha1, __m256i* dst)
        {
            __m256i _dst = Load<align>(dst);
            __m256i lo = _mm256_unpacklo_epi8(_dst, K_ZERO);
            __m256i hi = _mm256_unpackhi_epi8(_dst, K_ZERO);
            __m256i _src0 = Load<align>(src0);
            lo = AlphaBlending16i(_mm256_unpacklo_epi8(_src0, K_ZERO), lo, _mm256_unpacklo_epi8(alpha0, K_ZERO));
            hi = AlphaBlending16i(_mm256_unpackhi_epi8(_src0, K_ZERO), hi, _mm256_unpackhi_epi8(alpha0, K_ZERO));
            __m256i _src1 = Load<align>(src1);
            lo = AlphaBlending16i(_mm256_unpacklo_epi8(_src1, K_ZERO), lo, _mm256_unpacklo_epi8(alpha1, K_ZERO));
            hi = AlphaBlending16i(_mm256_unpackhi_epi8(_src1, K_ZERO), hi, _mm256_unpackhi_epi8(alpha1, K_ZERO));
            Store<align>(dst, _mm256_packus_epi16(lo, hi));
        }
    }
#endif

#ifdef SIMD_AVX512BW_ENABLE
    namespace Avx512bw
    {
        SIMD_INLINE __m512i Divide16iBy255(__m512i value)
        {
            return _mm512_mulhi_epi16(_mm512_add_epi16(value, K16_0001), K16_0101);
        }

        SIMD_INLINE __m512i Divide16uBy255(__m512i value)
        {
            return _mm512_mulhi_epu16(_mm512_add_epi16(value, K16_0001), K16_0101);
        }
    }
#endif
}
#endif
