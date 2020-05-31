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
#ifndef __SimdLoad_h__
#define __SimdLoad_h__

#include "Simd/SimdConst.h"

namespace Simd
{
    enum PadType
    {
        PadNose1,
        PadNone,
        PadTail1,
        PadTail2,
    };

#ifdef SIMD_SSE_ENABLE
    namespace Sse
    {
        template <bool align> SIMD_INLINE __m128 Load(const float * p);

        template <> SIMD_INLINE __m128 Load<false>(const float * p)
        {
            return _mm_loadu_ps(p);
        }

        template <> SIMD_INLINE __m128 Load<true>(const float * p)
        {
            return _mm_load_ps(p);
        }

        SIMD_INLINE __m128 Load(const float * p0, const float * p1)
        {
            return _mm_loadh_pi(_mm_loadl_pi(_mm_setzero_ps(), (__m64*)p0), (__m64*)p1);
        }

        SIMD_INLINE __m128 LoadPadZeroNose1(const float * p)
        {
            SIMD_ALIGNED(16) const int32_t m[F] = { 0, -1, -1, -1 };
            __m128 a = _mm_loadu_ps(p + 1);
            __m128 b = _mm_shuffle_ps(a, a, 0x90);
            return _mm_and_ps(b, _mm_load_ps((float*)m));
        }

        SIMD_INLINE __m128 LoadPadZeroTail1(const float * p)
        {
            SIMD_ALIGNED(16) const int32_t m[F] = { -1, -1, -1, 0 };
            __m128 a = _mm_loadu_ps(p - 1);
            __m128 b = _mm_shuffle_ps(a, a, 0xF9);
            return _mm_and_ps(b, _mm_load_ps((float*)m));
        }

        SIMD_INLINE __m128 LoadPadZeroTail2(const float * p)
        {
            SIMD_ALIGNED(16) const int32_t m[F] = { -1, -1, 0, 0 };
            __m128 a = _mm_loadu_ps(p - 2);
            __m128 b = _mm_shuffle_ps(a, a, 0xFE);
            return _mm_and_ps(b, _mm_load_ps((float*)m));
        }
    }
#endif//SIMD_SSE_ENABLE

#ifdef SIMD_SSE2_ENABLE
    namespace Sse2
    {
        using namespace Sse;

        template <bool align> SIMD_INLINE __m128i Load(const __m128i * p);

        template <> SIMD_INLINE __m128i Load<false>(const __m128i * p)
        {
            return _mm_loadu_si128(p);
        }

        template <> SIMD_INLINE __m128i Load<true>(const __m128i * p)
        {
            return _mm_load_si128(p);
        }

        template <bool align> SIMD_INLINE __m128i LoadMaskI8(const __m128i * p, __m128i index)
        {
            return _mm_cmpeq_epi8(Load<align>(p), index);
        }

        template <size_t count> SIMD_INLINE __m128i LoadBeforeFirst(__m128i first)
        {
            return _mm_or_si128(_mm_slli_si128(first, count), _mm_and_si128(first, _mm_srli_si128(K_INV_ZERO, A - count)));
        }

        template <size_t count> SIMD_INLINE __m128i LoadAfterLast(__m128i last)
        {
            return _mm_or_si128(_mm_srli_si128(last, count), _mm_and_si128(last, _mm_slli_si128(K_INV_ZERO, A - count)));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose3(const uint8_t * p, __m128i a[3])
        {
            a[1] = Load<align>((__m128i*)p);
            a[0] = LoadBeforeFirst<step>(a[1]);
            a[2] = _mm_loadu_si128((__m128i*)(p + step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody3(const uint8_t * p, __m128i a[3])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - step));
            a[1] = Load<align>((__m128i*)p);
            a[2] = _mm_loadu_si128((__m128i*)(p + step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail3(const uint8_t * p, __m128i a[3])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - step));
            a[1] = Load<align>((__m128i*)p);
            a[2] = LoadAfterLast<step>(a[1]);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose5(const uint8_t * p, __m128i a[5])
        {
            a[2] = Load<align>((__m128i*)p);
            a[1] = LoadBeforeFirst<step>(a[2]);
            a[0] = LoadBeforeFirst<step>(a[1]);
            a[3] = _mm_loadu_si128((__m128i*)(p + step));
            a[4] = _mm_loadu_si128((__m128i*)(p + 2 * step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody5(const uint8_t * p, __m128i a[5])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - 2 * step));
            a[1] = _mm_loadu_si128((__m128i*)(p - step));
            a[2] = Load<align>((__m128i*)p);
            a[3] = _mm_loadu_si128((__m128i*)(p + step));
            a[4] = _mm_loadu_si128((__m128i*)(p + 2 * step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail5(const uint8_t * p, __m128i a[5])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - 2 * step));
            a[1] = _mm_loadu_si128((__m128i*)(p - step));
            a[2] = Load<align>((__m128i*)p);
            a[3] = LoadAfterLast<step>(a[2]);
            a[4] = LoadAfterLast<step>(a[3]);
        }

        SIMD_INLINE void LoadNoseDx(const uint8_t * p, __m128i a[3])
        {
            a[0] = LoadBeforeFirst<1>(_mm_loadu_si128((__m128i*)p));
            a[2] = _mm_loadu_si128((__m128i*)(p + 1));
        }

        SIMD_INLINE void LoadBodyDx(const uint8_t * p, __m128i a[3])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - 1));
            a[2] = _mm_loadu_si128((__m128i*)(p + 1));
        }

        SIMD_INLINE void LoadTailDx(const uint8_t * p, __m128i a[3])
        {
            a[0] = _mm_loadu_si128((__m128i*)(p - 1));
            a[2] = LoadAfterLast<1>(_mm_loadu_si128((__m128i*)p));
        }
    }
#endif//SIMD_SSE2_ENABLE

#ifdef SIMD_SSE3_ENABLE
    namespace Sse3
    {
#if defined(_MSC_VER) && _MSC_VER >= 1700  && _MSC_VER < 1900 // Visual Studio 2012/2013 compiler bug      
        using Sse::Load;
        using Sse2::Load;
#endif
    }
#endif

#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
#if defined(_MSC_VER) && _MSC_VER >= 1700  && _MSC_VER < 1900 // Visual Studio 2012/2013 compiler bug      
        using Sse::Load;
        using Sse2::Load;
#endif
    }
#endif

#ifdef SIMD_AVX_ENABLE
    namespace Avx
    {
        template <bool align> SIMD_INLINE __m256 Load(const float * p);

        template <> SIMD_INLINE __m256 Load<false>(const float * p)
        {
            return _mm256_loadu_ps(p);
        }

        template <> SIMD_INLINE __m256 Load<true>(const float * p)
        {
            return _mm256_load_ps(p);
        }

        template<bool align> SIMD_INLINE __m256 Load(const float * p0, const float * p1)
        {
            return _mm256_insertf128_ps(_mm256_castps128_ps256(Sse::Load<align>(p0)), Sse::Load<align>(p1), 1);
        }

        SIMD_INLINE __m256 Load(const float * p0, const float * p1, const float * p2, const float * p3)
        {
            return _mm256_insertf128_ps(_mm256_castps128_ps256(Sse::Load(p0, p1)), Sse::Load(p2, p3), 1);
        }
    }
#endif//SIMD_AVX_ENABLE

#ifdef SIMD_AVX2_ENABLE
    namespace Avx2
    {
        using namespace Avx;

        template <bool align> SIMD_INLINE __m256i Load(const __m256i * p);

        template <> SIMD_INLINE __m256i Load<false>(const __m256i * p)
        {
            return _mm256_loadu_si256(p);
        }

        template <> SIMD_INLINE __m256i Load<true>(const __m256i * p)
        {
            return _mm256_load_si256(p);
        }

        template <bool align> SIMD_INLINE __m128i LoadHalf(const __m128i * p);

        template <> SIMD_INLINE __m128i LoadHalf<false>(const __m128i * p)
        {
            return _mm_loadu_si128(p);
        }

        template <> SIMD_INLINE __m128i LoadHalf<true>(const __m128i * p)
        {
            return _mm_load_si128(p);
        }

        template <size_t count> SIMD_INLINE __m128i LoadHalfBeforeFirst(__m128i first)
        {
            return _mm_or_si128(_mm_slli_si128(first, count), _mm_and_si128(first, _mm_srli_si128(Sse2::K_INV_ZERO, HA - count)));
        }

        template <size_t count> SIMD_INLINE __m128i LoadHalfAfterLast(__m128i last)
        {
            return _mm_or_si128(_mm_srli_si128(last, count), _mm_and_si128(last, _mm_slli_si128(Sse2::K_INV_ZERO, HA - count)));
        }

        template <bool align> SIMD_INLINE __m256i LoadPermuted(const __m256i * p)
        {
            return _mm256_permute4x64_epi64(Load<align>(p), 0xD8);
        }

        template <bool align> SIMD_INLINE __m256i LoadMaskI8(const __m256i * p, __m256i index)
        {
            return _mm256_cmpeq_epi8(Load<align>(p), index);
        }

        SIMD_INLINE __m256i PermutedUnpackLoU8(__m256i a, __m256i b = K_ZERO)
        {
            return _mm256_permute4x64_epi64(_mm256_unpacklo_epi8(a, b), 0xD8);
        }

        SIMD_INLINE __m256i PermutedUnpackHiU8(__m256i a, __m256i b = K_ZERO)
        {
            return _mm256_permute4x64_epi64(_mm256_unpackhi_epi8(a, b), 0xD8);
        }

        SIMD_INLINE __m256i PermutedUnpackLoU16(__m256i a, __m256i b = K_ZERO)
        {
            return _mm256_permute4x64_epi64(_mm256_unpacklo_epi16(a, b), 0xD8);
        }

        SIMD_INLINE __m256i PermutedUnpackHiU16(__m256i a, __m256i b = K_ZERO)
        {
            return _mm256_permute4x64_epi64(_mm256_unpackhi_epi16(a, b), 0xD8);
        }

        template <bool align, size_t step> SIMD_INLINE __m256i LoadBeforeFirst(const uint8_t * p)
        {
            __m128i lo = LoadHalfBeforeFirst<step>(LoadHalf<align>((__m128i*)p));
            __m128i hi = _mm_loadu_si128((__m128i*)(p + HA - step));
            return _mm256_inserti128_si256(_mm256_castsi128_si256(lo), hi, 0x1);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBeforeFirst(const uint8_t * p, __m256i & first, __m256i & second)
        {
            __m128i firstLo = LoadHalfBeforeFirst<step>(LoadHalf<align>((__m128i*)p));
            __m128i firstHi = _mm_loadu_si128((__m128i*)(p + HA - step));
            first = _mm256_inserti128_si256(_mm256_castsi128_si256(firstLo), firstHi, 0x1);

            __m128i secondLo = LoadHalfBeforeFirst<step>(firstLo);
            __m128i secondHi = _mm_loadu_si128((__m128i*)(p + HA - 2 * step));
            second = _mm256_inserti128_si256(_mm256_castsi128_si256(secondLo), secondHi, 0x1);
        }

        template <bool align, size_t step> SIMD_INLINE __m256i LoadAfterLast(const uint8_t * p)
        {
            __m128i lo = _mm_loadu_si128((__m128i*)(p + step));
            __m128i hi = LoadHalfAfterLast<step>(LoadHalf<align>((__m128i*)(p + HA)));
            return _mm256_inserti128_si256(_mm256_castsi128_si256(lo), hi, 0x1);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadAfterLast(const uint8_t * p, __m256i & first, __m256i & second)
        {
            __m128i firstLo = _mm_loadu_si128((__m128i*)(p + step));
            __m128i firstHi = LoadHalfAfterLast<step>(LoadHalf<align>((__m128i*)(p + HA)));
            first = _mm256_inserti128_si256(_mm256_castsi128_si256(firstLo), firstHi, 0x1);

            __m128i secondLo = _mm_loadu_si128((__m128i*)(p + 2 * step));
            __m128i secondHi = LoadHalfAfterLast<step>(firstHi);
            second = _mm256_inserti128_si256(_mm256_castsi128_si256(secondLo), secondHi, 0x1);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose3(const uint8_t * p, __m256i a[3])
        {
            a[0] = LoadBeforeFirst<align, step>(p);
            a[1] = Load<align>((__m256i*)p);
            a[2] = _mm256_loadu_si256((__m256i*)(p + step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody3(const uint8_t * p, __m256i a[3])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - step));
            a[1] = Load<align>((__m256i*)p);
            a[2] = _mm256_loadu_si256((__m256i*)(p + step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail3(const uint8_t * p, __m256i a[3])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - step));
            a[1] = Load<align>((__m256i*)p);
            a[2] = LoadAfterLast<align, step>(p);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose5(const uint8_t * p, __m256i a[5])
        {
            LoadBeforeFirst<align, step>(p, a[1], a[0]);
            a[2] = Load<align>((__m256i*)p);
            a[3] = _mm256_loadu_si256((__m256i*)(p + step));
            a[4] = _mm256_loadu_si256((__m256i*)(p + 2 * step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody5(const uint8_t * p, __m256i a[5])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - 2 * step));
            a[1] = _mm256_loadu_si256((__m256i*)(p - step));
            a[2] = Load<align>((__m256i*)p);
            a[3] = _mm256_loadu_si256((__m256i*)(p + step));
            a[4] = _mm256_loadu_si256((__m256i*)(p + 2 * step));
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail5(const uint8_t * p, __m256i a[5])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - 2 * step));
            a[1] = _mm256_loadu_si256((__m256i*)(p - step));
            a[2] = Load<align>((__m256i*)p);
            LoadAfterLast<align, step>(p, a[3], a[4]);
        }

        SIMD_INLINE void LoadNoseDx(const uint8_t * p, __m256i a[3])
        {
            a[0] = LoadBeforeFirst<false, 1>(p);
            a[2] = _mm256_loadu_si256((__m256i*)(p + 1));
        }

        SIMD_INLINE void LoadBodyDx(const uint8_t * p, __m256i a[3])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - 1));
            a[2] = _mm256_loadu_si256((__m256i*)(p + 1));
        }

        SIMD_INLINE void LoadTailDx(const uint8_t * p, __m256i a[3])
        {
            a[0] = _mm256_loadu_si256((__m256i*)(p - 1));
            a[2] = LoadAfterLast<false, 1>(p);
        }

        template <bool align> SIMD_INLINE __m256 Load(const float * p);

        template <> SIMD_INLINE __m256 Load<false>(const float * p)
        {
            return _mm256_loadu_ps(p);
        }

        template <> SIMD_INLINE __m256 Load<true>(const float * p)
        {
#ifdef _MSC_VER
            return _mm256_castsi256_ps(_mm256_load_si256((__m256i*)p));
#else
            return _mm256_load_ps(p);
#endif
        }
    }
#endif//SIMD_AVX2_ENABLE

#ifdef SIMD_NEON_ENABLE
    namespace Neon
    {
        template <bool align> SIMD_INLINE uint8x16_t Load(const uint8_t * p);

        template <> SIMD_INLINE uint8x16_t Load<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld1q_u8(p);
        }

        template <> SIMD_INLINE uint8x16_t Load<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            return vld1q_u8(_p);
#elif defined(_MSC_VER)
            return vld1q_u8_ex(p, 128);
#else
            return vld1q_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE int16x8_t Load(const int16_t * p)
        {
            return (int16x8_t)Load<align>((const uint8_t*)p);
        }

        template <bool align> SIMD_INLINE uint16x8_t Load(const uint16_t * p)
        {
            return (uint16x8_t)Load<align>((const uint8_t*)p);
        }

        template <bool align> SIMD_INLINE int32x4_t Load(const int32_t * p)
        {
            return (int32x4_t)Load<align>((const uint8_t*)p);
        }

        template <bool align> SIMD_INLINE uint32x4_t Load(const uint32_t * p)
        {
            return (uint32x4_t)Load<align>((const uint8_t*)p);
        }

        template <bool align> SIMD_INLINE float32x4_t Load(const float * p);

        template <> SIMD_INLINE float32x4_t Load<false>(const float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld1q_f32(p);
        }

        template <> SIMD_INLINE float32x4_t Load<true>(const float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            return vld1q_f32(_p);
#elif defined(_MSC_VER)
            return vld1q_f32_ex(p, 128);
#else
            return vld1q_f32(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x16x2_t Load2(const uint8_t * p);

        template <> SIMD_INLINE uint8x16x2_t Load2<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld2q_u8(p);
        }

        template <> SIMD_INLINE uint8x16x2_t Load2<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            return vld2q_u8(_p);
#elif defined(_MSC_VER)
            return vld2q_u8_ex(p, 128);
#else
            return vld2q_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE uint16x8x2_t Load2(const uint16_t * p);

        template <> SIMD_INLINE uint16x8x2_t Load2<false>(const uint16_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld2q_u16(p);
        }

        template <> SIMD_INLINE uint16x8x2_t Load2<true>(const uint16_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint16_t * _p = (uint16_t *)__builtin_assume_aligned(p, 16);
            return vld2q_u16(_p);
#elif defined(_MSC_VER)
            return vld2q_u16_ex(p, 128);
#else
            return vld2q_u16(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x16x3_t Load3(const uint8_t * p);

        template <> SIMD_INLINE uint8x16x3_t Load3<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld3q_u8(p);
        }

        template <> SIMD_INLINE uint8x16x3_t Load3<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            return vld3q_u8(_p);
#elif defined(_MSC_VER)
            return vld3q_u8_ex(p, 128);
#else
            return vld3q_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x16x4_t Load4(const uint8_t * p);

        template <> SIMD_INLINE uint8x16x4_t Load4<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld4q_u8(p);
        }

        template <> SIMD_INLINE uint8x16x4_t Load4<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            return vld4q_u8(_p);
#elif defined(_MSC_VER)
            return vld4q_u8_ex(p, 128);
#else
            return vld4q_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE float32x4x2_t Load2(const float * p);

        template <> SIMD_INLINE float32x4x2_t Load2<false>(const float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld2q_f32(p);
        }

        template <> SIMD_INLINE float32x4x2_t Load2<true>(const float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            return vld2q_f32(_p);
#elif defined(_MSC_VER)
            return vld2q_f32_ex(p, 128);
#else
            return vld2q_f32(p);
#endif
        }

        template <bool align> SIMD_INLINE float32x4x3_t Load3(const float * p);

        template <> SIMD_INLINE float32x4x3_t Load3<false>(const float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld3q_f32(p);
        }

        template <> SIMD_INLINE float32x4x3_t Load3<true>(const float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            return vld3q_f32(_p);
#elif defined(_MSC_VER)
            return vld3q_f32_ex(p, 128);
#else
            return vld3q_f32(p);
#endif
        }

        template <bool align> SIMD_INLINE float32x4x4_t Load4(const float * p);

        template <> SIMD_INLINE float32x4x4_t Load4<false>(const float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld4q_f32(p);
        }

        template <> SIMD_INLINE float32x4x4_t Load4<true>(const float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            return vld4q_f32(_p);
#elif defined(_MSC_VER)
            return vld4q_f32_ex(p, 128);
#else
            return vld4q_f32(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x8_t LoadHalf(const uint8_t * p);

        template <> SIMD_INLINE uint8x8_t LoadHalf<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld1_u8(p);
        }

        template <> SIMD_INLINE uint8x8_t LoadHalf<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            return vld1_u8(_p);
#elif defined(_MSC_VER)
            return vld1_u8_ex(p, 64);
#else
            return vld1_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE uint16x4_t LoadHalf(const uint16_t * p)
        {
            return (uint16x4_t)LoadHalf<align>((const uint8_t*)p);
        }

        template <bool align> SIMD_INLINE float32x2_t LoadHalf(const float * p);

        template <> SIMD_INLINE float32x2_t LoadHalf<false>(const  float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld1_f32(p);
        }

        template <> SIMD_INLINE float32x2_t LoadHalf<true>(const  float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 8);
            return vld1_f32(_p);
#elif defined(_MSC_VER)
            return vld1_f32_ex(p, 64);
#else
            return vld1_f32(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x8x2_t LoadHalf2(const uint8_t * p);

        template <> SIMD_INLINE uint8x8x2_t LoadHalf2<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld2_u8(p);
        }

        template <> SIMD_INLINE uint8x8x2_t LoadHalf2<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            return vld2_u8(_p);
#elif defined(_MSC_VER)
            return vld2_u8_ex(p, 64);
#else
            return vld2_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x8x3_t LoadHalf3(const uint8_t * p);

        template <> SIMD_INLINE uint8x8x3_t LoadHalf3<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld3_u8(p);
        }

        template <> SIMD_INLINE uint8x8x3_t LoadHalf3<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            return vld3_u8(_p);
#elif defined(_MSC_VER)
            return vld3_u8_ex(p, 64);
#else
            return vld3_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE uint8x8x4_t LoadHalf4(const uint8_t * p);

        template <> SIMD_INLINE uint8x8x4_t LoadHalf4<false>(const uint8_t * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld4_u8(p);
        }

        template <> SIMD_INLINE uint8x8x4_t LoadHalf4<true>(const uint8_t * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            return vld4_u8(_p);
#elif defined(_MSC_VER)
            return vld4_u8_ex(p, 64);
#else
            return vld4_u8(p);
#endif
        }

        template <bool align> SIMD_INLINE float32x2x4_t LoadHalf4(const float * p);

        template <> SIMD_INLINE float32x2x4_t LoadHalf4<false>(const float * p)
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            return vld4_f32(p);
        }

        template <> SIMD_INLINE float32x2x4_t LoadHalf4<true>(const float * p)
        {
#if defined(__GNUC__)
#if SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            float * _p = (float *)__builtin_assume_aligned(p, 8);
            return vld4_f32(_p);
#elif defined(_MSC_VER)
            return vld4_f32_ex(p, 64);
#else
            return vld4_f32(p);
#endif
        }

        template <size_t count> SIMD_INLINE uint8x16_t LoadBeforeFirst(uint8x16_t first)
        {
            return vextq_u8(vextq_u8(first, first, count), first, 16 - count);
        }

        template <size_t count> SIMD_INLINE uint8x16_t LoadAfterLast(uint8x16_t last)
        {
            return vextq_u8(last, vextq_u8(last, last, 16 - count), count);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose3(const uint8_t * p, uint8x16_t a[3])
        {
            a[1] = Load<align>(p);
            a[0] = LoadBeforeFirst<step>(a[1]);
            a[2] = vld1q_u8(p + step);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody3(const uint8_t * p, uint8x16_t a[3])
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            a[0] = vld1q_u8(p - step);
            a[1] = Load<align>(p);
            a[2] = vld1q_u8(p + step);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail3(const uint8_t * p, uint8x16_t a[3])
        {
            a[0] = vld1q_u8(p - step);
            a[1] = Load<align>(p);
            a[2] = LoadAfterLast<step>(a[1]);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadNose5(const uint8_t * p, uint8x16_t a[5])
        {
            a[2] = Load<align>(p);
            a[1] = LoadBeforeFirst<step>(a[2]);
            a[0] = LoadBeforeFirst<step>(a[1]);
            a[3] = vld1q_u8(p + step);
            a[4] = vld1q_u8(p + 2 * step);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadBody5(const uint8_t * p, uint8x16_t a[5])
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            a[0] = vld1q_u8(p - 2 * step);
            a[1] = vld1q_u8(p - step);
            a[2] = Load<align>(p);
            a[3] = vld1q_u8(p + step);
            a[4] = vld1q_u8(p + 2 * step);
        }

        template <bool align, size_t step> SIMD_INLINE void LoadTail5(const uint8_t * p, uint8x16_t a[5])
        {
            a[0] = vld1q_u8(p - 2 * step);
            a[1] = vld1q_u8(p - step);
            a[2] = Load<align>(p);
            a[3] = LoadAfterLast<step>(a[2]);
            a[4] = LoadAfterLast<step>(a[3]);
        }

        SIMD_INLINE void LoadNoseDx(const uint8_t * p, uint8x16_t a[3])
        {
            a[0] = LoadBeforeFirst<1>(vld1q_u8(p));
            a[2] = vld1q_u8(p + 1);
        }

        SIMD_INLINE void LoadBodyDx(const uint8_t * p, uint8x16_t a[3])
        {
#if defined(__GNUC__) && SIMD_NEON_PREFECH_SIZE
            __builtin_prefetch(p + SIMD_NEON_PREFECH_SIZE);
#endif
            a[0] = vld1q_u8(p - 1);
            a[2] = vld1q_u8(p + 1);
        }

        SIMD_INLINE void LoadTailDx(const uint8_t * p, uint8x16_t a[3])
        {
            a[0] = vld1q_u8(p - 1);
            a[2] = LoadAfterLast<1>(vld1q_u8(p));
        }

        template <size_t count> SIMD_INLINE uint8x8_t LoadBeforeFirst(uint8x8_t first)
        {
            return vext_u8(vext_u8(first, first, count), first, 8 - count);
        }

        template <size_t count> SIMD_INLINE uint8x8_t LoadAfterLast(uint8x8_t last)
        {
            return vext_u8(last, vext_u8(last, last, 8 - count), count);
        }

        SIMD_INLINE float32x4_t Load(const  float * p0, const  float * p1)
        {
            return vcombine_f32(vld1_f32(p0), vld1_f32(p1));
        }

        SIMD_INLINE float32x4_t LoadPadZeroNose1(const float * p)
        {
            return vextq_f32(vdupq_n_f32(0.0f), Load<false>(p + 1), 3);
        }

        SIMD_INLINE float32x4_t LoadPadZeroTail1(const float * p)
        {
            return vextq_f32(Load<false>(p - 1), vdupq_n_f32(0.0f), 1);
        }

        SIMD_INLINE float32x4_t LoadPadZeroTail2(const float * p)
        {
            return vextq_f32(Load<false>(p - 2), vdupq_n_f32(0.0f), 2);
        }
    }
#endif//SIMD_NEON_ENABLE
}
#endif//__SimdLoad_h__
