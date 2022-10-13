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
#ifndef __SimdStore_h__
#define __SimdStore_h__

#include "Simd/SimdDefs.h"
#include "Simd/SimdLoad.h"
#include "Simd/SimdMath.h"
#include "Simd/SimdLog.h"

namespace Simd
{
#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
        template <bool align> SIMD_INLINE void Store(float* p, __m128 a);

        template <> SIMD_INLINE void Store<false>(float* p, __m128 a)
        {
            _mm_storeu_ps(p, a);
        }

        template <> SIMD_INLINE void Store<true>(float* p, __m128 a)
        {
            _mm_store_ps(p, a);
        }

        template <int part> SIMD_INLINE void StoreHalf(float* p, __m128 a);

        template <> SIMD_INLINE void StoreHalf<0>(float* p, __m128 a)
        {
            _mm_storel_pi((__m64*)p, a);
        }

        template <> SIMD_INLINE void StoreHalf<1>(float* p, __m128 a)
        {
            _mm_storeh_pi((__m64*)p, a);
        }

        SIMD_INLINE void Store(float* ptr, __m128 val, size_t size)
        {
            SIMD_ALIGNED(16) float buf[F];
            _mm_store_ps(buf, val);
            for (size_t i = 0; i < size; ++i)
                ptr[i] = buf[i];
        }

        template<int step> SIMD_INLINE void Scater(float* ptr, __m128 val)
        {
            SIMD_ALIGNED(16) float buf[F];
            _mm_store_ps(buf, val);
            ptr[0 * step] = buf[0];
            ptr[1 * step] = buf[1];
            ptr[2 * step] = buf[2];
            ptr[3 * step] = buf[3];
        }

        template<int step> SIMD_INLINE void Scater(float* ptr, __m128 val, size_t size)
        {
            SIMD_ALIGNED(16) float buf[F];
            _mm_store_ps(buf, val);
            for (size_t i = 0; i < size; ++i)
                ptr[i * step] = buf[i];
        }

        template <bool align> SIMD_INLINE void Store(__m128i * p, __m128i a);

        template <> SIMD_INLINE void Store<false>(__m128i * p, __m128i a)
        {
            _mm_storeu_si128(p, a);
        }

        template <> SIMD_INLINE void Store<true>(__m128i * p, __m128i a)
        {
            _mm_store_si128(p, a);
        }

        template <int part> SIMD_INLINE void StoreHalf(__m128i * p, __m128i a)
        {
            StoreHalf<part>((float*)p, _mm_castsi128_ps(a));
        }

        template <bool align> SIMD_INLINE void StoreMasked(float* p, __m128 value, __m128 mask)
        {
            __m128 old = Load<align>(p);
            Store<align>(p, _mm_blendv_ps(old, value, mask));
        }

        template <bool align> SIMD_INLINE void StoreMasked(__m128i* p, __m128i value, __m128i mask)
        {
            __m128i old = Load<align>(p);
            Store<align>(p, _mm_blendv_epi8(old, value, mask));
        }
    }
#endif

#ifdef SIMD_AVX_ENABLE
    namespace Avx
    {
        template <bool align> SIMD_INLINE void Store(float * p, __m256 a);

        template <> SIMD_INLINE void Store<false>(float * p, __m256 a)
        {
            _mm256_storeu_ps(p, a);
        }

        template <> SIMD_INLINE void Store<true>(float * p, __m256 a)
        {
            _mm256_store_ps(p, a);
        }

        SIMD_INLINE void Store(float* ptr, __m256 val, size_t size)
        {
            SIMD_ALIGNED(32) float buf[F];
            _mm256_store_ps(buf, val);
            for (size_t i = 0; i < size; ++i)
                ptr[i] = buf[i];
        }

        template <bool align> SIMD_INLINE void Store(float * p0, float * p1, __m256 a)
        {
            Sse41::Store<align>(p0, _mm256_extractf128_ps(a, 0));
            Sse41::Store<align>(p1, _mm256_extractf128_ps(a, 1));
        }

        template <bool align> SIMD_INLINE void StoreMasked(float * p, __m256 value, __m256 mask)
        {
            __m256 old = Load<align>(p);
            Store<align>(p, _mm256_blendv_ps(old, value, mask));
        }

        template<int step> SIMD_INLINE void Scater(float* ptr, __m256 val)
        {
            SIMD_ALIGNED(32) float buf[F];
            _mm256_store_ps(buf, val);
            ptr[0 * step] = buf[0];
            ptr[1 * step] = buf[1];
            ptr[2 * step] = buf[2];
            ptr[3 * step] = buf[3];
            ptr[4 * step] = buf[4];
            ptr[5 * step] = buf[5];
            ptr[6 * step] = buf[6];
            ptr[7 * step] = buf[7];
        }

        template<int step> SIMD_INLINE void Scater(float* ptr, __m256 val, size_t size)
        {
            SIMD_ALIGNED(32) float buf[F];
            _mm256_store_ps(buf, val);
            for (size_t i = 0; i < size; ++i)
                ptr[i * step] = buf[i];
        }
    }
#endif

#ifdef SIMD_AVX2_ENABLE
    namespace Avx2
    {
        using namespace Avx;

        template <bool align> SIMD_INLINE void Store(__m256i * p, __m256i a);

        template <> SIMD_INLINE void Store<false>(__m256i * p, __m256i a)
        {
            _mm256_storeu_si256(p, a);
        }

        template <> SIMD_INLINE void Store<true>(__m256i * p, __m256i a)
        {
            _mm256_store_si256(p, a);
        }

        template <bool align> SIMD_INLINE void Store(__m128i* p0, __m128i* p1, __m256i a)
        {
            Sse41::Store<align>(p0, _mm256_extractf128_si256(a, 0));
            Sse41::Store<align>(p1, _mm256_extractf128_si256(a, 1));
        }

        template <bool align> SIMD_INLINE void StoreMasked(__m256i * p, __m256i value, __m256i mask)
        {
            __m256i old = Load<align>(p);
            Store<align>(p, _mm256_blendv_epi8(old, value, mask));
        }

        SIMD_INLINE __m256i PackI16ToI8(__m256i lo, __m256i hi)
        {
            return _mm256_permute4x64_epi64(_mm256_packs_epi16(lo, hi), 0xD8);
        }

        SIMD_INLINE __m256i PackI16ToU8(__m256i lo, __m256i hi)
        {
            return _mm256_permute4x64_epi64(_mm256_packus_epi16(lo, hi), 0xD8);
        }

        SIMD_INLINE __m256i PackI32ToI16(__m256i lo, __m256i hi)
        {
            return _mm256_permute4x64_epi64(_mm256_packs_epi32(lo, hi), 0xD8);
        }

        SIMD_INLINE __m256i PackU32ToI16(__m256i lo, __m256i hi)
        {
            return _mm256_permute4x64_epi64(_mm256_packus_epi32(lo, hi), 0xD8);
        }

        SIMD_INLINE void Permute2x128(__m256i & lo, __m256i & hi)
        {
            __m256i _lo = lo;
            lo = _mm256_permute2x128_si256(lo, hi, 0x20);
            hi = _mm256_permute2x128_si256(_lo, hi, 0x31);
        }

        template <bool align> SIMD_INLINE void Store24(uint8_t * p, __m256i a)
        {
            Sse41::Store<align>((__m128i*)p, _mm256_extractf128_si256(a, 0));
            Sse41::StoreHalf<0>((__m128i*)p + 1, _mm256_extractf128_si256(a, 1));
        }
    }
#endif//SIMD_SAVX2_ENABLE

#ifdef SIMD_NEON_ENABLE
    namespace Neon
    {
        template <bool align> SIMD_INLINE void Store(uint8_t * p, uint8x16_t a);

        template <> SIMD_INLINE void Store<false>(uint8_t * p, uint8x16_t a)
        {
            vst1q_u8(p, a);
        }

        template <> SIMD_INLINE void Store<true>(uint8_t * p, uint8x16_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            vst1q_u8(_p, a);
#elif defined(_MSC_VER)
            vst1q_u8_ex(p, a, 128);
#else
            vst1q_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store(int8_t* p, int8x16_t a)
        {
            Store<align>((uint8_t*)p, vreinterpretq_u8_s8(a));
        }

        template <bool align> SIMD_INLINE void Store(uint8_t * p, uint8x8_t a);

        template <> SIMD_INLINE void Store<false>(uint8_t * p, uint8x8_t a)
        {
            vst1_u8(p, a);
        }

        template <> SIMD_INLINE void Store<true>(uint8_t * p, uint8x8_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            vst1_u8(_p, a);
#elif defined(_MSC_VER)
            vst1_u8_ex(p, a, 64);
#else
            vst1_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store(uint16_t * p, uint16x8_t a)
        {
            Store<align>((uint8_t*)p, vreinterpretq_u8_u16(a));
        }

        template <bool align> SIMD_INLINE void Store(uint16_t * p, uint16x4_t a)
        {
            Store<align>((uint8_t*)p, vreinterpret_u8_u16(a));
        }

        template <bool align> SIMD_INLINE void Store(int16_t * p, int16x8_t a)
        {
            Store<align>((uint8_t*)p, vreinterpretq_u8_s16(a));
        }

        template <bool align> SIMD_INLINE void Store(uint32_t * p, uint32x4_t a)
        {
            Store<align>((uint8_t*)p, vreinterpretq_u8_u32(a));
        }

        template <bool align> SIMD_INLINE void Store(int32_t * p, int32x4_t a)
        {
            Store<align>((uint8_t*)p, vreinterpretq_u8_s32(a));
        }

        template <bool align> SIMD_INLINE void Store2(uint8_t * p, uint8x16x2_t a);

        template <> SIMD_INLINE void Store2<false>(uint8_t * p, uint8x16x2_t a)
        {
            vst2q_u8(p, a);
        }

        template <> SIMD_INLINE void Store2<true>(uint8_t * p, uint8x16x2_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            vst2q_u8(_p, a);
#elif defined(_MSC_VER)
            vst2q_u8_ex(p, a, 128);
#else
            vst2q_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store2(int16_t * p, int16x8x2_t a);

        template <> SIMD_INLINE void Store2<false>(int16_t * p, int16x8x2_t a)
        {
            vst2q_s16(p, a);
        }

        template <> SIMD_INLINE void Store2<true>(int16_t * p, int16x8x2_t a)
        {
#if defined(__GNUC__)
            int16_t * _p = (int16_t *)__builtin_assume_aligned(p, 16);
            vst2q_s16(_p, a);
#elif defined(_MSC_VER)
            vst2q_s16_ex(p, a, 128);
#else
            vst2q_s16(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store2(uint8_t * p, uint8x8x2_t a);

        template <> SIMD_INLINE void Store2<false>(uint8_t * p, uint8x8x2_t a)
        {
            vst2_u8(p, a);
        }

        template <> SIMD_INLINE void Store2<true>(uint8_t * p, uint8x8x2_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            vst2_u8(_p, a);
#elif defined(_MSC_VER)
            vst2_u8_ex(p, a, 64);
#else
            vst2_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store3(uint8_t * p, uint8x16x3_t a);

        template <> SIMD_INLINE void Store3<false>(uint8_t * p, uint8x16x3_t a)
        {
            vst3q_u8(p, a);
        }

        template <> SIMD_INLINE void Store3<true>(uint8_t * p, uint8x16x3_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            vst3q_u8(_p, a);
#elif defined(_MSC_VER)
            vst3q_u8_ex(p, a, 128);
#else
            vst3q_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store3(uint8_t * p, uint8x8x3_t a);

        template <> SIMD_INLINE void Store3<false>(uint8_t * p, uint8x8x3_t a)
        {
            vst3_u8(p, a);
        }

        template <> SIMD_INLINE void Store3<true>(uint8_t * p, uint8x8x3_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            vst3_u8(_p, a);
#elif defined(_MSC_VER)
            vst3_u8_ex(p, a, 64);
#else
            vst3_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store4(uint8_t * p, uint8x16x4_t a);

        template <> SIMD_INLINE void Store4<false>(uint8_t * p, uint8x16x4_t a)
        {
            vst4q_u8(p, a);
        }

        template <> SIMD_INLINE void Store4<true>(uint8_t * p, uint8x16x4_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 16);
            vst4q_u8(_p, a);
#elif defined(_MSC_VER)
            vst4q_u8_ex(p, a, 128);
#else
            vst4q_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store4(uint8_t * p, uint8x8x4_t a);

        template <> SIMD_INLINE void Store4<false>(uint8_t * p, uint8x8x4_t a)
        {
            vst4_u8(p, a);
        }

        template <> SIMD_INLINE void Store4<true>(uint8_t * p, uint8x8x4_t a)
        {
#if defined(__GNUC__)
            uint8_t * _p = (uint8_t *)__builtin_assume_aligned(p, 8);
            vst4_u8(_p, a);
#elif defined(_MSC_VER)
            vst4_u8_ex(p, a, 64);
#else
            vst4_u8(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store(float * p, float32x4_t a);

        template <> SIMD_INLINE void Store<false>(float * p, float32x4_t a)
        {
            vst1q_f32(p, a);
        }

        template <> SIMD_INLINE void Store<true>(float * p, float32x4_t a)
        {
#if defined(__GNUC__)
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            vst1q_f32(_p, a);
#elif defined(_MSC_VER)
            vst1q_f32_ex(p, a, 128);
#else
            vst1q_f32(p, a);
#endif
        }

        SIMD_INLINE void Store(float* ptr, float32x4_t val, size_t size)
        {
            SIMD_ALIGNED(16) float buf[F];
            Store<true>(buf, val);
            for (size_t i = 0; i < size; ++i)
                ptr[i] = buf[i];
        }

        template <bool align> SIMD_INLINE void Store(float * p, float32x2_t a);

        template <> SIMD_INLINE void Store<false>(float * p, float32x2_t a)
        {
            vst1_f32(p, a);
        }

        template <> SIMD_INLINE void Store<true>(float * p, float32x2_t a)
        {
#if defined(__GNUC__)
            float * _p = (float *)__builtin_assume_aligned(p, 8);
            vst1_f32(_p, a);
#elif defined(_MSC_VER)
            vst1_f32_ex(p, a, 64);
#else
            vst1_f32(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store2(float * p, float32x4x2_t a);

        template <> SIMD_INLINE void Store2<false>(float * p, float32x4x2_t a)
        {
            vst2q_f32(p, a);
        }

        template <> SIMD_INLINE void Store2<true>(float * p, float32x4x2_t a)
        {
#if defined(__GNUC__)
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            vst2q_f32(_p, a);
#elif defined(_MSC_VER)
            vst2q_f32_ex(p, a, 128);
#else
            vst2q_f32(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store3(float * p, float32x4x3_t a);

        template <> SIMD_INLINE void Store3<false>(float * p, float32x4x3_t a)
        {
            vst3q_f32(p, a);
        }

        template <> SIMD_INLINE void Store3<true>(float * p, float32x4x3_t a)
        {
#if defined(__GNUC__)
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            vst3q_f32(_p, a);
#elif defined(_MSC_VER)
            vst3q_f32_ex(p, a, 128);
#else
            vst3q_f32(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void Store4(float * p, float32x4x4_t a);

        template <> SIMD_INLINE void Store4<false>(float * p, float32x4x4_t a)
        {
            vst4q_f32(p, a);
        }

        template <> SIMD_INLINE void Store4<true>(float * p, float32x4x4_t a)
        {
#if defined(__GNUC__)
            float * _p = (float *)__builtin_assume_aligned(p, 16);
            vst4q_f32(_p, a);
#elif defined(_MSC_VER)
            vst4q_f32_ex(p, a, 128);
#else
            vst4q_f32(p, a);
#endif
        }

        template <bool align> SIMD_INLINE void StoreMasked(float * p, float32x4_t value, uint32x4_t mask)
        {
            float32x4_t old = Load<align>(p);
            Store<align>(p, vbslq_f32(mask, value, old));
        }
    }
#endif//SIMD_NEON_ENABLE
}
#endif//__SimdStore_h__
