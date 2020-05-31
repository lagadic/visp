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
#ifndef __SimdExp_h__
#define __SimdExp_h__

#include "Simd/SimdMath.h"

namespace Simd
{
    namespace Base
    {
        SIMD_INLINE float Exp(float value)
        {
            return ::expf(value);
        }
    }

#ifdef SIMD_SSE2_ENABLE    
    namespace Sse2
    {
        class Exp
        {
            __m128i _exponent, _mantissa, _127;
            __m128 _1_0, _0_5, _min, _max, _exp0, _exp1, _exp2, _exp3, _exp4, _exp5, _k;

            SIMD_INLINE __m128 Poly5(__m128 x) const
            {
                __m128 p = _exp5;
                p = _mm_add_ps(_mm_mul_ps(x, p), _exp4);
                p = _mm_add_ps(_mm_mul_ps(x, p), _exp3);
                p = _mm_add_ps(_mm_mul_ps(x, p), _exp2);
                p = _mm_add_ps(_mm_mul_ps(x, p), _exp1);
                p = _mm_add_ps(_mm_mul_ps(x, p), _exp0);
                return p;
            }

            SIMD_INLINE __m128 Exp2(__m128 x) const
            {
                x = _mm_max_ps(_mm_min_ps(x, _max), _min);
                __m128i ipart = _mm_cvtps_epi32(_mm_sub_ps(x, _0_5));
                __m128 fpart = _mm_sub_ps(x, _mm_cvtepi32_ps(ipart));
                __m128 expipart = _mm_castsi128_ps(_mm_slli_epi32(_mm_add_epi32(ipart, _127), 23));
                __m128 expfpart = Poly5(fpart);
                return _mm_mul_ps(expipart, expfpart);
            }

        public:

            SIMD_INLINE Exp(float k = 1.0f)
            {
                _exponent = _mm_set1_epi32(0x7F800000);
                _mantissa = _mm_set1_epi32(0x007FFFFF);
                _127 = _mm_set1_epi32(127);
                _1_0 = _mm_set1_ps(1.0f);
                _0_5 = _mm_set1_ps(0.5f);
                _min = _mm_set1_ps(-126.99999f);
                _max = _mm_set1_ps(129.00000f);
                _exp0 = _mm_set1_ps(9.9999994e-1f);
                _exp1 = _mm_set1_ps(6.9315308e-1f);
                _exp2 = _mm_set1_ps(2.4015361e-1f);
                _exp3 = _mm_set1_ps(5.5826318e-2f);
                _exp4 = _mm_set1_ps(8.9893397e-3f);
                _exp5 = _mm_set1_ps(1.8775767e-3f);
                _k = _mm_set1_ps(k / 0.69314718056f);
            }

            SIMD_INLINE __m128 Exponent(__m128 value) const
            {
                return Exp2(_mm_mul_ps(_k, value));
            }

            SIMD_INLINE __m128 Sigmoid(__m128 value) const
            {
                __m128 exp = Exp2(_mm_mul_ps(_k, value));
                return _mm_div_ps(_1_0, _mm_add_ps(_1_0, exp));
            }

            SIMD_INLINE __m128 Tanh(__m128 value) const
            {
                __m128 exp = Exp2(_mm_mul_ps(_k, value));
                return _mm_div_ps(_mm_sub_ps(_1_0, exp), _mm_add_ps(_1_0, exp));
            }

            SIMD_INLINE __m128 Elu(__m128 value, __m128 alpha) const
            {
                __m128 exp = Exp2(_mm_mul_ps(_k, value));
                __m128 neg = _mm_mul_ps(alpha, _mm_sub_ps(exp, _1_0));
                __m128 mask = _mm_cmpgt_ps(_mm_setzero_ps(), value);
                return Sse::Combine(mask, neg, value);
            }
        };

        namespace Detail
        {
            SIMD_INLINE __m128 Poly5(__m128 x)
            {
                __m128 p = _mm_set1_ps(1.8775767e-3f);
                p = _mm_add_ps(_mm_mul_ps(x, p), _mm_set1_ps(8.9893397e-3f));
                p = _mm_add_ps(_mm_mul_ps(x, p), _mm_set1_ps(5.5826318e-2f));
                p = _mm_add_ps(_mm_mul_ps(x, p), _mm_set1_ps(2.4015361e-1f));
                p = _mm_add_ps(_mm_mul_ps(x, p), _mm_set1_ps(6.9315308e-1f));
                p = _mm_add_ps(_mm_mul_ps(x, p), _mm_set1_ps(9.9999994e-1f));
                return p;
            }

            SIMD_INLINE __m128 Exp2(__m128 x)
            {
                x = _mm_max_ps(_mm_min_ps(x, _mm_set1_ps(129.00000f)), _mm_set1_ps(-126.99999f));
                __m128i ipart = _mm_cvtps_epi32(_mm_sub_ps(x, _mm_set1_ps(0.5f)));
                __m128 fpart = _mm_sub_ps(x, _mm_cvtepi32_ps(ipart));
                __m128 expipart = _mm_castsi128_ps(_mm_slli_epi32(_mm_add_epi32(ipart, _mm_set1_epi32(127)), 23));
                __m128 expfpart = Poly5(fpart);
                return _mm_mul_ps(expipart, expfpart);
            }
        }

        SIMD_INLINE __m128 Exponent(__m128 value)
        {
            return Detail::Exp2(_mm_mul_ps(_mm_set1_ps(1.44269504f), value));
        }

        SIMD_INLINE __m128 Elu(__m128 value, __m128 alpha) 
        {
            __m128 exp = Exponent(value);
            __m128 neg = _mm_mul_ps(alpha, _mm_sub_ps(exp, _mm_set1_ps(1.0f)));
            __m128 mask = _mm_cmpgt_ps(_mm_setzero_ps(), value);
            return Sse::Combine(mask, neg, value);
        }
    }
#endif //SIMD_SSE2_ENABLE   

#ifdef SIMD_AVX2_ENABLE    
    namespace Avx2
    {
        class Exp
        {
            __m256i _exponent, _mantissa, _127;
            __m256 _1_0, _0_5, _min, _max, _exp0, _exp1, _exp2, _exp3, _exp4, _exp5, _k;

            SIMD_INLINE __m256 Poly5(__m256 x) const
            {
                __m256 p = _exp5;
                p = _mm256_fmadd_ps(x, p, _exp4);
                p = _mm256_fmadd_ps(x, p, _exp3);
                p = _mm256_fmadd_ps(x, p, _exp2);
                p = _mm256_fmadd_ps(x, p, _exp1);
                p = _mm256_fmadd_ps(x, p, _exp0);
                return p;
            }

            SIMD_INLINE __m256 Exp2(__m256 x) const
            {
                x = _mm256_max_ps(_mm256_min_ps(x, _max), _min);
                __m256i ipart = _mm256_cvtps_epi32(_mm256_sub_ps(x, _0_5));
                __m256 fpart = _mm256_sub_ps(x, _mm256_cvtepi32_ps(ipart));
                __m256 expipart = _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_add_epi32(ipart, _127), 23));
                __m256 expfpart = Poly5(fpart);
                return _mm256_mul_ps(expipart, expfpart);
            }

        public:

            SIMD_INLINE Exp(float k = 1.0f)
            {
                _exponent = _mm256_set1_epi32(0x7F800000);
                _mantissa = _mm256_set1_epi32(0x007FFFFF);
                _127 = _mm256_set1_epi32(127);
                _1_0 = _mm256_set1_ps(1.0f);
                _0_5 = _mm256_set1_ps(0.5f);
                _min = _mm256_set1_ps(-126.99999f);
                _max = _mm256_set1_ps(129.00000f);
                _exp0 = _mm256_set1_ps(9.9999994e-1f);
                _exp1 = _mm256_set1_ps(6.9315308e-1f);
                _exp2 = _mm256_set1_ps(2.4015361e-1f);
                _exp3 = _mm256_set1_ps(5.5826318e-2f);
                _exp4 = _mm256_set1_ps(8.9893397e-3f);
                _exp5 = _mm256_set1_ps(1.8775767e-3f);
                _k = _mm256_set1_ps(k / 0.69314718056f);
            }

            SIMD_INLINE __m256 Exponent(__m256 value) const
            {
                return Exp2(_mm256_mul_ps(_k, value));
            }

            SIMD_INLINE __m256 Sigmoid(__m256 value) const
            {
                __m256 exp = Exp2(_mm256_mul_ps(_k, value));
                return _mm256_div_ps(_1_0, _mm256_add_ps(_1_0, exp));
            }

            SIMD_INLINE __m256 Tanh(__m256 value) const
            {
                __m256 exp = Exp2(_mm256_mul_ps(_k, value));
                return _mm256_div_ps(_mm256_sub_ps(_1_0, exp), _mm256_add_ps(_1_0, exp));
            }

            SIMD_INLINE __m256 Elu(__m256 value, __m256 alpha) const
            {
                __m256 exp = Exp2(_mm256_mul_ps(_k, value));
                __m256 neg = _mm256_mul_ps(alpha, _mm256_sub_ps(exp, _1_0));
                __m256 mask = _mm256_cmp_ps(_mm256_setzero_ps(), value, _CMP_GT_OS);
                return _mm256_blendv_ps(value, neg, mask);
            }
        };

        namespace Detail
        {
            SIMD_INLINE __m256 Poly5(__m256 x)
            {
                __m256 p = _mm256_set1_ps(1.8775767e-3f);
                p = _mm256_add_ps(_mm256_mul_ps(x, p), _mm256_set1_ps(8.9893397e-3f));
                p = _mm256_add_ps(_mm256_mul_ps(x, p), _mm256_set1_ps(5.5826318e-2f));
                p = _mm256_add_ps(_mm256_mul_ps(x, p), _mm256_set1_ps(2.4015361e-1f));
                p = _mm256_add_ps(_mm256_mul_ps(x, p), _mm256_set1_ps(6.9315308e-1f));
                p = _mm256_add_ps(_mm256_mul_ps(x, p), _mm256_set1_ps(9.9999994e-1f));
                return p;
            }

            SIMD_INLINE __m256 Exp2(__m256 x)
            {
                x = _mm256_max_ps(_mm256_min_ps(x, _mm256_set1_ps(129.00000f)), _mm256_set1_ps(-126.99999f));
                __m256i ipart = _mm256_cvtps_epi32(_mm256_sub_ps(x, _mm256_set1_ps(0.5f)));
                __m256 fpart = _mm256_sub_ps(x, _mm256_cvtepi32_ps(ipart));
                __m256 expipart = _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_add_epi32(ipart, _mm256_set1_epi32(127)), 23));
                __m256 expfpart = Poly5(fpart);
                return _mm256_mul_ps(expipart, expfpart);
            }
        }

        SIMD_INLINE __m256 Exponent(__m256 value)
        {
            return Detail::Exp2(_mm256_mul_ps(_mm256_set1_ps(1.44269504f), value));
        }

        SIMD_INLINE __m256 Elu(__m256 value, __m256 alpha)
        {
            __m256 exp = Exponent(value);
            __m256 neg = _mm256_mul_ps(alpha, _mm256_sub_ps(exp, _mm256_set1_ps(1.0f)));
            __m256 mask = _mm256_cmp_ps(_mm256_setzero_ps(), value, _CMP_GT_OS);
            return _mm256_blendv_ps(value, neg, mask);
        }
    }
#endif //SIMD_AVX2_ENABLE

#ifdef SIMD_NEON_ENABLE    
    namespace Neon
    {
        class Exp
        {
            int32x4_t _exponent, _mantissa, _127;
            float32x4_t _1_0, _0_5, _min, _max, _exp0, _exp1, _exp2, _exp3, _exp4, _exp5, _k;

            SIMD_INLINE float32x4_t Poly5(float32x4_t x) const
            {
                float32x4_t p = _exp5;
                p = vmlaq_f32(_exp4, x, p);
                p = vmlaq_f32(_exp3, x, p);
                p = vmlaq_f32(_exp2, x, p);
                p = vmlaq_f32(_exp1, x, p);
                p = vmlaq_f32(_exp0, x, p);
                return p;
            }

            SIMD_INLINE float32x4_t Exp2(float32x4_t x) const
            {
                x = vmaxq_f32(vminq_f32(x, _max), _min);
                int32x4_t ipart = vcvtq_s32_f32(vsubq_f32(x, _0_5));
                float32x4_t fpart = vsubq_f32(x, vcvtq_f32_s32(ipart));
                float32x4_t expipart = vreinterpretq_f32_s32(vshlq_n_s32(vaddq_s32(ipart, _127), 23));
                float32x4_t expfpart = Poly5(fpart);
                return vmulq_f32(expipart, expfpart);
            }

        public:

            SIMD_INLINE Exp(float k = 1.0f)
            {
                _exponent = vdupq_n_s32(0x7F800000);
                _mantissa = vdupq_n_s32(0x007FFFFF);
                _127 = vdupq_n_s32(127);
                _1_0 = vdupq_n_f32(1.0f);
                _0_5 = vdupq_n_f32(0.5f);
                _min = vdupq_n_f32(-126.99999f);
                _max = vdupq_n_f32(129.00000f);
                _exp0 = vdupq_n_f32(9.9999994e-1f);
                _exp1 = vdupq_n_f32(6.9315308e-1f);
                _exp2 = vdupq_n_f32(2.4015361e-1f);
                _exp3 = vdupq_n_f32(5.5826318e-2f);
                _exp4 = vdupq_n_f32(8.9893397e-3f);
                _exp5 = vdupq_n_f32(1.8775767e-3f);
                _k = vdupq_n_f32(k / 0.69314718056f);
            }

            SIMD_INLINE float32x4_t Exponent(float32x4_t value) const
            {
                return Exp2(vmulq_f32(_k, value));
            }

            template<int iter> SIMD_INLINE float32x4_t Sigmoid(float32x4_t value) const
            {
                float32x4_t exp = Exp2(vmulq_f32(_k, value));
                return Reciprocal<iter>(vaddq_f32(_1_0, exp));
            }

            template<int iter> SIMD_INLINE float32x4_t Tanh(float32x4_t value) const
            {
                float32x4_t exp = Exp2(vmulq_f32(_k, value));
                return Div<iter>(vsubq_f32(_1_0, exp), vaddq_f32(_1_0, exp));
            }

            SIMD_INLINE float32x4_t Elu(float32x4_t value, float32x4_t alpha) const
            {
                float32x4_t exp = Exp2(vmulq_f32(_k, value));
                float32x4_t neg = vmulq_f32(alpha, vsubq_f32(exp, _1_0));
                uint32x4_t mask = vcgtq_f32(vdupq_n_f32(0.0f), value);
                return vbslq_f32(mask, neg, value);
            }
        };

        namespace Detail
        {
            SIMD_INLINE float32x4_t Poly5(float32x4_t x)
            {
                float32x4_t p = vdupq_n_f32(1.8775767e-3f);
                p = vmlaq_f32(vdupq_n_f32(8.9893397e-3f), x, p);
                p = vmlaq_f32(vdupq_n_f32(5.5826318e-2f), x, p);
                p = vmlaq_f32(vdupq_n_f32(2.4015361e-1f), x, p);
                p = vmlaq_f32(vdupq_n_f32(6.9315308e-1f), x, p);
                p = vmlaq_f32(vdupq_n_f32(9.9999994e-1f), x, p);
                return p;
            }

            SIMD_INLINE float32x4_t Exp2(float32x4_t x)
            {
                x = vmaxq_f32(vminq_f32(x, vdupq_n_f32(129.00000f)), vdupq_n_f32(-126.99999f));
                int32x4_t ipart = vcvtq_s32_f32(vsubq_f32(x, vdupq_n_f32(0.5f)));
                float32x4_t fpart = vsubq_f32(x, vcvtq_f32_s32(ipart));
                float32x4_t expipart = vreinterpretq_f32_s32(vshlq_n_s32(vaddq_s32(ipart, vdupq_n_s32(127)), 23));
                float32x4_t expfpart = Poly5(fpart);
                return vmulq_f32(expipart, expfpart);
            }
        }

        SIMD_INLINE float32x4_t Exponent(float32x4_t value)
        {
            return Detail::Exp2(vmulq_f32(vdupq_n_f32(1.44269504f), value));
        }

        SIMD_INLINE float32x4_t Elu(float32x4_t value, float32x4_t alpha)
        {
            float32x4_t exp = Exponent(value);
            float32x4_t neg = vmulq_f32(alpha, vsubq_f32(exp, vdupq_n_f32(1.0f)));
            uint32x4_t mask = vcgtq_f32(vdupq_n_f32(0.0f), value);
            return vbslq_f32(mask, neg, value);
        }
    }
#endif //SIMD_NEON_ENABLE
}

#endif//__SimdExp_h__
