/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2019 Yermalayeu Ihar,
*               2014-2015 Antonenka Mikhail.
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
#ifndef __SimdConversion_h__
#define __SimdConversion_h__

#include "Simd/SimdConst.h"
#include "Simd/SimdMath.h"
#include "Simd/SimdLoad.h"

namespace Simd
{
    namespace Base
    {
        SIMD_INLINE int BgrToGray(int blue, int green, int red)
        {
            return (BLUE_TO_GRAY_WEIGHT*blue + GREEN_TO_GRAY_WEIGHT * green +
                RED_TO_GRAY_WEIGHT * red + BGR_TO_GRAY_ROUND_TERM) >> BGR_TO_GRAY_AVERAGING_SHIFT;
        }

        SIMD_INLINE int RgbToGray(int red, int green, int blue)
        {
            return (BLUE_TO_GRAY_WEIGHT*blue + GREEN_TO_GRAY_WEIGHT * green +
                RED_TO_GRAY_WEIGHT * red + BGR_TO_GRAY_ROUND_TERM) >> BGR_TO_GRAY_AVERAGING_SHIFT;
        }
    }

#ifdef SIMD_SSSE3_ENABLE    
    namespace Ssse3
    {
        template <int index> __m128i InterleaveBgr(__m128i blue, __m128i green, __m128i red);

        template<> SIMD_INLINE __m128i InterleaveBgr<0>(__m128i blue, __m128i green, __m128i red)
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(blue, K8_SHUFFLE_BLUE_TO_BGR0),
                    _mm_or_si128(_mm_shuffle_epi8(green, K8_SHUFFLE_GREEN_TO_BGR0),
                        _mm_shuffle_epi8(red, K8_SHUFFLE_RED_TO_BGR0)));
        }

        template<> SIMD_INLINE __m128i InterleaveBgr<1>(__m128i blue, __m128i green, __m128i red)
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(blue, K8_SHUFFLE_BLUE_TO_BGR1),
                    _mm_or_si128(_mm_shuffle_epi8(green, K8_SHUFFLE_GREEN_TO_BGR1),
                        _mm_shuffle_epi8(red, K8_SHUFFLE_RED_TO_BGR1)));
        }

        template<> SIMD_INLINE __m128i InterleaveBgr<2>(__m128i blue, __m128i green, __m128i red)
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(blue, K8_SHUFFLE_BLUE_TO_BGR2),
                    _mm_or_si128(_mm_shuffle_epi8(green, K8_SHUFFLE_GREEN_TO_BGR2),
                        _mm_shuffle_epi8(red, K8_SHUFFLE_RED_TO_BGR2)));
        }

        SIMD_INLINE __m128i BgrToBlue(__m128i bgr[3])
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_BLUE),
                    _mm_or_si128(_mm_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_BLUE),
                        _mm_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_BLUE)));
        }

        SIMD_INLINE __m128i BgrToGreen(__m128i bgr[3])
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_GREEN),
                    _mm_or_si128(_mm_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_GREEN),
                        _mm_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_GREEN)));
        }

        SIMD_INLINE __m128i BgrToRed(__m128i bgr[3])
        {
            return
                _mm_or_si128(_mm_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_RED),
                    _mm_or_si128(_mm_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_RED),
                        _mm_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_RED)));
        }
    }
#endif//SIMD_SSSE3_ENABLE

#ifdef SIMD_AVX2_ENABLE    
    namespace Avx2
    {
        template <int index> __m256i GrayToBgr(__m256i gray);

        template<> SIMD_INLINE __m256i GrayToBgr<0>(__m256i gray)
        {
            return _mm256_shuffle_epi8(_mm256_permute4x64_epi64(gray, 0x44), K8_SHUFFLE_GRAY_TO_BGR0);
        }

        template<> SIMD_INLINE __m256i GrayToBgr<1>(__m256i gray)
        {
            return _mm256_shuffle_epi8(_mm256_permute4x64_epi64(gray, 0x99), K8_SHUFFLE_GRAY_TO_BGR1);
        }

        template<> SIMD_INLINE __m256i GrayToBgr<2>(__m256i gray)
        {
            return _mm256_shuffle_epi8(_mm256_permute4x64_epi64(gray, 0xEE), K8_SHUFFLE_GRAY_TO_BGR2);
        }

        template <int index> __m256i InterleaveBgr(__m256i blue, __m256i green, __m256i red);

        template<> SIMD_INLINE __m256i InterleaveBgr<0>(__m256i blue, __m256i green, __m256i red)
        {
            return
                _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(blue, 0x44), K8_SHUFFLE_PERMUTED_BLUE_TO_BGR0),
                    _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(green, 0x44), K8_SHUFFLE_PERMUTED_GREEN_TO_BGR0),
                        _mm256_shuffle_epi8(_mm256_permute4x64_epi64(red, 0x44), K8_SHUFFLE_PERMUTED_RED_TO_BGR0)));
        }

        template<> SIMD_INLINE __m256i InterleaveBgr<1>(__m256i blue, __m256i green, __m256i red)
        {
            return
                _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(blue, 0x99), K8_SHUFFLE_PERMUTED_BLUE_TO_BGR1),
                    _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(green, 0x99), K8_SHUFFLE_PERMUTED_GREEN_TO_BGR1),
                        _mm256_shuffle_epi8(_mm256_permute4x64_epi64(red, 0x99), K8_SHUFFLE_PERMUTED_RED_TO_BGR1)));
        }

        template<> SIMD_INLINE __m256i InterleaveBgr<2>(__m256i blue, __m256i green, __m256i red)
        {
            return
                _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(blue, 0xEE), K8_SHUFFLE_PERMUTED_BLUE_TO_BGR2),
                    _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(green, 0xEE), K8_SHUFFLE_PERMUTED_GREEN_TO_BGR2),
                        _mm256_shuffle_epi8(_mm256_permute4x64_epi64(red, 0xEE), K8_SHUFFLE_PERMUTED_RED_TO_BGR2)));
        }

        SIMD_INLINE __m256i BgrToBlue(__m256i bgr[3])
        {
            __m256i b0 = _mm256_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_BLUE);
            __m256i b2 = _mm256_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_BLUE);
            return
                _mm256_or_si256(_mm256_permute2x128_si256(b0, b2, 0x20),
                    _mm256_or_si256(_mm256_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_BLUE),
                        _mm256_permute2x128_si256(b0, b2, 0x31)));
        }

        SIMD_INLINE __m256i BgrToGreen(__m256i bgr[3])
        {
            __m256i g0 = _mm256_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_GREEN);
            __m256i g2 = _mm256_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_GREEN);
            return
                _mm256_or_si256(_mm256_permute2x128_si256(g0, g2, 0x20),
                    _mm256_or_si256(_mm256_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_GREEN),
                        _mm256_permute2x128_si256(g0, g2, 0x31)));
        }

        SIMD_INLINE __m256i BgrToRed(__m256i bgr[3])
        {
            __m256i r0 = _mm256_shuffle_epi8(bgr[0], K8_SHUFFLE_BGR0_TO_RED);
            __m256i r2 = _mm256_shuffle_epi8(bgr[2], K8_SHUFFLE_BGR2_TO_RED);
            return
                _mm256_or_si256(_mm256_permute2x128_si256(r0, r2, 0x20),
                    _mm256_or_si256(_mm256_shuffle_epi8(bgr[1], K8_SHUFFLE_BGR1_TO_RED),
                        _mm256_permute2x128_si256(r0, r2, 0x31)));
        }

        template<bool tail> __m256i BgrToBgra(const __m256i & bgr, const __m256i & alpha);

        template<> SIMD_INLINE __m256i BgrToBgra<false>(const __m256i & bgr, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(bgr, 0x94), K8_BGRA_TO_BGR_SHUFFLE), alpha);
        }

        template<> SIMD_INLINE __m256i BgrToBgra<true>(const __m256i & bgr, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(bgr, 0xE9), K8_BGRA_TO_BGR_SHUFFLE), alpha);
        }

        template<bool tail> __m256i BgrToRgba(const __m256i & bgr, const __m256i & alpha);

        template<> SIMD_INLINE __m256i BgrToRgba<false>(const __m256i & bgr, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(bgr, 0x94), K8_BGRA_TO_RGB_SHUFFLE), alpha);
        }

        template<> SIMD_INLINE __m256i BgrToRgba<true>(const __m256i & bgr, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(bgr, 0xE9), K8_BGRA_TO_RGB_SHUFFLE), alpha);
        }

        SIMD_INLINE __m256i BgraToRgba(const __m256i & bgra)
        {
            return _mm256_shuffle_epi8(bgra, K8_BGRA_TO_RGBA_SHUFFLE);
        }

        template<bool tail> __m256i RgbToBgra(const __m256i & rgb, const __m256i & alpha);

        template<> SIMD_INLINE __m256i RgbToBgra<false>(const __m256i & rgb, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(rgb, 0x94), K8_BGRA_TO_RGB_SHUFFLE), alpha);
        }

        template<> SIMD_INLINE __m256i RgbToBgra<true>(const __m256i & rgb, const __m256i & alpha)
        {
            return _mm256_or_si256(_mm256_shuffle_epi8(_mm256_permute4x64_epi64(rgb, 0xE9), K8_BGRA_TO_RGB_SHUFFLE), alpha);
        }
    }
#endif// SIMD_AVX2_ENABLE

#ifdef SIMD_NEON_ENABLE    
    namespace Neon
    {
        template <int part> SIMD_INLINE uint32x4_t BgrToGray(const uint16x8_t & blue, const uint16x8_t & green, const uint16x8_t & red)
        {
            return vshrq_n_u32(vmlal_u16(vmlal_u16(vmlal_u16(K32_BGR_TO_GRAY_ROUND_TERM, Half<part>(blue), K16_BLUE_TO_GRAY_WEIGHT),
                Half<part>(green), K16_GREEN_TO_GRAY_WEIGHT), Half<part>(red), K16_RED_TO_GRAY_WEIGHT), Base::BGR_TO_GRAY_AVERAGING_SHIFT);
        }

        SIMD_INLINE uint16x8_t BgrToGray(const uint16x8_t & blue, const uint16x8_t & green, const uint16x8_t & red)
        {
            return PackU32(BgrToGray<0>(blue, green, red), BgrToGray<1>(blue, green, red));
        }

        template <int part> SIMD_INLINE int32x4_t BgrToU(uint16x8_t blue, uint16x8_t green, uint16x8_t red)
        {
            return vshrq_n_s32(vmlal_s16(vmlal_s16(vmlal_s16(K32_BGR_TO_YUV_ROUND_TERM, (int16x4_t)Half<part>(blue), K16_BLUE_TO_U_WEIGHT),
                (int16x4_t)Half<part>(green), K16_GREEN_TO_U_WEIGHT), (int16x4_t)Half<part>(red), K16_RED_TO_U_WEIGHT), Base::BGR_TO_YUV_AVERAGING_SHIFT);
        }
    }
#endif// SIMD_NEON_ENABLE
}
#endif//__SimdConversion_h__
