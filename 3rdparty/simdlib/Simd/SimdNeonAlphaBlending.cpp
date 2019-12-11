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
#include "Simd/SimdStore.h"
#include "Simd/SimdMemory.h"
#include "Simd/SimdBase.h"

namespace Simd
{
#ifdef SIMD_NEON_ENABLE    
    namespace Neon
    {
        template <int part> SIMD_INLINE uint8x8_t AlphaBlending(const uint8x16_t & src, const uint8x16_t & dst,
            const uint8x16_t & alpha, const uint8x16_t & ff_alpha)
        {
            uint16x8_t value = vaddq_u16(
                vmull_u8(Half<part>(src), Half<part>(alpha)),
                vmull_u8(Half<part>(dst), Half<part>(ff_alpha)));
            return vshrn_n_u16(vaddq_u16(vaddq_u16(value, K16_0001), vshrq_n_u16(value, 8)), 8);
        }

        template <bool align> SIMD_INLINE void AlphaBlending(const uint8_t * src, uint8_t * dst, const uint8x16_t & alpha)
        {
            uint8x16_t _src = Load<align>(src);
            uint8x16_t _dst = Load<align>(dst);
            uint8x16_t ff_alpha = vsubq_u8(K8_FF, alpha);
            uint8x8_t lo = AlphaBlending<0>(_src, _dst, alpha, ff_alpha);
            uint8x8_t hi = AlphaBlending<1>(_src, _dst, alpha, ff_alpha);
            Store<align>(dst, vcombine_u8(lo, hi));
        }

        template <bool align, size_t channelCount> struct AlphaBlender
        {
            void operator()(const uint8_t * src, uint8_t * dst, uint8x16_t alpha);
        };

        template <bool align> struct AlphaBlender<align, 1>
        {
            SIMD_INLINE void operator()(const uint8_t * src, uint8_t * dst, uint8x16_t alpha)
            {
                AlphaBlending<align>(src, dst, alpha);
            }
        };

        template <bool align> struct AlphaBlender<align, 2>
        {
            SIMD_INLINE void operator()(const uint8_t * src, uint8_t * dst, uint8x16_t alpha)
            {
                uint8x16x2_t _alpha = vzipq_u8(alpha, alpha);
                AlphaBlending<align>(src + 0, dst + 0, _alpha.val[0]);
                AlphaBlending<align>(src + A, dst + A, _alpha.val[1]);
            }
        };

        template <bool align> struct AlphaBlender<align, 3>
        {
            SIMD_INLINE void operator()(const uint8_t * src, uint8_t * dst, uint8x16_t alpha)
            {
                uint8x16x3_t _alpha;
                _alpha.val[0] = alpha;
                _alpha.val[1] = alpha;
                _alpha.val[2] = alpha;
                Store3<align>((uint8_t*)&_alpha, _alpha);
                AlphaBlending<align>(src + 0 * A, dst + 0 * A, _alpha.val[0]);
                AlphaBlending<align>(src + 1 * A, dst + 1 * A, _alpha.val[1]);
                AlphaBlending<align>(src + 2 * A, dst + 2 * A, _alpha.val[2]);
            }
        };

        template <bool align> struct AlphaBlender<align, 4>
        {
            SIMD_INLINE void operator()(const uint8_t * src, uint8_t * dst, uint8x16_t alpha)
            {
                uint8x16x2_t _alpha = vzipq_u8(alpha, alpha);
                AlphaBlender<align, 2>()(src + A * 0, dst + A * 0, _alpha.val[0]);
                AlphaBlender<align, 2>()(src + A * 2, dst + A * 2, _alpha.val[1]);
            }
        };

        template <bool align, size_t channelCount> void AlphaBlending(const uint8_t *src, size_t srcStride, size_t width, size_t height,
            const uint8_t *alpha, size_t alphaStride, uint8_t *dst, size_t dstStride)
        {
            size_t alignedWidth = AlignLo(width, A);
            uint8x16_t tailMask = ShiftLeft(K8_FF, A - width + alignedWidth);
            size_t step = channelCount*A;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, offset = 0; col < alignedWidth; col += A, offset += step)
                {
                    uint8x16_t _alpha = Load<align>(alpha + col);
                    AlphaBlender<align, channelCount>()(src + offset, dst + offset, _alpha);
                }
                if (alignedWidth != width)
                {
                    uint8x16_t _alpha = vandq_u8(Load<false>(alpha + width - A), tailMask);
                    AlphaBlender<false, channelCount>()(src + (width - A)*channelCount, dst + (width - A)*channelCount, _alpha);
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

        template <bool align> SIMD_INLINE void AlphaFilling(uint8_t * dst, const uint8x16_t & channel, const uint8x16_t & alpha)
        {
            uint8x16_t _dst = Load<align>(dst);
            uint8x16_t ff_alpha = vsubq_u8(K8_FF, alpha);
            uint8x8_t lo = AlphaBlending<0>(channel, _dst, alpha, ff_alpha);
            uint8x8_t hi = AlphaBlending<1>(channel, _dst, alpha, ff_alpha);
            Store<align>(dst, vcombine_u8(lo, hi));
        }

        template <bool align, size_t channelCount> struct AlphaFiller
        {
            void operator() (uint8x16_t * dst, const uint8x16_t * channel, const uint8x16_t & alpha);
        };

        template <bool align> struct AlphaFiller<align, 1>
        {
            SIMD_INLINE void operator()(uint8_t * dst, const uint8x16_t * channel, const uint8x16_t & alpha)
            {
                AlphaFilling<align>(dst, channel[0], alpha);
            }
        };

        template <bool align> struct AlphaFiller<align, 2>
        {
            SIMD_INLINE void operator()(uint8_t * dst, const uint8x16_t * channel, const uint8x16_t & alpha)
            {
                uint8x16x2_t _alpha = vzipq_u8(alpha, alpha);
                AlphaFilling<align>(dst + 0 * A, channel[0], _alpha.val[0]);
                AlphaFilling<align>(dst + 1 * A, channel[1], _alpha.val[1]);
            }
        };

        template <bool align> struct AlphaFiller<align, 3>
        {
            SIMD_INLINE void operator()(uint8_t * dst, const uint8x16_t * channel, const uint8x16_t & alpha)
            {
                uint8x16x3_t _alpha;
                _alpha.val[0] = alpha;
                _alpha.val[1] = alpha;
                _alpha.val[2] = alpha;
                Store3<align>((uint8_t*)&_alpha, _alpha);
                AlphaFilling<align>(dst + 0 * A, channel[0], _alpha.val[0]);
                AlphaFilling<align>(dst + 1 * A, channel[1], _alpha.val[1]);
                AlphaFilling<align>(dst + 2 * A, channel[2], _alpha.val[2]);
            }
        };

        template <bool align> struct AlphaFiller<align, 4>
        {
            SIMD_INLINE void operator()(uint8_t * dst, const uint8x16_t * channel, const uint8x16_t & alpha)
            {
                uint8x16x2_t _alpha = vzipq_u8(alpha, alpha);
                AlphaFiller<align, 2>()(dst + A * 0, channel + 0, _alpha.val[0]);
                AlphaFiller<align, 2>()(dst + A * 2, channel + 2, _alpha.val[1]);
            }
        };

        template <bool align, size_t channelCount> void AlphaFilling(uint8_t * dst, size_t dstStride, size_t width, size_t height, const uint8x16_t * channel, const uint8_t * alpha, size_t alphaStride)
        {
            size_t alignedWidth = AlignLo(width, A);
            uint8x16_t tailMask = ShiftLeft(K8_FF, A - width + alignedWidth);
            size_t step = channelCount * A;
            for (size_t row = 0; row < height; ++row)
            {
                for (size_t col = 0, offset = 0; col < alignedWidth; col += A, offset += step)
                {
                    uint8x16_t _alpha = Load<align>(alpha + col);
                    AlphaFiller<align, channelCount>()(dst + offset, channel, _alpha);
                }
                if (alignedWidth != width)
                {
                    uint8x16_t _alpha = vandq_u8(Load<false>(alpha + width - A), tailMask);
                    AlphaFiller<false, channelCount>()(dst + (width - A)*channelCount, channel, _alpha);
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

            switch (channelCount)
            {
            case 1:
            {
                uint8x16_t _channel = vdupq_n_u8(channel[0]);
                AlphaFilling<align, 1>(dst, dstStride, width, height, &_channel, alpha, alphaStride);
                break;
            }
            case 2:
            {
                uint8x16x2_t _channel;
                _channel.val[0] = vdupq_n_u8(channel[0]);
                _channel.val[1] = vdupq_n_u8(channel[1]);
                Store2<align>((uint8_t*)&_channel, _channel);
                AlphaFilling<align, 2>(dst, dstStride, width, height, _channel.val, alpha, alphaStride);
                break;
            }
            case 3:
            {
                uint8x16x3_t _channel;
                _channel.val[0] = vdupq_n_u8(channel[0]);
                _channel.val[1] = vdupq_n_u8(channel[1]);
                _channel.val[2] = vdupq_n_u8(channel[2]);
                Store3<align>((uint8_t*)&_channel, _channel);
                AlphaFilling<align, 3>(dst, dstStride, width, height, _channel.val, alpha, alphaStride);
                break;
            }
            case 4:
            {
                uint8x16x4_t _channel;
                _channel.val[0] = vdupq_n_u8(channel[0]);
                _channel.val[1] = vdupq_n_u8(channel[1]);
                _channel.val[2] = vdupq_n_u8(channel[2]);
                _channel.val[3] = vdupq_n_u8(channel[3]);
                Store4<align>((uint8_t*)&_channel, _channel);
                AlphaFilling<align, 4>(dst, dstStride, width, height, _channel.val, alpha, alphaStride);
                break;
            }
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
#endif// SIMD_NEON_ENABLE
}
