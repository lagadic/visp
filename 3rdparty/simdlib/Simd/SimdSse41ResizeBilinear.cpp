/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2021 Yermalayeu Ihar.
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
#include "Simd/SimdBase.h"
#include "Simd/SimdStore.h"

namespace Simd
{
#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
        namespace
        {
            struct Buffer
            {
                Buffer(size_t size, size_t width, size_t height)
                {
                    _p = Allocate(3 * size + sizeof(int)*(2 * height + width));
                    bx[0] = (uint8_t*)_p;
                    bx[1] = bx[0] + size;
                    ax = bx[1] + size;
                    ix = (int*)(ax + size);
                    iy = ix + width;
                    ay = iy + height;
                }

                ~Buffer()
                {
                    Free(_p);
                }

                uint8_t * bx[2];
                uint8_t * ax;
                int * ix;
                int * ay;
                int * iy;
            private:
                void *_p;
            };

            struct Index
            {
                int src, dst;
                uint8_t shuffle[A];
            };

            struct BufferG
            {
                BufferG(size_t width, size_t blocks, size_t height)
                {
                    _p = Allocate(3 * width + sizeof(int) * 2 * height + blocks * sizeof(Index) + 2 * A);
                    bx[0] = (uint8_t*)_p;
                    bx[1] = bx[0] + width + A;
                    ax = bx[1] + width + A;
                    ix = (Index*)(ax + width);
                    iy = (int*)(ix + blocks);
                    ay = iy + height;
                }

                ~BufferG()
                {
                    Free(_p);
                }

                uint8_t * bx[2];
                uint8_t * ax;
                Index * ix;
                int * ay;
                int * iy;
            private:
                void *_p;
            };
        }

        template <size_t channelCount> void EstimateAlphaIndexX(size_t srcSize, size_t dstSize, int * indexes, uint8_t * alphas)
        {
            float scale = (float)srcSize / dstSize;

            for (size_t i = 0; i < dstSize; ++i)
            {
                float alpha = (float)((i + 0.5)*scale - 0.5);
                ptrdiff_t index = (ptrdiff_t)::floor(alpha);
                alpha -= index;

                if (index < 0)
                {
                    index = 0;
                    alpha = 0;
                }

                if (index > (ptrdiff_t)srcSize - 2)
                {
                    index = srcSize - 2;
                    alpha = 1;
                }

                indexes[i] = (int)index;
                alphas[1] = (uint8_t)(alpha * Base::FRACTION_RANGE + 0.5);
                alphas[0] = (uint8_t)(Base::FRACTION_RANGE - alphas[1]);
                for (size_t channel = 1; channel < channelCount; channel++)
                    ((uint16_t*)alphas)[channel] = *(uint16_t*)alphas;
                alphas += 2 * channelCount;
            }
        }

        size_t BlockCountMax(size_t src, size_t dst)
        {
            return (size_t)Simd::Max(::ceil(float(src) / (A - 1)), ::ceil(float(dst) / HA));
        }

        void EstimateAlphaIndexX(int srcSize, int dstSize, Index * indexes, uint8_t * alphas, size_t & blockCount)
        {
            float scale = (float)srcSize / dstSize;
            int block = 0;
            indexes[0].src = 0;
            indexes[0].dst = 0;
            for (int dstIndex = 0; dstIndex < dstSize; ++dstIndex)
            {
                float alpha = (float)((dstIndex + 0.5)*scale - 0.5);
                int srcIndex = (int)::floor(alpha);
                alpha -= srcIndex;

                if (srcIndex < 0)
                {
                    srcIndex = 0;
                    alpha = 0;
                }

                if (srcIndex > srcSize - 2)
                {
                    srcIndex = srcSize - 2;
                    alpha = 1;
                }

                int dst = 2 * dstIndex - indexes[block].dst;
                int src = srcIndex - indexes[block].src;
                if (src >= A - 1 || dst >= A)
                {
                    block++;
                    indexes[block].src = Simd::Min(srcIndex, srcSize - (int)A);
                    indexes[block].dst = 2 * dstIndex;
                    dst = 0;
                    src = srcIndex - indexes[block].src;
                }
                indexes[block].shuffle[dst] = src;
                indexes[block].shuffle[dst + 1] = src + 1;

                alphas[1] = (uint8_t)(alpha * Base::FRACTION_RANGE + 0.5);
                alphas[0] = (uint8_t)(Base::FRACTION_RANGE - alphas[1]);
                alphas += 2;
            }
            blockCount = block + 1;
        }

        template <size_t channelCount> void InterpolateX(const __m128i * alpha, __m128i * buffer);

        template <> SIMD_INLINE void InterpolateX<1>(const __m128i * alpha, __m128i * buffer)
        {
            _mm_store_si128(buffer, _mm_maddubs_epi16(_mm_load_si128(buffer), _mm_load_si128(alpha)));
        }

        const __m128i K8_SHUFFLE_X2 = SIMD_MM_SETR_EPI8(0x0, 0x2, 0x1, 0x3, 0x4, 0x6, 0x5, 0x7, 0x8, 0xA, 0x9, 0xB, 0xC, 0xE, 0xD, 0xF);

        SIMD_INLINE void InterpolateX2(const __m128i * alpha, __m128i * buffer)
        {
            __m128i src = _mm_shuffle_epi8(_mm_load_si128(buffer), K8_SHUFFLE_X2);
            _mm_store_si128(buffer, _mm_maddubs_epi16(src, _mm_load_si128(alpha)));
        }

        template <> SIMD_INLINE void InterpolateX<2>(const __m128i * alpha, __m128i * buffer)
        {
            InterpolateX2(alpha + 0, buffer + 0);
            InterpolateX2(alpha + 1, buffer + 1);
        }

        const __m128i K8_SHUFFLE_X3_00 = SIMD_MM_SETR_EPI8(0x0, 0x3, 0x1, 0x4, 0x2, 0x5, 0x6, 0x9, 0x7, 0xA, 0x8, 0xB, 0xC, 0xF, 0xD, -1);
        const __m128i K8_SHUFFLE_X3_01 = SIMD_MM_SETR_EPI8(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0x0);
        const __m128i K8_SHUFFLE_X3_10 = SIMD_MM_SETR_EPI8(0xE, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
        const __m128i K8_SHUFFLE_X3_11 = SIMD_MM_SETR_EPI8(-1, 0x1, 0x2, 0x5, 0x3, 0x6, 0x4, 0x7, 0x8, 0xB, 0x9, 0xC, 0xA, 0xD, 0xE, -1);
        const __m128i K8_SHUFFLE_X3_12 = SIMD_MM_SETR_EPI8(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0x1);
        const __m128i K8_SHUFFLE_X3_21 = SIMD_MM_SETR_EPI8(0xF, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
        const __m128i K8_SHUFFLE_X3_22 = SIMD_MM_SETR_EPI8(-1, 0x2, 0x0, 0x3, 0x4, 0x7, 0x5, 0x8, 0x6, 0x9, 0xA, 0xD, 0xB, 0xE, 0xC, 0xF);

        template <> SIMD_INLINE void InterpolateX<3>(const __m128i * alpha, __m128i * buffer)
        {
            __m128i src[3], shuffled[3];
            src[0] = _mm_load_si128(buffer + 0);
            src[1] = _mm_load_si128(buffer + 1);
            src[2] = _mm_load_si128(buffer + 2);
            shuffled[0] = _mm_shuffle_epi8(src[0], K8_SHUFFLE_X3_00);
            shuffled[0] = _mm_or_si128(shuffled[0], _mm_shuffle_epi8(src[1], K8_SHUFFLE_X3_01));
            _mm_store_si128(buffer + 0, _mm_maddubs_epi16(shuffled[0], _mm_load_si128(alpha + 0)));
            shuffled[1] = _mm_shuffle_epi8(src[0], K8_SHUFFLE_X3_10);
            shuffled[1] = _mm_or_si128(shuffled[1], _mm_shuffle_epi8(src[1], K8_SHUFFLE_X3_11));
            shuffled[1] = _mm_or_si128(shuffled[1], _mm_shuffle_epi8(src[2], K8_SHUFFLE_X3_12));
            _mm_store_si128(buffer + 1, _mm_maddubs_epi16(shuffled[1], _mm_load_si128(alpha + 1)));
            shuffled[2] = _mm_shuffle_epi8(src[1], K8_SHUFFLE_X3_21);
            shuffled[2] = _mm_or_si128(shuffled[2], _mm_shuffle_epi8(src[2], K8_SHUFFLE_X3_22));
            _mm_store_si128(buffer + 2, _mm_maddubs_epi16(shuffled[2], _mm_load_si128(alpha + 2)));
        }

        const __m128i K8_SHUFFLE_X4 = SIMD_MM_SETR_EPI8(0x0, 0x4, 0x1, 0x5, 0x2, 0x6, 0x3, 0x7, 0x8, 0xC, 0x9, 0xD, 0xA, 0xE, 0xB, 0xF);

        SIMD_INLINE void InterpolateX4(const __m128i * alpha, __m128i * buffer)
        {
            __m128i src = _mm_shuffle_epi8(_mm_load_si128(buffer), K8_SHUFFLE_X4);
            _mm_store_si128(buffer, _mm_maddubs_epi16(src, _mm_load_si128(alpha)));
        }

        template <> SIMD_INLINE void InterpolateX<4>(const __m128i * alpha, __m128i * buffer)
        {
            InterpolateX4(alpha + 0, buffer + 0);
            InterpolateX4(alpha + 1, buffer + 1);
            InterpolateX4(alpha + 2, buffer + 2);
            InterpolateX4(alpha + 3, buffer + 3);
        }

        const __m128i K16_FRACTION_ROUND_TERM = SIMD_MM_SET1_EPI16(Base::BILINEAR_ROUND_TERM);

        template<bool align> SIMD_INLINE __m128i InterpolateY(const __m128i * pbx0, const __m128i * pbx1, __m128i alpha[2])
        {
            __m128i sum = _mm_add_epi16(_mm_mullo_epi16(Load<align>(pbx0), alpha[0]), _mm_mullo_epi16(Load<align>(pbx1), alpha[1]));
            return _mm_srli_epi16(_mm_add_epi16(sum, K16_FRACTION_ROUND_TERM), Base::BILINEAR_SHIFT);
        }

        template<bool align> SIMD_INLINE void InterpolateY(const uint8_t * bx0, const uint8_t * bx1, __m128i alpha[2], uint8_t * dst)
        {
            __m128i lo = InterpolateY<align>((__m128i*)bx0 + 0, (__m128i*)bx1 + 0, alpha);
            __m128i hi = InterpolateY<align>((__m128i*)bx0 + 1, (__m128i*)bx1 + 1, alpha);
            Store<false>((__m128i*)dst, _mm_packus_epi16(lo, hi));
        }

        template <size_t channelCount> void ResizeBilinear(
            const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
        {
            assert(dstWidth >= A);

            struct One { uint8_t channels[channelCount]; };
            struct Two { uint8_t channels[channelCount * 2]; };

            size_t size = 2 * dstWidth*channelCount;
            size_t bufferSize = AlignHi(dstWidth, A)*channelCount * 2;
            size_t alignedSize = AlignHi(size, DA) - DA;
            const size_t step = A*channelCount;

            Buffer buffer(bufferSize, dstWidth, dstHeight);

            Base::EstimateAlphaIndex(srcHeight, dstHeight, buffer.iy, buffer.ay, 1);

            EstimateAlphaIndexX<channelCount>(srcWidth, dstWidth, buffer.ix, buffer.ax);

            ptrdiff_t previous = -2;

            __m128i a[2];

            for (size_t yDst = 0; yDst < dstHeight; yDst++, dst += dstStride)
            {
                a[0] = _mm_set1_epi16(int16_t(Base::FRACTION_RANGE - buffer.ay[yDst]));
                a[1] = _mm_set1_epi16(int16_t(buffer.ay[yDst]));

                ptrdiff_t sy = buffer.iy[yDst];
                int k = 0;

                if (sy == previous)
                    k = 2;
                else if (sy == previous + 1)
                {
                    Swap(buffer.bx[0], buffer.bx[1]);
                    k = 1;
                }

                previous = sy;

                for (; k < 2; k++)
                {
                    Two * pb = (Two *)buffer.bx[k];
                    const One * psrc = (const One *)(src + (sy + k)*srcStride);
                    for (size_t x = 0; x < dstWidth; x++)
                        pb[x] = *(Two *)(psrc + buffer.ix[x]);

                    uint8_t * pbx = buffer.bx[k];
                    for (size_t i = 0; i < bufferSize; i += step)
                        InterpolateX<channelCount>((__m128i*)(buffer.ax + i), (__m128i*)(pbx + i));
                }

                for (size_t ib = 0, id = 0; ib < alignedSize; ib += DA, id += A)
                    InterpolateY<true>(buffer.bx[0] + ib, buffer.bx[1] + ib, a, dst + id);
                size_t i = size - DA;
                InterpolateY<false>(buffer.bx[0] + i, buffer.bx[1] + i, a, dst + i / 2);
            }
        }

        SIMD_INLINE void LoadGray(const uint8_t * src, const Index & index, uint8_t * dst)
        {
            __m128i _src = _mm_loadu_si128((__m128i*)(src + index.src));
            __m128i _shuffle = _mm_loadu_si128((__m128i*)&index.shuffle);
            _mm_storeu_si128((__m128i*)(dst + index.dst), _mm_shuffle_epi8(_src, _shuffle));
        }

        void ResizeBilinearGray(const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride, uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride)
        {
            assert(dstWidth >= A);

            size_t size = 2 * dstWidth;
            size_t bufferWidth = AlignHi(dstWidth, A) * 2;
            size_t blockCount = BlockCountMax(srcWidth, dstWidth);
            size_t alignedSize = AlignHi(size, DA) - DA;

            BufferG buffer(bufferWidth, blockCount, dstHeight);

            Base::EstimateAlphaIndex(srcHeight, dstHeight, buffer.iy, buffer.ay, 1);

            EstimateAlphaIndexX((int)srcWidth, (int)dstWidth, buffer.ix, buffer.ax, blockCount);

            ptrdiff_t previous = -2;

            __m128i a[2];

            for (size_t yDst = 0; yDst < dstHeight; yDst++, dst += dstStride)
            {
                a[0] = _mm_set1_epi16(int16_t(Base::FRACTION_RANGE - buffer.ay[yDst]));
                a[1] = _mm_set1_epi16(int16_t(buffer.ay[yDst]));

                ptrdiff_t sy = buffer.iy[yDst];
                int k = 0;

                if (sy == previous)
                    k = 2;
                else if (sy == previous + 1)
                {
                    Swap(buffer.bx[0], buffer.bx[1]);
                    k = 1;
                }

                previous = sy;

                for (; k < 2; k++)
                {
                    const uint8_t * psrc = src + (sy + k)*srcStride;
                    uint8_t * pdst = buffer.bx[k];
                    for (size_t i = 0; i < blockCount; ++i)
                        LoadGray(psrc, buffer.ix[i], pdst);

                    uint8_t * pbx = buffer.bx[k];
                    for (size_t i = 0; i < bufferWidth; i += A)
                        InterpolateX<1>((__m128i*)(buffer.ax + i), (__m128i*)(pbx + i));
                }

                for (size_t ib = 0, id = 0; ib < alignedSize; ib += DA, id += A)
                    InterpolateY<true>(buffer.bx[0] + ib, buffer.bx[1] + ib, a, dst + id);
                size_t i = size - DA;
                InterpolateY<false>(buffer.bx[0] + i, buffer.bx[1] + i, a, dst + i / 2);
            }
        }

        void ResizeBilinear(
            const uint8_t *src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t *dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount)
        {
            switch (channelCount)
            {
            case 1:
                if (srcWidth >= A && srcWidth < 4 * dstWidth)
                    ResizeBilinearGray(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
                else
                    ResizeBilinear<1>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
                break;
            case 2:
                ResizeBilinear<2>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
                break;
            case 3:
                ResizeBilinear<3>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
                break;
            case 4:
                ResizeBilinear<4>(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride);
                break;
            default:
                Base::ResizeBilinear(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, channelCount);
            }
        }
    }
#endif
}

