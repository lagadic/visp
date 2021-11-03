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
#include "Simd/SimdStore.h"
#include "Simd/SimdResizer.h"
#include "Simd/SimdSet.h"
#include "Simd/SimdUpdate.h"

namespace Simd
{
#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
        ResizerByteBilinear::ResizerByteBilinear(const ResParam& param)
            : Sse2::ResizerByteBilinear(param)
            , _blocks(0)
        {
        }

        size_t ResizerByteBilinear::BlockCountMax(size_t align)
        {
            return (size_t)Simd::Max(::ceil(float(_param.srcW) / (align - 1)), ::ceil(float(_param.dstW) * 2.0f / align));
        }

        void ResizerByteBilinear::EstimateParams()
        {
            if (_ax.data)
                return;
            if (_param.channels == 1 && _param.srcW < 4 * _param.dstW)
                _blocks = BlockCountMax(A);
            float scale = (float)_param.srcW / _param.dstW;
            _ax.Resize(AlignHi(_param.dstW, A) * _param.channels * 2, false, _param.align);
            uint8_t* alphas = _ax.data;
            if (_blocks)
            {
                _ixg.Resize(_blocks);
                int block = 0;
                _ixg[0].src = 0;
                _ixg[0].dst = 0;
                for (int dstIndex = 0; dstIndex < (int)_param.dstW; ++dstIndex)
                {
                    float alpha = (float)((dstIndex + 0.5) * scale - 0.5);
                    int srcIndex = (int)::floor(alpha);
                    alpha -= srcIndex;

                    if (srcIndex < 0)
                    {
                        srcIndex = 0;
                        alpha = 0;
                    }

                    if (srcIndex > (int)_param.srcW - 2)
                    {
                        srcIndex = (int)_param.srcW - 2;
                        alpha = 1;
                    }

                    int dst = 2 * dstIndex - _ixg[block].dst;
                    int src = srcIndex - _ixg[block].src;
                    if (src >= A - 1 || dst >= A)
                    {
                        block++;
                        _ixg[block].src = Simd::Min(srcIndex, int(_param.srcW - A));
                        _ixg[block].dst = 2 * dstIndex;
                        dst = 0;
                        src = srcIndex - _ixg[block].src;
                    }
                    _ixg[block].shuffle[dst] = src;
                    _ixg[block].shuffle[dst + 1] = src + 1;

                    alphas[1] = (uint8_t)(alpha * Base::FRACTION_RANGE + 0.5);
                    alphas[0] = (uint8_t)(Base::FRACTION_RANGE - alphas[1]);
                    alphas += 2;
                }
                _blocks = block + 1;
            }
            else
            {
                _ix.Resize(_param.dstW);
                for (size_t i = 0; i < _param.dstW; ++i)
                {
                    float alpha = (float)((i + 0.5) * scale - 0.5);
                    ptrdiff_t index = (ptrdiff_t)::floor(alpha);
                    alpha -= index;

                    if (index < 0)
                    {
                        index = 0;
                        alpha = 0;
                    }

                    if (index > (ptrdiff_t)_param.srcW - 2)
                    {
                        index = _param.srcW - 2;
                        alpha = 1;
                    }

                    _ix[i] = (int)index;
                    alphas[1] = (uint8_t)(alpha * Base::FRACTION_RANGE + 0.5);
                    alphas[0] = (uint8_t)(Base::FRACTION_RANGE - alphas[1]);
                    for (size_t channel = 1; channel < _param.channels; channel++)
                        ((uint16_t*)alphas)[channel] = *(uint16_t*)alphas;
                    alphas += 2 * _param.channels;
                }
            }
            size_t size = AlignHi(_param.dstW, _param.align) * _param.channels * 2;
            _bx[0].Resize(size, false, _param.align);
            _bx[1].Resize(size, false, _param.align);
        }

        template <size_t N> void ResizerByteBilinearInterpolateX(const __m128i* alpha, __m128i* buffer);

        template <> SIMD_INLINE void ResizerByteBilinearInterpolateX<1>(const __m128i* alpha, __m128i* buffer)
        {
            _mm_store_si128(buffer, _mm_maddubs_epi16(_mm_load_si128(buffer), _mm_load_si128(alpha)));
        }

        const __m128i K8_SHUFFLE_X2 = SIMD_MM_SETR_EPI8(0x0, 0x2, 0x1, 0x3, 0x4, 0x6, 0x5, 0x7, 0x8, 0xA, 0x9, 0xB, 0xC, 0xE, 0xD, 0xF);

        SIMD_INLINE void ResizerByteBilinearInterpolateX2(const __m128i* alpha, __m128i* buffer)
        {
            __m128i src = _mm_shuffle_epi8(_mm_load_si128(buffer), K8_SHUFFLE_X2);
            _mm_store_si128(buffer, _mm_maddubs_epi16(src, _mm_load_si128(alpha)));
        }

        template <> SIMD_INLINE void ResizerByteBilinearInterpolateX<2>(const __m128i* alpha, __m128i* buffer)
        {
            ResizerByteBilinearInterpolateX2(alpha + 0, buffer + 0);
            ResizerByteBilinearInterpolateX2(alpha + 1, buffer + 1);
        }

        const __m128i K8_SHUFFLE_X3_00 = SIMD_MM_SETR_EPI8(0x0, 0x3, 0x1, 0x4, 0x2, 0x5, 0x6, 0x9, 0x7, 0xA, 0x8, 0xB, 0xC, 0xF, 0xD, -1);
        const __m128i K8_SHUFFLE_X3_01 = SIMD_MM_SETR_EPI8(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0x0);
        const __m128i K8_SHUFFLE_X3_10 = SIMD_MM_SETR_EPI8(0xE, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
        const __m128i K8_SHUFFLE_X3_11 = SIMD_MM_SETR_EPI8(-1, 0x1, 0x2, 0x5, 0x3, 0x6, 0x4, 0x7, 0x8, 0xB, 0x9, 0xC, 0xA, 0xD, 0xE, -1);
        const __m128i K8_SHUFFLE_X3_12 = SIMD_MM_SETR_EPI8(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0x1);
        const __m128i K8_SHUFFLE_X3_21 = SIMD_MM_SETR_EPI8(0xF, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
        const __m128i K8_SHUFFLE_X3_22 = SIMD_MM_SETR_EPI8(-1, 0x2, 0x0, 0x3, 0x4, 0x7, 0x5, 0x8, 0x6, 0x9, 0xA, 0xD, 0xB, 0xE, 0xC, 0xF);

        template <> SIMD_INLINE void ResizerByteBilinearInterpolateX<3>(const __m128i* alpha, __m128i* buffer)
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

        SIMD_INLINE void ResizerByteBilinearInterpolateX4(const __m128i* alpha, __m128i* buffer)
        {
            __m128i src = _mm_shuffle_epi8(_mm_load_si128(buffer), K8_SHUFFLE_X4);
            _mm_store_si128(buffer, _mm_maddubs_epi16(src, _mm_load_si128(alpha)));
        }

        template <> SIMD_INLINE void ResizerByteBilinearInterpolateX<4>(const __m128i* alpha, __m128i* buffer)
        {
            ResizerByteBilinearInterpolateX4(alpha + 0, buffer + 0);
            ResizerByteBilinearInterpolateX4(alpha + 1, buffer + 1);
            ResizerByteBilinearInterpolateX4(alpha + 2, buffer + 2);
            ResizerByteBilinearInterpolateX4(alpha + 3, buffer + 3);
        }

        const __m128i K16_FRACTION_ROUND_TERM = SIMD_MM_SET1_EPI16(Base::BILINEAR_ROUND_TERM);

        template<bool align> SIMD_INLINE __m128i ResizerByteBilinearInterpolateY(const __m128i* pbx0, const __m128i* pbx1, __m128i alpha[2])
        {
            __m128i sum = _mm_add_epi16(_mm_mullo_epi16(Load<align>(pbx0), alpha[0]), _mm_mullo_epi16(Load<align>(pbx1), alpha[1]));
            return _mm_srli_epi16(_mm_add_epi16(sum, K16_FRACTION_ROUND_TERM), Base::BILINEAR_SHIFT);
        }

        template<bool align> SIMD_INLINE void ResizerByteBilinearInterpolateY(const uint8_t* bx0, const uint8_t* bx1, __m128i alpha[2], uint8_t* dst)
        {
            __m128i lo = ResizerByteBilinearInterpolateY<align>((__m128i*)bx0 + 0, (__m128i*)bx1 + 0, alpha);
            __m128i hi = ResizerByteBilinearInterpolateY<align>((__m128i*)bx0 + 1, (__m128i*)bx1 + 1, alpha);
            Store<false>((__m128i*)dst, _mm_packus_epi16(lo, hi));
        }

        template<size_t N> void ResizerByteBilinear::Run(const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride)
        {
            struct One { uint8_t val[N * 1]; };
            struct Two { uint8_t val[N * 2]; };

            size_t size = 2 * _param.dstW * N;
            size_t aligned = AlignHi(size, DA) - DA;
            const size_t step = A * N;
            ptrdiff_t previous = -2;
            __m128i a[2];
            uint8_t* bx[2] = { _bx[0].data, _bx[1].data };
            const uint8_t* ax = _ax.data;
            const int32_t* ix = _ix.data;
            size_t dstW = _param.dstW;

            for (size_t yDst = 0; yDst < _param.dstH; yDst++, dst += dstStride)
            {
                a[0] = _mm_set1_epi16(int16_t(Base::FRACTION_RANGE - _ay[yDst]));
                a[1] = _mm_set1_epi16(int16_t(_ay[yDst]));

                ptrdiff_t sy = _iy[yDst];
                int k = 0;

                if (sy == previous)
                    k = 2;
                else if (sy == previous + 1)
                {
                    Swap(bx[0], bx[1]);
                    k = 1;
                }

                previous = sy;

                for (; k < 2; k++)
                {
                    Two* pb = (Two*)bx[k];
                    const One* psrc = (const One*)(src + (sy + k) * srcStride);
                    for (size_t x = 0; x < dstW; x++)
                        pb[x] = *(Two*)(psrc + ix[x]);

                    uint8_t* pbx = bx[k];
                    for (size_t i = 0; i < size; i += step)
                        ResizerByteBilinearInterpolateX<N>((__m128i*)(ax + i), (__m128i*)(pbx + i));
                }

                for (size_t ib = 0, id = 0; ib < aligned; ib += DA, id += A)
                    ResizerByteBilinearInterpolateY<true>(bx[0] + ib, bx[1] + ib, a, dst + id);
                size_t i = size - DA;
                ResizerByteBilinearInterpolateY<false>(bx[0] + i, bx[1] + i, a, dst + i / 2);
            }
        }

        template <class Idx> SIMD_INLINE void ResizerByteBilinearLoadGrayInterpolated(const uint8_t* src, const Idx& index, const uint8_t* alpha, uint8_t* dst)
        {
            __m128i _src = _mm_loadu_si128((__m128i*)(src + index.src));
            __m128i _shuffle = _mm_loadu_si128((__m128i*) & index.shuffle);
            __m128i _alpha = _mm_loadu_si128((__m128i*)(alpha + index.dst));
            _mm_storeu_si128((__m128i*)(dst + index.dst), _mm_maddubs_epi16(_mm_shuffle_epi8(_src, _shuffle), _alpha));
        }

        void ResizerByteBilinear::RunG(const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride)
        {
            size_t bufW = AlignHi(_param.dstW, A) * 2;
            size_t size = 2 * _param.dstW;
            size_t aligned = AlignHi(size, DA) - DA;
            size_t blocks = _blocks;
            ptrdiff_t previous = -2;
            __m128i a[2];
            uint8_t* bx[2] = { _bx[0].data, _bx[1].data };
            const uint8_t* ax = _ax.data;
            const Idx* ixg = _ixg.data;

            for (size_t yDst = 0; yDst < _param.dstH; yDst++, dst += dstStride)
            {
                a[0] = _mm_set1_epi16(int16_t(Base::FRACTION_RANGE - _ay[yDst]));
                a[1] = _mm_set1_epi16(int16_t(_ay[yDst]));

                ptrdiff_t sy = _iy[yDst];
                int k = 0;

                if (sy == previous)
                    k = 2;
                else if (sy == previous + 1)
                {
                    Swap(bx[0], bx[1]);
                    k = 1;
                }

                previous = sy;

                for (; k < 2; k++)
                {
                    const uint8_t* psrc = src + (sy + k) * srcStride;
                    uint8_t* pdst = bx[k];
                    for (size_t i = 0; i < blocks; ++i)
                        ResizerByteBilinearLoadGrayInterpolated(psrc, ixg[i], ax, pdst);
                }

                for (size_t ib = 0, id = 0; ib < aligned; ib += DA, id += A)
                    ResizerByteBilinearInterpolateY<true>(bx[0] + ib, bx[1] + ib, a, dst + id);
                size_t i = size - DA;
                ResizerByteBilinearInterpolateY<false>(bx[0] + i, bx[1] + i, a, dst + i / 2);
            }
        }

        void ResizerByteBilinear::Run(const uint8_t* src, size_t srcStride, uint8_t* dst, size_t dstStride)
        {
            assert(_param.dstW >= A);

            EstimateParams();
            switch (_param.channels)
            {
            case 1:
                if (_blocks)
                    RunG(src, srcStride, dst, dstStride);
                else
                    Run<1>(src, srcStride, dst, dstStride);
                break;
            case 2: Run<2>(src, srcStride, dst, dstStride); break;
            case 3: Run<3>(src, srcStride, dst, dstStride); break;
            case 4: Run<4>(src, srcStride, dst, dstStride); break;
            default:
                assert(0);
            }
        }

        //---------------------------------------------------------------------

        ResizerByteArea::ResizerByteArea(const ResParam & param)
            : Sse2::ResizerByteArea(param)
        {
        }

        SIMD_INLINE __m128i SaveLoadTail(const uint8_t * ptr, size_t tail)
        {
            uint8_t buffer[DA];
            _mm_storeu_si128((__m128i*)(buffer), _mm_loadu_si128((__m128i*)(ptr + tail - A)));
            return _mm_loadu_si128((__m128i*)(buffer + A - tail));
        }

        template<UpdateType update> SIMD_INLINE void ResizerByteAreaRowUpdate(const uint8_t * src0, size_t size, int32_t a, int32_t * dst)
        {
            __m128i alpha = SetInt16(a, a);
            size_t sizeA = AlignLo(size, A);
            size_t i = 0;
            for (; i < sizeA; i += A, dst += A)
            {
                __m128i s0 = _mm_loadu_si128((__m128i*)(src0 + i));
                __m128i i0 = UnpackU8<0>(s0);
                __m128i i1 = UnpackU8<1>(s0);
                Update<update, true>(dst + 0 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i0)));
                Update<update, true>(dst + 1 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i0)));
                Update<update, true>(dst + 2 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i1)));
                Update<update, true>(dst + 3 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i1)));
            }
            if (i < size)
            {
                __m128i s0 = SaveLoadTail(src0 + i, size - i);
                __m128i i0 = UnpackU8<0>(s0);
                __m128i i1 = UnpackU8<1>(s0);
                Update<update, true>(dst + 0 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i0)));
                Update<update, true>(dst + 1 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i0)));
                Update<update, true>(dst + 2 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i1)));
                Update<update, true>(dst + 3 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i1)));
            }
        }

        template<UpdateType update> SIMD_INLINE void ResizerByteAreaRowUpdate(const uint8_t * src0, size_t stride, size_t size, int32_t a0, int32_t a1, int32_t * dst)
        {
            __m128i alpha = SetInt16(a0, a1);
            const uint8_t * src1 = src0 + stride;
            size_t sizeA = AlignLo(size, A);
            size_t i = 0;
            for (; i < sizeA; i += A, dst += A)
            {
                __m128i s0 = _mm_loadu_si128((__m128i*)(src0 + i));
                __m128i s1 = _mm_loadu_si128((__m128i*)(src1 + i));
                __m128i i0 = UnpackU8<0>(s0, s1);
                __m128i i1 = UnpackU8<1>(s0, s1);
                Update<update, true>(dst + 0 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i0)));
                Update<update, true>(dst + 1 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i0)));
                Update<update, true>(dst + 2 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i1)));
                Update<update, true>(dst + 3 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i1)));
            }
            if (i < size)
            {
                __m128i s0 = _mm_loadu_si128((__m128i*)(src0 + i));
                __m128i s1 = SaveLoadTail(src1 + i, size - i);
                __m128i i0 = UnpackU8<0>(s0, s1);
                __m128i i1 = UnpackU8<1>(s0, s1);
                Update<update, true>(dst + 0 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i0)));
                Update<update, true>(dst + 1 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i0)));
                Update<update, true>(dst + 2 * F, _mm_madd_epi16(alpha, UnpackU8<0>(i1)));
                Update<update, true>(dst + 3 * F, _mm_madd_epi16(alpha, UnpackU8<1>(i1)));
            }
        }

        SIMD_INLINE void ResizerByteAreaRowSum(const uint8_t * src, size_t stride, size_t count, size_t size, int32_t curr, int32_t zero, int32_t next, int32_t * dst)
        {
            if (count)
            {
                size_t i = 0;
                ResizerByteAreaRowUpdate<UpdateSet>(src, stride, size, curr, count == 1 ? zero - next : zero, dst), src += 2 * stride, i +=2;
                for (; i < count; i += 2, src += 2 * stride)
                    ResizerByteAreaRowUpdate<UpdateAdd>(src, stride, size, zero, i == count - 1 ? zero - next : zero, dst);
                if (i == count)
                    ResizerByteAreaRowUpdate<UpdateAdd>(src, size, zero - next, dst);
            }
            else
                ResizerByteAreaRowUpdate<UpdateSet>(src, size, curr - next, dst);
        }

        template<size_t N> SIMD_INLINE void ResizerByteAreaSet(const int32_t * src, int32_t value, int32_t * dst)
        {
            for (size_t c = 0; c < N; ++c)
                dst[c] = src[c] * value;
        }

        template<size_t N> SIMD_INLINE void ResizerByteAreaAdd(const int32_t * src, int32_t value, int32_t * dst)
        {
            for (size_t c = 0; c < N; ++c)
                dst[c] += src[c] * value;
        }

        template<size_t N> SIMD_INLINE void ResizerByteAreaRes(const int32_t * src, uint8_t * dst)
        {
            for (size_t c = 0; c < N; ++c)
                dst[c] = uint8_t((src[c] + Base::AREA_ROUND) >> Base::AREA_SHIFT);
        }

        template<size_t N> SIMD_INLINE void ResizerByteAreaResult(const int32_t * src, size_t count, int32_t curr, int32_t zero, int32_t next, uint8_t * dst)
        {
            int32_t sum[N];
            ResizerByteAreaSet<N>(src, curr, sum);
            for (size_t i = 0; i < count; ++i)
                src += N, ResizerByteAreaAdd<N>(src, zero, sum);
            ResizerByteAreaAdd<N>(src, -next, sum);
            ResizerByteAreaRes<N>(sum, dst);
        }

        template<size_t N> SIMD_INLINE void ResizerByteAreaResult34(const int32_t * src, size_t count, int32_t curr, int32_t zero, int32_t next, uint8_t * dst)
        {
            __m128i sum = _mm_mullo_epi32(_mm_loadu_si128((__m128i*)src), _mm_set1_epi32(curr));
            for (size_t i = 0; i < count; ++i)
                src += N, sum = _mm_add_epi32(sum, _mm_mullo_epi32(_mm_loadu_si128((__m128i*)src), _mm_set1_epi32(zero)));
            sum = _mm_add_epi32(sum, _mm_mullo_epi32(_mm_loadu_si128((__m128i*)src), _mm_set1_epi32(-next)));
            __m128i res = _mm_srai_epi32(_mm_add_epi32(sum, _mm_set1_epi32(Base::AREA_ROUND)), Base::AREA_SHIFT);
            *(int32_t*)dst = _mm_cvtsi128_si32(_mm_packus_epi16(_mm_packus_epi32(res, K_ZERO), K_ZERO));
        }

        template<> SIMD_INLINE void ResizerByteAreaResult<4>(const int32_t * src, size_t count, int32_t curr, int32_t zero, int32_t next, uint8_t * dst)
        {
            ResizerByteAreaResult34<4>(src, count, curr, zero, next, dst);
        }

        template<> SIMD_INLINE void ResizerByteAreaResult<3>(const int32_t * src, size_t count, int32_t curr, int32_t zero, int32_t next, uint8_t * dst)
        {
            ResizerByteAreaResult34<3>(src, count, curr, zero, next, dst);
        }

        template<size_t N> void ResizerByteArea::Run(const uint8_t * src, size_t srcStride, uint8_t * dst, size_t dstStride)
        {
            size_t dstW = _param.dstW, rowSize = _param.srcW*N, rowRest = dstStride - dstW*N;
            const int32_t * iy = _iy.data, * ix = _ix.data, * ay = _ay.data, * ax = _ax.data;
            int32_t ay0 = ay[0], ax0 = ax[0];
            for (size_t dy = 0; dy < _param.dstH; dy++, dst += rowRest)
            {
                int32_t * buf = _by.data;
                size_t yn = iy[dy + 1] - iy[dy];
                ResizerByteAreaRowSum(src, srcStride, yn, rowSize, ay[dy], ay0, ay[dy + 1], buf), src += yn * srcStride;
                for (size_t dx = 0; dx < dstW; dx++, dst += N)
                {
                    size_t xn = ix[dx + 1] - ix[dx];
                    ResizerByteAreaResult<N>(buf, xn, ax[dx], ax0, ax[dx + 1], dst), buf += xn * N;
                }
            }
        }

        void ResizerByteArea::Run(const uint8_t * src, size_t srcStride, uint8_t * dst, size_t dstStride)
        {
            switch (_param.channels)
            {
            case 1: Run<1>(src, srcStride, dst, dstStride); return;
            case 2: Run<2>(src, srcStride, dst, dstStride); return;
            case 3: Run<3>(src, srcStride, dst, dstStride); return;
            case 4: Run<4>(src, srcStride, dst, dstStride); return;
            default:
                assert(0);
            }
        }

        //---------------------------------------------------------------------

        void * ResizerInit(size_t srcX, size_t srcY, size_t dstX, size_t dstY, size_t channels, SimdResizeChannelType type, SimdResizeMethodType method)
        {
            ResParam param(srcX, srcY, dstX, dstY, channels, type, method, sizeof(__m128i));
            if (param.IsByteBilinear() && dstX >= A)
                return new ResizerByteBilinear(param);
            else if (param.IsByteArea())
                return new ResizerByteArea(param);
            else
                return Sse2::ResizerInit(srcX, srcY, dstX, dstY, channels, type, method);
        }
    }
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdSse41Resizer.cpp.o) has no symbols
    void dummy_SimdSse41Resizer(){};
#endif//SIMD_SSE41_ENABLE
}

