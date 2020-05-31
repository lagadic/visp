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
#ifndef __SimdArray_h__
#define __SimdArray_h__

#include "Simd/SimdMemory.h"

namespace Simd
{
    template <class T> struct Array
    {
        T * const data;
        size_t const size;

        SIMD_INLINE Array(size_t size_ = 0, bool clear = false, size_t align = SIMD_ALIGN)
            : data(0)
            , size(0)
        {
            Resize(size_, clear);
        }

        SIMD_INLINE ~Array()
        {
            if (data)
                Simd::Free(data);
        }

        SIMD_INLINE void Resize(size_t size_, bool clear = false, size_t align = SIMD_ALIGN)
        {
            if (size_ != size)
            {
                if (data)
                {
                    Simd::Free(data);
                    *(T**)&data = 0;
                }
                *(size_t*)&size = size_;
                if (size_)
                    *(T**)&data = (T*)Simd::Allocate(size * sizeof(T), align);
            }
            if (clear)
                Clear();
        }

        SIMD_INLINE void Clear()
        {
            ::memset(data, 0, size * sizeof(T));
        }

        SIMD_INLINE T & operator[] (size_t i)
        {
            return data[i];
        }

        SIMD_INLINE const T & operator[] (size_t i) const
        {
            return data[i];
        }
    };

    typedef Array<uint8_t> Array8u;
    typedef Array<int16_t> Array16i;
    typedef Array<uint16_t> Array16u;
    typedef Array<int32_t> Array32i;
    typedef Array<float> Array32f;

#if defined(__GNUC__) && __GNUC__ >= 6
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#endif

#ifdef SIMD_SSE_ENABLE
    namespace Sse
    {
        typedef Array<__m128> Array128f;
    }
#endif

#ifdef SIMD_AVX_ENABLE
    namespace Avx
    {
        typedef Array<__m256> Array256f;
    }
#endif

#ifdef SIMD_NEON_ENABLE
    namespace Neon
    {
        typedef Array<float32x4_t> Array128f;
    }
#endif

#if defined(__GNUC__) && __GNUC__ >= 6
#pragma GCC diagnostic pop
#endif
}

#endif//__SimdArray_h__
