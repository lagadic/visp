/*
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

namespace Simd
{
#ifdef SIMD_SSE2_ENABLE
    namespace Sse2
    {
        void ImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
        {
            const size_t buffWidth = width + 2;
            const size_t alignedSize = Simd::AlignLo(width, A);

            if (connexityType == SimdImageConnexity4) {
                size_t offset[5] = {1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1};

                for (size_t i = 0; i < height; i++) {
                    const uint8_t *ptr_buff = buff + i * buffWidth;
                    uint8_t *ptr_img = img + i * width;

                    for (size_t j = 0; j < alignedSize; j += A) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[0]));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[1])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[2])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[3])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[4])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + j), m);
                    }

                    if (alignedSize != width) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[0]));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[1])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[2])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[3])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[4])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + (width - A)), m);
                    }
                }
            } else {
                size_t offset[9] = { 0,
                                     1,
                                     2,
                                     buffWidth,
                                     buffWidth + 1,
                                     buffWidth + 2,
                                     buffWidth * 2,
                                     buffWidth * 2 + 1,
                                     buffWidth * 2 + 2 };

                for (size_t i = 0; i < height; i++) {
                    const uint8_t *ptr_buff = buff + i * buffWidth;
                    uint8_t *ptr_img = img + i * width;

                    for (size_t j = 0; j < alignedSize; j += A) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[0]));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[1])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[2])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[3])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[4])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[5])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[6])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[7])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[8])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + j), m);
                    }

                    if (alignedSize != width) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[0]));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[1])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[2])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[3])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[4])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[5])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[6])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[7])));
                        m = _mm_min_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[8])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + (width - A)), m);
                    }
                }
            }
        }

        void ImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
        {
            const size_t buffWidth = width + 2;
            const size_t alignedSize = Simd::AlignLo(width, A);

            if (connexityType == SimdImageConnexity4) {
                size_t offset[5] = {1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1};

                for (size_t i = 0; i < height; i++) {
                    const uint8_t *ptr_buff = buff + i * buffWidth;
                    uint8_t *ptr_img = img + i * width;

                    for (size_t j = 0; j < alignedSize; j += A) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[0]));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[1])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[2])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[3])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[4])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + j), m);
                    }

                    if (alignedSize != width) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[0]));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[1])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[2])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[3])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[4])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + (width - A)), m);
                    }
                }
            } else {
                size_t offset[9] = { 0,
                                     1,
                                     2,
                                     buffWidth,
                                     buffWidth + 1,
                                     buffWidth + 2,
                                     buffWidth * 2,
                                     buffWidth * 2 + 1,
                                     buffWidth * 2 + 2 };

                for (size_t i = 0; i < height; i++) {
                    const uint8_t *ptr_buff = buff + i * buffWidth;
                    uint8_t *ptr_img = img + i * width;

                    for (size_t j = 0; j < alignedSize; j += A) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[0]));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[1])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[2])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[3])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[4])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[5])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[6])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[7])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + j + offset[8])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + j), m);
                    }

                    if (alignedSize != width) {
                        __m128i m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[0]));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[1])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[2])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[3])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[4])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[5])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[6])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[7])));
                        m = _mm_max_epu8(m, _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_buff + (width - A) + offset[8])));

                        _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_img + (width - A)), m);
                    }
                }
            }
        }
    }
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdSse2CustomFunctions.cpp.o) has no symbols
    void dummy_SimdSse2CustomFunctions(){};
#endif// SIMD_SSE2_ENABLE
}
