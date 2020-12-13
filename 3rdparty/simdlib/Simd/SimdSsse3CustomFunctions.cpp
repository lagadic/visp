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
#include "Simd/SimdBase.h"
#include "Simd/SimdStore.h"

namespace Simd
{
#ifdef SIMD_SSSE3_ENABLE
    namespace Ssse3
    {
        void SimdImageDifference(const unsigned char * img1, const unsigned char * img2, size_t size, unsigned char * imgDiff)
        {
            const __m128i mask1 = _mm_set_epi8(-1, 14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0);
            const __m128i mask2 = _mm_set_epi8(-1, 15, -1, 13, -1, 11, -1, 9, -1, 7, -1, 5, -1, 3, -1, 1);
            const __m128i mask_out2 = _mm_set_epi8(14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0, -1);

            size_t i = 0;
            for (; i <= size-16; i+= 16) {
                const __m128i vdata1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(img1 + i));
                const __m128i vdata2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(img2 + i));

                __m128i vdata1_reorg = _mm_shuffle_epi8(vdata1, mask1);
                __m128i vdata2_reorg = _mm_shuffle_epi8(vdata2, mask1);

                const __m128i vshift = _mm_set1_epi16(128);
                __m128i vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);

                const __m128i v255 = _mm_set1_epi16(255);
                const __m128i vzero = _mm_setzero_si128();
                const __m128i vdata_diff_min_max1 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

                vdata1_reorg = _mm_shuffle_epi8(vdata1, mask2);
                vdata2_reorg = _mm_shuffle_epi8(vdata2, mask2);

                vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);
                const __m128i vdata_diff_min_max2 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

                _mm_storeu_si128(reinterpret_cast<__m128i *>(imgDiff + i), _mm_or_si128(_mm_shuffle_epi8(vdata_diff_min_max1, mask1),
                                                                                        _mm_shuffle_epi8(vdata_diff_min_max2, mask_out2)));
            }

            if (i < size) {
                Base::SimdImageDifference(img1 + i, img2 + i, size - i, imgDiff + i);
            }
        }
    }
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdSsse3CustomFunctions.cpp.o) has no symbols
    void dummy_SimdSsse3CustomFunctions(){};
#endif// SIMD_SSSE3_ENABLE
}
