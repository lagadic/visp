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
#ifdef SIMD_AVX_ENABLE
    namespace Avx
    {
        static void transpose4x4(const double* a, size_t rows, size_t cols, double * b, size_t i, size_t j)
        {
            __m256d a0 = _mm256_loadu_pd(&a[i*cols + j]);
            __m256d a1 = _mm256_loadu_pd(&a[(i+1)*cols + j]);
            __m256d a2 = _mm256_loadu_pd(&a[(i+2)*cols + j]);
            __m256d a3 = _mm256_loadu_pd(&a[(i+3)*cols + j]);

            __m256d T0 = _mm256_shuffle_pd(a0, a1, 15);
            __m256d T1 = _mm256_shuffle_pd(a0, a1, 0);
            __m256d T2 = _mm256_shuffle_pd(a2, a3, 15);
            __m256d T3 = _mm256_shuffle_pd(a2, a3, 0);

            a1 = _mm256_permute2f128_pd(T0, T2, 32);
            a3 = _mm256_permute2f128_pd(T0, T2, 49);
            a0 = _mm256_permute2f128_pd(T1, T3, 32);
            a2 = _mm256_permute2f128_pd(T1, T3, 49);

            _mm256_storeu_pd(&b[j*rows + i], a0);
            _mm256_storeu_pd(&b[(j+1)*rows + i], a1);
            _mm256_storeu_pd(&b[(j+2)*rows + i], a2);
            _mm256_storeu_pd(&b[(j+3)*rows + i], a3);
        }

        void transpose16x16(const double * a, size_t rows, size_t cols, double * b, size_t i, size_t j)
        {
            transpose4x4(a, rows, cols, b, i, j);
            transpose4x4(a, rows, cols, b, i, j + 4);
            transpose4x4(a, rows, cols, b, i, j + 8);
            transpose4x4(a, rows, cols, b, i, j + 12);

            transpose4x4(a, rows, cols, b, i + 4, j);
            transpose4x4(a, rows, cols, b, i + 4, j + 4);
            transpose4x4(a, rows, cols, b, i + 4, j + 8);
            transpose4x4(a, rows, cols, b, i + 4, j + 12);

            transpose4x4(a, rows, cols, b, i + 8, j);
            transpose4x4(a, rows, cols, b, i + 8, j + 4);
            transpose4x4(a, rows, cols, b, i + 8, j + 8);
            transpose4x4(a, rows, cols, b, i + 8, j + 12);

            transpose4x4(a, rows, cols, b, i + 12, j);
            transpose4x4(a, rows, cols, b, i + 12, j + 4);
            transpose4x4(a, rows, cols, b, i + 12, j + 8);
            transpose4x4(a, rows, cols, b, i + 12, j + 12);
        }

        void SimdMatTranspose(const double * mat, size_t rows, size_t cols, double * dst)
        {
            // Matrix transpose using tiling
            const int nrows = static_cast<int>(rows);
            const int ncols = static_cast<int>(cols);
            const int tileSize = 16;

            for (int i = 0; i < nrows;) {
                for (; i <= nrows - tileSize; i += tileSize) {
                    int j = 0;
                    for (; j <= ncols - tileSize; j += tileSize) {
                        transpose16x16(mat, rows, cols, dst, i, j);
                    }

                    for (int k = i; k < i + tileSize; k++) {
                        for (int l = j; l < ncols; l++) {
                            dst[l*rows + k] = mat[k*cols + l];
                        }
                    }
                }

                for (; i < nrows; i++) {
                    for (int j = 0; j < ncols; j++) {
                        dst[j*rows + i] = mat[i*cols + j];
                    }
                }
            }

            _mm256_zeroupper();
        }
    }
#else
    // Work arround to avoid warning: libvisp_simdlib.a(SimdAvx1CustomFunctions.cpp.o) has no symbols
    void dummy_SimdAvx1CustomFunctions(){};
#endif// SIMD_AVX_ENABLE
}
