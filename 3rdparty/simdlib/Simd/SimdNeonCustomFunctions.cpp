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
#ifdef SIMD_NEON_ENABLE
    namespace Neon
    {
        // https://developer.arm.com/architectures/instruction-sets/intrinsics/
        // https://github.com/DLTcollab/sse2neon/blob/master/sse2neon.h
        void SimdMatMulTwist(const double * mat, size_t rows, const double * twist, double * dst)
        {
#if defined(SIMD_ARM64_ENABLE)
            // Transpose twist matrix
            double transpose[36];
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 6; j++) {
                    transpose[i*6 + j] = twist[j*6 + i];
                }
            }

            for (size_t i = 0; i < rows; i++) {
                for (size_t j = 0; j < 6; j++) {
                    float64x2_t v_mul = vdupq_n_f64(0);
                    for (size_t k = 0; k < 6; k += 2) {
                        v_mul = vaddq_f64(v_mul, vmulq_f64(vld1q_f64(&mat[i*6 + k]), vld1q_f64(&transpose[j*6 + k])));
                    }

                    double v_tmp[2];
                    vst1q_f64(v_tmp, v_mul);
                    dst[i*6 + j] = v_tmp[0] + v_tmp[1];
                }
            }
#else
            (void) mat;
            (void) rows;
            (void) twist;
            (void) dst;
#endif
        }

        void SimdComputeJtR(const double * J, size_t rows, const double * R, double * dst)
        {
#if defined(SIMD_ARM64_ENABLE)
            float64x2_t v_JTR_0_1 = vdupq_n_f64(0);
            float64x2_t v_JTR_2_3 = vdupq_n_f64(0);
            float64x2_t v_JTR_4_5 = vdupq_n_f64(0);

            for (size_t i = 0; i < rows; i++) {
                const float64x2_t v_error = vdupq_n_f64(R[i]);

                float64x2_t v_interaction = vld1q_f64(&J[i*6]);
                v_JTR_0_1 = vaddq_f64(v_JTR_0_1, vmulq_f64(v_interaction, v_error));

                v_interaction = vld1q_f64(&J[i*6 + 2]);
                v_JTR_2_3 = vaddq_f64(v_JTR_2_3, vmulq_f64(v_interaction, v_error));

                v_interaction = vld1q_f64(&J[i*6 + 4]);
                v_JTR_4_5 = vaddq_f64(v_JTR_4_5, vmulq_f64(v_interaction, v_error));
            }

            vst1q_f64(dst, v_JTR_0_1);
            vst1q_f64(dst + 2, v_JTR_2_3);
            vst1q_f64(dst + 4, v_JTR_4_5);
#else
            (void) J;
            (void) rows;
            (void) R;
            (void) dst;
#endif
        }
    }
#else
    // Work around to avoid warning: libvisp_simdlib.a(SimdNeonCustomFunctions.cpp.o) has no symbols
    void dummy_SimdNeonCustomFunctions(){};
#endif// SIMD_NEON_ENABLE
}
