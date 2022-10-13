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
#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
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

        double SimdVectorSum(const double * vec, size_t size)
        {
            __m128d v_sum1 = _mm_setzero_pd(), v_sum2 = _mm_setzero_pd();

            size_t i = 0;
            for (; i <= size - 4; i += 4) {
                v_sum1 = _mm_add_pd(_mm_loadu_pd(vec + i), v_sum1);
                v_sum2 = _mm_add_pd(_mm_loadu_pd(vec + i + 2), v_sum2);
            }

            __m128d v_sum = _mm_add_pd(v_sum1, v_sum2);
            double res[2];
            _mm_storeu_pd(res, v_sum);
            double sum = res[0] + res[1];

            // tail processing
            for (; i < size; i++) {
                sum += vec[i];
            }

            return sum;
        }

        double SimdVectorSumSquare(const double * vec, size_t size)
        {
            double sum_square = 0.0;
            size_t i = 0;

            __m128d v_mul1, v_mul2;
            __m128d v_sum = _mm_setzero_pd();

            for (; i <= size - 4; i += 4) {
                v_mul1 = _mm_mul_pd(_mm_loadu_pd(vec + i), _mm_loadu_pd(vec + i));
                v_mul2 = _mm_mul_pd(_mm_loadu_pd(vec + i + 2), _mm_loadu_pd(vec + i + 2));

                v_sum = _mm_add_pd(v_mul1, v_sum);
                v_sum = _mm_add_pd(v_mul2, v_sum);
            }

            double res[2];
            _mm_storeu_pd(res, v_sum);

            sum_square = res[0] + res[1];

            for (; i < size; i++) {
                sum_square += vec[i] * vec[i];
            }

            return sum_square;
        }

        double SimdVectorStdev(const double * vec, size_t size, bool useBesselCorrection)
        {
            double mean_value = SimdVectorSum(vec, size) / size;
            double sum_squared_diff = 0.0;
            size_t i = 0;

            __m128d v_sub, v_mul, v_sum = _mm_setzero_pd();
            __m128d v_mean = _mm_set1_pd(mean_value);

            for (; i <= size - 4; i += 4) {
                v_sub = _mm_sub_pd(_mm_loadu_pd(vec + i), v_mean);
                v_mul = _mm_mul_pd(v_sub, v_sub);
                v_sum = _mm_add_pd(v_mul, v_sum);

                v_sub = _mm_sub_pd(_mm_loadu_pd(vec + i + 2), v_mean);
                v_mul = _mm_mul_pd(v_sub, v_sub);
                v_sum = _mm_add_pd(v_mul, v_sum);
            }

            double res[2];
            _mm_storeu_pd(res, v_sum);

            sum_squared_diff = res[0] + res[1];

            for (; i < size; i++) {
                sum_squared_diff += (vec[i] - mean_value) * (vec[i] - mean_value);
            }

            double divisor = (double)size;
            if (useBesselCorrection) {
                divisor = divisor - 1;
            }

            return std::sqrt(sum_squared_diff / divisor);
        }

        void SimdVectorHadamard(const double * src1, const double * src2, size_t size, double * dst)
        {
            size_t i = 0;
            for (; i <= size - 2; i += 2) {
                __m128d vout = _mm_mul_pd(_mm_loadu_pd(src1 + i), _mm_loadu_pd(src2 + i));
                _mm_storeu_pd(dst + i, vout);
            }

            for (; i < size; i++) {
                dst[i] = src1[i] * src2[i];
            }
        }

        void SimdMatMulTwist(const double * mat, size_t rows, const double * twist, double * dst)
        {
            // Transpose twist matrix
            double transpose[36];
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 6; j++) {
                    transpose[i*6 + j] = twist[j*6 + i];
                }
            }

            for (size_t i = 0; i < rows; i++) {
                for (size_t j = 0; j < 6; j++) {
                    __m128d v_mul = _mm_setzero_pd();
                    for (size_t k = 0; k < 6; k += 2) {
                        v_mul = _mm_add_pd(v_mul, _mm_mul_pd(_mm_loadu_pd(&mat[i*6 + k]), _mm_loadu_pd(&transpose[j*6 + k])));
                    }

                    double v_tmp[2];
                    _mm_storeu_pd(v_tmp, v_mul);
                    dst[i*6 + j] = v_tmp[0] + v_tmp[1];
                }
            }
        }

        void SimdNormalizedCorrelation(const double * img1, double mean1, const double * img2, double mean2, size_t size,
                                       double& a2, double& b2, double& ab)
        {
            const __m128d v_mean_a = _mm_set1_pd(mean1);
            const __m128d v_mean_b = _mm_set1_pd(mean2);
            __m128d v_ab = _mm_setzero_pd();
            __m128d v_a2 = _mm_setzero_pd();
            __m128d v_b2 = _mm_setzero_pd();

            size_t cpt = 0;
            for (; cpt <= size - 2; cpt += 2, img1 += 2, img2 += 2) {
                const __m128d v1 = _mm_loadu_pd(img1);
                const __m128d v2 = _mm_loadu_pd(img2);
                const __m128d norm_a = _mm_sub_pd(v1, v_mean_a);
                const __m128d norm_b = _mm_sub_pd(v2, v_mean_b);
                v_ab = _mm_add_pd(v_ab, _mm_mul_pd(norm_a, norm_b));
                v_a2 = _mm_add_pd(v_a2, _mm_mul_pd(norm_a, norm_a));
                v_b2 = _mm_add_pd(v_b2, _mm_mul_pd(norm_b, norm_b));
            }

            double v_res_ab[2], v_res_a2[2], v_res_b2[2];
            _mm_storeu_pd(v_res_ab, v_ab);
            _mm_storeu_pd(v_res_a2, v_a2);
            _mm_storeu_pd(v_res_b2, v_b2);

            ab = v_res_ab[0] + v_res_ab[1];
            a2 = v_res_a2[0] + v_res_a2[1];
            b2 = v_res_b2[0] + v_res_b2[1];

            for (; cpt < size; cpt++) {
                ab += (img1[cpt] - mean1) * (img2[cpt] - mean2);
                a2 += (img1[cpt] - mean1) * (img1[cpt] - mean1);
                b2 += (img2[cpt] - mean2) * (img2[cpt] - mean2);
            }
        }

        void SimdNormalizedCorrelation2(const double * img1_, size_t width1, const double * img2,
                                        size_t width2, size_t height2, size_t i0, size_t j0, double& ab)
        {
            const double *img1 = img1_;
            __m128d v_ab = _mm_setzero_pd();

            for (size_t i = 0; i < height2; i++) {
                size_t j = 0;
                img1 = &img1_[(i0 + i) * width1 + j0];

                for (; j <= width2 - 2; j += 2, img1 += 2, img2 += 2) {
                    const __m128d v1 = _mm_loadu_pd(img1);
                    const __m128d v2 = _mm_loadu_pd(img2);
                    v_ab = _mm_add_pd(v_ab, _mm_mul_pd(v1, v2));
                }

                for (; j < width2; j++) {
                    ab += img1[(i0 + i)*width1 + j0 + j] * img2[i*width2 + j];
                }
            }

            double v_res_ab[2];
            _mm_storeu_pd(v_res_ab, v_ab);

            ab += v_res_ab[0] + v_res_ab[1];
        }

        void SimdRemap(const unsigned char * src, size_t channels, size_t width, size_t height, size_t offset,
                       const int * mapU, const int * mapV, const float * mapDu, const float * mapDv, unsigned char * dst)
        {
            for (size_t j = 0; j < width; j++) {
                int u_round = mapU[offset + j];
                int v_round = mapV[offset + j];

                const __m128 vdu = _mm_set1_ps(mapDu[offset + j]);
                const __m128 vdv = _mm_set1_ps(mapDv[offset + j]);

                if (0 <= u_round && 0 <= v_round && u_round < static_cast<int>(width) - 1
                    && v_round < static_cast<int>(height) - 1) {
#define VLERP(va, vb, vt) _mm_add_ps(va, _mm_mul_ps(_mm_sub_ps(vb, va), vt))

                    // process interpolation
                    const __m128 vdata1 =
                        _mm_set_ps(static_cast<float>(src[(v_round*width + u_round)*channels + 3]), static_cast<float>(src[(v_round*width + u_round)*channels + 2]),
                                   static_cast<float>(src[(v_round*width + u_round)*channels + 1]), static_cast<float>(src[(v_round*width + u_round)*channels] + 0));

                    const __m128 vdata2 =
                        _mm_set_ps(static_cast<float>(src[(v_round*width + u_round + 1)*channels + 3]), static_cast<float>(src[(v_round*width + u_round + 1)*channels + 2]),
                                   static_cast<float>(src[(v_round*width + u_round + 1)*channels + 1]), static_cast<float>(src[(v_round*width + u_round + 1)*channels + 0]));

                    const __m128 vdata3 =
                        _mm_set_ps(static_cast<float>(src[((v_round + 1)*width + u_round)*channels + 3]), static_cast<float>(src[((v_round + 1)*width + u_round)*channels + 2]),
                                   static_cast<float>(src[((v_round + 1)*width + u_round)*channels + 1]), static_cast<float>(src[((v_round + 1)*width + u_round)*channels + 0]));

                    const __m128 vdata4 = _mm_set_ps(
                        static_cast<float>(src[((v_round + 1)*width + u_round + 1)*channels + 3]), static_cast<float>(src[((v_round + 1)*width + u_round + 1)*channels + 2]),
                        static_cast<float>(src[((v_round + 1)*width + u_round + 1)*channels + 1]), static_cast<float>(src[((v_round + 1)*width + u_round + 1)*channels + 0]));

                    const __m128 vcol0 = VLERP(vdata1, vdata2, vdu);
                    const __m128 vcol1 = VLERP(vdata3, vdata4, vdu);
                    const __m128 vvalue = VLERP(vcol0, vcol1, vdv);

  #undef VLERP

                    float values[4];
                    _mm_storeu_ps(values, vvalue);
                    dst[(offset + j)*channels + 0] = static_cast<unsigned char>(values[0]);
                    dst[(offset + j)*channels + 1] = static_cast<unsigned char>(values[1]);
                    dst[(offset + j)*channels + 2] = static_cast<unsigned char>(values[2]);
                    dst[(offset + j)*channels + 3] = static_cast<unsigned char>(values[3]);
                } else {
                    for (size_t c = 0; c < channels; c++) {
                        dst[(offset + j)*channels + c] = 0;
                    }
                }
            }
        }

        void SimdComputeJtR(const double * J, size_t rows, const double * R, double * dst)
        {
            __m128d v_JTR_0_1 = _mm_setzero_pd();
            __m128d v_JTR_2_3 = _mm_setzero_pd();
            __m128d v_JTR_4_5 = _mm_setzero_pd();

            for (size_t i = 0; i < rows; i++) {
                const __m128d v_error = _mm_set1_pd(R[i]);

                __m128d v_interaction = _mm_loadu_pd(&J[i*6]);
                v_JTR_0_1 = _mm_add_pd(v_JTR_0_1, _mm_mul_pd(v_interaction, v_error));

                v_interaction = _mm_loadu_pd(&J[i*6 + 2]);
                v_JTR_2_3 = _mm_add_pd(v_JTR_2_3, _mm_mul_pd(v_interaction, v_error));

                v_interaction = _mm_loadu_pd(&J[i*6 + 4]);
                v_JTR_4_5 = _mm_add_pd(v_JTR_4_5, _mm_mul_pd(v_interaction, v_error));
            }

            _mm_storeu_pd(dst, v_JTR_0_1);
            _mm_storeu_pd(dst + 2, v_JTR_2_3);
            _mm_storeu_pd(dst + 4, v_JTR_4_5);
        }

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
    // Work around to avoid warning: libvisp_simdlib.a(SimdSsse3CustomFunctions.cpp.o) has no symbols
    void dummy_SimdSse41CustomFunctions(){};
#endif// SIMD_SSE41_ENABLE
}
