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
#include <algorithm>

namespace Simd
{
namespace Base
{
void ImageErosion(uint8_t *img, const uint8_t *buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
  const size_t buffWidth = width + 2;
  if (connexityType == SimdImageConnexity4) {
    size_t offset[5] = { 1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1 };

    for (size_t i = 0; i < height; i++) {
      const uint8_t *ptr_buff = buff + i * buffWidth;
      uint8_t *ptr_img = img + i * width;

      for (size_t j = 0; j < width; j++) {
        uint8_t min_value = 255;
        for (int k = 0; k < 5; k++) {
          min_value = (std::min)(min_value, *(ptr_buff + j + offset[k]));
        }

        *(ptr_img + j) = min_value;
      }
    }
  }
  else {
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

      for (size_t j = 0; j < width; j++) {
        uint8_t min_value = 255;
        for (int k = 0; k < 9; k++) {
          min_value = (std::min)(min_value, *(ptr_buff + j + offset[k]));
        }

        *(ptr_img + j) = min_value;
      }
    }
  }
}

void ImageDilatation(uint8_t *img, const uint8_t *buff, size_t width, size_t height, SimdImageConnexityType connexityType)
{
  const size_t buffWidth = width + 2;
  if (connexityType == SimdImageConnexity4) {
    size_t offset[5] = { 1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1 };

    for (size_t i = 0; i < height; i++) {
      const uint8_t *ptr_buff = buff + i * buffWidth;
      uint8_t *ptr_img = img + i * width;

      for (size_t j = 0; j < width; j++) {
        uint8_t max_value = 0;
        for (int k = 0; k < 5; k++) {
          max_value = (std::max)(max_value, *(ptr_buff + j + offset[k]));
        }

        *(ptr_img + j) = max_value;
      }
    }
  }
  else {
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

      for (size_t j = 0; j < width; j++) {
        uint8_t max_value = 0;
        for (int k = 0; k < 9; k++) {
          max_value = (std::max)(max_value, *(ptr_buff + j + offset[k]));
        }

        *(ptr_img + j) = max_value;
      }
    }
  }
}

double SimdVectorSum(const double *vec, size_t size)
{
  double sum = 0.0;
  for (size_t i = 0; i < size; i++) {
    sum += vec[i];
  }
  return  sum;
}

double SimdVectorSumSquare(const double *vec, size_t size)
{
  double sum_square = 0.0;
  for (size_t i = 0; i < size; i++) {
    sum_square += vec[i] * vec[i];
  }
  return sum_square;
}

double SimdVectorStdev(const double *vec, size_t size, bool useBesselCorrection)
{
  double mean_value = SimdVectorSum(vec, size) / size;
  double sum_squared_diff = 0.0;
  for (size_t i = 0; i < size; i++) {
    sum_squared_diff += (vec[i] - mean_value) * (vec[i] - mean_value);
  }

  double divisor = (double)size;
  if (useBesselCorrection) {
    divisor = divisor - 1;
  }

  return std::sqrt(sum_squared_diff / divisor);
}

void SimdVectorHadamard(const double *src1, const double *src2, size_t size, double *dst)
{
  for (size_t i = 0; i < size; i++) {
    dst[i] = src1[i] * src2[i];
  }
}

void SimdMatMulTwist(const double *mat, size_t rows, const double *twist, double *dst)
{
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < 6; j++) {
      double s = 0;
      for (size_t k = 0; k < 6; k++) {
        s += mat[i*6 + k] * twist[k*6 + j];
      }
      dst[i*6 + j] = s;
    }
  }
}

void SimdMatTranspose(const double *mat, size_t rows, size_t cols, double *dst)
{
  if (rows <= 16 || cols <= 16) {
    for (size_t i = 0; i < rows; i++) {
      for (size_t j = 0; j < cols; j++) {
        dst[j*cols + i] = mat[i*cols + j];
      }
    }
  }
  else {
   // https://stackoverflow.com/a/21548079
    const size_t tileSize = 32;
    for (size_t i = 0; i < rows; i += tileSize) {
      for (size_t j = 0; j < cols; j++) {
        for (size_t b = 0; b < tileSize && i + b < rows; b++) {
          dst[j*rows + i+b] = mat[(i+b)*cols + j];
        }
      }
    }
  }
}

void SimdImageDifference(const unsigned char *img1, const unsigned char *img2, size_t size, unsigned char *imgDiff)
{
  for (size_t i = 0; i < size; i++) {
    int diff = img1[i] - img2[i] + 128;
    imgDiff[i] = static_cast<unsigned char>(std::max<int>(std::min<int>(diff, 255), 0));
  }
}

void SimdNormalizedCorrelation(const double *img1, double mean1, const double *img2, double mean2, size_t size,
                               double &a2, double &b2, double &ab)
{
  for (size_t cpt = 0; cpt < size; cpt++) {
    ab += (img1[cpt] - mean1) * (img2[cpt] - mean2);
    a2 += (img1[cpt] - mean1) * (img1[cpt] - mean1);
    b2 += (img2[cpt] - mean2) * (img2[cpt] - mean2);
  }
}

void SimdNormalizedCorrelation2(const double *img1, size_t width1, const double *img2,
                                size_t width2, size_t height2, size_t i0, size_t j0, double &ab)
{
  for (size_t i = 0; i < height2; i++) {
    for (size_t j = 0; j < width2; j++) {
      ab += img1[(i0 + i)*width1 + j0 + j] * img2[i*width2 + j];
    }
  }
}

static float lerp(float A, float B, float t)
{
  return A * (1.0f - t) + B * t;
}

void SimdRemap(const unsigned char *src, size_t channels, size_t width, size_t height, size_t offset,
               const int *mapU, const int *mapV, const float *mapDu, const float *mapDv, unsigned char *dst)
{
  for (size_t j = 0; j < width; j++) {
    int u_round = mapU[offset + j];
    int v_round = mapV[offset + j];

    float du = mapDu[offset + j];
    float dv = mapDv[offset + j];

    if (0 <= u_round && 0 <= v_round && u_round < static_cast<int>(width) - 1
        && v_round < static_cast<int>(height) - 1) {
      for (size_t c = 0; c < channels; c++) {
          // process interpolation
        float col0 = lerp(src[(v_round*width + u_round)*channels + c], src[(v_round*width + u_round + 1)*channels + c], du);
        float col1 = lerp(src[((v_round + 1)*width + u_round)*channels + c], src[((v_round + 1)*width + u_round + 1)*channels + c], du);
        float value = lerp(col0, col1, dv);

        dst[(offset + j)*channels + c] = static_cast<unsigned char>(value);
      }
    }
    else {
      for (size_t c = 0; c < channels; c++) {
        dst[(offset + j)*channels + c] = 0;
      }
    }
  }
}

void SimdComputeJtR(const double *J, size_t rows, const double *R, double *dst)
{
  for (size_t i = 0; i < 6; i++) {
    double ssum = 0;
    for (size_t j = 0; j < rows; j++) {
      ssum += J[j*6 + i] * R[j];
    }
    dst[i] = ssum;
  }
}
}
}
