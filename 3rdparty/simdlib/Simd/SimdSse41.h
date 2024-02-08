/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2022 Yermalayeu Ihar.
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
#ifndef __SimdSse41_h__
#define __SimdSse41_h__

#include "Simd/SimdDefs.h"

namespace Simd
{
#ifdef SIMD_SSE41_ENABLE
    namespace Sse41
    {
        void BgraToBgr(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* bgr, size_t bgrStride);

        void BgraToGray(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* gray, size_t grayStride);

        void BgraToRgb(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgb, size_t rgbStride);

        void BgraToRgba(const uint8_t* bgra, size_t width, size_t height, size_t bgraStride, uint8_t* rgba, size_t rgbaStride);

        void Bgr48pToBgra32(const uint8_t* blue, size_t blueStride, size_t width, size_t height,
            const uint8_t* green, size_t greenStride, const uint8_t* red, size_t redStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha);

        void BgrToBgra(const uint8_t* bgr, size_t width, size_t height, size_t bgrStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha);

        void BgrToGray(const uint8_t* bgr, size_t width, size_t height, size_t bgrStride, uint8_t* gray, size_t grayStride);

        void BgrToRgb(const uint8_t* bgr, size_t width, size_t height, size_t bgrStride, uint8_t* rgb, size_t rgbStride);

        void DeinterleaveBgr(const uint8_t* bgr, size_t bgrStride, size_t width, size_t height, uint8_t* b, size_t bStride, uint8_t* g, size_t gStride, uint8_t* r, size_t rStride);

        void DeinterleaveBgra(const uint8_t* bgra, size_t bgraStride, size_t width, size_t height, uint8_t* b, size_t bStride, uint8_t* g, size_t gStride, uint8_t* r, size_t rStride, uint8_t* a, size_t aStride);

        void GaussianBlur3x3(const uint8_t* src, size_t srcStride, size_t width, size_t height, size_t channelCount, uint8_t* dst, size_t dstStride);

        void GrayToBgr(const uint8_t* gray, size_t width, size_t height, size_t grayStride, uint8_t* bgr, size_t bgrStride);

        void GrayToBgra(const uint8_t* gray, size_t width, size_t height, size_t grayStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha);

        void InterleaveBgr(const uint8_t* b, size_t bStride, const uint8_t* g, size_t gStride, const uint8_t* r, size_t rStride, size_t width, size_t height, uint8_t* bgr, size_t bgrStride);

        void InterleaveBgra(const uint8_t* b, size_t bStride, const uint8_t* g, size_t gStride, const uint8_t* r, size_t rStride, const uint8_t* a, size_t aStride, size_t width, size_t height, uint8_t* bgra, size_t bgraStride);

        void OperationBinary8u(const uint8_t* a, size_t aStride, const uint8_t* b, size_t bStride,
            size_t width, size_t height, size_t channelCount, uint8_t* dst, size_t dstStride, SimdOperationBinary8uType type);

        void RgbaToGray(const uint8_t* rgba, size_t width, size_t height, size_t rgbaStride, uint8_t* gray, size_t grayStride);

        void ReduceColor2x2(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

        void ReduceGray2x2(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        void ReduceGray3x3(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

        void ReduceGray4x4(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        void ReduceGray5x5(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride, int compensation);

        void ResizeBilinear(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride, size_t channelCount);

        void RgbToBgra(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* bgra, size_t bgraStride, uint8_t alpha);

        void RgbToGray(const uint8_t* rgb, size_t width, size_t height, size_t rgbStride, uint8_t* gray, size_t grayStride);

        void StretchGray2x2(const uint8_t* src, size_t srcWidth, size_t srcHeight, size_t srcStride,
            uint8_t* dst, size_t dstWidth, size_t dstHeight, size_t dstStride);

        // ViSP custom SIMD code
        void ImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

        void ImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType);

        double SimdVectorSum(const double * vec, size_t size);

        double SimdVectorSumSquare(const double * vec, size_t size);

        double SimdVectorStdev(const double * vec, size_t size, bool useBesselCorrection);

        void SimdVectorHadamard(const double * src1, const double * src2, size_t size, double * dst);

        void SimdMatMulTwist(const double * mat, size_t rows, const double * twist, double * dst);

        void SimdNormalizedCorrelation(const double * img1, double mean1, const double * img2, double mean2, size_t size,
                                       double& a2, double& b2, double& ab);

        void SimdNormalizedCorrelation2(const double * img1, size_t width1, const double * img2,
                                        size_t width2, size_t height2, size_t i0, size_t j0, double& ab);

        void SimdRemap(const unsigned char * src, size_t channels, size_t width, size_t height, size_t offset,
                       const int * mapU, const int * mapV, const float * mapDu, const float * mapDv, unsigned char * dst);

        void SimdComputeJtR(const double * J, size_t rows, const double * R, double * dst);

        void SimdImageDifference(const unsigned char * img1, const unsigned char * img2, size_t size, unsigned char * imgDiff);
    }
#endif// SIMD_SSE41_ENABLE
}
#endif//__SimdSse41_h__
