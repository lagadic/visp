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
        void ImageErosion(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
        {
            const size_t buffWidth = width + 2;
            if (connexityType == SimdImageConnexity4) {
                size_t offset[5] = {1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1};

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

        void ImageDilatation(uint8_t * img, const uint8_t * buff, size_t width, size_t height, SimdImageConnexityType connexityType)
        {
            const size_t buffWidth = width + 2;
            if (connexityType == SimdImageConnexity4) {
                size_t offset[5] = {1, buffWidth, buffWidth + 1, buffWidth + 2, buffWidth * 2 + 1};

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
    }
}
