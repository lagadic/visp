/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "math_util.h"

#include "image_f32.h"

#ifndef HUGE
//# define HUGE 3.40282347e+38F
#include <float.h>
#endif


static inline float sqf(float v)
{
    return v*v;
}

image_f32_t *image_f32_create_stride(int width, int height, int stride)
{
    float *buf = calloc(height * stride, sizeof(float));
    // const initializer
    image_f32_t tmp = { .width = width, .height = height, .stride = stride, .buf = buf };

    image_f32_t *fim = (image_f32_t*) calloc(1, sizeof(image_f32_t));

    memcpy(fim, &tmp, sizeof(image_f32_t));

    return fim;
}

image_f32_t *image_f32_create(int width, int height)
{
    int stride = width;
    while (stride & 7)
        stride++;
    return image_f32_create_stride(width, height, stride);
}

// scales by 1/255u
image_f32_t *image_f32_create_from_u8(const image_u8_t *im)
{
    image_f32_t *fim = image_f32_create(im->width, im->height);

    for (int y = 0; y < fim->height; y++)
        for (int x = 0; x < fim->width; x++)
            fim->buf[y*fim->stride + x] = im->buf[y*im->stride + x] / 255.0f;

    return fim;
}

void image_f32_destroy(image_f32_t *im)
{
    free(im->buf);
    free(im);
}

static void convolve(const float *x, float *y, int sz, const float *k, int ksz)
{
    assert((ksz&1)==1);

    for (int i = 0; i < ksz/2 && i < sz; i++)
        y[i] = x[i];

    for (int i = 0; i < sz - ksz; i++) {
        float acc = 0;

        for (int j = 0; j < ksz; j++)
            acc += k[j]*x[i+j];

        y[ksz/2 + i] = acc;
    }

    for (int i = sz - ksz + ksz/2; i < sz; i++)
        y[i] = x[i];
}

void image_f32_gaussian_blur(image_f32_t *im, double sigma, int ksz)
{
    assert((ksz & 1) == 1); // ksz must be odd.

    // build the kernel.
#ifdef _MSC_VER
    float *k = malloc(ksz*sizeof *k);
#else
    float k[ksz];
#endif

    // for kernel of length 5:
    // dk[0] = f(-2), dk[1] = f(-1), dk[2] = f(0), dk[3] = f(1), dk[4] = f(2)
    for (int i = 0; i < ksz; i++) {
        int x = -ksz/2 + i;
        float v = exp(-.5*sqf(x / sigma));
        k[i] = v;
    }

    // normalize
    float acc = 0;
    for (int i = 0; i < ksz; i++)
        acc += k[i];

    for (int i = 0; i < ksz; i++)
        k[i] /= acc;

#ifdef _MSC_VER
    free(k);
#endif

    for (int y = 0; y < im->height; y++) {
#ifdef _MSC_VER
        float *x = malloc(im->stride*sizeof *x);
#else
        float x[im->stride];
#endif
        memcpy(x, &im->buf[y*im->stride], im->stride * sizeof(float));
        convolve(x, &im->buf[y*im->stride], im->width, k, ksz);

#ifdef _MSC_VER
        free(x);
#endif
    }

    for (int x = 0; x < im->width; x++) {
#ifdef _MSC_VER
        float *xb = malloc(im->height*sizeof *xb);
        float *yb = malloc(im->height*sizeof *yb);
#else
        float xb[im->height];
        float yb[im->height];
#endif

        for (int y = 0; y < im->height; y++)
            xb[y] = im->buf[y*im->stride + x];

        convolve(xb, yb, im->height, k, ksz);

        for (int y = 0; y < im->height; y++)
            im->buf[y*im->stride + x] = yb[y];

#ifdef _MSC_VER
        free(xb);
        free(yb);
#endif
    }
}

// remap all values to [0, 1]
void image_f32_normalize(image_f32_t *im)
{
#ifndef HUGE
  float min = FLT_MAX, max = -FLT_MAX;
#else
    float min = HUGE, max = -HUGE;
#endif

    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            float v = im->buf[y*im->stride + x];
            if (v < min)
                min = v;
            if (v > max)
                max = v;
        }
    }

    if (min == max) {
        for (int y = 0; y < im->height; y++) {
            for (int x = 0; x < im->width; x++) {
                im->buf[y*im->stride + x] = 0.5;
            }
        }
    } else {

        for (int y = 0; y < im->height; y++) {
            for (int x = 0; x < im->width; x++) {
                float v = im->buf[y*im->stride + x];

                im->buf[y*im->stride + x] = (v - min) / (max - min);
            }
        }
    }
}

// image is assumed to be [0, 1]
int image_f32_write_pnm(const image_f32_t *im, const char *path)
{
    FILE *f = fopen(path, "wb");
    int res = 0;

    if (f == NULL) {
        res = -1;
        goto finish;
    }

    // Only outputs to grayscale
    fprintf(f, "P5\n%d %d\n255\n", im->width, im->height);

    for (int y = 0; y < im->height; y++) {
#ifdef _MSC_VER
        uint8_t *line = malloc(im->width*sizeof *line);
#else
        uint8_t line[im->width];
#endif
        for (int x = 0; x < im->width; x++) {
            float v = im->buf[y*im->stride + x];
            if (v < 0)
                v = 0;
            if (v > 1)
                v = 1;
            line[x] = v * 255.0;
        }

        if (im->width != fwrite(line, 1, im->width, f)) {
            res = -2;
            goto finish;
        }

#ifdef _MSC_VER
        free(line);
#endif
    }

finish:
    if (f != NULL)
        fclose(f);

    return res;
}
