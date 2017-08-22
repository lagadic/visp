/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _IMAGE_U8X3_H
#define _IMAGE_U8X3_H

#include <stdint.h>
#include "common/image_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/////////////////////////////////////
// IMPORTANT NOTE ON BYTE ORDER
//
// Format conversion routines will (unless otherwise specified) assume
// R, G, B, ordering of bytes. This is consistent with GTK, PNM, etc.
//
/////////////////////////////////////

// Create or load an image. returns NULL on failure
image_u8x3_t *image_u8x3_create(unsigned int width, unsigned int height);
image_u8x3_t *image_u8x3_create_alignment(unsigned int width, unsigned int height, unsigned int alignment);
image_u8x3_t *image_u8x3_create_from_pnm(const char *path);

image_u8x3_t *image_u8x3_copy(const image_u8x3_t *in);

void image_u8x3_gaussian_blur(image_u8x3_t *im, double sigma, int ksz);

void image_u8x3_destroy(image_u8x3_t *im);

int image_u8x3_write_pnm(const image_u8x3_t *im, const char *path);

// only width 1 supported
void image_u8x3_draw_line(image_u8x3_t *im, float x0, float y0, float x1, float y1, uint8_t rgb[3], int width);

#ifdef __cplusplus
}
#endif

#endif
