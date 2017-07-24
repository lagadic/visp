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

#ifndef _IMAGE_U8X4_H
#define _IMAGE_U8X4_H

#include <stdint.h>
#include "common/image_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/////////////////////////////////////
// IMPORTANT NOTE ON BYTE ORDER
//
// Format conversion routines will (unless otherwise specified) assume
// R, G, B, A ordering of bytes.
//
/////////////////////////////////////

// Create or load an image. returns NULL on failure
image_u8x4_t *image_u8x4_create(unsigned int width, unsigned int height);
image_u8x4_t *image_u8x4_create_alignment(unsigned int width, unsigned int height, unsigned int alignment);
image_u8x4_t *image_u8x4_create_from_pnm(const char *path);

image_u8x4_t *image_u8x4_copy(const image_u8x4_t *in);

void image_u8x4_destroy(image_u8x4_t *im);

// Write a pnm. Return 0 on success.
// Currently supports GRAY and RGB
int image_u8x4_write_pnm(const image_u8x4_t *im, const char *path);

image_u8x4_t *image_u8x4_create_from_pam(const char *path);

    void image_u8x4_write_pam(const image_u8x4_t *im, const char *path);

#ifdef __cplusplus
}
#endif

#endif
