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

#ifndef _PAM_H
#define _PAM_H

#include <stdint.h>

enum { PAM_GRAYSCALE_ALPHA = 5000, PAM_RGB_ALPHA, PAM_RGB, PAM_GRAYSCALE };

typedef struct pam pam_t;
struct pam
{
    int type; // one of PAM_*

    int width, height; // note, stride always width.
    int depth; // bytes per pixel
    int maxval; // maximum value per channel, e.g. 255 for 8bpp

    int datalen; // in bytes
    uint8_t *data;
};

pam_t *pam_create_from_file(const char *inpath);
int pam_write_file(pam_t *pam, const char *outpath);
void pam_destroy(pam_t *pam);

pam_t *pam_copy(pam_t *pam);

// NB doesn't handle many conversions yet.
pam_t *pam_convert(pam_t *in, int type);

#endif
