/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

An unlimited license is granted to use, adapt, modify, or embed the 2D
barcodes into any medium.

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

#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t *tag16h5_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag16h5");
   tf->black_border = 1;
   tf->d = 4;
   tf->h = 5;
   tf->ncodes = 30;
   tf->codes = calloc(30, sizeof(uint64_t));
   tf->codes[0] = 0x000000000000231bUL;
   tf->codes[1] = 0x0000000000002ea5UL;
   tf->codes[2] = 0x000000000000346aUL;
   tf->codes[3] = 0x00000000000045b9UL;
   tf->codes[4] = 0x00000000000079a6UL;
   tf->codes[5] = 0x0000000000007f6bUL;
   tf->codes[6] = 0x000000000000b358UL;
   tf->codes[7] = 0x000000000000e745UL;
   tf->codes[8] = 0x000000000000fe59UL;
   tf->codes[9] = 0x000000000000156dUL;
   tf->codes[10] = 0x000000000000380bUL;
   tf->codes[11] = 0x000000000000f0abUL;
   tf->codes[12] = 0x0000000000000d84UL;
   tf->codes[13] = 0x0000000000004736UL;
   tf->codes[14] = 0x0000000000008c72UL;
   tf->codes[15] = 0x000000000000af10UL;
   tf->codes[16] = 0x000000000000093cUL;
   tf->codes[17] = 0x00000000000093b4UL;
   tf->codes[18] = 0x000000000000a503UL;
   tf->codes[19] = 0x000000000000468fUL;
   tf->codes[20] = 0x000000000000e137UL;
   tf->codes[21] = 0x0000000000005795UL;
   tf->codes[22] = 0x000000000000df42UL;
   tf->codes[23] = 0x0000000000001c1dUL;
   tf->codes[24] = 0x000000000000e9dcUL;
   tf->codes[25] = 0x00000000000073adUL;
   tf->codes[26] = 0x000000000000ad5fUL;
   tf->codes[27] = 0x000000000000d530UL;
   tf->codes[28] = 0x00000000000007caUL;
   tf->codes[29] = 0x000000000000af2eUL;
   return tf;
}

void tag16h5_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
