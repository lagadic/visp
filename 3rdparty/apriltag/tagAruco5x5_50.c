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

#include <stdlib.h>
#include "tagAruco5x5_50.h"

static uint64_t codedata[50] = {
    0x000000000152c6e3UL,
    0x0000000000158da8UL,
    0x0000000001b967e8UL,
    0x00000000010bcdc5UL,
    0x0000000001bb840fUL,
    0x0000000001d4a600UL,
    0x0000000000cfc7b5UL,
    0x0000000000e98b01UL,
    0x000000000110c85eUL,
    0x000000000125e4a7UL,
    0x000000000137283aUL,
    0x0000000001aa21b1UL,
    0x0000000001f82066UL,
    0x00000000005c8d1fUL,
    0x0000000001f7a29bUL,
    0x000000000046cf76UL,
    0x00000000009f4366UL,
    0x0000000000be8f2aUL,
    0x0000000000fd2594UL,
    0x000000000119a385UL,
    0x000000000132e579UL,
    0x0000000001470112UL,
    0x000000000169e088UL,
    0x0000000000ac0fe9UL,
    0x0000000001968819UL,
    0x0000000001b2e945UL,
    0x0000000001cb6ab0UL,
    0x0000000000280d30UL,
    0x00000000002fc969UL,
    0x0000000000308a62UL,
    0x000000000039cf27UL,
    0x00000000003ea940UL,
    0x0000000000420f91UL,
    0x00000000004a23aaUL,
    0x0000000000588af4UL,
    0x00000000006dabbeUL,
    0x000000000086e6a8UL,
    0x00000000008a0015UL,
    0x00000000008f264eUL,
    0x000000000099a943UL,
    0x0000000000b0e40aUL,
    0x0000000000b16ee6UL,
    0x0000000000c4addcUL,
    0x0000000000c863c4UL,
    0x0000000000cda07bUL,
    0x0000000000e0cb7aUL,
    0x0000000000e26ccfUL,
    0x0000000000e46c18UL,
    0x0000000000f6c405UL,
    0x0000000000ff0de7UL,
};
apriltag_family_t *tagAruco5x5_50_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagAruco5x5_50");
   tf->h = 7;
   tf->ncodes = 50;
   tf->codes = codedata;
   tf->nbits = 25;
   tf->bit_x = calloc(25, sizeof(uint32_t));
   tf->bit_y = calloc(25, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 4;
   tf->bit_y[3] = 1;
   tf->bit_x[4] = 2;
   tf->bit_y[4] = 2;
   tf->bit_x[5] = 3;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 5;
   tf->bit_y[6] = 1;
   tf->bit_x[7] = 5;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 5;
   tf->bit_y[8] = 3;
   tf->bit_x[9] = 5;
   tf->bit_y[9] = 4;
   tf->bit_x[10] = 4;
   tf->bit_y[10] = 2;
   tf->bit_x[11] = 4;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 5;
   tf->bit_y[12] = 5;
   tf->bit_x[13] = 4;
   tf->bit_y[13] = 5;
   tf->bit_x[14] = 3;
   tf->bit_y[14] = 5;
   tf->bit_x[15] = 2;
   tf->bit_y[15] = 5;
   tf->bit_x[16] = 4;
   tf->bit_y[16] = 4;
   tf->bit_x[17] = 3;
   tf->bit_y[17] = 4;
   tf->bit_x[18] = 1;
   tf->bit_y[18] = 5;
   tf->bit_x[19] = 1;
   tf->bit_y[19] = 4;
   tf->bit_x[20] = 1;
   tf->bit_y[20] = 3;
   tf->bit_x[21] = 1;
   tf->bit_y[21] = 2;
   tf->bit_x[22] = 2;
   tf->bit_y[22] = 4;
   tf->bit_x[23] = 2;
   tf->bit_y[23] = 3;
   tf->bit_x[24] = 3;
   tf->bit_y[24] = 3;
   tf->width_at_border = 7;
   tf->total_width = 9;
   tf->reversed_border = false;
   return tf;
}

void tagAruco5x5_50_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
