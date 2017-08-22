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

apriltag_family_t *tag25h9_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag25h9");
   tf->black_border = 1;
   tf->d = 5;
   tf->h = 9;
   tf->ncodes = 35;
   tf->codes = calloc(35, sizeof(uint64_t));
   tf->codes[0] = 0x000000000155cbf1UL;
   tf->codes[1] = 0x0000000001e4d1b6UL;
   tf->codes[2] = 0x00000000017b0b68UL;
   tf->codes[3] = 0x0000000001eac9cdUL;
   tf->codes[4] = 0x00000000012e14ceUL;
   tf->codes[5] = 0x00000000003548bbUL;
   tf->codes[6] = 0x00000000007757e6UL;
   tf->codes[7] = 0x0000000001065dabUL;
   tf->codes[8] = 0x0000000001baa2e7UL;
   tf->codes[9] = 0x0000000000dea688UL;
   tf->codes[10] = 0x000000000081d927UL;
   tf->codes[11] = 0x000000000051b241UL;
   tf->codes[12] = 0x0000000000dbc8aeUL;
   tf->codes[13] = 0x0000000001e50e19UL;
   tf->codes[14] = 0x00000000015819d2UL;
   tf->codes[15] = 0x00000000016d8282UL;
   tf->codes[16] = 0x000000000163e035UL;
   tf->codes[17] = 0x00000000009d9b81UL;
   tf->codes[18] = 0x000000000173eec4UL;
   tf->codes[19] = 0x0000000000ae3a09UL;
   tf->codes[20] = 0x00000000005f7c51UL;
   tf->codes[21] = 0x0000000001a137fcUL;
   tf->codes[22] = 0x0000000000dc9562UL;
   tf->codes[23] = 0x0000000001802e45UL;
   tf->codes[24] = 0x0000000001c3542cUL;
   tf->codes[25] = 0x0000000000870fa4UL;
   tf->codes[26] = 0x0000000000914709UL;
   tf->codes[27] = 0x00000000016684f0UL;
   tf->codes[28] = 0x0000000000c8f2a5UL;
   tf->codes[29] = 0x0000000000833ebbUL;
   tf->codes[30] = 0x000000000059717fUL;
   tf->codes[31] = 0x00000000013cd050UL;
   tf->codes[32] = 0x0000000000fa0ad1UL;
   tf->codes[33] = 0x0000000001b763b0UL;
   tf->codes[34] = 0x0000000000b991ceUL;
   return tf;
}

void tag25h9_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
