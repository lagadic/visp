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
#include "tagAruco5x5_250.h"

static uint64_t codedata[250] = {
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
    0x00000000010ce599UL,
    0x00000000010e8379UL,
    0x00000000011c2a0bUL,
    0x00000000011f0501UL,
    0x00000000013a6b3fUL,
    0x00000000014626bcUL,
    0x000000000147c1ecUL,
    0x0000000001492fe6UL,
    0x0000000001588c29UL,
    0x00000000015fa835UL,
    0x00000000016ee1f5UL,
    0x000000000172a654UL,
    0x0000000001762cafUL,
    0x0000000001802ca8UL,
    0x00000000018547f4UL,
    0x0000000001922ab6UL,
    0x000000000193c68cUL,
    0x0000000001afa294UL,
    0x0000000001afa94fUL,
    0x0000000001e3c859UL,
    0x0000000001f9c216UL,
    0x0000000000f54a13UL,
    0x0000000001a46961UL,
    0x000000000007a538UL,
    0x00000000002a026bUL,
    0x000000000086209eUL,
    0x0000000000a10668UL,
    0x0000000001424346UL,
    0x0000000001904707UL,
    0x00000000019e8615UL,
    0x0000000001ee88bdUL,
    0x000000000000c4eaUL,
    0x000000000007e18bUL,
    0x000000000009c759UL,
    0x00000000000b8f34UL,
    0x00000000000c42a2UL,
    0x00000000000e8cb1UL,
    0x000000000010293cUL,
    0x00000000001825b7UL,
    0x00000000001a662aUL,
    0x00000000001d656dUL,
    0x000000000026a35aUL,
    0x00000000002ca827UL,
    0x0000000000310d2bUL,
    0x0000000000334a4eUL,
    0x00000000003b268dUL,
    0x00000000003bc914UL,
    0x00000000003fa5dfUL,
    0x00000000004da1a1UL,
    0x000000000053a7e0UL,
    0x00000000005520a6UL,
    0x00000000005743b3UL,
    0x00000000005a4212UL,
    0x00000000005ae3edUL,
    0x00000000005b0355UL,
    0x00000000005f0f7bUL,
    0x000000000060428eUL,
    0x00000000006086a5UL,
    0x000000000063a00bUL,
    0x00000000006f01c6UL,
    0x0000000000708980UL,
    0x000000000073ea78UL,
    0x0000000000814209UL,
    0x000000000082e7ddUL,
    0x000000000089a5abUL,
    0x00000000008b8a0dUL,
    0x000000000091876eUL,
    0x0000000000954868UL,
    0x00000000009b885eUL,
    0x00000000009dcbc8UL,
    0x0000000000a52a7eUL,
    0x0000000000a6efb3UL,
    0x0000000000a9e1f6UL,
    0x0000000000ad8dd7UL,
    0x0000000000b4c867UL,
    0x0000000000b881a4UL,
    0x0000000000ba6f81UL,
    0x0000000000bb2811UL,
    0x0000000000bc2f74UL,
    0x0000000000be62eeUL,
    0x0000000000cb2498UL,
    0x0000000000cb6f6cUL,
    0x0000000000d1a8baUL,
    0x0000000000d82a8dUL,
    0x0000000000e1eac2UL,
    0x0000000000e7003cUL,
    0x0000000000ed4975UL,
    0x0000000000f026b3UL,
    0x0000000000fae534UL,
    0x000000000101225cUL,
    0x00000000010e4832UL,
    0x00000000011169ddUL,
    0x00000000011589f4UL,
    0x000000000120a9baUL,
    0x000000000126ac68UL,
    0x000000000129232fUL,
    0x000000000130e02dUL,
    0x000000000139af1cUL,
    0x00000000013d2fb2UL,
    0x00000000013f6184UL,
    0x00000000014147c3UL,
    0x000000000143060eUL,
    0x0000000001444a16UL,
    0x00000000014d44abUL,
    0x00000000015ec0daUL,
    0x0000000001612d91UL,
    0x000000000161a061UL,
    0x000000000167c51dUL,
    0x00000000017662c8UL,
    0x000000000179c5c4UL,
    0x00000000017a8343UL,
    0x00000000017e27ecUL,
    0x00000000017fad0aUL,
    0x000000000181819fUL,
    0x0000000001864c3dUL,
    0x000000000189003aUL,
    0x00000000018d0d98UL,
    0x000000000190044dUL,
    0x000000000190ef4aUL,
    0x0000000001976215UL,
    0x00000000019de438UL,
    0x00000000019e05b4UL,
    0x0000000001a109c4UL,
    0x0000000001a566baUL,
    0x0000000001ad4225UL,
    0x0000000001afe505UL,
    0x0000000001b040a3UL,
    0x0000000001b2ce22UL,
    0x0000000001b707cbUL,
    0x0000000001b724a0UL,
    0x0000000001bca013UL,
    0x0000000001bd68e5UL,
    0x0000000001c40d87UL,
    0x0000000001c82fcbUL,
    0x0000000001c9e151UL,
    0x0000000001cccdfbUL,
    0x0000000001cec510UL,
    0x0000000001de2263UL,
    0x0000000001deca7fUL,
    0x0000000001e40822UL,
    0x0000000001e5aca4UL,
    0x0000000001f10108UL,
    0x0000000001f18eebUL,
    0x0000000001f46bfeUL,
    0x0000000001f58172UL,
    0x0000000001f70885UL,
    0x0000000001f9a1b9UL,
    0x0000000001fbc3ddUL,
    0x0000000001fc820fUL,
    0x0000000001fdc6f0UL,
    0x0000000000c44f53UL,
    0x00000000012d2c61UL,
    0x00000000012e4703UL,
    0x0000000001c3266bUL,
    0x00000000002c07caUL,
    0x000000000056c120UL,
    0x0000000000620da8UL,
    0x00000000007ac388UL,
    0x00000000007c2cfeUL,
    0x0000000000b042d8UL,
    0x0000000000b8895bUL,
    0x0000000000d0c14fUL,
    0x0000000000f8ee30UL,
    0x0000000000fcabddUL,
    0x00000000011f29e4UL,
    0x00000000014bce2cUL,
    0x00000000014c4df5UL,
    0x0000000001688f2fUL,
    0x000000000173c847UL,
    0x000000000182c3acUL,
    0x000000000194c673UL,
    0x0000000001a72d35UL,
    0x0000000001bc666dUL,
    0x0000000001cb2e5cUL,
    0x0000000001cfa468UL,
    0x0000000001e32634UL,
    0x00000000000007fdUL,
    0x0000000000010eb0UL,
    0x000000000001cdd4UL,
    0x00000000000246cdUL,
    0x0000000000034986UL,
    0x0000000000050e49UL,
    0x00000000000546a5UL,
    0x000000000005ecc8UL,
    0x000000000008cbecUL,
    0x00000000000a8672UL,
    0x00000000000bca00UL,
    0x00000000000be9b0UL,
    0x00000000000c8f75UL,
    0x00000000000c8c4aUL,
    0x00000000000ca068UL,
    0x00000000000e6f79UL,
    0x00000000000fa6dcUL,
    0x00000000000fc792UL,
    0x0000000000124406UL,
    0x00000000001308d4UL,
    0x000000000013497eUL,
    0x0000000000136d39UL,
    0x000000000015a41bUL,
    0x000000000015cac7UL,
};
apriltag_family_t *tagAruco5x5_250_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagAruco5x5_250");
   tf->h = 6;
   tf->ncodes = 250;
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

void tagAruco5x5_250_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
