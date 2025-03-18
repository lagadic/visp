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
#include "tagAruco6x6_250.h"

static uint64_t codedata[250] = {
    0x0000000187341537UL,
    0x00000000bebc30f0UL,
    0x00000001674816c5UL,
    0x0000000cca04346cUL,
    0x0000000d0d1c269eUL,
    0x0000000db17005e8UL,
    0x0000000410703b74UL,
    0x00000008a0ac3603UL,
    0x00000003136c312eUL,
    0x000000038c242fe6UL,
    0x000000047fdc132aUL,
    0x00000004b3c82425UL,
    0x0000000741441c8fUL,
    0x00000008371808ffUL,
    0x00000008def03d41UL,
    0x0000000a22f032bdUL,
    0x00000000f26c1aa7UL,
    0x0000000154941a67UL,
    0x0000000308c01164UL,
    0x0000000489dc37c3UL,
    0x000000053e8439beUL,
    0x0000000624480c7cUL,
    0x0000000735b81c39UL,
    0x000000099bd4319dUL,
    0x0000000af8140320UL,
    0x0000000c17a4301bUL,
    0x0000000c7a500881UL,
    0x0000000e54000939UL,
    0x0000000e897c00b3UL,
    0x0000000ed8bc3f4cUL,
    0x0000000f8ba01752UL,
    0x000000001f9c3f7dUL,
    0x0000000055dc3604UL,
    0x00000000be24174aUL,
    0x0000000142f819f0UL,
    0x0000000164f825acUL,
    0x00000003910c36ddUL,
    0x00000004c75c1458UL,
    0x00000005401c0bfeUL,
    0x00000005ab1c25edUL,
    0x00000006051404e1UL,
    0x000000061b0c30c5UL,
    0x00000006520c3ba0UL,
    0x00000006fb6429ffUL,
    0x00000007207c06a6UL,
    0x0000000765882e4bUL,
    0x00000007a2c43517UL,
    0x00000008050c1374UL,
    0x00000008812423baUL,
    0x0000000979242cf5UL,
    0x00000009a054306bUL,
    0x00000009bec81c73UL,
    0x0000000a45343d68UL,
    0x0000000b6e54284dUL,
    0x0000000b77700770UL,
    0x0000000b85682404UL,
    0x0000000c10d80f0aUL,
    0x0000000c3c442640UL,
    0x0000000c64ac18adUL,
    0x0000000c9f3c0fd0UL,
    0x0000000cc4582762UL,
    0x0000000cd9c4020dUL,
    0x0000000cee302dbaUL,
    0x0000000e56e423a6UL,
    0x0000000efcd01392UL,
    0x0000000f5ff82d7eUL,
    0x00000002b4e82647UL,
    0x00000002f398337fUL,
    0x000000037dd43e97UL,
    0x0000000a6920068cUL,
    0x0000000ae9903d80UL,
    0x0000000d8b840ca1UL,
    0x0000000052801cf9UL,
    0x0000000074c0391bUL,
    0x00000000fd682b50UL,
    0x000000011448026cUL,
    0x0000000121ac3fafUL,
    0x0000000128902b1cUL,
    0x0000000134e400d4UL,
    0x000000019ba03529UL,
    0x00000001b0382a30UL,
    0x00000001ca1c1341UL,
    0x00000001d8b426d6UL,
    0x0000000276900eaeUL,
    0x00000002a1c42de8UL,
    0x000000035200171eUL,
    0x0000000371443b41UL,
    0x00000003784401bdUL,
    0x00000003bd140af2UL,
    0x0000000414042597UL,
    0x000000042f600de4UL,
    0x0000000463481d62UL,
    0x00000004b294110cUL,
    0x0000000536f00d8fUL,
    0x000000055e1022b5UL,
    0x000000056d840f84UL,
    0x000000056f2c0a56UL,
    0x0000000586681fceUL,
    0x0000000595502f6dUL,
    0x000000059db8027bUL,
    0x00000005ccec355dUL,
    0x00000005d228255bUL,
    0x000000064c9c1d62UL,
    0x000000067ecc1a0cUL,
    0x00000006a2001b7aUL,
    0x00000006f1dc0046UL,
    0x00000006fb401109UL,
    0x00000007177c19e8UL,
    0x00000007388c15faUL,
    0x000000076734221dUL,
    0x0000000784882632UL,
    0x000000079164047cUL,
    0x00000007ab300c0dUL,
    0x00000007f0943e0aUL,
    0x000000081de41cbfUL,
    0x000000083b683d5eUL,
    0x000000085c88226bUL,
    0x0000000982942b90UL,
    0x00000009972430c8UL,
    0x00000009f6543e8cUL,
    0x0000000a0a0825beUL,
    0x0000000a96200395UL,
    0x0000000ac4c418c0UL,
    0x0000000b02ec1b87UL,
    0x0000000b1d503035UL,
    0x0000000b4ea4314eUL,
    0x0000000badb827b8UL,
    0x0000000bd0e01ef5UL,
    0x0000000c1ff81654UL,
    0x0000000c79a41f83UL,
    0x0000000cd31c3a5bUL,
    0x0000000ce1a80d1cUL,
    0x0000000d00cc33a9UL,
    0x0000000d106828efUL,
    0x0000000d72642958UL,
    0x0000000e3cac1955UL,
    0x0000000e5a4c06d5UL,
    0x0000000e88182e83UL,
    0x0000000e8d14380aUL,
    0x0000000eba743c10UL,
    0x0000000f0eac1af8UL,
    0x0000000f48542d17UL,
    0x0000000f9c1c39f5UL,
    0x0000000ae1e403d7UL,
    0x0000000c330c1948UL,
    0x0000000179c815f7UL,
    0x00000004a594265dUL,
    0x0000000535280f54UL,
    0x0000000692780819UL,
    0x00000007203024b5UL,
    0x00000008530c1caaUL,
    0x0000000cd5d828baUL,
    0x0000000d2fe403f0UL,
    0x0000000ebe8c2584UL,
    0x0000000f45b40644UL,
    0x0000000fcad43beaUL,
    0x0000000001301f4eUL,
    0x00000000018012a1UL,
    0x0000000007380daaUL,
    0x000000001f242424UL,
    0x000000001f3022f8UL,
    0x00000000283813a4UL,
    0x00000000360434cdUL,
    0x0000000046503041UL,
    0x0000000050100044UL,
    0x00000000625417afUL,
    0x000000008388278cUL,
    0x0000000098fc0fa4UL,
    0x00000000b430047dUL,
    0x00000000d1283ef6UL,
    0x00000000f6d010f7UL,
    0x00000000fd981f31UL,
    0x0000000104b83a8fUL,
    0x000000010c443942UL,
    0x00000001444428acUL,
    0x0000000153980e36UL,
    0x00000001581c3d87UL,
    0x00000001a6b42561UL,
    0x00000001b6cc0f2aUL,
    0x00000001cd600677UL,
    0x00000001d968210aUL,
    0x00000001db14128eUL,
    0x00000001e5300efcUL,
    0x00000001e9483698UL,
    0x00000001e9f03172UL,
    0x0000000220781e56UL,
    0x000000022a340936UL,
    0x0000000240e00eb6UL,
    0x000000025ed00569UL,
    0x0000000264bc36d3UL,
    0x000000027628030bUL,
    0x0000000299e0328eUL,
    0x000000029d481345UL,
    0x00000002a3643ef5UL,
    0x00000002d99009caUL,
    0x00000002ec20204bUL,
    0x00000002fa2419f0UL,
    0x000000032518373eUL,
    0x000000032dcc1e4eUL,
    0x0000000337dc00e3UL,
    0x0000000367dc375cUL,
    0x0000000388782b11UL,
    0x000000038e94025dUL,
    0x00000003a78811a8UL,
    0x00000003d2d02b25UL,
    0x00000003d378158fUL,
    0x00000003d4342f7fUL,
    0x00000003dd340828UL,
    0x00000003e8ac3268UL,
    0x000000040d482611UL,
    0x000000042c183155UL,
    0x0000000431b41b39UL,
    0x00000004349c2fe4UL,
    0x0000000439541b06UL,
    0x000000043e5012dcUL,
    0x0000000443cc093cUL,
    0x0000000449783ac8UL,
    0x000000045e941bd0UL,
    0x000000048f9004feUL,
    0x000000049d5009b0UL,
    0x000000049e5c0173UL,
    0x00000004a04c08f6UL,
    0x00000004c2142f86UL,
    0x00000004d59c052fUL,
    0x00000004e9a414daUL,
    0x00000004f298188aUL,
    0x00000005046c0fb9UL,
    0x000000055bf03fbeUL,
    0x0000000576fc37c2UL,
    0x00000005775c044bUL,
    0x000000058adc3861UL,
    0x00000005ae78343eUL,
    0x00000005c79039d5UL,
    0x00000005cfd826a7UL,
    0x00000005d6b40e99UL,
    0x00000005f46c030dUL,
    0x000000060afc0445UL,
    0x0000000616481dadUL,
    0x0000000671d027b5UL,
    0x000000067ac033e5UL,
    0x000000068e6835f0UL,
    0x0000000694e42f90UL,
    0x00000006a09414f9UL,
    0x00000006aa4c315aUL,
    0x00000006cb980090UL,
    0x00000006d1a80531UL,
    0x00000006e7d0291aUL,
    0x00000006ed3806c7UL,
    0x00000006fd782039UL,
    0x00000007026405e7UL,
};
apriltag_family_t *tagAruco6x6_250_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagAruco6x6_250");
   tf->h = 11;
   tf->ncodes = 250;
   tf->codes = codedata;
   tf->nbits = 36;
   tf->bit_x = calloc(36, sizeof(uint32_t));
   tf->bit_y = calloc(36, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 4;
   tf->bit_y[3] = 1;
   tf->bit_x[4] = 5;
   tf->bit_y[4] = 1;
   tf->bit_x[5] = 2;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 3;
   tf->bit_y[6] = 2;
   tf->bit_x[7] = 4;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 3;
   tf->bit_y[8] = 3;
   tf->bit_x[9] = 6;
   tf->bit_y[9] = 1;
   tf->bit_x[10] = 6;
   tf->bit_y[10] = 2;
   tf->bit_x[11] = 6;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 6;
   tf->bit_y[12] = 4;
   tf->bit_x[13] = 6;
   tf->bit_y[13] = 5;
   tf->bit_x[14] = 5;
   tf->bit_y[14] = 2;
   tf->bit_x[15] = 5;
   tf->bit_y[15] = 3;
   tf->bit_x[16] = 5;
   tf->bit_y[16] = 4;
   tf->bit_x[17] = 4;
   tf->bit_y[17] = 3;
   tf->bit_x[18] = 6;
   tf->bit_y[18] = 6;
   tf->bit_x[19] = 5;
   tf->bit_y[19] = 6;
   tf->bit_x[20] = 4;
   tf->bit_y[20] = 6;
   tf->bit_x[21] = 3;
   tf->bit_y[21] = 6;
   tf->bit_x[22] = 2;
   tf->bit_y[22] = 6;
   tf->bit_x[23] = 5;
   tf->bit_y[23] = 5;
   tf->bit_x[24] = 4;
   tf->bit_y[24] = 5;
   tf->bit_x[25] = 3;
   tf->bit_y[25] = 5;
   tf->bit_x[26] = 4;
   tf->bit_y[26] = 4;
   tf->bit_x[27] = 1;
   tf->bit_y[27] = 6;
   tf->bit_x[28] = 1;
   tf->bit_y[28] = 5;
   tf->bit_x[29] = 1;
   tf->bit_y[29] = 4;
   tf->bit_x[30] = 1;
   tf->bit_y[30] = 3;
   tf->bit_x[31] = 1;
   tf->bit_y[31] = 2;
   tf->bit_x[32] = 2;
   tf->bit_y[32] = 5;
   tf->bit_x[33] = 2;
   tf->bit_y[33] = 4;
   tf->bit_x[34] = 2;
   tf->bit_y[34] = 3;
   tf->bit_x[35] = 3;
   tf->bit_y[35] = 4;
   tf->width_at_border = 8;
   tf->total_width = 10;
   tf->reversed_border = false;
   return tf;
}

void tagAruco6x6_250_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
