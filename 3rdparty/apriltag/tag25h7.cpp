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
#include "apriltag.h"

apriltag_family_t *tag25h7_create()
{
   apriltag_family_t *tf = (apriltag_family_t *)calloc(1, sizeof(apriltag_family_t));
#ifdef WINRT
   tf->name = _strdup("tag25h7");
#else
   tf->name = strdup("tag25h7");
#endif
   tf->h = 7;
   tf->ncodes = 242;
   tf->codes = (uint64_t *)calloc(242, sizeof(uint64_t));
   tf->codes[0] = 0x000000000049563fUL;
   tf->codes[1] = 0x00000000011e8da5UL;
   tf->codes[2] = 0x0000000001b2fa85UL;
   tf->codes[3] = 0x0000000000cbd446UL;
   tf->codes[4] = 0x00000000014eabe2UL;
   tf->codes[5] = 0x0000000000b322dcUL;
   tf->codes[6] = 0x0000000001caa81dUL;
   tf->codes[7] = 0x000000000031b6aeUL;
   tf->codes[8] = 0x0000000000adc94fUL;
   tf->codes[9] = 0x000000000148bc6fUL;
   tf->codes[10] = 0x0000000001981b31UL;
   tf->codes[11] = 0x0000000000c3670dUL;
   tf->codes[12] = 0x00000000002beba7UL;
   tf->codes[13] = 0x000000000142e574UL;
   tf->codes[14] = 0x0000000001c69255UL;
   tf->codes[15] = 0x000000000178fba0UL;
   tf->codes[16] = 0x000000000091d561UL;
   tf->codes[17] = 0x0000000001440f27UL;
   tf->codes[18] = 0x00000000007923ebUL;
   tf->codes[19] = 0x0000000000f556d8UL;
   tf->codes[20] = 0x0000000000a9879dUL;
   tf->codes[21] = 0x0000000001c08d5eUL;
   tf->codes[22] = 0x00000000015ea22dUL;
   tf->codes[23] = 0x00000000007788eaUL;
   tf->codes[24] = 0x00000000015a5ab4UL;
   tf->codes[25] = 0x0000000001de2154UL;
   tf->codes[26] = 0x00000000008966a8UL;
   tf->codes[27] = 0x00000000010d1149UL;
   tf->codes[28] = 0x00000000015cb526UL;
   tf->codes[29] = 0x00000000013c43bdUL;
   tf->codes[30] = 0x0000000000d7eb44UL;
   tf->codes[31] = 0x00000000008b33a1UL;
   tf->codes[32] = 0x0000000000efb1d8UL;
   tf->codes[33] = 0x00000000013f1412UL;
   tf->codes[34] = 0x00000000006d2c2eUL;
   tf->codes[35] = 0x0000000001015b8bUL;
   tf->codes[36] = 0x00000000016a2d6cUL;
   tf->codes[37] = 0x0000000001ba8b22UL;
   tf->codes[38] = 0x0000000000e5151dUL;
   tf->codes[39] = 0x000000000196c5b0UL;
   tf->codes[40] = 0x000000000032b290UL;
   tf->codes[41] = 0x00000000012b6cdbUL;
   tf->codes[42] = 0x0000000001608aadUL;
   tf->codes[43] = 0x0000000001b0aee2UL;
   tf->codes[44] = 0x00000000015dc403UL;
   tf->codes[45] = 0x0000000001119954UL;
   tf->codes[46] = 0x0000000000a549b1UL;
   tf->codes[47] = 0x00000000005c305aUL;
   tf->codes[48] = 0x000000000172efe5UL;
   tf->codes[49] = 0x0000000001d4f931UL;
   tf->codes[50] = 0x00000000004ed51bUL;
   tf->codes[51] = 0x0000000000eb9d20UL;
   tf->codes[52] = 0x0000000001e2656bUL;
   tf->codes[53] = 0x0000000001756b5bUL;
   tf->codes[54] = 0x0000000001f668a2UL;
   tf->codes[55] = 0x0000000001aa31e7UL;
   tf->codes[56] = 0x00000000000eb3deUL;
   tf->codes[57] = 0x0000000001337161UL;
   tf->codes[58] = 0x0000000001359a20UL;
   tf->codes[59] = 0x0000000000e1cb65UL;
   tf->codes[60] = 0x0000000000f1c2acUL;
   tf->codes[61] = 0x00000000016ba423UL;
   tf->codes[62] = 0x0000000000445aa9UL;
   tf->codes[63] = 0x0000000001793ea3UL;
   tf->codes[64] = 0x00000000000e1bacUL;
   tf->codes[65] = 0x0000000000527367UL;
   tf->codes[66] = 0x0000000001814b68UL;
   tf->codes[67] = 0x00000000004ac16fUL;
   tf->codes[68] = 0x00000000017b506fUL;
   tf->codes[69] = 0x00000000012a9d50UL;
   tf->codes[70] = 0x0000000000f28b4eUL;
   tf->codes[71] = 0x0000000000cec5dcUL;
   tf->codes[72] = 0x000000000082193dUL;
   tf->codes[73] = 0x000000000047374aUL;
   tf->codes[74] = 0x0000000000ac83a8UL;
   tf->codes[75] = 0x00000000000d36cdUL;
   tf->codes[76] = 0x0000000001d9e7a2UL;
   tf->codes[77] = 0x00000000008c35b4UL;
   tf->codes[78] = 0x0000000000d6b8f8UL;
   tf->codes[79] = 0x000000000081345eUL;
   tf->codes[80] = 0x000000000166b51cUL;
   tf->codes[81] = 0x00000000004f4af8UL;
   tf->codes[82] = 0x000000000164d132UL;
   tf->codes[83] = 0x00000000017607b2UL;
   tf->codes[84] = 0x0000000001f0c5c9UL;
   tf->codes[85] = 0x000000000014e2b2UL;
   tf->codes[86] = 0x00000000003c2aaaUL;
   tf->codes[87] = 0x000000000038d7b1UL;
   tf->codes[88] = 0x00000000014f5170UL;
   tf->codes[89] = 0x0000000001758161UL;
   tf->codes[90] = 0x00000000007c3435UL;
   tf->codes[91] = 0x000000000106dad8UL;
   tf->codes[92] = 0x00000000016f18a6UL;
   tf->codes[93] = 0x00000000005bf08aUL;
   tf->codes[94] = 0x0000000000ec5d61UL;
   tf->codes[95] = 0x00000000013087d4UL;
   tf->codes[96] = 0x000000000011432fUL;
   tf->codes[97] = 0x0000000000c93465UL;
   tf->codes[98] = 0x0000000000b09c59UL;
   tf->codes[99] = 0x00000000018308d6UL;
   tf->codes[100] = 0x000000000074406bUL;
   tf->codes[101] = 0x0000000000a81c97UL;
   tf->codes[102] = 0x00000000002bfebaUL;
   tf->codes[103] = 0x00000000003b3d77UL;
   tf->codes[104] = 0x00000000006168a8UL;
   tf->codes[105] = 0x00000000007d4773UL;
   tf->codes[106] = 0x0000000001a25ddeUL;
   tf->codes[107] = 0x0000000001432a63UL;
   tf->codes[108] = 0x0000000001a9aa69UL;
   tf->codes[109] = 0x0000000001952774UL;
   tf->codes[110] = 0x00000000018cc859UL;
   tf->codes[111] = 0x00000000001e3cc1UL;
   tf->codes[112] = 0x00000000011bb2e0UL;
   tf->codes[113] = 0x0000000000ede6c4UL;
   tf->codes[114] = 0x00000000011a1146UL;
   tf->codes[115] = 0x0000000001d574e3UL;
   tf->codes[116] = 0x000000000137af80UL;
   tf->codes[117] = 0x0000000000d96bbcUL;
   tf->codes[118] = 0x00000000016eb4e4UL;
   tf->codes[119] = 0x0000000000d5a7a9UL;
   tf->codes[120] = 0x0000000000627bdaUL;
   tf->codes[121] = 0x000000000179980dUL;
   tf->codes[122] = 0x000000000112f433UL;
   tf->codes[123] = 0x00000000018a262fUL;
   tf->codes[124] = 0x0000000000d11766UL;
   tf->codes[125] = 0x000000000073ed68UL;
   tf->codes[126] = 0x0000000000e31bf9UL;
   tf->codes[127] = 0x0000000000e17df6UL;
   tf->codes[128] = 0x0000000001b108aaUL;
   tf->codes[129] = 0x00000000007c5b74UL;
   tf->codes[130] = 0x0000000000a877d5UL;
   tf->codes[131] = 0x00000000002f2f09UL;
   tf->codes[132] = 0x00000000012a4ac5UL;
   tf->codes[133] = 0x0000000001e83c51UL;
   tf->codes[134] = 0x00000000000aac42UL;
   tf->codes[135] = 0x000000000015c494UL;
   tf->codes[136] = 0x0000000001233681UL;
   tf->codes[137] = 0x0000000000adf33dUL;
   tf->codes[138] = 0x0000000001d5282dUL;
   tf->codes[139] = 0x000000000055ef7dUL;
   tf->codes[140] = 0x000000000131508cUL;
   tf->codes[141] = 0x00000000013f49a0UL;
   tf->codes[142] = 0x0000000000e15211UL;
   tf->codes[143] = 0x0000000001a912cfUL;
   tf->codes[144] = 0x00000000012663eaUL;
   tf->codes[145] = 0x0000000000a7aa05UL;
   tf->codes[146] = 0x0000000000543e90UL;
   tf->codes[147] = 0x000000000135064fUL;
   tf->codes[148] = 0x00000000009cd352UL;
   tf->codes[149] = 0x0000000000ac2875UL;
   tf->codes[150] = 0x0000000001b2e47dUL;
   tf->codes[151] = 0x0000000001c8ceaeUL;
   tf->codes[152] = 0x00000000011dc275UL;
   tf->codes[153] = 0x000000000124a2e7UL;
   tf->codes[154] = 0x0000000000d50c77UL;
   tf->codes[155] = 0x000000000183abb8UL;
   tf->codes[156] = 0x0000000000aebd1fUL;
   tf->codes[157] = 0x000000000129b256UL;
   tf->codes[158] = 0x0000000001367ad6UL;
   tf->codes[159] = 0x0000000000373586UL;
   tf->codes[160] = 0x000000000038370bUL;
   tf->codes[161] = 0x0000000000868bf5UL;
   tf->codes[162] = 0x0000000000295866UL;
   tf->codes[163] = 0x000000000174f0dcUL;
   tf->codes[164] = 0x0000000000ab0dc8UL;
   tf->codes[165] = 0x0000000000cba3d5UL;
   tf->codes[166] = 0x0000000001790d31UL;
   tf->codes[167] = 0x0000000000595f4bUL;
   tf->codes[168] = 0x00000000002990f3UL;
   tf->codes[169] = 0x0000000001317dd5UL;
   tf->codes[170] = 0x0000000001796798UL;
   tf->codes[171] = 0x00000000004ba7eeUL;
   tf->codes[172] = 0x0000000000d1ac96UL;
   tf->codes[173] = 0x000000000093d8f4UL;
   tf->codes[174] = 0x0000000001257ba5UL;
   tf->codes[175] = 0x0000000000953f1aUL;
   tf->codes[176] = 0x00000000019a7d15UL;
   tf->codes[177] = 0x000000000090e593UL;
   tf->codes[178] = 0x0000000000eb3a27UL;
   tf->codes[179] = 0x0000000000a02725UL;
   tf->codes[180] = 0x0000000000a0fa54UL;
   tf->codes[181] = 0x00000000006f8d3dUL;
   tf->codes[182] = 0x0000000000d328bbUL;
   tf->codes[183] = 0x0000000001843b6dUL;
   tf->codes[184] = 0x0000000001cb3498UL;
   tf->codes[185] = 0x00000000005d3bc2UL;
   tf->codes[186] = 0x00000000014de368UL;
   tf->codes[187] = 0x00000000009955baUL;
   tf->codes[188] = 0x00000000002cdee3UL;
   tf->codes[189] = 0x000000000133057eUL;
   tf->codes[190] = 0x000000000000ed7aUL;
   tf->codes[191] = 0x0000000001d6caf6UL;
   tf->codes[192] = 0x00000000003a7132UL;
   tf->codes[193] = 0x0000000001501fe1UL;
   tf->codes[194] = 0x00000000007ae4c7UL;
   tf->codes[195] = 0x0000000000cf0a4eUL;
   tf->codes[196] = 0x0000000000167c7cUL;
   tf->codes[197] = 0x000000000050f7f8UL;
   tf->codes[198] = 0x00000000006955c7UL;
   tf->codes[199] = 0x0000000000f8846eUL;
   tf->codes[200] = 0x0000000001757578UL;
   tf->codes[201] = 0x0000000001a681cbUL;
   tf->codes[202] = 0x000000000142f98dUL;
   tf->codes[203] = 0x000000000145ebc7UL;
   tf->codes[204] = 0x00000000007da9d1UL;
   tf->codes[205] = 0x00000000005f20e6UL;
   tf->codes[206] = 0x00000000013986a9UL;
   tf->codes[207] = 0x0000000001cee711UL;
   tf->codes[208] = 0x0000000000fd5aeeUL;
   tf->codes[209] = 0x00000000018017eaUL;
   tf->codes[210] = 0x000000000087c359UL;
   tf->codes[211] = 0x00000000009249d8UL;
   tf->codes[212] = 0x0000000000378d23UL;
   tf->codes[213] = 0x0000000000dbcc0dUL;
   tf->codes[214] = 0x0000000001999650UL;
   tf->codes[215] = 0x00000000006fb228UL;
   tf->codes[216] = 0x000000000194b4baUL;
   tf->codes[217] = 0x0000000001ee9762UL;
   tf->codes[218] = 0x000000000128cc4eUL;
   tf->codes[219] = 0x0000000000c73614UL;
   tf->codes[220] = 0x000000000132802fUL;
   tf->codes[221] = 0x0000000001767a3dUL;
   tf->codes[222] = 0x000000000011da47UL;
   tf->codes[223] = 0x00000000014dfe8aUL;
   tf->codes[224] = 0x0000000001f35c06UL;
   tf->codes[225] = 0x0000000001229597UL;
   tf->codes[226] = 0x0000000001140b52UL;
   tf->codes[227] = 0x00000000015b273bUL;
   tf->codes[228] = 0x00000000005e8444UL;
   tf->codes[229] = 0x00000000010f68edUL;
   tf->codes[230] = 0x0000000001769cb1UL;
   tf->codes[231] = 0x000000000060766eUL;
   tf->codes[232] = 0x000000000027cfdaUL;
   tf->codes[233] = 0x0000000000abb67fUL;
   tf->codes[234] = 0x00000000009c0722UL;
   tf->codes[235] = 0x000000000137bd6dUL;
   tf->codes[236] = 0x0000000001073eb2UL;
   tf->codes[237] = 0x0000000000e4ebb7UL;
   tf->codes[238] = 0x000000000086777fUL;
   tf->codes[239] = 0x00000000002c5117UL;
   tf->codes[240] = 0x00000000000b1707UL;
   tf->codes[241] = 0x000000000128e163UL;
   tf->nbits = 25;
   tf->bit_x = (uint32_t *)calloc(25, sizeof(uint32_t));
   tf->bit_y = (uint32_t *)calloc(25, sizeof(uint32_t));
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

void tag25h7_destroy(apriltag_family_t *tf)
{
   free(tf->codes);
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
