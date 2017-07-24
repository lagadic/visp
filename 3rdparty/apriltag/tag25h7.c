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

apriltag_family_t *tag25h7_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag25h7");
   tf->black_border = 1;
   tf->d = 5;
   tf->h = 7;
   tf->ncodes = 242;
   tf->codes = calloc(242, sizeof(uint64_t));
   tf->codes[0] = 0x00000000004b770dUL;
   tf->codes[1] = 0x00000000011693e6UL;
   tf->codes[2] = 0x0000000001a599abUL;
   tf->codes[3] = 0x0000000000c3a535UL;
   tf->codes[4] = 0x000000000152aafaUL;
   tf->codes[5] = 0x0000000000accd98UL;
   tf->codes[6] = 0x0000000001cad922UL;
   tf->codes[7] = 0x00000000002c2fadUL;
   tf->codes[8] = 0x0000000000bb3572UL;
   tf->codes[9] = 0x00000000014a3b37UL;
   tf->codes[10] = 0x000000000186524bUL;
   tf->codes[11] = 0x0000000000c99d4cUL;
   tf->codes[12] = 0x000000000023bfeaUL;
   tf->codes[13] = 0x000000000141cb74UL;
   tf->codes[14] = 0x0000000001d0d139UL;
   tf->codes[15] = 0x0000000001670aebUL;
   tf->codes[16] = 0x0000000000851675UL;
   tf->codes[17] = 0x000000000150334eUL;
   tf->codes[18] = 0x00000000006e3ed8UL;
   tf->codes[19] = 0x0000000000fd449dUL;
   tf->codes[20] = 0x0000000000aa55ecUL;
   tf->codes[21] = 0x0000000001c86176UL;
   tf->codes[22] = 0x00000000015e9b28UL;
   tf->codes[23] = 0x00000000007ca6b2UL;
   tf->codes[24] = 0x000000000147c38bUL;
   tf->codes[25] = 0x0000000001d6c950UL;
   tf->codes[26] = 0x00000000008b0e8cUL;
   tf->codes[27] = 0x00000000011a1451UL;
   tf->codes[28] = 0x0000000001562b65UL;
   tf->codes[29] = 0x00000000013f53c8UL;
   tf->codes[30] = 0x0000000000d58d7aUL;
   tf->codes[31] = 0x0000000000829ec9UL;
   tf->codes[32] = 0x0000000000faccf1UL;
   tf->codes[33] = 0x000000000136e405UL;
   tf->codes[34] = 0x00000000007a2f06UL;
   tf->codes[35] = 0x00000000010934cbUL;
   tf->codes[36] = 0x00000000016a8b56UL;
   tf->codes[37] = 0x0000000001a6a26aUL;
   tf->codes[38] = 0x0000000000f85545UL;
   tf->codes[39] = 0x000000000195c2e4UL;
   tf->codes[40] = 0x000000000024c8a9UL;
   tf->codes[41] = 0x00000000012bfc96UL;
   tf->codes[42] = 0x00000000016813aaUL;
   tf->codes[43] = 0x0000000001a42abeUL;
   tf->codes[44] = 0x0000000001573424UL;
   tf->codes[45] = 0x0000000001044573UL;
   tf->codes[46] = 0x0000000000b156c2UL;
   tf->codes[47] = 0x00000000005e6811UL;
   tf->codes[48] = 0x0000000001659bfeUL;
   tf->codes[49] = 0x0000000001d55a63UL;
   tf->codes[50] = 0x00000000005bf065UL;
   tf->codes[51] = 0x0000000000e28667UL;
   tf->codes[52] = 0x0000000001e9ba54UL;
   tf->codes[53] = 0x00000000017d7c5aUL;
   tf->codes[54] = 0x0000000001f5aa82UL;
   tf->codes[55] = 0x0000000001a2bbd1UL;
   tf->codes[56] = 0x00000000001ae9f9UL;
   tf->codes[57] = 0x0000000001259e51UL;
   tf->codes[58] = 0x000000000134062bUL;
   tf->codes[59] = 0x0000000000e1177aUL;
   tf->codes[60] = 0x0000000000ed07a8UL;
   tf->codes[61] = 0x000000000162be24UL;
   tf->codes[62] = 0x000000000059128bUL;
   tf->codes[63] = 0x0000000001663e8fUL;
   tf->codes[64] = 0x00000000001a83cbUL;
   tf->codes[65] = 0x000000000045bb59UL;
   tf->codes[66] = 0x000000000189065aUL;
   tf->codes[67] = 0x00000000004bb370UL;
   tf->codes[68] = 0x00000000016fb711UL;
   tf->codes[69] = 0x000000000122c077UL;
   tf->codes[70] = 0x0000000000eca17aUL;
   tf->codes[71] = 0x0000000000dbc1f4UL;
   tf->codes[72] = 0x000000000088d343UL;
   tf->codes[73] = 0x000000000058ac5dUL;
   tf->codes[74] = 0x0000000000ba02e8UL;
   tf->codes[75] = 0x00000000001a1d9dUL;
   tf->codes[76] = 0x0000000001c72eecUL;
   tf->codes[77] = 0x0000000000924bc5UL;
   tf->codes[78] = 0x0000000000dccab3UL;
   tf->codes[79] = 0x0000000000886d15UL;
   tf->codes[80] = 0x000000000178c965UL;
   tf->codes[81] = 0x00000000005bc69aUL;
   tf->codes[82] = 0x0000000001716261UL;
   tf->codes[83] = 0x000000000174e2ccUL;
   tf->codes[84] = 0x0000000001ed10f4UL;
   tf->codes[85] = 0x0000000000156aa8UL;
   tf->codes[86] = 0x00000000003e2a8aUL;
   tf->codes[87] = 0x00000000002752edUL;
   tf->codes[88] = 0x000000000153c651UL;
   tf->codes[89] = 0x0000000001741670UL;
   tf->codes[90] = 0x0000000000765b05UL;
   tf->codes[91] = 0x000000000119c0bbUL;
   tf->codes[92] = 0x000000000172a783UL;
   tf->codes[93] = 0x00000000004faca1UL;
   tf->codes[94] = 0x0000000000f31257UL;
   tf->codes[95] = 0x00000000012441fcUL;
   tf->codes[96] = 0x00000000000d3748UL;
   tf->codes[97] = 0x0000000000c21f15UL;
   tf->codes[98] = 0x0000000000ac5037UL;
   tf->codes[99] = 0x000000000180e592UL;
   tf->codes[100] = 0x00000000007d3210UL;
   tf->codes[101] = 0x0000000000a27187UL;
   tf->codes[102] = 0x00000000002beeafUL;
   tf->codes[103] = 0x000000000026ff57UL;
   tf->codes[104] = 0x0000000000690e82UL;
   tf->codes[105] = 0x000000000077765cUL;
   tf->codes[106] = 0x0000000001a9e1d7UL;
   tf->codes[107] = 0x000000000140be1aUL;
   tf->codes[108] = 0x0000000001aa1e3aUL;
   tf->codes[109] = 0x0000000001944f5cUL;
   tf->codes[110] = 0x00000000019b5032UL;
   tf->codes[111] = 0x0000000000169897UL;
   tf->codes[112] = 0x0000000001068eb9UL;
   tf->codes[113] = 0x0000000000f30dbcUL;
   tf->codes[114] = 0x000000000106a151UL;
   tf->codes[115] = 0x0000000001d53e95UL;
   tf->codes[116] = 0x0000000001348ceeUL;
   tf->codes[117] = 0x0000000000cf4fcaUL;
   tf->codes[118] = 0x0000000001728bb5UL;
   tf->codes[119] = 0x0000000000dc1eecUL;
   tf->codes[120] = 0x000000000069e8dbUL;
   tf->codes[121] = 0x00000000016e1523UL;
   tf->codes[122] = 0x000000000105fa25UL;
   tf->codes[123] = 0x00000000018abb0cUL;
   tf->codes[124] = 0x0000000000c4275dUL;
   tf->codes[125] = 0x00000000006d8e76UL;
   tf->codes[126] = 0x0000000000e8d6dbUL;
   tf->codes[127] = 0x0000000000e16fd7UL;
   tf->codes[128] = 0x0000000001ac2682UL;
   tf->codes[129] = 0x000000000077435bUL;
   tf->codes[130] = 0x0000000000a359ddUL;
   tf->codes[131] = 0x00000000003a9c4eUL;
   tf->codes[132] = 0x000000000123919aUL;
   tf->codes[133] = 0x0000000001e25817UL;
   tf->codes[134] = 0x000000000002a836UL;
   tf->codes[135] = 0x00000000001545a4UL;
   tf->codes[136] = 0x0000000001209c8dUL;
   tf->codes[137] = 0x0000000000bb5f69UL;
   tf->codes[138] = 0x0000000001dc1f02UL;
   tf->codes[139] = 0x00000000005d5f7eUL;
   tf->codes[140] = 0x00000000012d0581UL;
   tf->codes[141] = 0x00000000013786c2UL;
   tf->codes[142] = 0x0000000000e15409UL;
   tf->codes[143] = 0x0000000001aa3599UL;
   tf->codes[144] = 0x000000000139aad8UL;
   tf->codes[145] = 0x0000000000b09d2aUL;
   tf->codes[146] = 0x000000000054488fUL;
   tf->codes[147] = 0x00000000013c351cUL;
   tf->codes[148] = 0x0000000000976079UL;
   tf->codes[149] = 0x0000000000b25b12UL;
   tf->codes[150] = 0x0000000001addb34UL;
   tf->codes[151] = 0x0000000001cb23aeUL;
   tf->codes[152] = 0x0000000001175738UL;
   tf->codes[153] = 0x0000000001303bb8UL;
   tf->codes[154] = 0x0000000000d47716UL;
   tf->codes[155] = 0x000000000188ceeaUL;
   tf->codes[156] = 0x0000000000baf967UL;
   tf->codes[157] = 0x0000000001226d39UL;
   tf->codes[158] = 0x000000000135e99bUL;
   tf->codes[159] = 0x000000000034adc5UL;
   tf->codes[160] = 0x00000000002e384dUL;
   tf->codes[161] = 0x000000000090d3faUL;
   tf->codes[162] = 0x0000000000232713UL;
   tf->codes[163] = 0x00000000017d49b1UL;
   tf->codes[164] = 0x0000000000aa84d6UL;
   tf->codes[165] = 0x0000000000c2ddf8UL;
   tf->codes[166] = 0x0000000001665646UL;
   tf->codes[167] = 0x00000000004f345fUL;
   tf->codes[168] = 0x00000000002276b1UL;
   tf->codes[169] = 0x0000000001255dd7UL;
   tf->codes[170] = 0x00000000016f4cccUL;
   tf->codes[171] = 0x00000000004aaffcUL;
   tf->codes[172] = 0x0000000000c46da6UL;
   tf->codes[173] = 0x000000000085c7b3UL;
   tf->codes[174] = 0x0000000001311fcbUL;
   tf->codes[175] = 0x00000000009c6c4fUL;
   tf->codes[176] = 0x000000000187d947UL;
   tf->codes[177] = 0x00000000008578e4UL;
   tf->codes[178] = 0x0000000000e2bf0bUL;
   tf->codes[179] = 0x0000000000a01b4cUL;
   tf->codes[180] = 0x0000000000a1493bUL;
   tf->codes[181] = 0x00000000007ad766UL;
   tf->codes[182] = 0x0000000000ccfe82UL;
   tf->codes[183] = 0x0000000001981b5bUL;
   tf->codes[184] = 0x0000000001cacc85UL;
   tf->codes[185] = 0x0000000000562cdbUL;
   tf->codes[186] = 0x00000000015b0e78UL;
   tf->codes[187] = 0x00000000008f66c5UL;
   tf->codes[188] = 0x00000000003332bfUL;
   tf->codes[189] = 0x00000000012ce754UL;
   tf->codes[190] = 0x0000000000096a76UL;
   tf->codes[191] = 0x0000000001d5e3baUL;
   tf->codes[192] = 0x000000000027ea41UL;
   tf->codes[193] = 0x00000000014412dfUL;
   tf->codes[194] = 0x000000000067b9b4UL;
   tf->codes[195] = 0x0000000000daa51aUL;
   tf->codes[196] = 0x00000000001dcb17UL;
   tf->codes[197] = 0x00000000004d4afdUL;
   tf->codes[198] = 0x00000000006335d5UL;
   tf->codes[199] = 0x0000000000ee2334UL;
   tf->codes[200] = 0x00000000017d4e55UL;
   tf->codes[201] = 0x0000000001b8b0f0UL;
   tf->codes[202] = 0x00000000014999e3UL;
   tf->codes[203] = 0x0000000001513dfaUL;
   tf->codes[204] = 0x0000000000765cf2UL;
   tf->codes[205] = 0x000000000056af90UL;
   tf->codes[206] = 0x00000000012e16acUL;
   tf->codes[207] = 0x0000000001d3d86cUL;
   tf->codes[208] = 0x0000000000ff279bUL;
   tf->codes[209] = 0x00000000018822ddUL;
   tf->codes[210] = 0x000000000099d478UL;
   tf->codes[211] = 0x00000000008dc0d2UL;
   tf->codes[212] = 0x000000000034b666UL;
   tf->codes[213] = 0x0000000000cf9526UL;
   tf->codes[214] = 0x000000000186443dUL;
   tf->codes[215] = 0x00000000007a8e29UL;
   tf->codes[216] = 0x00000000019c6aa5UL;
   tf->codes[217] = 0x0000000001f2a27dUL;
   tf->codes[218] = 0x00000000012b2136UL;
   tf->codes[219] = 0x0000000000d0cd0dUL;
   tf->codes[220] = 0x00000000012cb320UL;
   tf->codes[221] = 0x00000000017ddb0bUL;
   tf->codes[222] = 0x000000000005353bUL;
   tf->codes[223] = 0x00000000015b2cafUL;
   tf->codes[224] = 0x0000000001e5a507UL;
   tf->codes[225] = 0x000000000120f1e5UL;
   tf->codes[226] = 0x000000000114605aUL;
   tf->codes[227] = 0x00000000014efe4cUL;
   tf->codes[228] = 0x0000000000568134UL;
   tf->codes[229] = 0x00000000011b9f92UL;
   tf->codes[230] = 0x000000000174d2a7UL;
   tf->codes[231] = 0x0000000000692b1dUL;
   tf->codes[232] = 0x000000000039e4feUL;
   tf->codes[233] = 0x0000000000aaff3dUL;
   tf->codes[234] = 0x000000000096224cUL;
   tf->codes[235] = 0x00000000013c9f77UL;
   tf->codes[236] = 0x000000000110ee8fUL;
   tf->codes[237] = 0x0000000000f17beaUL;
   tf->codes[238] = 0x000000000099fb5dUL;
   tf->codes[239] = 0x0000000000337141UL;
   tf->codes[240] = 0x000000000002b54dUL;
   tf->codes[241] = 0x0000000001233a70UL;
   return tf;
}

void tag25h7_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
