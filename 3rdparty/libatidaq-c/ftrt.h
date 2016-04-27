/* ATIDAQ F/T C Library * 
 * Copyright (c) ATI Industrial Automation
 *
 * The MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/* ftrt.c - routines for realtime calculation of forces and torques from voltages
modifications:
june.15.2005 - Sam Skuce (ATI Industrial Automation) - moved some structs and #defines into
ftsharedrt.h
 */

#include "ftsharedrt.h" /*june.15.2005 - ss - added */

void RTConvertToFT(RTCoefs *coefs, float voltages[],float result[],BOOL tempcomp);
void RTBias(RTCoefs *coefs, float voltages[]);

//-------------------------------------------------
//private routines
//void mmult(float *array1, float *array2, float *result,unsigned short r1,unsigned short c1,unsigned short c2);
void mmult(float *a, unsigned short ra, unsigned short ca, unsigned short dca,
		   float *b, unsigned short cb, unsigned short dcb,
		   float *c, unsigned short dcc);
float TempComp(RTCoefs *coefs,float G,float T,unsigned short i);

