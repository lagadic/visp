/* ATIDAQ F/T C Library
 * v1.0.1
 * Copyright (c) 2001 ATI Industrial Automation
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

/* ftrt.h - routines for realtime calculation of forces and torques from voltages
 */

#include "ftrt.h"

//--------------------------------------------------
// public routines definitions

void RTConvertToFT(RTCoefs *coefs, float voltages[],float result[],BOOL tempcomp) {
	// perform temp. comp., if applicable
	float cvoltages[MAX_GAUGES];
	unsigned short i;

	for (i=0; i<coefs->NumChannels-1; i++) {
		if (tempcomp==TRUE) {
			cvoltages[i]=TempComp(coefs,voltages[i],voltages[coefs->NumChannels-1],i) - coefs->TCbias_vector[i];
		} else {
			cvoltages[i]=voltages[i]-coefs->bias_vector[i];
		}
	}
	// perform matrix math
	mmult(*coefs->working_matrix,coefs->NumAxes,(unsigned short)(coefs->NumChannels-1),MAX_GAUGES,
		cvoltages,1,1,
		result,1);
}
void RTBias(RTCoefs *coefs, float voltages[]) {
	unsigned short i;
	for (i=0; i<coefs->NumChannels-1; i++) {
		coefs->bias_vector[i]=voltages[i];
		coefs->TCbias_vector[i]=TempComp(coefs,voltages[i],voltages[coefs->NumChannels-1],i);
	}
	// store bias thermistor value, although it is no longer needed
	coefs->bias_vector[coefs->NumChannels-1]=voltages[coefs->NumChannels-1];
}

//--------------------------------------
// private routines definitions
void mmult(float *a, unsigned short ra, unsigned short ca, unsigned short dca,
		   float *b, unsigned short cb, unsigned short dcb,
		   float *c, unsigned short dcc) {
    //multiplies two matrices; must be properly dimensioned
    unsigned short i,j,k;
	for (i=0;i<ra;i++)
		for (j=0;j<cb;j++) {
			c[i*dcc+j]=0;
            for (k=0;k<ca;k++)
                c[i*dcc+j] = c[i*dcc+j] + a[i*dca+k] * b[k*dcb+j];
		}
}

float TempComp(RTCoefs *coefs,float G,float T,unsigned short i) {
	return ((G + coefs->bias_slopes[i] * (T - coefs->thermistor)) / (1 - coefs->gain_slopes[i] * (T - coefs->thermistor)));
}
