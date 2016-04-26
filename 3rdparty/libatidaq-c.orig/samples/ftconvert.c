/* ATIDAQ F/T C Library
 * v1.0.2
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

/* ftconvert.c - demonstrates configuring a DAQ F/T system and performing force/torque 
calculations; converts a sample set of voltages into forces and torques.

Sam Skuce - v.1.0.2 - February.19.2004 - added some comments just before the declaration
	of SampleBias explaining the difference	between hardware and software temp comp
 */

#include <stdio.h>
#include <stdlib.h>
#include "ftconfig.h"

int main(int argc, char *argv[]) {
	char *calfilepath;      // name of calibration file
	unsigned short index;   // index of calibration in file (second parameter; default = 1)
	Calibration *cal;		// struct containing calibration information
	unsigned short i;       // loop variable used to print results
	short sts;              // return value from functions

	// In this sample application, readings have been hard-coded for demonstration.
	// A working application would retrieve these vectors from a data acquisition system.
	// PLEASE NOTE:  There are 7 elements in the bias and reading arrays.  The first 6 are
	//	the gage values you would retrieve from the transducer.  The seventh represents the 
	//	thermistor gage, which is only meaningful if your sensor uses software temperature
	//	compensation.  If your sensor uses hardware temperature compensation (all sensors
	//	sold after mid-2003 do), the last value is meaningless, and you can just use a 6 element
	//	array.
	float SampleBias[7]={0.2651,-0.0177,-0.0384,-0.0427,-0.1891,0.1373,-3.2423};
	float SampleReading[7]={-3.2863,0.3875,-3.4877,0.4043,-3.9341,0.5474,-3.2106};

	// This sample transform includes a translation along the Z-axis and a rotation about the X-axis.
	float SampleTT[6]={0,0,20,45,0,0};

	float FT[6];            // This array will hold the resultant force/torque vector.

	// parse arguments
	switch (argc) {
		case 2:
			index=1;
			break;
		case 3:
			index=atoi(argv[2]);
			break;
		default:
			printf("\nUsage: calinfo [calfile] [index]\n");
			printf("  [calfile]: path to calibration file\n");
			printf("  [index]  : index of calibration in file (default=1)\n");
			scanf(".");
			return 0;
	}
	calfilepath=argv[1];

	// create Calibration struct
	cal=createCalibration(calfilepath,index);
	if (cal==NULL) {
		printf("\nSpecified calibration could not be loaded.\n");
		scanf(".");
		return 0;
	}
	
	// Set force units.
	// This step is optional; by default, the units are inherited from the calibration file.
	sts=SetForceUnits(cal,"N");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid force units"); return 0;
		default: printf("Unknown error"); return 0;
	}


	// Set torque units.
	// This step is optional; by default, the units are inherited from the calibration file.
	sts=SetTorqueUnits(cal,"N-m");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid torque units"); return 0;
		default: printf("Unknown error"); return 0;
	}


	// Set tool transform.
	// This line is only required if you want to move or rotate the sensor's coordinate system.
	// This example tool transform translates the coordinate system 20 mm along the Z-axis 
	// and rotates it 45 degrees about the X-axis.
	sts=SetToolTransform(cal,SampleTT,"mm","degrees");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid distance units"); return 0;
		case 3: printf("Invalid angle units"); return 0;
		default: printf("Unknown error"); return 0;
	}

	
	// Temperature compensation is on by default if it is available.
	// To explicitly disable temperature compensation, uncomment the following code
	/*SetTempComp(cal,FALSE);                   // disable temperature compensation
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Temperature Compensation not available on this transducer"); return 0;
		default: printf("Unknown error"); return 0;
	}*/

	// store an unloaded measurement; this removes the effect of tooling weight
	Bias(cal,SampleBias);

	// convert a loaded measurement into forces and torques
	ConvertToFT(cal,SampleReading,FT);
	
	
	// print results
	printf("Bias reading:\n");
	for (i=0;i<7;i++)
		printf("%9.6f ",SampleBias[i]);
	printf("\nMeasurement:\n");
	for (i=0;i<7;i++)
		printf("%9.6f ",SampleReading[i]);
	printf("\nResult:\n");
	for (i=0;i<6;i++)
		printf("%9.6f ",FT[i]);

	// free memory allocated to Calibration structure
	destroyCalibration(cal);

	//wait for a keypress
	scanf(".");

	return 0;
}
