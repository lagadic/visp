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

/* calinfo.c - demonstrates retrieving information from the calibration file such as the 
   transducer's rated loads, calibration date, and other information.
 */

#include <stdio.h>
#include <stdlib.h>
#include "..\ATIDAQ\ftconfig.h"

int main(int argc, char *argv[]) {
	char *calfilepath;      // name of calibration file
	unsigned short index;   // index of calibration in file (second parameter; default = 1)
	Calibration *cal;		// struct containing calibration information
	unsigned short i,j;     // loop variables

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
	
	// display info from calibration file
	printf("Calibration Information for %s, index #%i\n",calfilepath,index);
	printf("                  Serial: %s\n",cal->Serial);
	printf("              Body Style: %s\n",cal->BodyStyle);
	printf("             Calibration: %s\n",cal->PartNumber);
	printf("        Calibration Date: %s\n",cal->CalDate);
	printf("                  Family: %s\n",cal->Family);
	printf("              # Channels: %i\n",cal->rt.NumChannels);
	printf("                  # Axes: %i\n",cal->rt.NumAxes);
	printf("             Force Units: %s\n",cal->ForceUnits);
	printf("            Torque Units: %s\n",cal->TorqueUnits);
	printf("Temperature Compensation: %s\n",(cal->TempCompAvailable ? "Yes" : "No"));
	
	// print maximum loads of axes
	printf("\nRated Loads\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		char *units;
		if ((cal->AxisNames[i])[0]=='F') {
			units=cal->ForceUnits;
		} else units=cal->TorqueUnits;
		printf("%s: %f %s\n",cal->AxisNames[i],cal->MaxLoads[i],units);
	}

	// print working calibration matrix
	printf("\nWorking Calibration Matrix\n");
	printf("     ");
	for (i=0;i<cal->rt.NumChannels-1;i++)
		printf("G%i            ",i);
	printf("\n");
	for (i=0;i<cal->rt.NumAxes;i++) {
		printf("%s: ",cal->AxisNames[i]);
		for (j=0;j<cal->rt.NumChannels-1;j++)
			printf("%13.5e ",cal->rt.working_matrix[i][j]);
		printf("\n");
	}

	// print temperature compensation information, if available
	if (cal->TempCompAvailable) {
		printf("\nTemperature Compensation Information\n");
		printf("BS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.bias_slopes[i]);
		}
		printf("\nGS: ");
		for (i=0;i<cal->rt.NumChannels-1;i++) {
			printf("%13.5e ",cal->rt.gain_slopes[i]);
		}
		printf("\nTherm: %f\n",cal->rt.thermistor);
	}

	// free memory allocated to Calibration structure
	destroyCalibration(cal);

	//wait for a keypress
	scanf(".");
	
	return 0;
}
