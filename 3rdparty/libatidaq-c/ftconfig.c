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

/* ftconfig.c - calibration file and configuration routines
History
dec.14.2007 - Sam Skuce (ATI Industrial Automation)
  Added support for kN-m torque units.
 */

#include "dom.h"				// GBB: added
#include "ftconfig.h"			// GBB: ftconfig.h modified to include strcutures in ftrt.h
#include <math.h>              // sin(), cos()
#include <stdio.h>

//-------------------------------------------------
// private functions 
//-------------------------------------------------

// from ftrt.h (these functions are defined in ftrt.c)
extern void RTConvertToFT(RTCoefs *coefs, float voltages[],float result[],BOOL tempcomp);
extern void RTBias(RTCoefs *coefs, float voltages[]);

// void mmult(float *array1, float *array2, float *result,unsigned short r1,unsigned short c1,unsigned short c2);
extern void mmult(float *a, unsigned short ra, unsigned short ca, unsigned short dca,
		   float *b, unsigned short cb, unsigned short dcb,
		   float *c, unsigned short dcc);

// GBB: moved here these definitions from config.h
void ResetDefaults(Calibration *cal);
short CalcMatrix(Calibration *cal);
short GetMatrix(Calibration *cal, float *result);
short TTM(Transform xform,float result[6][6],Units ForceUnits,Units TorqueUnits);
float ForceConv(char *Units);
float TorqueConv(char *Units);
float DistConv(char *Units);
float AngleConv(char *Units);
short ReadAttribute(const DOM_Element *elem, char **attValue, char *attName, BOOL required, char *defaultValue);
void Separate(char *ValueList,float results[],unsigned short numValues);
unsigned short FindText(char *str, unsigned short StartPos);
unsigned short FindSpace(char *str, unsigned short StartPos);
char *mid(char *instr,unsigned short startpos,unsigned short length);


//----------------------------------------------
// "public" functions
//----------------------------------------------
short SetToolTransform(Calibration *cal, float Vector[6],char *DistUnits,char *AngleUnits) {
	short i;
	if (cal==NULL) return 1;
	if (DistConv(DistUnits)==0) return 2;
	if (AngleConv(AngleUnits)==0) return 3;
	for(i=0;i<6;i++)
		cal->cfg.UserTransform.TT[i]=Vector[i];
	free(cal->cfg.UserTransform.DistUnits);
	cal->cfg.UserTransform.DistUnits=ATI_strdup(DistUnits);
	free(cal->cfg.UserTransform.AngleUnits);
	cal->cfg.UserTransform.AngleUnits=ATI_strdup(AngleUnits);
	return CalcMatrix(cal);
} // SetToolTransform()

short SetForceUnits(Calibration *cal, char *NewUnits) {
	if (cal==NULL) return 1;
	if (ForceConv(NewUnits)==0) return 2;
	free(cal->cfg.ForceUnits);
	cal->cfg.ForceUnits=ATI_strdup(NewUnits);
	return CalcMatrix(cal);
} // SetForceUnits()

short SetTorqueUnits(Calibration *cal, char *NewUnits) {
	if (cal==NULL) return 1;
	if (TorqueConv(NewUnits)==0) return 2;
	free(cal->cfg.TorqueUnits);
	cal->cfg.TorqueUnits=ATI_strdup(NewUnits);
	return CalcMatrix(cal);
} // SetTorqueUnits()

short SetTempComp(Calibration *cal, int TCEnabled) {
	if (cal==NULL) return 1;
	if (TCEnabled) 
		if (cal->TempCompAvailable) {
			cal->cfg.TempCompEnabled=TRUE;
		} else return 2;
	else cal->cfg.TempCompEnabled=FALSE;
	return 0;
} // SetTempComp();

Calibration *createCalibration(char *CalFilePath,unsigned short index) {
// This function creates and populate a Calibration structure from a file.
	Calibration *cal;
	DOM_DocumentLS *doc;            // contains DOM document of calibration file
	DOM_Element *eRoot;             // points to document root element ("FTSensor")
	DOM_Node *node;                 // multipurpose variable for nodes in the cal file
	DOM_Element *eCalibration;      // points to Calibration element
	DOM_NodeList *calibrationNodelist;         //node lists in the cal file
	DOM_NodeList *axisNodelist;
	DOM_NodeList *childNodelist;
	char *temp;                     // temporary string value for reading in attributes
	unsigned short i,j;             // counter variables
	float temparray[MAX_GAUGES];    // used when loading calibration rows
	float scale;                    // used when loading calibration rows
	
	cal=(Calibration *) calloc(1,sizeof(Calibration));	
	doc = DOM_Implementation_createDocument(NULL, NULL, NULL);  // initialize DOM document	
	if (DOM_DocumentLS_load(doc,CalFilePath)!=1) {              // open calibration file
		free(cal);
		DOM_Document_destroyNode(doc, doc);    // clean up DOM stuff
		return NULL;
	}
	eRoot=doc->u.Document.documentElement;
	if (strcmp(eRoot->nodeName,"FTSensor")!=0) {        // make sure we're loading the right kind of file
		free(cal);
		DOM_Document_destroyNode(doc, doc);    // clean up DOM stuff
		return NULL;
	}

	ReadAttribute(eRoot,&temp,"Serial",TRUE,"");
	cal->Serial=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eRoot,&temp,"BodyStyle",TRUE,"");
	cal->BodyStyle=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eRoot,&temp,"NumGages",TRUE,"");
	cal->rt.NumChannels=atoi(temp)+1;	// add one to NumGages for the temperature channel.
	free(temp);
	ReadAttribute(eRoot,&temp,"Family",TRUE,"");
	cal->Family=ATI_strdup(temp);
	free(temp);
	// find calibration specified by index
	calibrationNodelist=DOM_Element_getElementsByTagName(eRoot,"Calibration");
	if ((calibrationNodelist->length) < index || index < 1) {    // determine if invalid calibration index was used
		return NULL;
	}
	eCalibration=DOM_NodeList_item(calibrationNodelist,index-1);
	
	// set Calibration structure attributes found in Calibration element
	ReadAttribute(eCalibration,&temp,"PartNumber",TRUE,"");
	cal->PartNumber=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eCalibration,&temp,"CalDate",TRUE,"");
	cal->CalDate=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eCalibration,&temp,"ForceUnits",TRUE,"");
	cal->ForceUnits=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eCalibration,&temp,"TorqueUnits",TRUE,"");
	cal->TorqueUnits=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eCalibration,&temp,"DistUnits",TRUE,"");
	cal->BasicTransform.DistUnits=ATI_strdup(temp);
	cal->cfg.UserTransform.DistUnits=ATI_strdup(temp);
	free(temp);
	ReadAttribute(eCalibration,&temp,"AngleUnits",FALSE,"degrees");
	cal->BasicTransform.AngleUnits=ATI_strdup(temp);
	cal->cfg.UserTransform.AngleUnits=ATI_strdup(temp);
	free(temp);
	// initialize temp comp variables
	cal->TempCompAvailable=FALSE;
	for (i=0;i<MAX_GAUGES;i++) {
		cal->rt.bias_slopes[i]=0;
		cal->rt.gain_slopes[i]=0;
	}
	cal->rt.thermistor=0;
	
	// store basic matrix
	axisNodelist=DOM_Element_getElementsByTagName(eCalibration,"Axis");

	cal->rt.NumAxes=(unsigned short) axisNodelist->length;
	for (i=0;i<axisNodelist->length;i++) {
		node=DOM_NodeList_item(axisNodelist,i);		
		ReadAttribute(node, &temp, "scale", FALSE,"1");
		scale=(float) atof(temp);
		free(temp);
		ReadAttribute(node, &temp, "values", TRUE,"");
		Separate(temp,temparray,(unsigned short)(cal->rt.NumChannels-1));
		for(j=0;j<cal->rt.NumChannels-1;j++) {
			cal->BasicMatrix[i][j]=temparray[j]/scale;
		}
		free(temp);
		ReadAttribute(node, &temp, "max", FALSE,"0");
		cal->MaxLoads[i]=(float) atof(temp);
		free(temp);		
		ReadAttribute(node, &temp, "Name", TRUE,"");
		cal->AxisNames[i]=ATI_strdup(temp);
		free(temp);
	}
    childNodelist=eCalibration->childNodes;
	for (i=0; i < childNodelist->length; i++) {
		node=DOM_NodeList_item(childNodelist,i);

		if (strcmp(node->nodeName,"BasicTransform")==0) {
			ReadAttribute(node, &temp, "Dx", FALSE,"0");
            cal->BasicTransform.TT[0]=(float) atof(temp);
			free(temp);
            ReadAttribute(node, &temp, "Dy", FALSE,"0");
            cal->BasicTransform.TT[1]=(float) atof(temp);
			free(temp);
			ReadAttribute(node, &temp, "Dz", FALSE,"0");
            cal->BasicTransform.TT[2]=(float) atof(temp);
			free(temp);
			ReadAttribute(node, &temp, "Rx", FALSE,"0");
            cal->BasicTransform.TT[3]=(float) atof(temp);
			free(temp);
			ReadAttribute(node, &temp, "Ry", FALSE,"0");
            cal->BasicTransform.TT[4]=(float) atof(temp);
			free(temp);
			ReadAttribute(node, &temp, "Rz", FALSE,"0");
            cal->BasicTransform.TT[5]=(float) atof(temp);
			free(temp);
		} else if (strcmp(node->nodeName,"BiasSlope")==0) {
			ReadAttribute(node, &temp, "values", TRUE,"");
            Separate(temp,cal->rt.bias_slopes,(unsigned short)(cal->rt.NumChannels-1));
			free(temp);
            cal->TempCompAvailable = TRUE;
		} else if (strcmp(node->nodeName,"GainSlope")==0) {
			ReadAttribute(node, &temp, "values", TRUE,"");
            Separate(temp,cal->rt.gain_slopes,(unsigned short)(cal->rt.NumChannels-1));
			free(temp);
            cal->TempCompAvailable = TRUE;
		} else if (strcmp(node->nodeName,"Thermistor")==0) {
            ReadAttribute(node, &temp, "value", TRUE,"");
			cal->rt.thermistor = (float) atof(temp);
			free(temp);
		}

	}

	DOM_Document_destroyNodeList(eRoot->ownerDocument, calibrationNodelist,0);
	DOM_Document_destroyNodeList(eRoot->ownerDocument, axisNodelist,0);	
	DOM_Document_destroyNode(doc, doc);    // clean up DOM stuff		
	ResetDefaults(cal);                    // calculate working matrix and set default values		
	return cal;	
} // createCalibration();

void destroyCalibration(Calibration *cal) {
// frees all memory allocated for a Calibration structure
	int i;
	if (cal==NULL) return;
	for(i=0;i<MAX_AXES;i++)
		free(cal->AxisNames[i]);
	free(cal->BasicTransform.AngleUnits);
	free(cal->BasicTransform.DistUnits);
	free(cal->cfg.UserTransform.AngleUnits);
	free(cal->cfg.UserTransform.DistUnits);
	free(cal->cfg.ForceUnits);
	free(cal->cfg.TorqueUnits);
	free(cal->BodyStyle);
	free(cal->Family);
	free(cal->Serial);
	free(cal->CalDate);
	free(cal->PartNumber);
	free(cal->ForceUnits);
	free(cal->TorqueUnits);
	free(cal);
	return;
} // destroyCalibration()

void Bias(Calibration *cal, float voltages[]) {
	RTBias(&cal->rt,voltages);
} // Bias()

void ConvertToFT(Calibration *cal, float voltages[],float result[]) {
	RTConvertToFT(&cal->rt,voltages,result,cal->cfg.TempCompEnabled);
} // ConvertToFT()



// from calinfo.c
void printCalInfo(Calibration *cal) {
	unsigned short i,j;     // loop variables

	
	// display info from calibration file
	printf("Calibration Information:\n");
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
}

//----------------------------------------------
// "private" functions
//----------------------------------------------
void ResetDefaults(Calibration *cal) {
// Restores all configuration options to their default values
	unsigned short i;	
	cal->cfg.ForceUnits=ATI_strdup(cal->ForceUnits);  // set output units to calibration file defaults
	cal->cfg.TorqueUnits=ATI_strdup(cal->TorqueUnits);	
	for (i=0;i<6;i++)                                 // clear user Tool Transform
		cal->cfg.UserTransform.TT[i]=0;
	for (i=0;i < cal->rt.NumChannels-1;i++) {         // clear bias vectors
		cal->rt.bias_vector[i]=0;
		cal->rt.TCbias_vector[i]=0;
	}
	cal->rt.bias_vector[cal->rt.NumChannels-1]=0;     // thermistor reading of bias vector
	cal->cfg.TempCompEnabled=cal->TempCompAvailable;  // turn temp comp on by default, if available
	
	CalcMatrix(cal);                                  // compute working matrix and store in rt
} // ResetDefaults()
short CalcMatrix(Calibration *cal) {
// Computes a working matrix (based on BasicMatrix, tool transforms, and units)
// and stores it in the rt structure
	return GetMatrix(cal,*cal->rt.working_matrix);
} //CalcMatrix()
short GetMatrix(Calibration *cal, float *result) {
// Calculates a working matrix based on the basic matrix,
// basic tool transform, user tool transform, and user units,
// and stores in result.
	float UserTTM[6][6];           // the User tool transform matrix 
    float BasicTTM[6][6];          // basic (built-in) tool transform matrix
	float result1[6][MAX_GAUGES];  // temporary intermediate result
	float FConv, TConv;            // unit conversion factors
	unsigned short i, j;           // loop variables
	unsigned short NumGauges=cal->rt.NumChannels-1;  // number of strain gages
	short sts;                     // return value 

	if (cal->rt.NumAxes==6) {
		sts=TTM(cal->BasicTransform,BasicTTM,cal->ForceUnits,cal->TorqueUnits);
		if (sts!=0) return 1;      // error in tool transform units
		sts=TTM(cal->cfg.UserTransform,UserTTM,cal->ForceUnits,cal->TorqueUnits);
		if (sts!=0) return 1;      // error in tool transform units
		mmult(*BasicTTM,6,6,6,
			*cal->BasicMatrix,NumGauges,MAX_GAUGES,
			*result1,MAX_GAUGES);
		mmult(*UserTTM,6,6,6,                // compute working matrix
			*result1,NumGauges,MAX_GAUGES,
			result,MAX_GAUGES);
	} else {
		// No transforms allowed except for 6-axis transducers
		result=*cal->BasicMatrix;
	}
	//Apply units change
	FConv = ForceConv(cal->cfg.ForceUnits) / ForceConv(cal->ForceUnits);
	TConv = TorqueConv(cal->cfg.TorqueUnits) / TorqueConv(cal->TorqueUnits);
	for(i=0;i<cal->rt.NumAxes;i++)  //forces
		for(j=0;j<NumGauges;j++)
			if ((cal->AxisNames[i])[0]=='F') {
				result[i*MAX_GAUGES+j] = result[i*MAX_GAUGES+j] * FConv;
				if (FConv==0) return 2;
			}
			else {
				result[i*MAX_GAUGES+j] = result[i*MAX_GAUGES+j] * TConv;
				if (TConv==0) return 2;
			}
	return 0;
} // GetMatrix()
short TTM(Transform xform,float result[6][6],Units ForceUnits,Units TorqueUnits) {
// calculates a tool transform matrix and stores in result
	float R[3][3];	// rotation matrix
	float sx, sy, sz, cx, cy, cz, dx, dy, dz;
	unsigned short i, j;	// loop variables
	float DC,AC;	// distance conversion factor and angle conversion factor

	if ((DistConv(xform.DistUnits)==0) || (AngleConv(xform.AngleUnits)==0)) return 1; // invalid units
	DC = TorqueConv(TorqueUnits) / (ForceConv(ForceUnits) * DistConv(xform.DistUnits));
	AC = (float) 1.0/AngleConv(xform.AngleUnits);
	

	// calculate sin & cos of angles
	sx = (float) sin(PI/180 * xform.TT[3] * AC);
	cx = (float) cos(PI/180 * xform.TT[3] * AC);
	
	sy = (float) sin(PI/180 * xform.TT[4] * AC);
	cy = (float) cos(PI/180 * xform.TT[4] * AC);

	sz = (float) sin(PI/180 * xform.TT[5] * AC);
	cz = (float) cos(PI/180 * xform.TT[5] * AC);


	// calculate Dx, Dy, Dz
	dx = xform.TT[0]*DC;
	dy = xform.TT[1]*DC;
	dz = xform.TT[2]*DC;

	// calculate rotation matrix
	R[0][0] =  cy * cz;
	R[0][1] =  sx * sy * cz + cx * sz;
	R[0][2] =  sx * sz - cx * sy * cz;
	R[1][0] = -cy * sz;
	R[1][1] = -sx * sy * sz + cx * cz;
	R[1][2] =  sx * cz + cx * sy * sz;
	R[2][0] =  sy;
	R[2][1] = -sx * cy;
	R[2][2] =  cx * cy;

	// calculate transformation matrix
	for(i=0;i<=5;i++)
		for(j=0;j<=5;j++)
			result[i][j]=0;

	result[0][0] = R[0][0];
	result[0][1] = R[0][1];
	result[0][2] = R[0][2];

	result[1][0] = R[1][0];
	result[1][1] = R[1][1];
	result[1][2] = R[1][2];

	result[2][0] = R[2][0];
	result[2][1] = R[2][1];
	result[2][2] = R[2][2];

	result[3][0] = R[0][2] * dy - R[0][1] * dz;
	result[3][1] = R[0][0] * dz - R[0][2] * dx;
	result[3][2] = R[0][1] * dx - R[0][0] * dy;
	result[3][3] = R[0][0];
	result[3][4] = R[0][1];
	result[3][5] = R[0][2];

	result[4][0] = R[1][2] * dy - R[1][1] * dz;
	result[4][1] = R[1][0] * dz - R[1][2] * dx;
	result[4][2] = R[1][1] * dx - R[1][0] * dy;
	result[4][3] = R[1][0];
	result[4][4] = R[1][1];
	result[4][5] = R[1][2];

	result[5][0] = R[2][2] * dy - R[2][1] * dz;
	result[5][1] = R[2][0] * dz - R[2][2] * dx;
	result[5][2] = R[2][1] * dx - R[2][0] * dy;
	result[5][3] = R[2][0];
	result[5][4] = R[2][1];
	result[5][5] = R[2][2];
	return 0;
} // TTM()
float ForceConv(Units units) {
	if ((strcmp(units,"lb")==0) || (strcmp(units,"lbf")==0)) {
        return (float) 1;
	} else if ((strcmp(units,"klb")==0) || (strcmp(units,"klbf")==0)) {
		return (float) 0.001;
	} else if (strcmp(units,"N")==0) {
        return (float) 4.44822161526;
	} else if (strcmp(units,"kN")==0) {
        return (float) 0.00444822161526;
	} else if (strcmp(units,"kg")==0) {
        return (float) 0.45359237;
	} else if (strcmp(units,"g")==0) {
        return (float) 453.59237;
	} else return (float) 0;
} // ForceConv()
float TorqueConv(Units units) {
	if ((strcmp(units,"in-lb")==0) || (strcmp(units,"in-lbf")==0) || (strcmp(units,"lb-in")==0) || (strcmp(units,"lbf-in")==0)) {
		return (float) 1;
	} else if ((strcmp(units,"ft-lb")==0) || (strcmp(units,"lb-ft")==0) || (strcmp(units,"ft-lbf")==0) || (strcmp(units,"lbf-ft")==0)) {
        return (float) 0.08333333333;
	} else if ((strcmp(units,"N-m")==0) || (strcmp(units,"Nm")==0)) {
        return (float) 0.112984829028;
	} else if ((strcmp(units,"N-mm")==0) || (strcmp(units,"Nmm")==0)) {
        return (float) 112.984829028;
	} else if ((strcmp(units,"kg-cm")==0) || (strcmp(units,"kgcm")==0)) {
        return (float) 1.1521246198;
	}else if ( (strcmp(units, "kN-m" ) == 0 ) || ( strcmp( units, "kNm" ) == 0 ) ){
	      return (float) 0.000112984829028;	
	} else return (float) 0;
} // TorqueConv()
float DistConv(Units units) {
	if (strcmp(units,"in")==0) {
        return (float) 1;
	} else if (strcmp(units,"m")==0) {
        return (float) 0.0254;
	} else if (strcmp(units,"cm")==0) {
        return (float) 2.54;
	} else if (strcmp(units,"mm")==0) {
        return (float) 25.4;
	} else if (strcmp(units,"ft")==0) {
        return (float) 0.08333333333;
	} else return (float) 0;
} // DistConv()

float AngleConv(Units u) {
	if ((strcmp(u,"deg")==0) || (strcmp(u,"degrees")==0) || (strcmp(u,"degree")==0)) {
		return (float) 1;
	} else if ((strcmp(u,"rad")==0) || (strcmp(u,"radians")==0) || (strcmp(u,"radian")==0)) {
		return (float) PI / 180;
	} else return (float) 0;
} // AngleConv()

short ReadAttribute(const DOM_Element *elem, char **attValue, char *attName, BOOL required, char *defaultValue) {
	(*attValue)=DOM_Element_getAttribute(elem,attName);
	if (strcmp(*attValue,"")==0) {
		if (required==TRUE) {
			return 1;
		} else {
			free(*attValue);
			*attValue=ATI_strdup(defaultValue);
		}
	}
	return 0;
} // ReadAttribute()

void Separate(char *ValueList,float results[],unsigned short numValues) {
// parses a space- or tab- delimited string into an array of floats
    unsigned short i;
    unsigned short StartPos, EndPos;
	char *word;
    StartPos=FindText(ValueList,0);    // find start of first value
	for (i=0;i<numValues;i++) {
		EndPos=FindSpace(ValueList,StartPos);    // find end of value
		word=mid(ValueList, StartPos, (unsigned short)(EndPos-StartPos));
        results[i] = (float) atof(word);  // store value in array
		free(word);
        StartPos=FindText(ValueList,EndPos);   // find start of next value
	}
} // Separate()
unsigned short FindText(char *str, unsigned short StartPos) {
// Finds the first character in a string after StartPos that is not a tab or space
	int i;
	for(i=StartPos;(str[i]=='\t') || (str[i]==' ');i++)
		if (str[i]=='\0')
			return 0;
	return i;
} // FindText()
unsigned short FindSpace(char *str, unsigned short StartPos) {
// Finds the first tab or space in a string after StartPos
	int i;
	for(i=StartPos;(str[i]!='\t') && (str[i]!=' ') && (str[i]!='\0');i++)
		;
	return i;
} // FindSpace()
char *mid(char *instr,unsigned short startpos, unsigned short length) {
// Returns a string containing a subset of a larger string
	char *p;
	unsigned short i;
	p=calloc(length+1,sizeof(char));
	for(i=0;i<length;i++)
		p[i]=instr[i+startpos];
	p[length]='\0';	// terminate string
	return p;
} // mid()
