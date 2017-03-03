ATI DAQ F/T C Library (v1.0.6)
-----------------------------------------

The purpose of this C library is to load and parse calibration files for ATI 
DAQ F/T transducers, and perform the necessary calculations to convert raw 
voltages from a data acquisition system into forces and torques.

Neither the library itself or the sample applications provide hardware I/O.
These functions must be provided by your hardware's device drivers.

If you are developing an application for the Windows environment, we 
recommend you use the ATIDAQFT ActiveX Automation Server or the 
ATICombinedDAQFT .NET assembly.


Licensing
---------
All of the software including in this package is distributed under the MIT License:

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.

http://www.opensource.org/licenses/mit-license.html


Library Files
-----------------
The following files should be compiled as part of your project:
ftconfig.c
ftrt.c
dom.c
expatls.c
node.c
stack.c
xmlparse.c
xmlrole.c
xmltok.c

In addition to the above files, the files "events.c", "xmltok_impl.c", and 
"xmltok_ns.c" are also included in this library, but are not meant to be
directly compiled, as they are '#include'd into other library files.

None of these files are intended to be modified.


Sample Applications
----------------------------
Two sample applications are included in the Samples folder.

ftconvert.c - demonstrates configuring a DAQ F/T system and performing 
force/torque calculations; converts a sample set of voltages into forces and 
torques.
calinfo.c - demonstrates retrieving information from the calibration file such
as the transducer's rated loads, calibration date, and other information.


Programming Interface
-------------------------------
The functions supported for public use are:

createCalibration - Loads calibration info for a transducer from a calibration
file
destroyCalibration - Frees memory allocated by a successful call to 
createCalibration
SetToolTransform - Performs a 6-axis translation/rotation on the transducer's 
coordinate system
SetForceUnits - Sets the units of force output
SetTorqueUnits - Sets the units of torque outputEnables or disables 
temperature compensation, if available
Bias - Stores a voltage reading to be subtracted from subsequent readings, 
effectively "zeroing" the transducer output to remove tooling weight, etc.
ConvertToFT - Converts an array of voltages into forces and torques

These functions are documented in ftconfig.h.

You can also read information about the transducer system and calibration from the Calibration and other structures.  These structures should be treated as read-only and should only be manipulated through the functions listed above.  The sample application calinfo.c shows how to retrieve this information.

The general structure of an application should follow the following structure:

1.  Call the createCalibration() method to load a calibration file into 
    memory.
2.  Call the SetForceUnits(), SetTorqueUnits(), SetCalibration(), and 
    SetToolTransform() functions to configure the system.
3.  Use device driver calls to retrieve an array of voltages from your data 
    acquisition system.
4.  Use the Bias() function to store a baseline reading (zero the transducer)
5.  Use the ConvertToFT() function to convert another array of voltages into 
    forces and torques.  The Bias reading will be subtracted automatically.
6.  Call the destroyCalibration() function to free memory allocated for the 
    Calibration structure created in step 1.


Optimizing Your Application
--------------------------------------
The bare minimum calculations for converting voltages into forces and torques 
are in ftrt.c.  To isolate this section of the code, simply compile ftrc.c.  
This file #includes ftrc.h, but none of the other files.  This can be useful, 
for example, in an embedded application where the configuration never changes.
You can write a separate configuration program to run on a PC, which sends the
contents of the Calibration.rt structure to a local copy of the RTCoefs 
structure in the embedded system.


Version History
----------------------
1.0.6	Added support for kN-m torque units, changed reference to ActiveX 
	server to also reference .NET assembly.
1.0.5	Moved some declarations out of .h files into .c files to avoid 
	multiple definition errors.  Added ftsharedrt.h
1.0.4	Added newlines to the ends of several files to avoid warnings when 
	compiling with gcc.  FTconfig.c was using '/t' instead of '\t' for tab
	character, which caused warnings under gcc, but no warnings or errors 
	under Visual C++.
1.0.3	Added comments to ftconvert.c explaining hardware temperature 
	compensation.
1.0.2   Fixed memory leaks
1.0.0	No changes
	Released 2-1-02
0.7.1	Added necessary licensing info to all files
	Beta released 10-5-01
0.7.0	Initial beta release; no known bugs; compiles with no errors or warnings
	in Microsoft Visual Studio.NET
	Beta released 10-2-01