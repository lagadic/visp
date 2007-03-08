/****************************************************************************
*
* $Id: grabDirectShowMulti.cpp,v 1.3 2007-03-08 13:35:43 fspindle Exp $
*
* Copyright (C) 1998-2006 Inria. All rights reserved.
*
* This software was developed at:
* IRISA/INRIA Rennes
* Projet Lagadic
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* http://www.irisa.fr/lagadic
*
* This file is part of the ViSP toolkit
*
* This file may be distributed under the terms of the Q Public License
* as defined by Trolltech AS of Norway and appearing in the file
* LICENSE included in the packaging of this file.
*
* Licensees holding valid ViSP Professional Edition licenses may
* use this file in accordance with the ViSP Commercial License
* Agreement provided with the Software.
*
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* Contact visp@irisa.fr if any conditions of this licensing are
* not clear to you.
*
* Description:
* Acquire images using DirectShow (under Windows only) and display it
* using GTK or GDI.
*
* Authors:
* Bruno Renier
* Fabien Spindler
* Anthony Saunier
*
*****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
\file grabDirectShowMulti.cpp

\brief Example of framegrabbing using vpDirectShowGrabber class.

*/

#include <vector>
using namespace std;
#include <iostream>
#include <sstream>


#if defined (VISP_HAVE_DIRECTSHOW)

#include <visp/vpDirectShowGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpParseArgv.h>
#include <visp/vpTime.h>


// List of allowed command line options
//#define GETOPTARGS	"dhn:o:"
#define GETOPTARGS	"c:df:hmn:io:st:?"

#define GRAB_COLOR

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param camera : Active camera identifier.
\param nframes : Number of frames to acquire.
\param opath : Image filename when saving.

*/
void usage(char *name, char *badparam, unsigned int camera, unsigned int &nframes,
		   string &opath)
{
	if (badparam)
		fprintf(stderr, "\nERREUR: Bad parameter [%s]\n", badparam);

	fprintf(stdout, "\n\
Acquire images using DirectShow (under Windows only) and display\n\
it using GTK or the windows GDI if GTK is not available.\n\
For a given camera, mediatype (or video mode) as well as framerate\n\
can be set.\n\
If more than one camera is connected, this example allows also to \n\
acquire images from all the cameras.\n\
\n\
SYNOPSIS\n\
%s [-t <mediatype>] [-f <framerate>] \n\
  [-c <camera id>] [-m] [-n <frames>] [-i] [-s] [-d] \n\
  [-o <filename>] [-h]\n\
	\n\
OPTIONS                                                    Default\n\
  -t [%%u] \n\
     MediaType (or video mode) to set for the active \n\
     camera. Use -s option so see which are the supported \n\
     Mediatypes. You can select the active camera \n\
     using -c option.\n\
\n\
  -f [%%d] \n\
     Framerate to set for the active camera.\n\
     You can select the active camera using -c option.\n",
		name);

	fprintf(stdout, "\n\
  -c [%%u]                                                    %u\n\
     Active camera identifier.\n\
     Zero is for the first camera found on the bus.\n\
\n\
  -m      \n\
     Flag to active multi camera acquisition.       \n\
     You need at least two cameras connected on the bus.\n\
\n\
  -n [%%u]                                                    %u\n\
     Number of frames to acquire.\n\
\n\
  -i      \n\
     Flag to print camera informations.\n\
\n\
  -s      \n\
     Print camera settings capabilities such as MediaType \n\
     and sizes available and exit.\n\
\n\
  -d      \n\
     Flag to turn off image display.\n\
\n\
  -o [%%s] \n\
     Filename for image saving.                     \n\
     Example: -o %s\n\
     The first %%d is for the camera id, %%04d\n\
     is for the image numbering.\n\
\n\
  -h \n\
     Print the help.\n\
\n",
		camera, nframes, opath.c_str());

	exit(0);
}

/*!

Set the program options.

\param argc : Command line number of parameters.
\param argv : Array of command line parameters.
\param multi : Multi camera framegrabbing activation.
\param camera : Active camera identifier.
\param nframes : Number of frames to acquire.

\param verbose_info : Camera informations printing.
\param verbose_settings : Camera settings printing.

\param mediatype_is_set : New mediatype setting.
\param mediatypeID : Mediatype setting.

\param framerate_is_set : New framerate setting.
\param framerate : Framerate setting.

\param colorcoding_is_set : New color coding setting.
\param colorcoding : Color coding setting (usefull only for format 7).

\param display : Display activation.
\param save : Image saving activation.
\param opath : Image filename when saving.

*/

void read_options(int argc, char **argv, bool &multi, unsigned int &camera,
				  unsigned int &nframes, bool &verbose_info,
				  bool &verbose_settings,
				  bool &mediatype_is_set,
				  unsigned int &mediatypeID,
				  bool &framerate_is_set,
				  double &framerate,
				  bool &display, bool &save, string &opath)
{
	char *optarg;
	int	c;
	/*
	* Lecture des options.
	*/

	while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS,&optarg)) > 1) {
		switch (c) {
			case 'c':
				camera = atoi(optarg); break;
			case 'd':
				display = false; break;
			case 'f':
				framerate_is_set = true;
				framerate = atoi(optarg); break;
			case 'i':
				verbose_info = true; break;
			case 'm':
				multi = true; break;
			case 'n':
				nframes = atoi(optarg); break;
			case 'o':
				save = true;
				opath = optarg; break;
			case 's':
				verbose_settings = true; break;
			case 't':
				mediatype_is_set = true;
				mediatypeID = atoi(optarg); break;
			default:
				usage(argv[0], NULL, camera, nframes, opath);
				break;
		}
	}

	if ((c == 1) || (c == -1)) {
	// standalone param or error
	usage(argv[0], NULL, camera, nframes, opath);
	cerr << "ERROR: " << endl;
	cerr << "  Bad argument " << optarg << endl << endl;
	}
}


/*!
\example grabDirectShowMulti.cpp

Example of framegrabbing using vpDirectShowGrabber class.

Grab grey level images using DirectShow frame grabbing capabilities. Display the
images using the GTK or GDI display.
*/
int
main(int argc, char ** argv)
{
	try  {
		unsigned int camera = 0;
		bool multi = false;
		bool verbose_info = false;
		bool verbose_settings = false;
		bool display = true;
		unsigned int nframes = 50;
		bool mediatype_is_set = false;
		unsigned int mediatypeID;
		bool framerate_is_set = false;
		double framerate;
		bool save = false;

#ifdef GRAB_COLOR
		vpImage<vpRGBa> *I;
		string opath = "C:/temp/I%d-%04d.ppm";
#else
		vpImage<unsigned char> *I;
		string opath = "C:/temp/I%d-%04d.pgm";
#endif
#ifdef VISP_HAVE_GTK
		vpDisplayGTK *d;
#else
		vpDisplayGDI *d;
#endif
		read_options(argc, argv, multi, camera, nframes,
			verbose_info, verbose_settings,
			mediatype_is_set, mediatypeID,
			framerate_is_set, framerate,
			display, save, opath);

		// Number of cameras connected on the bus
		vpDirectShowGrabber *g;
		g = new vpDirectShowGrabber();
		unsigned int ncameras = g->getDeviceNumber();
		// Check the consistancy of the options
		if (multi) {
			// ckeck if two cameras are connected
			if (ncameras < 2) {
				cout << "You have only " << ncameras << " camera connected on the bus." << endl;
				cout << "It is not possible to active multi-camera acquisition." << endl;
				cout << "Disable -m command line option, or connect an other " << endl;
				cout << "cameras on the bus." << endl;
				g->close();
				return(0);
			}
		}
		if (camera >= ncameras) {
			cout << "You have only " << ncameras;
			cout << " camera connected on the bus." << endl;
			cout << "It is not possible to select camera " << camera << endl;
			cout << "Check your -c <camera> command line option." << endl;
			g->close();
			return(0);
		}
		if (multi) {
			camera = 0; // to over write a bad option usage
			//reinitialize the grabbers with the right number of devices (one grabber per device)
			delete g;
			g = new vpDirectShowGrabber[ncameras];
			for(unsigned int i=0; i<ncameras ; i++)
			{
				g[i].open();
			}

		}
		else {
			ncameras = 1; // acquisition from only one camera
			delete g;
			g = new vpDirectShowGrabber[1];
			g[0].open();
			g[0].setDevice(camera);

		}

		// allocate an image and display for each camera to consider
#ifdef GRAB_COLOR
		I = new vpImage<vpRGBa> [ncameras];
#else
		I = new vpImage<unsigned char> [ncameras];
#endif
		if (display)

#ifdef VISP_HAVE_GTK
			d = new vpDisplayGTK [ncameras];
#else
			d = new vpDisplayGDI [ncameras];
#endif

		unsigned int width;
		unsigned int height;
		// Display information for each camera
		if (verbose_info || verbose_settings) {

			cout << "----------------------------------------------------------" << endl;
			cout << "---- Device List : " << endl;
			cout << "----------------------------------------------------------" << endl;
			g[0].displayDevices();
			for (unsigned i=0; i < ncameras; i ++) {
				unsigned int c;
				if (multi) c = i;
				else c = camera;

				if (verbose_info)
					g[i].getFormat(width, height, framerate);
					cout << "----------------------------------------------------------"
						<< endl
						<< "---- MediaType and framerate currently used by device " << endl
						<< "---- (or camera) " << c <<  endl
						<< "---- Current MediaType : " << g[i].getMediaType() << endl
						<< "---- Current format : " << width <<" x "<< height <<" at "<< framerate << " fps" << endl
						<< "----------------------------------------------------------" << endl;

				if (verbose_settings) {
					cout << "----------------------------------------------------------"
						   << endl
						   << "---- MediaTypes supported by device (or camera) "
						   << c << endl
						   << "---- One of the MediaType below can be set using " << endl
						   << "---- option -t <mediatype>."
						   << endl
						   << "----------------------------------------------------------"
						   << endl;
					g[i].getStreamCapabilities();
				}

			}
			return 0;
		}

		// If required modify camera settings

		if (mediatype_is_set) {
			g[0].setMediaType(mediatypeID);
		}
		else {
			// get The actual video mode
			mediatypeID = g[0].getMediaType();
		}
		if (framerate_is_set) {
			for(unsigned int i=0; i<ncameras ; i++)
			{
				unsigned int c;
				if (multi) c = i;
				else c = camera;
				cout<<"camera " << c <<endl;
				if (!g[i].setFramerate(framerate))
					cout << "Set Framerate failed !!" <<endl<<endl;
			}
		}

		// Do a first acquisition to initialise the display
		for (unsigned int i=0; i < ncameras; i ++) {
			// Acquire the first image
			g[i].acquire(I[i]);
			unsigned int c;
			if (multi) c = i;
			else c = camera;

			cout << "Image size for camera " << c << " : width: "
				<< I[i].getWidth() << " height: " << I[i].getHeight() << endl;

			if (display) {
				// Initialise the display
				char title[100];
				sprintf(title, "Images captured by camera %u", c );
				d[i].init(I[i], 100+i*50, 100+i*50, title) ;
			}
		}

		// Main loop for single or multi-camera acquisition and display
		cout << "Capture in process..." << endl;

		double tbegin=0, tend=0, tloop=0, ttotal=0;

		ttotal = 0;
		tbegin = vpTime::measureTimeMs();
		for (unsigned i = 0; i < nframes; i++) {
			for (unsigned c = 0; c < ncameras; c++) {
				// Acquire an image
				g[c].acquire(I[c]);
				if (display) {
					// Display the last image acquired
					vpDisplay::display(I[c]);
				}
				if (save) {
					char buf[FILENAME_MAX];
					sprintf(buf, opath.c_str(), c, i);
					string filename(buf);
					cout << "Write: " << filename << endl;
#ifdef GRAB_COLOR
					vpImageIo::writePPM(I[c], filename);
#else
					vpImageIo::writePGM(I[c], filename);
#endif
				}
			}
			tend = vpTime::measureTimeMs();
			tloop = tend - tbegin;
			tbegin = tend;
			cout << "loop time: " << tloop << " ms" << endl;
			ttotal += tloop;
		}

		cout << "Mean loop time: " << ttotal / nframes << " ms" << endl;
		cout << "Mean frequency: " << 1000./(ttotal / nframes) << " fps" << endl;

		// Release the framegrabber
		delete [] g;

		// Free memory
		delete [] I;
		if (display)
			delete [] d;

	}
	catch (...) {
		vpCERROR << "Failure: exit" << endl;
	}

	cout << " the end" << endl;
}
#else
int
main()
{
	vpTRACE("DirectShow grabber capabilities are not available...\n"
		"You should install DirectShow to use this example.") ;
}

#endif

/*
* Local variables:
* c-basic-offset: 2
* End:
*/
