

#ifndef vpDirectShowGrabber_hh
#define vpDirectShowGrabber_hh

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) ) 

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpFrameGrabber.h>

class vpDirectShowGrabberImpl;

/*
	This class' goal is to totally hide the implementation of the framegrabber from
	the rest of the application. This is necessary to avoid conflicts between dshow's
	Qedit.h and Direct3D's includes.
	
*/

/*!
	\class vpDirectShowGrabber
	\brief class for windows direct show devices

	\ingroup libdevice
*/

class VISP_EXPORT vpDirectShowGrabber : public vpFrameGrabber
{
	vpDirectShowGrabberImpl * grabber;

	public:
		int getRows();
		int getCols();
		
		vpDirectShowGrabber();
		~vpDirectShowGrabber();

		void open(vpImage<unsigned char> &I);
		void open(vpImage<vpRGBa> &I);

		void acquire(vpImage<unsigned char> &I);
		void acquire(vpImage<vpRGBa> &I);

		void close();
			
		//get the number of capture devices
		int getDeviceNumber();

		//change the capture device
		bool setInput(unsigned int n);

		//displays a list of available devices
		void displayDevices();

};
#endif
#endif
