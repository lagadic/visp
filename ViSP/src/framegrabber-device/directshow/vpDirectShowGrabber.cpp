
#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_DIRECTSHOW) )

#include <visp/vpDirectShowGrabber.h>
#include <visp/vpDirectShowGrabberImpl.h>

/*!
	Returns the number of rows in the grabbed image
*/
unsigned int vpDirectShowGrabber::getHeight(){ return grabber->getHeight(); }

/*!
	Returns the number of colunms in the grabbed image
*/
unsigned int vpDirectShowGrabber::getWidth(){ return grabber->getWidth(); }


/*!
	Constructor.
	Initializes COM.
*/
vpDirectShowGrabber::vpDirectShowGrabber()
{
	grabber = new vpDirectShowGrabberImpl();
}

/*!
	Destructor
*/
vpDirectShowGrabber::~vpDirectShowGrabber()
{
	delete grabber;
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<unsigned char> &I)
{
	grabber->open(I);
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<vpRGBa> &I)
{
	grabber->open(I);
}


/*!
	Grabs a grayscale image from the video stream.
	Uses a semaphore to synchronize with the framegrabber callback.

	\param I The image to fill with the grabbed frame

	\exception otherError Can't grab the frame
*/
void vpDirectShowGrabber::acquire(vpImage<unsigned char> &I)
{
	grabber->acquire(I);
}

/*!
	Grabs a rgba image from the video stream.
	Uses a semaphore to synchronize with the framegrabber callback.

	\param I The image to fill with the grabbed frame

	\exception otherError Can't grab the frame
*/
void vpDirectShowGrabber::acquire(vpImage<vpRGBa> &I)
{
	grabber->acquire(I);
}


/*!
	Stops the framegrabber
*/
void vpDirectShowGrabber::close() { grabber->close(); }

/*!
	Gets the number of capture devices
*/
int vpDirectShowGrabber::getDeviceNumber()
{
	return grabber->getDeviceNumber();
}

/*!
	Change the input device of the grabber
	\param n number of the device to use

	\return true was the change successful
*/
bool vpDirectShowGrabber::setInput(unsigned int n)
{
	return grabber->setInput(n);
}

/*!
	Displays the list of devices on the standard output
*/
void vpDirectShowGrabber::displayDevices()
{
	grabber->displayDevices();
}

#endif

