
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
void vpDirectShowGrabber::open()
{
	grabber->open();
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<unsigned char> &I)
{
	grabber->open();
}

/*!
	Creates the filter graph and select the first available device.
	\exception initializationError
*/
void vpDirectShowGrabber::open(vpImage<vpRGBa> &I)
{
	grabber->open();
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
	\return the number of capture devices
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
bool vpDirectShowGrabber::setDevice(unsigned int n)
{
	return grabber->setDevice(n);
}

/*!
	Displays the list of devices on the standard output
*/
void vpDirectShowGrabber::displayDevices()
{
	grabber->displayDevices();
}
/*!
	Set image Size
	\param width : Image width
	\param height : Image height

	\return true if successful
*/
bool vpDirectShowGrabber::setImageSize(unsigned int _width,unsigned int _height)
{
	return grabber->setImageSize(_width, _height);
}
/*!
	Set capture framerate
	\param width : Framerate in fps
	
	\return true if successful
*/
bool vpDirectShowGrabber::setFramerate(double _framerate)
{
	return grabber->setFramerate(_framerate);
}
/*!
	Set capture format
	\param _width : Image width in pixel
	\param _height : Image height in pixel
	\param _framerate : Framerate in fps

	\return true if successful
*/
bool vpDirectShowGrabber::setFormat(unsigned int _width,unsigned int _height,double _framerate)
{
	return grabber->setFormat(_width, _height, _framerate);
}
/*
	Get capture format
	\param width : Pointer to a variable that receives the width in pixel
	\param height : Pointer to a variable that receives the height in pixel
	\param framerate : Pointer to a variable that receives the framerate in fps
*/
void vpDirectShowGrabber::getFormat(unsigned int* pWidth,unsigned int* pHeight, double* pFramerate)
{
	grabber->getFormat(pWidth,pHeight,pFramerate);
}

/*!
	Get the available capture formats

	\return true if successful
*/
bool vpDirectShowGrabber::getStreamCapabilities()
{
	return grabber->getStreamCapabilities();
}

/*!
	Set capture MediaType
	\param mediaTypeID : mediaTypeID (available in calling getStreamCapabilities)

	\return true if successful
*/
bool vpDirectShowGrabber::setMediaType(int mediaTypeID)
{
	return grabber->setMediaType(mediaTypeID);
}

/*
	Get current capture MediaType

	\return the current mediaTypeID
*/
int vpDirectShowGrabber::getMediaType()
{
	return grabber->getMediaType();
}


#endif

