#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

/*!
  \example testDirectShow.cpp

  Test frame grabbing capabilities using DirectShow video device.
*/

#ifdef VISP_HAVE_DIRECTSHOW

#include <visp/vpDirectShowGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayGTK.h>


int
main(int argc, char ** argv)
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  "        test frame grabbing with DirectShow            " << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;

  
  vpImage<vpRGBa> I ;
  vpImage<unsigned char> grayI;

  //Creates the grabber
  vpDirectShowGrabber grabber;
  
  //Initializes the grabber
  grabber.open(I);
  
  //Acquires an RGBa image
  grabber.acquire(I);
  
  //Creates a display
  vpDisplayGTK display(I,100,100,"DirectShow Framegrabber Test");

  try{
	
	for(int i=0 ; i<100 ; i++)
	{
		//Displays the grabbed rgba image
		vpDisplay::display(I);

		//Acquires an RGBa image
		grabber.acquire(I);
	}
	
	cout << "Click to quit!" << endl;

    //Waits for user input
    vpDisplay::getClick(I);
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  grabber.close();

}
#else
int
main()
{
  vpTRACE("DirectShow is not available") ;
}
#endif


