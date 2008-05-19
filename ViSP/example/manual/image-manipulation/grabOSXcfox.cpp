#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpOSXcfoxGrabber.h>

main(){
#ifdef VISP_HAVE_CFOX

  vpImage<unsigned char> I; // Grey level image
  
  vpOSXcfoxGrabber g;
  g.setInput(2);    // Input 2 on the board
  g.setScale(2);    // Use half-size images
  g.open(I);        // Open the grabber
  while(1)
    g.acquire(I);     // Acquire an image

#endif
}
