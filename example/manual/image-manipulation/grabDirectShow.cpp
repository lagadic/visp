#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpDirectShowGrabber.h>

main(){
#ifdef VISP_HAVE_DIRECTSHOW

  vpImage<unsigned char> I; // Grey level image
  
  vpDirectShowGrabber g; // Create the grabber
  if(g.getDeviceNumber() == 0) //test if a camera is connected
  {
    g.close();
    exit(0);
  }
  
  g.open(); // Initialize the grabber
  
  setImageSize(640,480); // If the camera supports 640x480 image size
  setFramerate(30); // If the camera supports 30fps framerate
  
  while(1)
    g.acquire(I); // Acquire an image

#endif
}
