#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpV4l2Grabber.h>
main(){
#ifdef VISP_HAVE_V4L2

  vpImage<unsigned char> I; // Grey level image
  
  vpV4l2Grabber g;
  g.setInput(2);    // Input 2 on the board
  g.setWidth(768);  // Acquired images are 768 width
  g.setHeight(576); // Acquired images are 576 height
  g.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
  g.open(I);        // Open the grabber
  while(1)
    g.acquire(I);     // Acquire a 768x576 grey image

#endif
}
