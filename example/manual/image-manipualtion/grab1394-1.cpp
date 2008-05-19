#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vp1394Grabber.h>

int main(){

#ifdef VISP_HAVE_DC1394_1
  unsigned int ncameras; // Number of cameras on the bus
  vp1394Grabber g;
  g.getNumCameras(ncameras);
  vpImage<unsigned char> *I = new vpImage<unsigned char> [ncameras];
  
  // If the first camera supports MODE_640x480_YUV422 video mode
  g.setCamera(0);
  g.setFormat(FORMAT_VGA_NON_COMPRESSED);
  g.setMode(MODE_640x480_YUV422);
  
  // If all cameras support 30 fps acquisition
  for (unsigned int camera=0; camera < ncameras; camera ++) {
    g.setCamera(camera);
    g.setFramerate(FRAMERATE_30);
  }
  
  while(1) {
    for (unsigned int camera=0; camera < ncameras; camera ++) {
      // Acquire successively images from the different cameras
      g.setCamera(camera);
      g.acquire(I[camera]);
    }
  }
  delete [] I;
#endif

}
