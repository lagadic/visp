/*! \example tutorial-grabber-1394.cpp */
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

int main()
{
#ifdef VISP_HAVE_DC1394_2
  try {
    vpImage<unsigned char> I; // Create a gray level image container
    bool reset = true; // Enable bus reset during construction (default)
    vp1394TwoGrabber g(reset); // Create a grabber based on libdc1394-2.x third party lib

    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
