//! \example tutorial-grabber-1394.cpp
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImage.h>

int main()
{
#ifdef VISP_HAVE_DC1394
  try {
    vpImage<unsigned char> I; // Create a gray level image container
    bool reset = true; // Enable bus reset during construction (default)
    //! [vp1394TwoGrabber construction]
    vp1394TwoGrabber g(reset); // Create a grabber based on libdc1394-2.x third party lib
    //! [vp1394TwoGrabber construction]

    //! [vp1394TwoGrabber settings]
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    //! [vp1394TwoGrabber settings]
    //! [vp1394TwoGrabber open]
    g.open(I);
    //! [vp1394TwoGrabber open]

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      //! [vp1394TwoGrabber acquire]
      g.acquire(I);
      //! [vp1394TwoGrabber acquire]
      vpDisplay::display(I);
      vpDisplay::flush(I);
      //! [vp1394TwoGrabber click to exit]
      if (vpDisplay::getClick(I, false))
        break;
      //! [vp1394TwoGrabber click to exit]
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
