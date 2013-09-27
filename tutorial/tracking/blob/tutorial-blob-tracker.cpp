/*!
  \example tutorial-blob-tracker.cpp
  */
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

int main()
{
#if (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394))
  try {
    vpImage<unsigned char> I; // Create a gray level image container

#if defined(VISP_HAVE_DC1394_2)
    vp1394TwoGrabber g(false);
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g;
#endif
    g.open(I);
    g.acquire(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);
    blob.initTracking(I);

    while(1) {
      g.acquire(I); // Acquire an image
      vpDisplay::display(I);
      blob.track(I);
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
