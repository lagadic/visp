/*!
  \example tutorial-blob-tracker-live-firewire.cpp
  */
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

int main()
{
#if (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
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
#else
  vpDisplayGDI d(I, 0, 0, "Camera view");
#endif

  vpDot2 blob;
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);

  vpImagePoint germ;
  bool init_done = false;

  while(1) {
    try {
      g.acquire(I); // Acquire an image
      vpDisplay::display(I);

      if (! init_done) {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          blob.initTracking(I, germ);
          init_done = true;
        }
      }
      else {
        blob.track(I);
      }
      vpDisplay::flush(I);
    }
    catch(...) {
      init_done = false;
    }
  }
#endif
}
