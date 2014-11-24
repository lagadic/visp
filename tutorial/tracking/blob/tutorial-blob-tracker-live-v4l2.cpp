/*!
  \example tutorial-blob-tracker-live-v4l2.cpp
  */
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

int main()
{
#if (defined(VISP_HAVE_V4L2) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)))
  vpImage<unsigned char> I; // Create a gray level image container

  vpV4l2Grabber g;

  g.open(I);
  g.acquire(I);

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK d(I, 0, 0, "Camera view");
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
