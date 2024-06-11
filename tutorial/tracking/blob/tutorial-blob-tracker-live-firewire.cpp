//! \example tutorial-blob-tracker-live-firewire.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#endif
#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
#endif

int main()
{
#if (defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(HAVE_OPENCV_VIDEOIO)) &&             \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I; // Create a gray level image container

#if defined(VISP_HAVE_DC1394)
  vp1394TwoGrabber g(false);
  g.open(I);
#elif defined(VISP_HAVE_CMU1394)
  vp1394CMUGrabber g;
  g.open(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
  cv::VideoCapture g(0); // open the default camera
  if (!g.isOpened()) {   // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return EXIT_FAILURE;
  }
  cv::Mat frame;
  g >> frame; // get a new frame from camera
  vpImageConvert::convert(frame, I);
#endif

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d(I, 0, 0, "Camera view");
#endif

  //! [Construction]
  vpDot2 blob;
  //! [Construction]
  //! [Setting]
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);
  //! [Setting]

  vpImagePoint germ;
  bool init_done = false;

  while (1) {
    try {
#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394)
      g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      if (!init_done) {
        vpDisplay::displayText(I, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          //! [Init]
          blob.initTracking(I, germ);
          //! [Init]
          init_done = true;
        }
      }
      else {
        //! [Track]
        blob.track(I);
        //! [Track]
      }

      vpDisplay::flush(I);
    }
    catch (...) {
      init_done = false;
    }
}
#endif
}
