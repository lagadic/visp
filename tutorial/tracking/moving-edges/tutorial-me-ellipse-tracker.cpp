//! \example tutorial-me-ellipse-tracker.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/me/vpMeEllipse.h>

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
#endif

int main()
{
#if (defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_V4L2) || defined(HAVE_OPENCV_VIDEOIO))
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    vpImage<unsigned char> I;

#if defined(VISP_HAVE_DC1394)
    vp1394TwoGrabber g(false);
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g;
#elif defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
#elif defined(HAVE_OPENCV_VIDEOIO)
    cv::VideoCapture g(0); // open the default camera
    if (!g.isOpened()) {   // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
#endif

#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_CMU1394)
    g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, 0, 0, "Camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpMe me;
    me.setRange(25);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20);
    me.setSampleStep(10);

    //! [me ellipse container]
    vpMeEllipse ellipse;
    ellipse.setMe(&me);
    ellipse.setDisplay(vpMeSite::RANGE_RESULT);
    ellipse.initTracking(I);
    //! [me ellipse container]

    while (1) {
#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_CMU1394)
      g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      ellipse.track(I);
      ellipse.display(I, vpColor::red);
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
