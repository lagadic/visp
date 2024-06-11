//! \example tutorial-me-line-tracker.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
//! [camera headers]
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>
//! [camera headers]
#endif
//! [display headers]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [display headers]
//! [me line headers]
#include <visp3/me/vpMeLine.h>
//! [me line headers]

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
    //! [image container]
    vpImage<unsigned char> I;
    //! [image container]

    //! [grabber container]
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
    //! [grabber container]
    //! [first image acquisition]
#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_CMU1394)
    g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [first image acquisition]

    //! [display container]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, 0, 0, "Camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    //! [display container]
    //! [display image]
    vpDisplay::display(I);
    vpDisplay::flush(I);
    //! [display image]

    //! [me container]
    vpMe me;
    me.setRange(25);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20);
    me.setSampleStep(10);
    //! [me container]

    //! [me line container]
    vpMeLine line;
    line.setMe(&me);
    line.setDisplay(vpMeSite::RANGE_RESULT);
    line.initTracking(I);
    //! [me line container]

    //! [loop]
    while (1) {
#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_CMU1394)
      g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      line.track(I);
      line.display(I, vpColor::red);
      vpDisplay::flush(I);
    }
    //! [loop]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
