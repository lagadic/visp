//! \example tutorial-me-ellipse-tracker.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>
//! [Undef grabber]
// #undef VISP_HAVE_V4L2
// #undef VISP_HAVE_DC1394
// #undef VISP_HAVE_CMU1394
// #undef VISP_HAVE_FLYCAPTURE
// #undef VISP_HAVE_REALSENSE2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || \
   defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2) || \
   ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
   ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))

#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif

#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/me/vpMeEllipse.h>

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif

  try {
    vpImage<unsigned char> I;
    int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device

    //! [Grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.open(I);
#elif defined(VISP_HAVE_DC1394)
    (void)opt_device;         // To avoid non used warning
    std::cout << "Use DC1394 grabber" << std::endl;
    vp1394TwoGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_CMU1394)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use CMU1394 grabber" << std::endl;
    vp1394CMUGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_FLYCAPTURE)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use FlyCapture grabber" << std::endl;
    vpFlyCaptureGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_REALSENSE2)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);
    g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
    std::cout << "Use OpenCV grabber on device " << opt_device << std::endl;
    cv::VideoCapture g(opt_device); // Open the default camera
    if (!g.isOpened()) {            // Check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [Grabber]

#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
    g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

#if defined(VISP_HAVE_DISPLAY)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I, 0, 0, "Camera view");
#else
    display = vpDisplayFactory::allocateDisplay(I, 0, 0, "Camera view");
#endif
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

    bool quit = false;
    while (!quit) {
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "Click to quit", vpColor::red);
      ellipse.track(I);
      ellipse.display(I, vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
}

#else

int main()
{
#if defined(VISP_HAVE_OPENCV)
  std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, "
    << "Realsense2), configure and build ViSP again to use this tutorial."
    << std::endl;
#else
  std::cout << "Install OpenCV 3rd party, configure and build ViSP again to use this tutorial." << std::endl;
#endif
  return EXIT_SUCCESS;
  }
#endif
