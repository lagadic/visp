//! \example tutorial-me-line-tracker.cpp
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
   ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))) && \
  ((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_FEATURES2D)) || \
  ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_3D) && defined(HAVE_OPENCV_FEATURES))

#ifdef VISP_HAVE_MODULE_SENSOR
//! [camera headers]
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
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
  try {
    //! [image container]
    vpImage<unsigned char> I;
    //! [image container]

    //! [grabber container]
    int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device
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

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
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
    //! [grabber container]
    //! [first image acquisition]
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
    g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
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
    bool quit = false;
    while (! quit) {
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "Click to quit", vpColor::red);
      line.track(I);
      line.display(I, vpColor::red);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }
      vpDisplay::flush(I);
    }
    //! [loop]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
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
