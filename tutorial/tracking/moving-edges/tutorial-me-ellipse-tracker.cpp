//! \example tutorial-me-ellipse-tracker.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>
//! [Undef grabber]
// If openCV available, priority to OpenCV capture, otherwise the user has to modify the code uncommenting/commenting
// one of the following lines
#if ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
    ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
#undef VISP_HAVE_V4L2
#undef VISP_HAVE_DC1394
#undef VISP_HAVE_CMU1394
#undef VISP_HAVE_FLYCAPTURE
#undef VISP_HAVE_REALSENSE2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
#else
// Use the first grabber that is available. Uncomment/comment the following lines to disable usage of a grabber
// #undef VISP_HAVE_V4L2
// #undef VISP_HAVE_DC1394
// #undef VISP_HAVE_CMU1394
// #undef VISP_HAVE_FLYCAPTURE
// #undef VISP_HAVE_REALSENSE2
#undef HAVE_OPENCV_HIGHGUI
#undef HAVE_OPENCV_VIDEOIO
#endif
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

#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoWriter.h>
//! [Header for vpMeEllipse]
#include <visp3/me/vpMeEllipse.h>
//! [Header for vpMeEllipse]

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif

  int opt_me_range = 10;
  int opt_me_sample_step = 5;
  int opt_me_threshold = 20;     // Value in [0 ; 255]
  int opt_track_circle = false;  // By default we will track an ellipse
  int opt_track_arc = false;     // By default we will track a full circle or ellipse
  std::string opt_save_filename; // Saved images filename

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--me-range" && i + 1 < argc) {
      opt_me_range = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--me-sample-step" && i + 1 < argc) {
      opt_me_sample_step = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--me-threshold" && i + 1 < argc) {
      opt_me_threshold = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--track-circle") {
      opt_track_circle = true;
    }
    else if (std::string(argv[i]) == "--track-arc") {
      opt_track_arc = true;
    }
    else if (std::string(argv[i]) == "--save" && i + 1 < argc) {
      opt_save_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--me-range <range>]"
        << " [--me-sample-step <sample step>]"
        << " [--me-threshold <threshold>]"
        << " [--track-arc]"
        << " [--track-circle]"
        << " [--save <filename>]"
        << " [--help] [-h]\n"
        << std::endl;
      return EXIT_SUCCESS;
    }
    else {
      std::cout << "\nError: wrong parameter " << argv[i] << std::endl;
      return EXIT_FAILURE;
    }
  }

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
    int i = 0;
    cv::Mat frame;
    while ((i++ < 20) && !g.read(frame)) {
    }; // warm up camera by skiping unread frames
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

    vpVideoWriter *writer = nullptr;
    vpImage<vpRGBa> O;
    if (!opt_save_filename.empty()) {
      std::string parent = vpIoTools::getParent(opt_save_filename);
      if (!parent.empty()) {
        std::cout << "Create output directory: " << parent << std::endl;
        vpIoTools::makeDirectory(parent);
      }

      writer = new vpVideoWriter();
      writer->setFileName(opt_save_filename);
      writer->open(O);
    }

    vpMe me;
    me.setRange(opt_me_range);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(opt_me_threshold);
    me.setSampleStep(opt_me_sample_step);

    std::cout << "Moving-edges settings" << std::endl;
    me.print();

    std::cout << "\nTracker settings" << std::endl;
    std::cout << " Tracker type....................."
      << (opt_track_arc ? std::string("arc of ") : std::string())
      << (opt_track_circle ? std::string("circle") : std::string("ellipse")) << std::endl;
    std::cout << " Save results....................." << (opt_save_filename.empty() ? std::string("n/a") : opt_save_filename) << std::endl << std::endl;
    //! [me ellipse container]
    vpMeEllipse ellipse;
    ellipse.setMe(&me);
    ellipse.setDisplay(vpMeSite::RANGE_RESULT);
    ellipse.initTracking(I, opt_track_circle, opt_track_arc);
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

      if (!opt_save_filename.empty()) {
        vpDisplay::getImage(I, O);
        writer->saveFrame(O);
      }
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
