/*! \example tutorial-pose-from-points-live.cpp */
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
// #undef VISP_HAVE_V4L2
// #undef VISP_HAVE_DC1394
// #undef VISP_HAVE_CMU1394
// #undef VISP_HAVE_FLYCAPTURE
// #undef VISP_HAVE_REALSENSE2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML) && \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || \
     defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)) || defined(VISP_HAVE_OPENCV) && \
     (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
      ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))

#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

#include "pose_helper.h"

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--camera-device <id>]"
#if defined(VISP_HAVE_PUGIXML)
    << " [--intrinsic <xmlfile>]"
    << " [--camera-name <name>]"
#endif
    << " [--square-width <width>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute the pose of a square from its 4 corners." << std::endl
    << std::endl
    << "  --camera-device <id>" << std::endl
    << "    Camera device id." << std::endl
    << "    Default: 0" << std::endl
    << std::endl
#if defined(VISP_HAVE_PUGIXML)
    << "  --intrinsic <xmlfile>" << std::endl
    << "    Camera intrinsic parameters file in xml format." << std::endl
    << "    Default: empty" << std::endl
    << std::endl
    << "  --camera-name <name>" << std::endl
    << "    Camera name in the intrinsic parameters file in xml format." << std::endl
    << "    Default: empty" << std::endl
    << std::endl
#endif
    << "  --square-width <width>" << std::endl
    << "    Square width in meter." << std::endl
    << "    Default: 0.12" << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout
    << std::endl
    << "Example using default camera parameters and square size:\n"
    << "  " << argv[0] << "\n"
    << std::endl
    << "Example fully tuned for a 0.1m x 0.1m square:\n"
    << "  " << argv[0] << " --intrinsic camera.xml --camera-name Camera --square-width 0.1\n"
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
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
    std::string opt_intrinsic_file; // xml file obtained from camera calibration
    std::string opt_camera_name;    // corresponding camera name in the xml calibration file
    double opt_square_width = 0.12;
    int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--camera-device" && i + 1 < argc) {
        opt_device = atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
        opt_intrinsic_file = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--square-width" && i + 1 < argc) {
        opt_device = atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    vpImage<unsigned char> I;
    vpCameraParameters cam;

    //! [Grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.open(I);
    cam.initPersProjWithoutDistortion(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
#elif defined(VISP_HAVE_DC1394)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use DC1394 grabber" << std::endl;
    vp1394TwoGrabber g;
    g.open(I);
    cam.initPersProjWithoutDistortion(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
#elif defined(VISP_HAVE_CMU1394)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use CMU1394 grabber" << std::endl;
    vp1394CMUGrabber g;
    g.open(I);
    cam.initPersProjWithoutDistortion(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
#elif defined(VISP_HAVE_FLYCAPTURE)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use FlyCapture grabber" << std::endl;
    vpFlyCaptureGrabber g;
    g.open(I);
    cam.initPersProjWithoutDistortion(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
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
    cam.initPersProjWithoutDistortion(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
#endif
    //! [Grabber]

#if defined(VISP_HAVE_PUGIXML)
    // Parameters of our camera
    vpXmlParserCamera parser;
    if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
      std::cout << "Intrinsic file: " << opt_intrinsic_file << std::endl;
      std::cout << "Camera name   : " << opt_camera_name << std::endl;
      if (parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) ==
          vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Succeed to read camera parameters from xml file" << std::endl;
      }
      else {
        std::cout << "Unable to read camera parameters from xml file" << std::endl;
      }
    }
#endif

    std::cout << "Square width  : " << opt_square_width << std::endl;
    std::cout << cam << std::endl;

    // The pose container
    vpHomogeneousMatrix cMo;

    std::vector<vpDot2> dot(4);
    std::vector<vpPoint> point;   // 3D coordinates of the points
    std::vector<vpImagePoint> ip; // 2D coordinates of the points in pixels
    double L = opt_square_width / 2.;
    point.push_back(vpPoint(-L, -L, 0));
    point.push_back(vpPoint(L, -L, 0));
    point.push_back(vpPoint(L, L, 0));
    point.push_back(vpPoint(-L, L, 0));

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif

    bool quit = false;
    bool apply_cv = false; // apply computer vision
    bool init_cv = true;   // initialize tracking and pose computation

    while (!quit) {
      double t_begin = vpTime::measureTimeMs();
      // Image Acquisition
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || \
    defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      if (apply_cv) {
        try {
          ip = track(I, dot, init_cv);
          computePose(point, ip, cam, init_cv, cMo);
          vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 3);
          if (init_cv)
            init_cv = false; // turn off the computer vision initialisation specific stuff

          { // Display estimated pose in [m] and [deg]
            vpPoseVector pose(cMo);
            std::stringstream ss;
            ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
            vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::red);
            ss.str(""); // erase ss
            ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " "
              << vpMath::deg(pose[5]) << " [deg]";
            vpDisplay::displayText(I, 80, 20, ss.str(), vpColor::red);
          }
        }
        catch (...) {
          std::cout << "Computer vision failure." << std::endl;
          apply_cv = false;
          init_cv = true;
        }
      }
      vpDisplay::displayText(I, 20, 20, "Right click: quit", vpColor::red);
      if (apply_cv) {
        vpDisplay::displayText(I, 40, 20, "Computer vision in progress...", vpColor::red);
      }
      else {
        vpDisplay::displayText(I, 40, 20, "Left click : start", vpColor::red);
      }
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        }
        else if (button == vpMouseButton::button1) {
          apply_cv = true;
        }
      }
      {
        std::stringstream ss;
        ss << "Time: " << vpTime::measureTimeMs() - t_begin << " ms";
        vpDisplay::displayText(I, 20, I.getWidth() - 100, ss.str(), vpColor::red);
      }
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
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
  std::cout << "There are missing 3rd parties to run this tutorial" << std::endl;
}

#endif
