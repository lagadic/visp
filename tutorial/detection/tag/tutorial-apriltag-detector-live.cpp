//! \example tutorial-apriltag-detector-live.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// If openCV available, priority to OpenCV capture, otherwise the user has to modify the code uncommenting/commenting
// one of the following lines
#if defined(VISP_HAVE_OPENCV) && (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
    ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
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

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && \
  (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || \
    defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2) || defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))))

//! [Macro defined]

#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp>
#endif

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--camera-device <id>]"
    << " [--tag-size <size>]"
    << " [--tag-family <family>]"
    << " [--tag-decision-margin-threshold <threshold>]"
    << " [--tag-hamming-distance-threshold <threshold>]"
    << " [--tag-z-aligned]"
    << " [--tag-quad-decimate <factor>]"
    << " [--tag-n-threads <number>]"
    << " [--tag-pose-method <method>]"
#if defined(VISP_HAVE_PUGIXML)
    << " [--intrinsic <xmlfile>]"
    << " [--camera-name <name>]"
#endif
#if defined(VISP_HAVE_DISPLAY)
    << " [--display-tag]"
    << " [--display-off]"
    << " [--color <id>]"
    << " [--thickness <thickness>"
#endif
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute the pose of an Apriltag in images acquired with a camera." << std::endl
    << std::endl
    << "  --camera-device <id>" << std::endl
    << "    Camera device id." << std::endl
    << "    Default: 0" << std::endl
    << std::endl
    << "  --tag-size <size>" << std::endl
    << "    Apriltag size in [m]." << std::endl
    << "    Default: 0.03" << std::endl
    << std::endl
    << "  --tag-family <family>" << std::endl
    << "    Apriltag family. Supported values are:" << std::endl
    << "       0: TAG_36h11" << std::endl
    << "       1: TAG_36h10 (DEPRECATED)" << std::endl
    << "       2: TAG_36ARTOOLKIT (DEPRECATED)" << std::endl
    << "       3: TAG_25h9" << std::endl
    << "       4: TAG_25h7 (DEPRECATED)" << std::endl
    << "       5: TAG_16h5" << std::endl
    << "       6: TAG_CIRCLE21h7" << std::endl
    << "       7: TAG_CIRCLE49h12" << std::endl
    << "       8: TAG_CUSTOM48h12" << std::endl
    << "       9: TAG_STANDARD41h12" << std::endl
    << "      10: TAG_STANDARD52h13" << std::endl
    << "      11: TAG_ARUCO_4x4_50" << std::endl
    << "      12: TAG_ARUCO_4x4_100" << std::endl
    << "      13: TAG_ARUCO_4x4_250" << std::endl
    << "      14: TAG_ARUCO_4x4_1000" << std::endl
    << "      15: TAG_ARUCO_5x5_50" << std::endl
    << "      16: TAG_ARUCO_5x5_100" << std::endl
    << "      17: TAG_ARUCO_5x5_250" << std::endl
    << "      18: TAG_ARUCO_5x5_1000" << std::endl
    << "      19: TAG_ARUCO_6x6_50" << std::endl
    << "      20: TAG_ARUCO_6x6_100" << std::endl
    << "      21: TAG_ARUCO_6x6_250" << std::endl
    << "      22: TAG_ARUCO_6x6_1000" << std::endl
    << "      23: TAG_ARUCO_MIP_36h12" << std::endl
    << "    Default: 0 (36h11)" << std::endl
    << std::endl
    << "  --tag-decision-margin-threshold <threshold>" << std::endl
    << "    Threshold used to discard low-confident detections. A typical value is " << std::endl
    << "    around 100. The higher this value, the more false positives will be filtered" << std::endl
    << "    out. When this value is set to -1, false positives are not filtered out." << std::endl
    << "    Default: 50" << std::endl
    << std::endl
    << "  --tag-hamming-distance-threshold <threshold>" << std::endl
    << "    Threshold used to discard low-confident detections with corrected bits." << std::endl
    << "    A typical value is between 0 and 3. The lower this value, the more false" << std::endl
    << "    positives will be filtered out." << std::endl
    << "    Default: 0" << std::endl
    << std::endl
    << "  --tag-quad-decimate <factor>" << std::endl
    << "    Decimation factor used to detect a tag. " << std::endl
    << "    Default: 1" << std::endl
    << std::endl
    << "  --tag-n-threads <number>" << std::endl
    << "    Number of threads used to detect a tag." << std::endl
    << "    Default: 1" << std::endl
    << std::endl
    << "  --tag-z-aligned" << std::endl
    << "    When enabled, tag z-axis and camera z-axis are aligned." << std::endl
    << "    Default: false" << std::endl
    << std::endl
    << "  --tag-pose-method <method>" << std::endl
    << "    Algorithm used to compute the tag pose from its 4 corners." << std::endl
    << "    Possible values are:" << std::endl
    << "       0: HOMOGRAPHY" << std::endl
    << "       1: HOMOGRAPHY_VIRTUAL_VS" << std::endl
    << "       2: DEMENTHON_VIRTUAL_VS" << std::endl
    << "       3: LAGRANGE_VIRTUAL_VS" << std::endl
    << "       4: BEST_RESIDUAL_VIRTUAL_VS" << std::endl
    << "       5: HOMOGRAPHY_ORTHOGONAL_ITERATION" << std::endl
    << "    Default: 1 (HOMOGRAPHY_VIRTUAL_VS)" << std::endl
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
#if defined(VISP_HAVE_DISPLAY)
    << "  --display-tag" << std::endl
    << "    Flag used to enable displaying the edges of a tag." << std::endl
    << "    Default: disabled" << std::endl
    << std::endl
    << "  --display-off" << std::endl
    << "    Flag used to turn display off." << std::endl
    << "    Default: enabled" << std::endl
    << std::endl
    << "  --color <id>" << std::endl
    << "    Color id used to display the frame over each tag." << std::endl
    << "    Possible values are:" << std::endl
    << "      -1: R-G-B colors for X, Y, Z axis respectively" << std::endl
    << "       0: all axis in black" << std::endl
    << "       1: all axis in white" << std::endl
    << "       ..." << std::endl
    << "    Default: -1" << std::endl
    << std::endl
    << "  --thickness <thickness>" << std::endl
    << "    Thickness of the drawings in overlay." << std::endl
    << "    Default: 2" << std::endl
    << std::endl
#endif
    << "  --verbose, -v" << std::endl
    << "    Enable verbosity." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
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

  int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device
  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod opt_tag_pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double opt_tag_size = 0.053;
  float opt_tag_quad_decimate = 1.0;
  float opt_tag_decision_margin_threshold = 50;
  float opt_tag_hamming_distance_threshold = 2;
  int opt_tag_nThreads = 1;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool opt_display_tag = false;
  int opt_color_id = -1;
  unsigned int thickness = 2;
  bool opt_tag_z_align_frame = false;
  bool opt_verbose = false;

#if !(defined(VISP_HAVE_DISPLAY))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to display images..." << std::endl;
#else
  bool display_off = false;
#endif

  vpImage<unsigned char> I;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--camera-device" && i + 1 < argc) {
      opt_device = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      opt_tag_family = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-quad-decimate" && i + 1 < argc) {
      opt_tag_quad_decimate = (float)atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-n-threads" && i + 1 < argc) {
      opt_tag_nThreads = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-z-aligned") {
      opt_tag_z_align_frame = true;
    }
    else if (std::string(argv[i]) == "--tag-pose-method" && i + 1 < argc) {
      opt_tag_pose_estimation_method = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-decision-margin-threshold" && i + 1 < argc) {
      opt_tag_decision_margin_threshold = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-hamming-distance-threshold" && i + 1 < argc) {
      opt_tag_hamming_distance_threshold = atoi(argv[++i]);
    }
#if defined(VISP_HAVE_PUGIXML)
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      camera_name = std::string(argv[++i]);
    }
#endif
#if defined(VISP_HAVE_DISPLAY)
    else if (std::string(argv[i]) == "--display-tag") {
      opt_display_tag = true;
    }
    else if (std::string(argv[i]) == "--display-off") {
      display_off = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      opt_color_id = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = static_cast<unsigned int>(atoi(argv[++i]));
    }
#endif
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif

  try {
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
#if defined(VISP_HAVE_PUGIXML)
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty())
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
#endif

//! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.open(I);
#elif defined(VISP_HAVE_DC1394)
    (void)opt_device; // To avoid non used warning
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
#elif defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
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
    //! [Construct grabber]

    std::cout << cam << std::endl;
    std::cout << "Tag detector settings" << std::endl;
    std::cout << "  Tag size [m]              : " << opt_tag_size << std::endl;
    std::cout << "  Tag family                : " << opt_tag_family << std::endl;
    std::cout << "  Quad decimate             : " << opt_tag_quad_decimate << std::endl;
    std::cout << "  Decision margin threshold : " << opt_tag_decision_margin_threshold << std::endl;
    std::cout << "  Hamming distance threshold: " << opt_tag_hamming_distance_threshold << std::endl;
    std::cout << "  Num threads               : " << opt_tag_nThreads << std::endl;
    std::cout << "  Z aligned                 : " << opt_tag_z_align_frame << std::endl;
    std::cout << "  Pose estimation           : " << opt_tag_pose_estimation_method << std::endl;

    if (!display_off) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      display = vpDisplayFactory::createDisplay(I);
#else
      display = vpDisplayFactory::allocateDisplay(I);
#endif
    }

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(opt_tag_family);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(opt_tag_quad_decimate);
    detector.setAprilTagPoseEstimationMethod(opt_tag_pose_estimation_method);
    detector.setAprilTagNbThreads(opt_tag_nThreads);
    detector.setDisplayTag(opt_display_tag, opt_color_id < 0 ? vpColor::none : vpColor::getColor(opt_color_id), thickness);
    detector.setZAlignedWithCameraAxis(opt_tag_z_align_frame);
    detector.setAprilTagDecisionMarginThreshold(opt_tag_decision_margin_threshold);
    detector.setAprilTagHammingDistanceThreshold(opt_tag_hamming_distance_threshold);
    //! [AprilTag detector settings]

    std::vector<double> time_vec;
    bool quit = false;
    while (!quit) {
      //! [Acquisition]
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) ||                              \
    defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      //! [Acquisition]
      if (opt_verbose) {
        std::cout << "-- Process new image --" << std::endl;
      }

      vpDisplay::display(I);

      double t = vpTime::measureTimeMs();
      //! [Detect and compute pose]
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, opt_tag_size, cam, cMo_vec);
      //! [Detect and compute pose]
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

      //! [Display camera pose for each tag]
      for (size_t i = 0; i < cMo_vec.size(); ++i) {
        vpDisplay::displayFrame(I, cMo_vec[i], cam, opt_tag_size / 2, vpColor::none, 3);
      }
      //! [Display camera pose for each tag]

      //! [Display tags]
      std::vector< std::vector<vpImagePoint> > tags_corners = detector.getTagsCorners();
      detector.displayTags(I, tags_corners);
      //! [Display tags]

      //! [Display id for each tag]
      std::vector<int> tags_id = detector.getTagsId();
      for (size_t i = 0; i < tags_id.size(); ++i) {
        std::stringstream ss;
        ss << "id=" << tags_id[i];
        vpDisplay::displayText(I, detector.getCog(i) + vpImagePoint(-10, 10), ss.str(), vpColor::blue);
      }
      //! [Display id for each tag]
      vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }

      if (opt_verbose) {
        std::vector<float> tag_decision_margins = detector.getTagsDecisionMargin();
        std::vector<int> tag_hamming_distances = detector.getTagsHammingDistance();
        for (size_t i = 0; i < tags_id.size(); ++i) {
          std::string message = detector.getMessage(i);
          std::stringstream ss;
          ss << "  Found " << message
            << " with decision margin: " << tag_decision_margins[i]
            << " and hamming distance: " << tag_hamming_distances[i];;
          std::cout << ss.str() << std::endl;
        }
      }
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif

  return EXIT_SUCCESS;
}

#else

int main()
{
#ifndef VISP_HAVE_APRILTAG
  std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#else
  std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, "
    << "Realsense2), configure and build ViSP again to use this example"
    << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif
