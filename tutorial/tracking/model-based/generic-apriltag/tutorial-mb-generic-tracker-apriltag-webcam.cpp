//! \example tutorial-mb-generic-tracker-apriltag-webcam.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// #undef VISP_HAVE_V4L2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_MODULE_MBT) && \
  (defined(VISP_HAVE_V4L2) || \
   ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
   ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
//! [Macro defined]

#include <fstream>
#include <ios>

#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

typedef enum { state_detection, state_tracking, state_quit } state_t;

// Creates a cube.cao file in your current directory
// cubeEdgeSize : size of cube edges in meters
void createCaoFile(double cubeEdgeSize)
{
  std::ofstream fileStream;
  fileStream.open("cube.cao", std::ofstream::out | std::ofstream::trunc);
  fileStream << "V1\n";
  fileStream << "# 3D Points\n";
  fileStream << "8                  # Number of points\n";
  fileStream << cubeEdgeSize / 2 << " " << cubeEdgeSize / 2 << " " << 0 << "    # Point 0: (X, Y, Z)\n";
  fileStream << cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << 0 << "    # Point 1\n";
  fileStream << -cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << 0 << "    # Point 2\n";
  fileStream << -cubeEdgeSize / 2 << " " << cubeEdgeSize / 2 << " " << 0 << "    # Point 3\n";
  fileStream << -cubeEdgeSize / 2 << " " << cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 4\n";
  fileStream << -cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 5\n";
  fileStream << cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 6\n";
  fileStream << cubeEdgeSize / 2 << " " << cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 7\n";
  fileStream << "# 3D Lines\n";
  fileStream << "0                  # Number of lines\n";
  fileStream << "# Faces from 3D lines\n";
  fileStream << "0                  # Number of faces\n";
  fileStream << "# Faces from 3D points\n";
  fileStream << "6                  # Number of faces\n";
  fileStream << "4 0 3 2 1          # Face 0: [number of points] [index of the 3D points]...\n";
  fileStream << "4 1 2 5 6\n";
  fileStream << "4 4 7 6 5\n";
  fileStream << "4 0 7 4 3\n";
  fileStream << "4 5 2 3 4\n";
  fileStream << "4 0 1 6 7          # Face 5\n";
  fileStream << "# 3D cylinders\n";
  fileStream << "0                  # Number of cylinders\n";
  fileStream << "# 3D circles\n";
  fileStream << "0                  # Number of circles\n";
  fileStream.close();
}

state_t detectAprilTag(const vpImage<unsigned char> &I, vpDetectorAprilTag &detector, double tagSize,
                       const vpCameraParameters &cam, vpHomogeneousMatrix &cMo)
{
  std::vector<vpHomogeneousMatrix> cMo_vec;

  // Detection
  bool ret = detector.detect(I, tagSize, cam, cMo_vec);

  // Display camera pose
  for (size_t i = 0; i < cMo_vec.size(); i++) {
    vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
  }

  vpDisplay::displayText(I, 40, 20, "State: waiting tag detection", vpColor::red);

  if (ret && detector.getNbObjects() > 0) { // if tag detected, we pick the first one
    cMo = cMo_vec[0];
    return state_tracking;
  }

  return state_detection;
}

state_t track(const vpImage<unsigned char> &I, vpMbGenericTracker &tracker, double projection_error_threshold,
              vpHomogeneousMatrix &cMo)
{
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);

  // Track the object
  try {
    tracker.track(I);
  }
  catch (...) {
    return state_detection;
  }

  tracker.getPose(cMo);

  // Detect tracking error
  double projection_error = tracker.computeCurrentProjectionError(I, cMo, cam);
  if (projection_error > projection_error_threshold) {
    return state_detection;
  }

  // Display
  tracker.display(I, cMo, cam, vpColor::red, 2);
  vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
  vpDisplay::displayText(I, 40, 20, "State: tracking in progress", vpColor::red);
  {
    std::stringstream ss;
    ss << "Features: edges " << tracker.getNbFeaturesEdge() << ", klt " << tracker.getNbFeaturesKlt();
    vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::red);
  }

  return state_tracking;
}

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--device <id>]"
    << " [--tag-size <size>]"
    << " [--tag-family <family>]"
    << " [--tag-quad-decimate <factor>]"
    << " [--tag-n-threads <number>]"
#if defined(VISP_HAVE_PUGIXML)
    << " [--intrinsic <xmlfile>]"
    << " [--camera-name <name>]"
#endif
#if defined(VISP_HAVE_DISPLAY)
    << " [--display-off]"
#endif
    << " [--cube-size <size]"
    << " [--use-texture]"
    << " [--projection-error-threshold <threshold>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Live execution on images acquired by a webcam of the generic model-based tracker" << std::endl
    << "  The considered object is a cube to which an Apriltag is attached on one of its" << std::endl
    << "  faces. Once detected, the pose of the Apriltag is used to initialise the tracker." << std::endl
    << "  The Apriltag must be centred on a face of the cube. If the tracker fails, the " << std::endl
    << "  tag is used to reset the tracker." << std::endl
    << std::endl
    << "  --device <id>" << std::endl
    << "    Camera id." << std::endl
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
    << "    Default: 0 (36h11)" << std::endl
    << std::endl
    << "  --tag-quad-decimate <factor>" << std::endl
    << "    Decimation factor used to detect a tag. " << std::endl
    << "    Default: 1" << std::endl
    << std::endl
    << "  --tag-n-threads <number>" << std::endl
    << "    Number of threads used to detect a tag." << std::endl
    << "    Default: 1" << std::endl
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
    << "  --display-off" << std::endl
    << "    Flag used to turn display off." << std::endl
    << "    Default: enabled" << std::endl
    << std::endl
#endif
    << "  --cube-size <size>" << std::endl
    << "    Cube size in meter." << std::endl
    << "    Default: 0.125" << std::endl
    << std::endl
#if defined(VISP_HAVE_OPENCV)
    << "  --use-texture" << std::endl
    << "    Flag to enable usage of keypoint features." << std::endl
    << "    Default: disabled" << std::endl
    << std::endl
#endif
    << "  --projection-error-threshold <threshold>" << std::endl
    << "    Threshold in the range [0:90] deg used to restart the tracker when the projection"
    << "    error is below this threshold." << std::endl
    << "    Default: 40" << std::endl
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
  int opt_device = 0;
  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  double opt_tag_size = 0.08;
  float opt_quad_decimate = 1.0;
  int opt_nthreads = 1;
  std::string opt_intrinsic_file = "";
  std::string opt_camera_name = "";
  double opt_cube_size = 0.125; // 12.5cm by default
#ifdef VISP_HAVE_OPENCV
  bool opt_use_texture = false;
#endif
  double opt_projection_error_threshold = 40.;

#if !(defined(VISP_HAVE_DISPLAY))
  bool opt_display_off = true;
#else
  bool opt_display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      opt_tag_family = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--tag-quad-decimate" && i + 1 < argc) {
      opt_quad_decimate = (float)atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-n-threads" && i + 1 < argc) {
      opt_nthreads = atoi(argv[++i]);
    }
#if defined(VISP_HAVE_PUGIXML)
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[++i]);
    }
#endif
#if defined(VISP_HAVE_DISPLAY)
    else if (std::string(argv[i]) == "--display-off") {
      opt_display_off = true;
    }
#endif
    else if (std::string(argv[i]) == "--cube-size" && i + 1 < argc) {
      opt_cube_size = atof(argv[++i]);
    }
#ifdef VISP_HAVE_OPENCV
    else if (std::string(argv[i]) == "--use-texture") {
      opt_use_texture = true;
    }
#endif
    else if (std::string(argv[i]) == "--projection-error-threshold" && i + 1 < argc) {
      opt_projection_error_threshold = atof(argv[++i]);
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

  createCaoFile(opt_cube_size);

  vpCameraParameters cam;
  bool camIsInit = false;
#if defined(VISP_HAVE_PUGIXML)
  vpXmlParserCamera parser;
  if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
    parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    camIsInit = true;
  }
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif

  try {
    vpImage<unsigned char> I;

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.open(I);
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
    //! [Construct grabber]
    if (!camIsInit) {
      cam.initPersProjWithoutDistortion(600, 600, I.getWidth() / 2., I.getHeight() / 2.);
    }

    std::cout << "Cube size: " << opt_cube_size << std::endl;
    std::cout << "AprilTag size: " << opt_tag_size << std::endl;
    std::cout << "AprilTag family: " << opt_tag_family << std::endl;
    std::cout << "Camera parameters:\n" << cam << std::endl;
    std::cout << "Detection: " << std::endl;
    std::cout << "  Quad decimate: " << opt_quad_decimate << std::endl;
    std::cout << "  Threads number: " << opt_nthreads << std::endl;
    std::cout << "Tracker: " << std::endl;
    std::cout << "  Use edges  : 1" << std::endl;
    std::cout << "  Use texture: "
#ifdef VISP_HAVE_OPENCV
      << opt_use_texture << std::endl;
#else
      << " na" << std::endl;
#endif
    std::cout << "  Projection error: " << opt_projection_error_threshold << std::endl;

    // Construct display
    if (!opt_display_off) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      display = vpDisplayFactory::createDisplay(I);
#else
      display = vpDisplayFactory::allocateDisplay(I);
#endif
    }

    // Initialize AprilTag detector
    vpDetectorAprilTag detector(opt_tag_family);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    detector.setAprilTagNbThreads(opt_nthreads);

    // Prepare MBT
    vpMbGenericTracker tracker;
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    if (opt_use_texture)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    else
#endif
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
    // edges
    vpMe me;
    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(12);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20);
    me.setMu1(0.5);
    me.setMu2(0.5);
    me.setSampleStep(4);
    tracker.setMovingEdge(me);

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    if (opt_use_texture) {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      tracker.setKltOpencv(klt_settings);
      tracker.setKltMaskBorder(5);
    }
#endif

    // camera calibration params
    tracker.setCameraParameters(cam);
    // model definition
    tracker.loadModel("cube.cao");
    tracker.setDisplayFeatures(true);
    tracker.setAngleAppear(vpMath::rad(70));
    tracker.setAngleDisappear(vpMath::rad(80));

    vpHomogeneousMatrix cMo;
    state_t state = state_detection;

    // wait for a tag detection
    while (state != state_quit) {

#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif

      vpDisplay::display(I);

      if (state == state_detection) {
        state = detectAprilTag(I, detector, opt_tag_size, cam, cMo);

        // Initialize the tracker with the result of the detection
        if (state == state_tracking) {
          //! [Init]
          tracker.initFromPose(I, cMo);
          //! [Init]
        }
      }

      if (state == state_tracking) {
        state = track(I, tracker, opt_projection_error_threshold, cMo);
      }

      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I, false)) { // exit
        state = state_quit;
      }

      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (!opt_display_off)
    delete display;
#endif
  return EXIT_SUCCESS;
}

#else

int main()
{
#if !defined(VISP_HAVE_APRILTAG)
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  std::cout << "ViSP is not build with v4l2 or OpenCV support" << std::endl;
#else
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
#endif

  return EXIT_SUCCESS;
}

#endif
