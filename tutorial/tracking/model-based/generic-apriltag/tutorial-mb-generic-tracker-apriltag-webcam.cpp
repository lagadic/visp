//! \example tutorial-mb-generic-tracker-apriltag-webcam.cpp
#include <fstream>
#include <ios>
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
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

#if defined(VISP_HAVE_APRILTAG)
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
#endif // #if defined(VISP_HAVE_APRILTAG)

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

int main(int argc, const char **argv)
{
  //! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_V4L2) || defined(HAVE_OPENCV_VIDEOIO)) &&  defined(VISP_HAVE_MODULE_MBT)
  //! [Macro defined]

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

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
#else
  bool display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      opt_tag_size = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      opt_quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      opt_nthreads = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
    }
    else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      opt_tag_family = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--cube_size" && i + 1 < argc) {
      opt_cube_size = atof(argv[i + 1]);
#ifdef VISP_HAVE_OPENCV
    }
    else if (std::string(argv[i]) == "--texture") {
      opt_use_texture = true;
#endif
    }
    else if (std::string(argv[i]) == "--projection_error" && i + 1 < argc) {
      opt_projection_error_threshold = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <camera id>] [--cube_size <size in m>] [--tag_size <size in m>]"
        " [--quad_decimate <decimation>] [--nthreads <nb>]"
        " [--intrinsic <xml intrinsic file>] [--camera_name <camera name in xml file>]"
        " [--tag_family <0: TAG_36h11, 1: TAG_36h10, 2: TAG_36ARTOOLKIT, "
        " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5>]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off]";
#endif
      std::cout << " [--texture] [--projection_error <30 - 100>] [--help]" << std::endl;
      return EXIT_SUCCESS;
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

  try {
    vpImage<unsigned char> I;

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use device " << device.str() << " (v4l2 grabber)" << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
    std::cout << "Use device " << opt_device << " (OpenCV grabber)" << std::endl;
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
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
    vpDisplay *d = nullptr;
    if (!display_off) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
      d = new vpDisplayOpenCV(I);
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
#elif defined(HAVE_OPENCV_VIDEOIO)
      cap >> frame;
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

    if (!display_off)
      delete d;
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  std::cout << "ViSP is not build with v4l2 or OpenCV support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
#endif
  return EXIT_SUCCESS;
}
