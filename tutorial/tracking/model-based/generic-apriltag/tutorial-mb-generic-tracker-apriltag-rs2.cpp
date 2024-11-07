//! \example tutorial-mb-generic-tracker-apriltag-rs2.cpp
#include <fstream>
#include <ios>
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>

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

state_t track(std::map<std::string, const vpImage<unsigned char> *> mapOfImages,
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> mapOfPointclouds,
#else
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds,
  std::map<std::string, unsigned int> mapOfWidths, std::map<std::string, unsigned int> mapOfHeights,
#endif
  const vpImage<unsigned char> &I_gray, const vpImage<unsigned char> &I_depth,
  const vpHomogeneousMatrix &depth_M_color, vpMbGenericTracker &tracker, double projection_error_threshold,
  vpHomogeneousMatrix &cMo)
{
  vpCameraParameters cam_color, cam_depth;
  tracker.getCameraParameters(cam_color, cam_depth);

  // Track the object
  try {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    tracker.track(mapOfImages, mapOfPointclouds);
#else
    tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
#endif
  }
  catch (...) {
    return state_detection;
  }

  tracker.getPose(cMo);

  // Detect tracking error
  double projection_error = tracker.computeCurrentProjectionError(I_gray, cMo, cam_color);
  if (projection_error > projection_error_threshold) {
    return state_detection;
  }

  // Display
  tracker.display(I_gray, I_depth, cMo, depth_M_color * cMo, cam_color, cam_depth, vpColor::red, 3);
  vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
  vpDisplay::displayFrame(I_depth, depth_M_color * cMo, cam_depth, 0.05, vpColor::none, 3);

  return state_tracking;
}

int main(int argc, const char **argv)
{
  //! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_MODULE_MBT)
  //! [Macro defined]

  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  double opt_tag_size = 0.08;
  float opt_quad_decimate = 1.0;
  int opt_nthreads = 1;
  double opt_cube_size = 0.125; // 12.5cm by default
#ifdef VISP_HAVE_OPENCV
  bool opt_use_texture = false;
#endif
  bool opt_use_depth = false;
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
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      opt_quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      opt_nthreads = atoi(argv[i + 1]);
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
    else if (std::string(argv[i]) == "--depth") {
      opt_use_depth = true;
    }
    else if (std::string(argv[i]) == "--projection_error" && i + 1 < argc) {
      opt_projection_error_threshold = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--cube_size <size in m>] [--tag_size <size in m>]"
        " [--quad_decimate <decimation>] [--nthreads <nb>]"
        " [--tag_family <0: TAG_36h11, 1: TAG_36h10, 2: TAG_36ARTOOLKIT, "
        " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5>]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off]";
#endif
      std::cout << " [--texture] [--depth] [--projection_error <30 - 100>] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  createCaoFile(opt_cube_size);

  try {
    //! [Construct grabber]
    vpRealSense2 realsense;
    rs2::config config;
    int width = 640, height = 480, stream_fps = 30;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, stream_fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, stream_fps);
    config.disable_stream(RS2_STREAM_INFRARED);
    realsense.open(config);

    vpCameraParameters cam_color, cam_depth;
    vpHomogeneousMatrix depth_M_color;
    cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    if (opt_use_depth) {
      cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);
      depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
    }

    vpImage<vpRGBa> I_color(height, width);
    vpImage<unsigned char> I_gray(height, width);
    vpImage<unsigned char> I_depth(height, width);
    vpImage<uint16_t> I_depth_raw(height, width);
    std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> mapOfPointclouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
#else
    std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
    std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
    std::vector<vpColVector> pointcloud;
#endif

    std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;

    std::cout << "Cube size: " << opt_cube_size << std::endl;
    std::cout << "AprilTag size: " << opt_tag_size << std::endl;
    std::cout << "AprilTag family: " << opt_tag_family << std::endl;
    std::cout << "Camera parameters:" << std::endl;
    std::cout << "  Color:\n" << cam_color << std::endl;
    if (opt_use_depth)
      std::cout << "  Depth:\n" << cam_depth << std::endl;
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
    std::cout << "  Use depth  : " << opt_use_depth << std::endl;
    std::cout << "  Projection error: " << opt_projection_error_threshold << std::endl;

    // Construct display
    vpDisplay *d_gray = nullptr;
    vpDisplay *d_depth = nullptr;

    if (!display_off) {
#ifdef VISP_HAVE_X11
      d_gray = new vpDisplayX(I_gray, 50, 50, "Color stream");
      if (opt_use_depth)
        d_depth = new vpDisplayX(I_depth, 80 + I_gray.getWidth(), 50, "Depth stream");
#elif defined(VISP_HAVE_GDI)
      d_gray = new vpDisplayGDI(I_gray);
      if (opt_use_depth)
        d_depth = new vpDisplayGDI(I_depth);
#elif defined(HAVE_OPENCV_HIGHGUI)
      d_gray = new vpDisplayOpenCV(I_gray);
      if (opt_use_depth)
        d_depth = new vpDisplayOpenCV(I_depth);
#endif
    }

    // Initialize AprilTag detector
    vpDetectorAprilTag detector(opt_tag_family);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    detector.setAprilTagNbThreads(opt_nthreads);

    // Prepare MBT
    std::vector<int> trackerTypes;
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    if (opt_use_texture)
      trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    else
#endif
      trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER);

    if (opt_use_depth)
      trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

    vpMbGenericTracker tracker(trackerTypes);
    // edges
    vpMe me;
    me.setMaskSize(5);
    me.setMaskNumber(180);
    //! [Range]
    me.setRange(12);
    //! [Range]
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

    if (opt_use_depth) {
      // camera calibration params
      tracker.setCameraParameters(cam_color, cam_depth);
      // model definition
      tracker.loadModel("cube.cao", "cube.cao");
      mapOfCameraTransformations["Camera2"] = depth_M_color;
      tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
      tracker.setAngleAppear(vpMath::rad(70), vpMath::rad(70));
      tracker.setAngleDisappear(vpMath::rad(80), vpMath::rad(80));
    }
    else {
      // camera calibration params
      tracker.setCameraParameters(cam_color);
      // model definition
      tracker.loadModel("cube.cao");
      tracker.setAngleAppear(vpMath::rad(70));
      tracker.setAngleDisappear(vpMath::rad(80));
    }
    tracker.setDisplayFeatures(true);

    vpHomogeneousMatrix cMo;
    state_t state = state_detection;

    // wait for a tag detection
    while (state != state_quit) {
      if (opt_use_depth) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
        realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, pointcloud, nullptr);
#else
        realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointcloud, nullptr,
          nullptr);
#endif
        vpImageConvert::convert(I_color, I_gray);
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        vpDisplay::display(I_gray);
        vpDisplay::display(I_depth);

        mapOfImages["Camera1"] = &I_gray;
        mapOfImages["Camera2"] = &I_depth;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
        mapOfPointclouds["Camera2"] = pointcloud;
#else
        mapOfPointclouds["Camera2"] = &pointcloud;
        mapOfWidths["Camera2"] = width;
        mapOfHeights["Camera2"] = height;
#endif
      }
      else {
        realsense.acquire(I_gray);
        vpDisplay::display(I_gray);
      }

      if (state == state_detection) {
        state = detectAprilTag(I_gray, detector, opt_tag_size, cam_color, cMo);

        // Initialize the tracker with the result of the detection
        if (state == state_tracking) {
          if (opt_use_depth) {
            mapOfCameraPoses["Camera1"] = cMo;
            mapOfCameraPoses["Camera2"] = depth_M_color * cMo;
            tracker.initFromPose(mapOfImages, mapOfCameraPoses);
          }
          else {
            tracker.initFromPose(I_gray, cMo);
          }
        }
      }

      if (state == state_tracking) {
        if (opt_use_depth) {
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
          state = track(mapOfImages, mapOfPointclouds, I_gray, I_depth, depth_M_color, tracker,
            opt_projection_error_threshold, cMo);
#else
          state = track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights, I_gray, I_depth, depth_M_color,
            tracker, opt_projection_error_threshold, cMo);
#endif
        }
        else {
          state = track(I_gray, tracker, opt_projection_error_threshold, cMo);
        }
        {
          std::stringstream ss;
          ss << "Features: edges " << tracker.getNbFeaturesEdge() << ", klt " << tracker.getNbFeaturesKlt()
            << ", depth " << tracker.getNbFeaturesDepthDense();
          vpDisplay::displayText(I_gray, I_gray.getHeight() - 30, 20, ss.str(), vpColor::red);
        }
      }

      vpDisplay::displayText(I_gray, 20, 20, "Click to quit...", vpColor::red);
      if (vpDisplay::getClick(I_gray, false)) { // exit
        state = state_quit;
      }
      vpDisplay::flush(I_gray);

      if (opt_use_depth) {
        vpDisplay::displayText(I_depth, 20, 20, "Click to quit...", vpColor::red);
        if (vpDisplay::getClick(I_depth, false)) { // exit
          state = state_quit;
        }
        vpDisplay::flush(I_depth);
      }
    }

    if (!display_off) {
      delete d_gray;
      if (opt_use_depth)
        delete d_depth;
    }
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
#ifndef VISP_HAVE_REALSENSE2
  std::cout << "ViSP is not build with librealsense2 support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
#endif
  return EXIT_SUCCESS;
}
