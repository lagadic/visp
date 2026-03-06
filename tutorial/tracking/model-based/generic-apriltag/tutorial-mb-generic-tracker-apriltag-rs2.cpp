//! \example tutorial-mb-generic-tracker-apriltag-rs2.cpp

#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_MODULE_MBT)
//! [Macro defined]

#include <fstream>
#include <ios>

#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
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

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--tag-size <size>]"
    << " [--tag-family <family>]"
    << " [--tag-decision-margin-threshold <threshold>]"
    << " [--tag-hamming-distance-threshold <threshold>]"
    << " [--tag-quad-decimate <factor>]"
    << " [--tag-n-threads <number>]"
#if defined(VISP_HAVE_DISPLAY)
    << " [--display-off]"
#endif
    << " [--cube-size <size]"
    << " [--use-texture]"
    << " [--use-depth]"
    << " [--projection-error-threshold <threshold>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Live execution on images acquired by a realsense camera of the generic model-based" << std::endl
    << "  tracker. The considered object is a cube to which an Apriltag is attached on one of its" << std::endl
    << "  faces. Once detected, the pose of the Apriltag is used to initialise the tracker." << std::endl
    << "  The Apriltag must be centred on a face of the cube. If the tracker fails, the " << std::endl
    << "  tag is used to reset the tracker." << std::endl
    << std::endl
    << "  --tag-size <size>" << std::endl
    << "    Apriltag size in [m]." << std::endl
    << "    Default: 0.03" << std::endl
    << std::endl
    << "  --tag-family <family>" << std::endl
    << "    Apriltag family. Supported values are:" << std::endl
    << "       0: TAG_36h11" << std::endl
    << "       1: TAG_25h9" << std::endl
    << "       2: TAG_16h5" << std::endl
    << "       3: TAG_CIRCLE21h7" << std::endl
    << "       4: TAG_CIRCLE49h12" << std::endl
    << "       5: TAG_CUSTOM48h12" << std::endl
    << "       6: TAG_STANDARD41h12" << std::endl
    << "       7: TAG_STANDARD52h13" << std::endl
#if defined(VISP_HAVE_APRILTAG_ARUCO)
    << "       8: TAG_ARUCO_4x4_50" << std::endl
    << "       9: TAG_ARUCO_4x4_100" << std::endl
    << "      10: TAG_ARUCO_4x4_250" << std::endl
    << "      11: TAG_ARUCO_4x4_1000" << std::endl
    << "      12: TAG_ARUCO_5x5_50" << std::endl
    << "      13: TAG_ARUCO_5x5_100" << std::endl
    << "      14: TAG_ARUCO_5x5_250" << std::endl
    << "      15: TAG_ARUCO_5x5_1000" << std::endl
    << "      16: TAG_ARUCO_6x6_50" << std::endl
    << "      17: TAG_ARUCO_6x6_100" << std::endl
    << "      18: TAG_ARUCO_6x6_250" << std::endl
    << "      19: TAG_ARUCO_6x6_1000" << std::endl
    << "      20: TAG_ARUCO_7x7_50" << std::endl
    << "      21: TAG_ARUCO_7x7_100" << std::endl
    << "      22: TAG_ARUCO_7x7_250" << std::endl
    << "      23: TAG_ARUCO_7x7_1000" << std::endl
    << "      24: TAG_ARUCO_MIP_36h12" << std::endl
#endif
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
    << "  --use-depth" << std::endl
    << "    Flag to enable usage of depth map as features." << std::endl
    << "    Default: disabled" << std::endl
    << std::endl
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
  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  double opt_tag_size = 0.08;
  float opt_tag_quad_decimate = 1.0;
  float opt_tag_decision_margin_threshold = 50;
  int opt_tag_hamming_distance_threshold = 2;
  int opt_tag_nthreads = 1;
  double opt_cube_size = 0.125; // 12.5cm by default
#ifdef VISP_HAVE_OPENCV
  bool opt_use_texture = false;
#endif
  bool opt_use_depth = false;
  double opt_projection_error_threshold = 40.;

#if !(defined(VISP_HAVE_DISPLAY))
  bool opt_display_off = true;
#else
  bool opt_display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      opt_tag_family = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-decision-margin-threshold" && i + 1 < argc) {
      opt_tag_decision_margin_threshold = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-hamming-distance-threshold" && i + 1 < argc) {
      opt_tag_hamming_distance_threshold = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-quad-decimate" && i + 1 < argc) {
      opt_tag_quad_decimate = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-n-threads" && i + 1 < argc) {
      opt_tag_nthreads = atoi(argv[++i]);
    }
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
    else if (std::string(argv[i]) == "--use-depth") {
      opt_use_depth = true;
    }
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d_gray;
  std::shared_ptr<vpDisplay> d_depth;
#else
  vpDisplay *d_gray = nullptr;
  vpDisplay *d_depth = nullptr;
#endif

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

    std::cout << "Cube size         : " << opt_cube_size << std::endl;
    std::cout << "AprilTag size     : " << opt_tag_size << std::endl;
    std::cout << "AprilTag family   : " << opt_tag_family << std::endl;
    std::cout << "Camera parameters" << std::endl;
    std::cout << "  Color           :\n" << cam_color << std::endl;
    if (opt_use_depth) {
      std::cout << "  Depth           :\n" << cam_depth << std::endl;
    }
    std::cout << "Detection " << std::endl;
    std::cout << "  Quad decimate   : " << opt_tag_quad_decimate << std::endl;
    std::cout << "  Threads number  : " << opt_tag_nthreads << std::endl;
    std::cout << "  Decision margin : " << opt_tag_decision_margin_threshold << " (applied to ArUco tags only)" << std::endl;
    std::cout << "Tracker " << std::endl;
    std::cout << "  Use edges       : 1" << std::endl;
    std::cout << "  Use texture     : "
#ifdef VISP_HAVE_OPENCV
      << opt_use_texture << std::endl;
#else
      << " na" << std::endl;
#endif
    std::cout << "  Use depth       : " << opt_use_depth << std::endl;
    std::cout << "  Projection error: " << opt_projection_error_threshold << std::endl;

    // Construct display
    if (!opt_display_off) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      d_gray = vpDisplayFactory::createDisplay(I_gray, 50, 50, "Color stream");
      if (opt_use_depth)
        d_depth = vpDisplayFactory::createDisplay(I_depth, 80 + I_gray.getWidth(), 50, "Depth stream");
#else
      d_gray = vpDisplayFactory::allocateDisplay(I_gray, 50, 50, "Color stream");
      if (opt_use_depth)
        d_depth = vpDisplayFactory::allocateDisplay(I_depth, 80 + I_gray.getWidth(), 50, "Depth stream");
#endif
    }

    // Initialize AprilTag detector
    vpDetectorAprilTag detector(opt_tag_family);
    detector.setAprilTagQuadDecimate(opt_tag_quad_decimate);
    detector.setAprilTagNbThreads(opt_tag_nthreads);
    detector.setAprilTagDecisionMarginThreshold(opt_tag_decision_margin_threshold);
    detector.setAprilTagHammingDistanceThreshold(opt_tag_hamming_distance_threshold);

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
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (!opt_display_off) {
    delete d_gray;
    if (opt_use_depth)
      delete d_depth;
  }
#endif

  return EXIT_SUCCESS;
}

#else

int main()
{
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#ifndef VISP_HAVE_REALSENSE2
  std::cout << "ViSP is not build with librealsense2 support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;

  return EXIT_SUCCESS;
}

#endif
