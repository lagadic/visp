//! \example tutorial-apriltag-detector-live-T265-realsense.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/vision/vpPose.h>

void usage(const char **argv, int error);

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
    << " [--tag-z-aligned]"
    << " [--tag-pose-method <method>]"
#if defined(VISP_HAVE_DISPLAY)
    << " [--display-tag]"
    << " [--display-off]"
    << " [--color <id>]"
    << " [--opt_thickness <opt_thickness>"
#endif
    << " [--verbose, -v]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute the pose of an Apriltag in images acquired with a realsense T265 camera." << std::endl
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
    << "      23: TAG_ARUCO_7x7_50" << std::endl
    << "      24: TAG_ARUCO_7x7_100" << std::endl
    << "      25: TAG_ARUCO_7x7_250" << std::endl
    << "      26: TAG_ARUCO_7x7_1000" << std::endl
    << "      27: TAG_ARUCO_MIP_36h12" << std::endl
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
    << "    Enable extra verbosity." << std::endl
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
  //! [Macro defined]
  // Realsense T265 is only supported if realsense API > 2.31.0
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  //! [Macro defined]

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod opt_tag_pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double opt_tag_size = 0.053;
  float opt_tag_quad_decimate = 1.0;
  float opt_tag_decision_margin_threshold = 50;
  int opt_tag_hamming_distance_threshold = 2;
  int opt_tag_nThreads = 1;
  bool opt_display_tag = false;
  int opt_color_id = -1;
  unsigned int opt_thickness = 2;
  bool opt_tag_z_align_frame = false;

#if !(defined(VISP_HAVE_DISPLAY))
  bool opt_display_off = true;
  std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to display images..." << std::endl;
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
    else if (std::string(argv[i]) == "--tag-quad-decimate" && i + 1 < argc) {
      opt_tag_quad_decimate = static_cast<float>(atof(argv[++i]));
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
#if defined(VISP_HAVE_DISPLAY)
    else if (std::string(argv[i]) == "--display-tag") {
      opt_display_tag = true;
    }
    else if (std::string(argv[i]) == "--display-off") {
      opt_display_off = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      opt_color_id = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      opt_thickness = static_cast<unsigned int>(atoi(argv[++i]));
    }
#endif
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
  std::shared_ptr<vpDisplay> display_left, display_undistort;
#else
  vpDisplay *display_left = nullptr;
  vpDisplay *display_undistort = nullptr;
#endif

  try {
    //! [Construct grabber]
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    unsigned int width = 848, height = 800;
    config.disable_stream(RS2_STREAM_FISHEYE, 1);
    config.disable_stream(RS2_STREAM_FISHEYE, 2);
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    vpImage<unsigned char> I_left(height, width);
    vpImage<unsigned char> I_undist(height, width);

    g.open(config);
    g.acquire(&I_left, nullptr, nullptr);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    vpCameraParameters cam_left, cam_undistort;
    cam_left = g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 1);
    cam_undistort.initPersProjWithoutDistortion(cam_left.get_px(), cam_left.get_py(), cam_left.get_u0(),
                                                cam_left.get_v0());
    //! [Construct grabber]

    std::cout << cam_left << std::endl;
    std::cout << "Tag detector settings" << std::endl;
    std::cout << "  Tag size [m]              : " << opt_tag_size << std::endl;
    std::cout << "  Tag family                : " << opt_tag_family << std::endl;
    std::cout << "  Quad decimate             : " << opt_tag_quad_decimate << std::endl;
    std::cout << "  Decision margin threshold : " << opt_tag_decision_margin_threshold << std::endl;
    std::cout << "  Hamming distance threshold: " << opt_tag_hamming_distance_threshold << std::endl;
    std::cout << "  Num threads               : " << opt_tag_nThreads << std::endl;
    std::cout << "  Z aligned                 : " << opt_tag_z_align_frame << std::endl;
    std::cout << "  Pose estimation           : " << opt_tag_pose_estimation_method << std::endl;

    if (!opt_display_off) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      display_left = vpDisplayFactory::createDisplay(I_left, 100, 30, "Left image");
      display_undistort = vpDisplayFactory::createDisplay(I_undist, I_left.getWidth(), 30, "Undistorted image");
#else
      display_left = vpDisplayFactory::allocateDisplay(I_left, 100, 30, "Left image");
      display_undistort = vpDisplayFactory::allocateDisplay(I_undist, I_left.getWidth(), 30, "Undistorted image");
#endif
    }

    //! [Initializing undistort map]
    vpArray2D<int> mapU, mapV;
    vpArray2D<float> mapDu, mapDv;
    vpImageTools::initUndistortMap(cam_left, I_left.getWidth(), I_left.getHeight(), mapU, mapV, mapDu, mapDv);
    //! [Initializing undistort map]

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(opt_tag_family);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(opt_tag_quad_decimate);
    detector.setAprilTagPoseEstimationMethod(opt_tag_pose_estimation_method);
    detector.setAprilTagNbThreads(opt_tag_nThreads);
    detector.setDisplayTag(opt_display_tag, opt_color_id < 0 ? vpColor::none : vpColor::getColor(opt_color_id), opt_thickness);
    detector.setZAlignedWithCameraAxis(opt_tag_z_align_frame);
    detector.setAprilTagDecisionMarginThreshold(opt_tag_decision_margin_threshold);
    detector.setAprilTagHammingDistanceThreshold(opt_tag_hamming_distance_threshold);
    //! [AprilTag detector settings]

    std::vector<double> time_vec;

    std::vector<std::vector<vpImagePoint> > tag_corners;

    for (;;) {
      double t = vpTime::measureTimeMs();

      //! [Acquisition]
      g.acquire(&I_left, nullptr, nullptr);

      //! [Undistorting image]
      vpImageTools::undistort(I_left, mapU, mapV, mapDu, mapDv, I_undist);
      //! [Undistorting image]

      //! [Display images]
      vpDisplay::display(I_left);
      vpDisplay::display(I_undist);
      //! [Display images]

      //! [Detect tags in undistorted image]
      std::vector<vpHomogeneousMatrix> cMo_vec, cMo_vec1;
      detector.detect(I_undist, opt_tag_size, cam_undistort, cMo_vec);
      //! [Detect tags in undistorted image]

      // Display tag corners, bounding box and pose
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        tag_corners = detector.getTagsCorners();
        for (size_t j = 0; j < 4; j++) {
          vpDisplay::displayCross(I_undist, tag_corners[i][j], 20, vpColor::green, 2);
        }

        vpDisplay::displayRectangle(I_undist, detector.getBBox(i), vpColor::yellow, false, 3);
        vpDisplay::displayFrame(I_undist, cMo_vec[i], cam_undistort, opt_tag_size / 2, vpColor::red, 3);
      }

      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I_left, 50, 20, ss.str(), vpColor::red);
      vpDisplay::displayText(I_undist, 50, 20, ss.str(), vpColor::red);

      if (vpDisplay::getClick(I_left, false) || vpDisplay::getClick(I_undist, false))
        break;

      vpDisplay::flush(I_left);
      vpDisplay::flush(I_undist);
    }

    std::cout << "Benchmark loop processing time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (!opt_display_off) {
    if (display_left != nullptr) {
      delete display_left;
    }

    if (display_undistort != nullptr) {
      delete display_undistort;
    }
  }
#endif

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#elif defined(VISP_HAVE_REALSENSE2) && !(RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  std::cout << "Realsense T265 device needs librealsense API > 2.31.0. ViSP is linked with librealsense API "
    << RS2_API_VERSION_STR << ". You need to upgrade librealsense to use this example." << std::endl;
#else
  std::cout << "Install librealsense 3rd party, configure and build ViSP again to use this example." << std::endl;
#endif
#endif
  return EXIT_SUCCESS;
}
