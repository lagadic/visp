//! \example tutorial-apriltag-detector-live-rgbd-realsense.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/vision/vpPose.h>

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
    << " [--thickness <thickness>"
#endif
    << " [--verbose, -v]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute the pose of an Apriltag in images acquired with a realsense camera." << std::endl
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
    << "    Default: -1" << std::endl
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
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)
  //! [Macro defined]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod opt_tag_pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double opt_tag_size = 0.053;
  float opt_tag_quad_decimate = 1.0;
  float opt_tag_decision_margin_threshold = -1;
  float opt_tag_hamming_distance_threshold = 2;
  int opt_tag_nThreads = 1;
  bool opt_display_tag = false;
  int opt_color_id = -1;
  unsigned int opt_thickness = 2;
  bool opt_tag_z_align_frame = false;
  bool opt_verbose = false;

#if !(defined(VISP_HAVE_DISPLAY))
  bool opt_display_off = true;
  std::cout << "Warning: There is no 3rd party to display images..." << std::endl;
#else
  bool opt_display_off = false;
#endif

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
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
      opt_thickness = (unsigned int)atoi(argv[++i]);
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
  std::shared_ptr<vpDisplay> d1, d2, d3;
#else
  vpDisplay *d1 = nullptr;
  vpDisplay *d2 = nullptr;
  vpDisplay *d3 = nullptr;
#endif

  try {
    //! [Construct grabber]
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    unsigned int width = 640, height = 480;
    config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Y8, 30);

    vpImage<unsigned char> I;
    vpImage<vpRGBa> I_color(height, width);
    vpImage<uint16_t> I_depth_raw(height, width);
    vpImage<vpRGBa> I_depth;

    g.open(config);
    const float depth_scale = g.getDepthScale();
    std::cout << "I_color: " << I_color.getWidth() << " " << I_color.getHeight() << std::endl;
    std::cout << "I_depth_raw: " << I_depth_raw.getWidth() << " " << I_depth_raw.getHeight() << std::endl;

    rs2::align align_to_color = RS2_STREAM_COLOR;
    g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
              nullptr, nullptr, &align_to_color);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
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

    vpImage<vpRGBa> I_color2 = I_color;
    vpImage<float> depthMap;
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    if (!opt_display_off) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      d1 = vpDisplayFactory::createDisplay(I_color, 100, 30, "Pose from Homography");
      d2 = vpDisplayFactory::createDisplay(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");
      d3 = vpDisplayFactory::createDisplay(I_depth, 100, I_color.getHeight() + 70, "Depth");
#else
      d1 = vpDisplayFactory::allocateDisplay(I_color, 100, 30, "Pose from Homography");
      d2 = vpDisplayFactory::allocateDisplay(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");
      d3 = vpDisplayFactory::allocateDisplay(I_depth, 100, I_color.getHeight() + 70, "Depth");
#endif
    }

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
    //! [AprilTag detector settings]
    std::vector<double> time_vec, time_vec_detection;
    for (;;) {
      double t = vpTime::measureTimeMs();

      //! [Acquisition]
      g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
                reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), nullptr, nullptr, &align_to_color);
      //! [Acquisition]

      I_color2 = I_color;
      vpImageConvert::convert(I_color, I);
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      depthMap.resize(I_depth_raw.getHeight(), I_depth_raw.getWidth());
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < static_cast<int>(I_depth_raw.getHeight()); i++) {
        for (int j = 0; j < static_cast<int>(I_depth_raw.getWidth()); j++) {
          if (I_depth_raw[i][j]) {
            float Z = I_depth_raw[i][j] * depth_scale;
            depthMap[i][j] = Z;
          }
          else {
            depthMap[i][j] = 0;
          }
        }
      }

      vpDisplay::display(I_color);
      vpDisplay::display(I_color2);
      vpDisplay::display(I_depth);

      double t_detection = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, opt_tag_size, cam, cMo_vec);
      t_detection = vpTime::measureTimeMs() - t_detection;
      time_vec_detection.push_back(t_detection);

      // Display camera pose for each tag
      std::vector<std::vector<vpImagePoint> > tagsCorners = detector.getTagsCorners();
      detector.displayTags(I_color, tagsCorners, vpColor::none, opt_thickness);
      detector.displayFrames(I_color, cMo_vec, cam, opt_tag_size / 2, vpColor::none, opt_thickness);
      detector.displayTags(I_color2, tagsCorners, vpColor::none, opt_thickness);
      detector.displayFrames(I_color2, cMo_vec, cam, opt_tag_size / 2, vpColor::none, opt_thickness);

      //! [Pose from depth map]
      std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
      std::vector<int> tags_id = detector.getTagsId();
      std::map<int, double> tags_size;
      tags_size[-1] = opt_tag_size; // Default tag size
      std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
      for (size_t i = 0; i < tags_corners.size(); i++) {
        vpHomogeneousMatrix cMo;
        double confidence_index;
        if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo,
                                                    &confidence_index)) {
          if (confidence_index > 0.5) {
            vpDisplay::displayFrame(I_color2, cMo, cam, opt_tag_size / 2, vpColor::none, opt_thickness);
          }
          else if (confidence_index > 0.25) {
            vpDisplay::displayFrame(I_color2, cMo, cam, opt_tag_size / 2, vpColor::orange, opt_thickness);
          }
          else {
            vpDisplay::displayFrame(I_color2, cMo, cam, opt_tag_size / 2, vpColor::red, opt_thickness);
          }
          std::stringstream ss;
          ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
          vpDisplay::displayText(I_color2, 35 + static_cast<int>(i) * 15, 20, ss.str(), vpColor::red);

          if (opt_verbose) {
            std::cout << ss.str() << std::endl;
            std::cout << "cMo[" << i << "]: \n" << cMo_vec[i] << std::endl;
            std::cout << "cMo[" << i << "] using depth: \n" << cMo << std::endl;
          }
        }
        else {
          vpDisplay::displayText(I_color2, 35, 20, "Unable to compuee a valid pose from RGB-D fusion", vpColor::red);
          vpDisplay::displayText(I_color2, 50, 20, "Check your tag size parameter...", vpColor::red);
        }
      }
      //! [Pose from depth map]

      vpDisplay::displayText(I_color, 20, 20, "Pose from homography + VVS", vpColor::red);
      vpDisplay::displayText(I_color2, 20, 20, "Pose from RGBD fusion", vpColor::red);
      vpDisplay::displayText(I_color, 35, 20, "Click to quit.", vpColor::red);
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I_color, 50, 20, ss.str(), vpColor::red);

      if (vpDisplay::getClick(I_color, false))
        break;

      vpDisplay::flush(I_color);
      vpDisplay::flush(I_color2);
      vpDisplay::flush(I_depth);
    }

    std::cout << "Benchmark loop processing time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    std::cout << "Benchmark detection processing time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec_detection) << " ms"
      << " ; " << vpMath::getMedian(time_vec_detection) << " ms"
      << " ; " << vpMath::getStdev(time_vec_detection) << " ms" << std::endl;
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (!opt_display_off) {
    if (d1 != nullptr) {
      delete d1;
    }
    if (d2 != nullptr) {
      delete d2;
    }
    if (d3 != nullptr) {
      delete d3;
    }
  }
#endif

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#else
  std::cout << "Install librealsense 3rd party, configure and build ViSP again to use this example" << std::endl;
#endif
#endif
  return EXIT_SUCCESS;
}
