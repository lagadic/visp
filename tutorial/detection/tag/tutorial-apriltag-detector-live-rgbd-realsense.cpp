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

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)
  //! [Macro defined]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  bool opt_verbose = false;

#if !(defined(VISP_HAVE_DISPLAY))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose-method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      tagSize = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--quad-decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--display-tag") {
      display_tag = true;
    }
    else if (std::string(argv[i]) == "--display-off") {
      display_off = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = (unsigned int)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--z-aligned") {
      align_frame = true;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--tag-size <tag_size in m> (default: 0.053)]"
        << " [--quad-decimate <quad_decimate> (default: 1)]"
        << " [--nthreads <nb> (default: 1)]"
        << " [--pose-method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
        << " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
        << " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
        << " [--tag-family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
        << " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
        << " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
        << " [--display-tag] [--z-aligned]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display-off] [--color <color id>] [--thickness <line thickness>]";
#endif
      std::cout << " [--verbose,-v] [--help,-h]" << std::endl;
      return EXIT_SUCCESS;
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
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads : " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;

    vpImage<vpRGBa> I_color2 = I_color;
    vpImage<float> depthMap;
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

    if (!display_off) {
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
    vpDetectorAprilTag detector(tagFamily);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(align_frame);
    //! [AprilTag detector settings]
    std::vector<double> time_vec;
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

      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tagSize, cam, cMo_vec);

      // Display camera pose for each tag
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
      }

      //! [Pose from depth map]
      std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
      std::vector<int> tags_id = detector.getTagsId();
      std::map<int, double> tags_size;
      tags_size[-1] = tagSize; // Default tag size
      std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
      for (size_t i = 0; i < tags_corners.size(); i++) {
        vpHomogeneousMatrix cMo;
        double confidence_index;
        if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo,
                                                    &confidence_index)) {
          if (confidence_index > 0.5) {
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::none, 3);
          }
          else if (confidence_index > 0.25) {
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::orange, 3);
          }
          else {
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::red, 3);
          }
          std::stringstream ss;
          ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
          vpDisplay::displayText(I_color2, 35 + i * 15, 20, ss.str(), vpColor::red);

          if (opt_verbose) {
            std::cout << "cMo[" << i << "]: \n" << cMo_vec[i] << std::endl;
            std::cout << "cMo[" << i << "] using depth: \n" << cMo << std::endl;
          }
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
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (!display_off) {
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
