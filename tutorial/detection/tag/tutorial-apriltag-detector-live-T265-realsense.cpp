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
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>

int main(int argc, const char **argv)
{
  //! [Macro defined]
  // Realsense T265 is only supported if realsense API > 2.31.0
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
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

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    }
    else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = (unsigned int)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--z_aligned") {
      align_frame = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--tag_size <tag_size in m> (default: 0.053)]"
        " [--quad_decimate <quad_decimate> (default: 1)]"
        " [--nthreads <nb> (default: 1)]"
        " [--pose_method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
        " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
        " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
        " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
        " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
        " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
        " [--display_tag] [--z_aligned]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
#endif
      std::cout << " [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

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
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads : " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;

    vpDisplay *display_left = nullptr;
    vpDisplay *display_undistort = nullptr;
    if (!display_off) {
#ifdef VISP_HAVE_X11
      display_left = new vpDisplayX(I_left, 100, 30, "Left image");
      display_undistort = new vpDisplayX(I_undist, I_left.getWidth(), 30, "Undistorted image");
#elif defined(VISP_HAVE_GDI)
      display_left = new vpDisplayGDI(I_left, 100, 30, "Left image");
#elif defined(HAVE_OPENCV_HIGHGUI)
      display_left = new vpDisplayOpenCV(I_left, 100, 30, "Left image");
#endif
    }

    //! [Initializing undistort map]
    vpArray2D<int> mapU, mapV;
    vpArray2D<float> mapDu, mapDv;
    vpImageTools::initUndistortMap(cam_left, I_left.getWidth(), I_left.getHeight(), mapU, mapV, mapDu, mapDv);
    //! [Initializing undistort map]

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
      detector.detect(I_undist, tagSize, cam_undistort, cMo_vec);
      //! [Detect tags in undistorted image]

      // Display tag corners, bounding box and pose
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        tag_corners = detector.getTagsCorners();
        for (size_t j = 0; j < 4; j++) {
          vpDisplay::displayCross(I_undist, tag_corners[i][j], 20, vpColor::green, 2);
        }

        vpDisplay::displayRectangle(I_undist, detector.getBBox(i), vpColor::yellow, false, 3);
        vpDisplay::displayFrame(I_undist, cMo_vec[i], cam_undistort, tagSize / 2, vpColor::red, 3);
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

    if (!display_off) {
      delete display_left;
      delete display_undistort;
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
