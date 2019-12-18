//! \example tutorial-apriltag-detector-live-rgbd-realsense.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/vision/vpPose.h>

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)
  //! [Macro defined]

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
    } else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    } else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
    } else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = (unsigned int) atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--z_aligned") {
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
    unsigned int width = 640, height = 480;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);
    rs2::align align_to = RS2_STREAM_COLOR;

    vpImage<unsigned char> I;
    vpImage<vpRGBa> I_color(height, width);
    vpImage<float> depthMap(height, width);
    std::vector<vpColVector> vp_pointcloud;

    g.open(config);
    g.acquire(I_color);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    //! [Construct grabber]

    std::cout << cam << std::endl;
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads : " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;

    vpDisplay *d = NULL;
    if (! display_off) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I_color);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I_color);
#elif defined(VISP_HAVE_OPENCV)
      d = new vpDisplayOpenCV(I_color);
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
      //! [Acquisition]
      g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), NULL, &vp_pointcloud, NULL, &align_to);
      vpImageConvert::convert(I_color, I);
      if (depthMap.getSize() != vp_pointcloud.size()) {
        throw (vpException(vpException::fatalError, "Cannot compute depth map from point cloud"));
      }
      for (unsigned int i=0; i < depthMap.getHeight(); i++) {
        for (unsigned int j=0; j < depthMap.getWidth(); j++) {
          depthMap[i][j] = static_cast<float>(vp_pointcloud[i*depthMap.getWidth() + j][2]);
        }
      }
      //! [Acquisition]

      vpDisplay::display(I_color);

      double t = vpTime::measureTimeMs();
      //! [Detect and compute pose]
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tagSize, cam, cMo_vec);
      //! [Detect and compute pose]
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I_color, 40, 20, ss.str(), vpColor::red);

      //! [Display camera pose for each tag]
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);

        // Compute pose from depth map
        vpHomogeneousMatrix cMo_rgbd;
        std::vector<vpImagePoint> corners = detector.getPolygon(i);
        bool ret = vpPose::computePoseRGBD(depthMap, corners, cam, tagSize, cMo_rgbd);
        if(ret) {
          vpDisplay::displayFrame(I_color, cMo_rgbd, cam, tagSize / 1.5, vpColor::none, 1);
        }
      }
      //! [Display camera pose for each tag]

      vpDisplay::displayText(I_color, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I_color);
      if (vpDisplay::getClick(I_color, false))
        break;
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
              << " ; " << vpMath::getMedian(time_vec) << " ms"
              << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    if (! display_off)
      delete d;

  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#else
  std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, Realsense2), configure and build ViSP again to use this example" << std::endl;
#endif
#endif
  return EXIT_SUCCESS;
}
