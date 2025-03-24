//! \example tutorial-apriltag-detector.cpp
#include <visp3/core/vpConfig.h>
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--input <filename>]"
    << " [--tag-size <size>]"
    << " [--tag-family <family>]"
    << " [--tag-quad-decimate <factor>]"
    << " [--tag-n-threads <number>]"
    << " [--tag-z-aligned]"
    << " [--tag-pose-method <method>]"
#if defined(VISP_HAVE_PUGIXML)
    << " [--intrinsic <xmlfile>]"
    << " [--camera <name>]"
#endif
#if defined(VISP_HAVE_DISPLAY)
    << " [--display-tag]"
    << " [--color <id>]"
    << " [--thickness <thickness>"
#endif
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Detect AprilTags in an image and compute their corresponding pose." << std::endl
    << std::endl
    << "  --input <filename>" << std::endl
    << "    Image filename to process." << std::endl
    << "    Default: AprilTag.jpg" << std::endl
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
    << "  --camera <name>" << std::endl
    << "    Camera name in the intrinsic parameters file in xml format." << std::endl
    << "    Default: empty" << std::endl
    << std::endl
#endif
#if defined(VISP_HAVE_DISPLAY)
    << "  --display-tag" << std::endl
    << "    Flag used to enable displaying the edges of a tag." << std::endl
    << "    Default: disabled" << std::endl
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
#endif
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
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_DISPLAY)
  //! [Macro defined]

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string opt_input_filename = "AprilTag.jpg";
  vpDetectorAprilTag::vpAprilTagFamily opt_tag_family = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod opt_tag_pose_estimation_method = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double opt_tag_size = 0.053;
  float opt_tag_quad_decimate = 1.0;
  int opt_tag_nThreads = 1;
  std::string opt_intrinsic_file = "";
  std::string opt_camera_name = "";
  bool opt_display_tag = false;
  int opt_color_id = -1;
  unsigned int opt_thickness = 2;
  bool opt_tag_z_aligned = false;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_input_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      opt_tag_size = atof(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      opt_tag_family = static_cast<vpDetectorAprilTag::vpAprilTagFamily>(atoi(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-quad-decimate" && i + 1 < argc) {
      opt_tag_quad_decimate = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--tag-n-threads" && i + 1 < argc) {
      opt_tag_nThreads = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-z-aligned") {
      opt_tag_z_aligned = true;
    }
    else if (std::string(argv[i]) == "--tag-pose-method" && i + 1 < argc) {
      opt_tag_pose_estimation_method = static_cast<vpDetectorAprilTag::vpPoseEstimationMethod>(atoi(argv[++i]));
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
    else if (std::string(argv[i]) == "--display-tag") {
      opt_display_tag = true;
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

  std::cout << "Input data" << std::endl;
  std::cout << "  Image          : " << opt_input_filename << std::endl;

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
#if defined(VISP_HAVE_PUGIXML)
  vpXmlParserCamera parser;
  if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
    std::cout << "  Intrinsics     : " << opt_intrinsic_file << std::endl << std::endl;
    parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
  }
  else {
    std::cout << "  Intrinsics     : default" << std::endl << std::endl;
  }
#else
  std::cout << "  Intrinsics     : default" << std::endl << std::endl;
#endif

  std::cout << cam << std::endl;
  std::cout << "Tag detector settings" << std::endl;
  std::cout << "  Tag size [m]   : " << opt_tag_size << std::endl;
  std::cout << "  Tag family     : " << opt_tag_family << std::endl;
  std::cout << "  Quad decimate  : " << opt_tag_quad_decimate << std::endl;
  std::cout << "  Thread numbers : " << opt_tag_nThreads << std::endl;
  std::cout << "  Z aligned      : " << opt_tag_z_aligned << std::endl;
  std::cout << "  Pose estimation: " << opt_tag_pose_estimation_method << std::endl;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display, display2;
#else
  vpDisplay *display = nullptr;
  vpDisplay *display2 = nullptr;
#endif
  try {
    vpImage<vpRGBa> I_color;
    vpImageIo::read(I_color, opt_input_filename);
    vpImage<unsigned char> I;
    vpImageConvert::convert(I_color, I);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I, -1, -1, "AprilTag detection (for loop display)");
#else
    display = vpDisplayFactory::allocateDisplay(I, -1, -1, "AprilTag detection (for loop display)");
#endif

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(opt_tag_family);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(opt_tag_quad_decimate);
    detector.setAprilTagPoseEstimationMethod(opt_tag_pose_estimation_method);
    detector.setAprilTagNbThreads(opt_tag_nThreads);
    detector.setDisplayTag(opt_display_tag, opt_color_id < 0 ? vpColor::none : vpColor::getColor(opt_color_id), opt_thickness);
    detector.setZAlignedWithCameraAxis(opt_tag_z_aligned);
    //! [AprilTag detector settings]
    // TODO:
    detector.setAprilTagDebugOption(true);

    vpDisplay::display(I);

    double t = vpTime::measureTimeMs();
    //! [Detect and compute pose]
    std::vector<vpHomogeneousMatrix> cMo_vec;
    detector.detect(I, opt_tag_size, cam, cMo_vec);
    //! [Detect and compute pose]
    t = vpTime::measureTimeMs() - t;

    std::stringstream ss;
    ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

    //! [Get tag ids]
    std::vector<int> tag_ids = detector.getTagsId();
    //! [Get tag ids]

    //! [Parse detected codes]
    std::cout << "\nDetected tags" << std::endl;
    for (size_t i = 0; i < detector.getNbObjects(); i++) {
      //! [Parse detected codes]
      //! [Get location]
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      vpRect bbox = detector.getBBox(i);
      //! [Get location]
      vpDisplay::displayRectangle(I, bbox, vpColor::green);
      //! [Get message]
      std::string message = detector.getMessage(i);
      std::cout << "  " << message << std::endl;
      //! [Get message]
      //! [Display tag ids]
      ss.str("");
      ss << "Tag id: " << tag_ids[i];
      vpDisplay::displayText(I, static_cast<int>(bbox.getTop()), static_cast<int>(bbox.getLeft()), ss.str(), vpColor::red);
      //! [Display tag ids]
      //! [Display corner indexes]
      for (size_t j = 0; j < p.size(); j++) {
        vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
        std::ostringstream number;
        number << j;
        vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
      }
      //! [Display corner indexes]
    }

    vpDisplay::displayText(I, 20, 20, "Click to display tag poses", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    vpDisplay::display(I);

    //! [Display camera pose for each tag]
    for (size_t i = 0; i < cMo_vec.size(); i++) {
      vpDisplay::displayFrame(I, cMo_vec[i], cam, opt_tag_size / 2, vpColor::none, 3);
    }
    //! [Display camera pose for each tag]

    vpDisplay::displayText(I, 20, 20, "Click.", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);
    vpDisplay::close(I);


#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display2 = vpDisplayFactory::createDisplay(I_color, 50, 50, "AprilTag detection (class display)");
#else
    display2 = vpDisplayFactory::allocateDisplay(I_color, 50, 50, "AprilTag detection (class display)");
#endif
    // To test the displays on a vpRGBa image
    vpDisplay::display(I_color);

    // Display frames and tags
    std::vector<std::vector<vpImagePoint> > tagsCorners = detector.getTagsCorners();
    detector.displayTags(I_color, tagsCorners, vpColor::none, 3);
    detector.displayFrames(I_color, cMo_vec, cam, opt_tag_size / 2, vpColor::none, 3);

    vpDisplay::displayText(I_color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I_color);
    vpDisplay::getClick(I_color);
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
  if (display2 != nullptr) {
    delete display2;
  }
#endif
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
