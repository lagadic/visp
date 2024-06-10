//! \example tutorial-apriltag-detector.cpp
#include <visp3/core/vpConfig.h>
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main(int argc, const char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string input_filename = "AprilTag.pgm";
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
  float quad_decimate = 1.0;
  int nThreads = 1;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool z_aligned = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = static_cast<vpDetectorAprilTag::vpPoseEstimationMethod>(atoi(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = static_cast<float>(atof(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    }
#if defined(VISP_HAVE_PUGIXML)
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    }
#endif
    else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = static_cast<unsigned int>(atoi(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = static_cast<vpDetectorAprilTag::vpAprilTagFamily>(atoi(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--z_aligned") {
      z_aligned = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input file>] [--tag_size <tag_size in m>]"
        " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
        " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
        " [--pose_method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
        " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
        " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
        " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
        " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
        " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
        " [--display_tag] [--color <color_id (0, 1, ...)>]"
        " [--thickness <thickness>] [--z_aligned]"
        " [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
#if defined(VISP_HAVE_PUGIXML)
  vpXmlParserCamera parser;
  if (!intrinsic_file.empty() && !camera_name.empty()) {
    parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
  }
#endif

  std::cout << cam << std::endl;
  std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
  std::cout << "tagFamily: " << tagFamily << std::endl;
  std::cout << "nThreads : " << nThreads << std::endl;
  std::cout << "Z aligned: " << z_aligned << std::endl;

  try {
    vpImage<vpRGBa> I_color;
    vpImageIo::read(I_color, input_filename);
    vpImage<unsigned char> I;
    vpImageConvert::convert(I_color, I);

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I);
#endif

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(tagFamily);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(z_aligned);
    //! [AprilTag detector settings]

    vpDisplay::display(I);

    double t = vpTime::measureTimeMs();
    //! [Detect and compute pose]
    std::vector<vpHomogeneousMatrix> cMo_vec;
    detector.detect(I, tagSize, cam, cMo_vec);
    //! [Detect and compute pose]
    t = vpTime::measureTimeMs() - t;

    std::stringstream ss;
    ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

    //! [Parse detected codes]
    for (size_t i = 0; i < detector.getNbObjects(); i++) {
      //! [Parse detected codes]
      //! [Get location]
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      vpRect bbox = detector.getBBox(i);
      //! [Get location]
      vpDisplay::displayRectangle(I, bbox, vpColor::green);
      //! [Get message]
      std::string message = detector.getMessage(i);
      //! [Get message]
      //! [Get tag id]
      std::size_t tag_id_pos = message.find("id: ");
      if (tag_id_pos != std::string::npos) {
        int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
        ss.str("");
        ss << "Tag id: " << tag_id;
        vpDisplay::displayText(I, static_cast<int>(bbox.getTop() - 10), static_cast<int>(bbox.getLeft()), ss.str(), vpColor::red);
      }
      //! [Get tag id]
      for (size_t j = 0; j < p.size(); j++) {
        vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
        std::ostringstream number;
        number << j;
        vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
      }
    }

    vpDisplay::displayText(I, 20, 20, "Click to display tag poses", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    vpDisplay::display(I);

    //! [Display camera pose for each tag]
    for (size_t i = 0; i < cMo_vec.size(); i++) {
      vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
    }
    //! [Display camera pose for each tag]

    vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

#ifdef VISP_HAVE_X11
    vpDisplayX d2(I_color, 50, 50);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d2(I_color, 50, 50);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d2(I_color, 50, 50);
#endif
    // To test the displays on a vpRGBa image
    vpDisplay::display(I_color);

    // Display frames and tags but remove the display of the last element
    std::vector<std::vector<vpImagePoint> > tagsCorners = detector.getTagsCorners();
    tagsCorners.pop_back();
    detector.displayTags(I_color, tagsCorners, vpColor::none, 3);

    cMo_vec.pop_back();
    detector.displayFrames(I_color, cMo_vec, cam, tagSize / 2, vpColor::none, 3);

    vpDisplay::displayText(I_color, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I_color);
    vpDisplay::getClick(I_color);
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
