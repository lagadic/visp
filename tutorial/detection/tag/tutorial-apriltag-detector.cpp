//! \example tutorial-apriltag-detector.cpp
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#ifdef VISP_HAVE_XML2
#include <visp3/core/vpXmlParserCamera.h>
#endif

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]

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

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    } else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = (unsigned int) atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <input file>] [--tag_size <tag_size in m>]"
                   " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                   " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                   " [--pose_method <method> (0: HOMOGRAPHY, 1: "
                   "HOMOGRAPHY_VIRTUAL_VS,"
                   " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS,"
                   " 4: BEST_RESIDUAL_VIRTUAL_VS)]"
                   " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: "
                   "TAG_36ARTOOLKIT,"
                   " 3: TAG_25h9, 4: TAG_25h7)]"
                   " [--display_tag] [--color <color_id (0, 1, ...)>]"
                   " [--thickness <thickness>]"
                   " [--help]"
                << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
#ifdef VISP_HAVE_XML2
  vpXmlParserCamera parser;
  if (!intrinsic_file.empty() && !camera_name.empty())
    parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
#endif
  std::cout << "cam:\n" << cam << std::endl;
  std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
  std::cout << "tagFamily: " << tagFamily << std::endl;

  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, input_filename);

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
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
        vpDisplay::displayText(I, (int)(bbox.getTop() - 10), (int)bbox.getLeft(),
                               ss.str(), vpColor::red);
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
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
  return 0;
#endif
}
