//! \example tutorial-apriltag-detector.cpp
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpXmlParserCamera.h>

int main(int argc, const char** argv) {
  //! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]

  std::string input_filename = "AprilTag.pgm";
  bool poseFromHomography = false;
  double tagSize = 0.053;
  float quad_decimate = 1.0;
  int nThreads = 1;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_homography") {
      poseFromHomography = true;
    } else if (std::string(argv[i]) == "--tag_size" && i+1 < argc) {
      tagSize = atof(argv[i+1]);
    } else if (std::string(argv[i]) == "--input" && i+1 < argc) {
      input_filename = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i+1 < argc) {
      quad_decimate = atof(argv[i+1]);
    } else if (std::string(argv[i]) == "--nthreads" && i+1 < argc) {
      nThreads = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i+1 < argc) {
      intrinsic_file = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--camera_name" && i+1 < argc) {
      camera_name = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <input file>] [--tag_size <tag_size in m>] [--pose_homography]"
                   " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                   " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                   " [--help]"
                << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpCameraParameters cam;
  vpXmlParserCamera parser;
  if (parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
  }
  std::cout << "cam:\n" << cam << std::endl;

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

    //! [Create base detector]
    vpDetectorBase *detector = new vpDetectorAprilTag;
    //! [Create base detector]

    //! [AprilTag detector settings]
    dynamic_cast<vpDetectorAprilTag*>(detector)->setAprilTagQuadDecimate(quad_decimate);
    dynamic_cast<vpDetectorAprilTag*>(detector)->setAprilTagPoseFromHomography(poseFromHomography);
    dynamic_cast<vpDetectorAprilTag*>(detector)->setAprilTagNbThreads(nThreads);
    dynamic_cast<vpDetectorAprilTag*>(detector)->setDisplayTag(display_tag);
    //! [AprilTag detector settings]

    vpDisplay::display(I);

    double t = vpTime::measureTimeMs();
    //! [Detect and compute pose]
    std::vector<vpHomogeneousMatrix> cMo_vec;
    dynamic_cast<vpDetectorAprilTag*>(detector)->detect(I, tagSize, cam, cMo_vec);
    //! [Detect and compute pose]
    t = vpTime::measureTimeMs() - t;

    std::stringstream ss;
    ss << "Detection time: " << t << " ms for " << detector->getNbObjects() << " tags";
    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

    //! [Display camera pose for each tag]
    for (size_t i = 0; i < cMo_vec.size() ; i++) {
      vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize/2, vpColor::none, 3);
    }
    //! [Display camera pose for each tag]

    vpDisplay::displayText(I, 20, 20, "Click to display bounding boxes.", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    vpDisplay::display(I);

    for(size_t i=0; i < detector->getNbObjects(); i++) {
      //! [Parse detected codes]
      //! [Get location]
      std::vector<vpImagePoint> p = detector->getPolygon(i);
      vpRect bbox = detector->getBBox(i);
      //! [Get location]
      vpDisplay::displayRectangle(I, bbox, vpColor::green);
      //! [Get message]
      vpDisplay::displayText(I, (int) (bbox.getTop()-10), (int) bbox.getLeft(),
                             "Message: \"" + detector->getMessage(i) + "\"",
                             vpColor::red);
      //! [Get message]
      for(size_t j=0; j < p.size(); j++) {
        vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
        std::ostringstream number;
        number << j;
        vpDisplay::displayText(I, p[j]+vpImagePoint(15,5), number.str(), vpColor::blue);
      }
    }

    vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    delete detector;
  } catch(const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
  return 0;
#endif
}
