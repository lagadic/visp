//! \example tutorial-mb-generic-tracker-stereo.cpp
#include <cstdlib>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && defined(VISP_HAVE_PUGIXML)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    std::string opt_videoname_left = "teabox_left.mp4";
    std::string opt_videoname_right = "teabox_right.mp4";
    int opt_tracker1 = vpMbGenericTracker::EDGE_TRACKER;
    int opt_tracker2 = vpMbGenericTracker::EDGE_TRACKER;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name" && i + 2 < argc) {
        opt_videoname_left = std::string(argv[i + 1]);
        opt_videoname_right = std::string(argv[i + 2]);
      }
      else if (std::string(argv[i]) == "--tracker" && i + 2 < argc) {
        opt_tracker1 = atoi(argv[i + 1]);
        opt_tracker2 = atoi(argv[i + 2]);
      }
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0]
          << " [--name <video name left> <video name right>]"
          " [--tracker <1=egde|2=klt|3=hybrid> <1=egde|2=klt|3=hybrid>]"
          " [--help]\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }

    if ((opt_tracker1 < 1 || opt_tracker1 > 3) && (opt_tracker2 < 1 || opt_tracker2 > 3)) {
      std::cerr << "Wrong tracker type. Correct values are: "
        "1=egde|2=keypoint|3=hybrid."
        << std::endl;
      return EXIT_SUCCESS;
    }

    std::string parentname = vpIoTools::getParent(opt_videoname_left);
    std::string objectname_left = vpIoTools::getNameWE(opt_videoname_left);
    std::string objectname_right = vpIoTools::getNameWE(opt_videoname_right);

    if (!parentname.empty()) {
      objectname_left = parentname + "/" + objectname_left;
    }

    std::cout << "Video name: " << opt_videoname_left << " ; " << opt_videoname_right << std::endl;
    std::cout << "Tracker requested config files: " << objectname_left << ".[init, cao]"
      << " and " << objectname_right << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << opt_videoname_left << ".ppm"
      << " and " << opt_videoname_right << ".ppm" << std::endl;

//! [Images]
    vpImage<unsigned char> I_left, I_right;
    //! [Images]

    vpVideoReader g_left, g_right;
    g_left.setFileName(opt_videoname_left);
    g_left.open(I_left);
    g_right.setFileName(opt_videoname_right);
    g_right.open(I_right);

#if defined(VISP_HAVE_X11)
    vpDisplayX display_left;
    vpDisplayX display_right;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_left;
    vpDisplayGDI display_right;
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV display_left;
    vpDisplayOpenCV display_right;
#endif
    display_left.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display_right.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display_left.init(I_left, 100, 100, "Model-based tracker (Left)");
    display_right.init(I_right, 110 + (int)I_left.getWidth(), 100, "Model-based tracker (Right)");

    //! [Constructor]
    std::vector<int> trackerTypes(2);
    trackerTypes[0] = opt_tracker1;
    trackerTypes[1] = opt_tracker2;
    vpMbGenericTracker tracker(trackerTypes);
    //! [Constructor]

#if !defined(VISP_HAVE_MODULE_KLT)
    if (opt_tracker >= 2) {
      std::cout << "klt and hybrid model-based tracker are not available "
        "since visp_klt module is missing"
        << std::endl;
      return EXIT_SUCCESS;
    }
#endif

    //! [Load config file]
    tracker.loadConfigFile(objectname_left + ".xml", objectname_right + ".xml");
    //! [Load config file]

    //! [Load cao]
    tracker.loadModel(objectname_left + ".cao", objectname_right + ".cao");
    //! [Load cao]
    //! [Set display features]
    tracker.setDisplayFeatures(true);
    //! [Set display features]

    //! [Set camera transformation matrix]
    vpHomogeneousMatrix cRightMcLeft;
    std::ifstream file_cRightMcLeft("cRightMcLeft.txt");
    cRightMcLeft.load(file_cRightMcLeft);

    std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformationMatrix;
    mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix();
    mapOfCameraTransformationMatrix["Camera2"] = cRightMcLeft;

    tracker.setCameraTransformationMatrix(mapOfCameraTransformationMatrix);
    //! [Set camera transformation matrix]

    //! [Init]
    tracker.initClick(I_left, I_right, objectname_left + ".init", objectname_right + ".init", true);
    //! [Init]

    while (!g_left.end() && !g_right.end()) {
      g_left.acquire(I_left);
      g_right.acquire(I_right);

      vpDisplay::display(I_left);
      vpDisplay::display(I_right);

      //! [Track]
      tracker.track(I_left, I_right);
      //! [Track]

      //! [Get pose]
      vpHomogeneousMatrix cLeftMo, cRightMo;
      tracker.getPose(cLeftMo, cRightMo);
      //! [Get pose]

      //! [Display]
      vpCameraParameters cam_left, cam_right;
      tracker.getCameraParameters(cam_left, cam_right);
      tracker.display(I_left, I_right, cLeftMo, cRightMo, cam_left, cam_right, vpColor::red, 2);
      //! [Display]

      vpDisplay::displayFrame(I_left, cLeftMo, cam_left, 0.025, vpColor::none, 3);
      vpDisplay::displayFrame(I_right, cRightMo, cam_right, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I_left, 10, 10, "A click to exit...", vpColor::red);

      vpDisplay::flush(I_left);
      vpDisplay::flush(I_right);

      if (vpDisplay::getClick(I_left, false)) {
        break;
      }
    }
    vpDisplay::getClick(I_left);
}
  catch (const vpException &e) {
    std::cerr << "Catch a ViSP exception: " << e.what() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
