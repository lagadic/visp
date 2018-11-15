/*! \example tutorial-grabber-basler-pylon.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpPylonFactory.h>

#include "record_helper.h"

/*!
  Usage :
    To get the help    : ./tutorial-grabber-basler-pylon --help
    To set the device  : ./tutorial-grabber-basler-pylon --device GigE --camera 1
 */
int main(int argc, const char *argv[])
{
#ifdef VISP_HAVE_PYLON
  try {
    unsigned int opt_camera = 0;
    std::string opt_device("GigE");
    std::string opt_seqname;
    int opt_record_mode = 0;
    bool opt_change_settings = false;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--camera")
        opt_camera = (unsigned int)atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--device")
        opt_device = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--change_settings")
        opt_change_settings = true;
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--camera <0...9> (default: 0)] [--device <\"GigE\"|\"USB\" (default: GigE)>]"
                  << " [--seqname <sequence name (default: empty>] [--record <0: continuous | 1: single shot (default: 0)>]"
                  <<  " [--change_settings] [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << " \n"
                  << "  " << argv[0] << " --device GigE --camera 0\n"
                  << "\nExamples to record a sequence:\n"
                  << "  " << argv[0] << " --seqname I%04d.png \n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 0\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --seqname I%04d.png --record 1\n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 1\n"
                  << std::endl;
        return 0;
      }
    }

    std::cout << "Settings   : " << (opt_change_settings ? "modified" : "current") << std::endl;
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (! opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }

    vpImage<unsigned char> I;

    vpPylonFactory &factory = vpPylonFactory::instance();

    vpPylonGrabber *g;
    if (opt_device == "GigE" || opt_device == "gige") {
      g = factory.createPylonGrabber(vpPylonFactory::BASLER_GIGE);
      std::cout << "Opening Basler GigE camera: " << opt_camera << std::endl;
    } else if (opt_device == "USB" || opt_device == "usb") {
      g = factory.createPylonGrabber(vpPylonFactory::BASLER_USB);
      std::cout << "Opening Basler USB camera: " << opt_camera << std::endl;
    } else {
      std::cout << "Error: only Basler GigE or USB cameras are supported." << std::endl;
      return EXIT_SUCCESS;
    }
    g->setCameraIndex(opt_camera);

    g->open(I);

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    bool quit = false;
    while (! quit) {
      double t = vpTime::measureTimeMs();
      g->acquire(I);

      vpDisplay::display(I);

      quit = record_helper(opt_seqname, opt_record_mode, I);

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, I.getHeight() - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
