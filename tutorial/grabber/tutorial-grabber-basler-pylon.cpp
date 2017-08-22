/*! \example tutorial-grabber-basler-pylon.cpp */
#include <visp3/sensor/vpPylonFactory.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpImage.h>

/*!
  Usage :
    To get the help    : ./tutorial-grabber-basler-pylon --help
    To set the device  : ./tutorial-grabber-basler-pylon --device GigE --camera 1
 */
int main(int argc, const char* argv[])
{
#ifdef VISP_HAVE_PYLON
  try {
    unsigned int opt_camera = 0;
    std::string opt_device("GigE");

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--camera")
        opt_camera = (unsigned int)atoi(argv[i+1]);
      if (std::string(argv[i]) == "--device")
        opt_device = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--camera <0...9>] [--device <\"GigE\"|\"USB\">] [--help]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;

    vpPylonFactory &factory = vpPylonFactory::instance();

    vpPylonGrabber *g;
    if (opt_device == "GigE" || opt_device == "gige") {
      g = factory.createPylonGrabber(vpPylonFactory::BASLER_GIGE);
      std::cout << "Opening Basler GigE camera: " << opt_camera << std::endl;
    }
    else if (opt_device == "USB" || opt_device == "usb") {
      g = factory.createPylonGrabber(vpPylonFactory::BASLER_USB);
      std::cout << "Opening Basler USB camera: " << opt_camera << std::endl;
    }
    else {
      std::cout << "Error: only Basler GigE or USB cameras are supported." << std::endl;
      return EXIT_SUCCESS;
    }
    g->setCameraIndex(opt_camera);

    g->open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      g->acquire(I);
      vpDisplay::display(I);
      vpDisplay::displayText(I, 10, 10, "A click to quit", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) break;
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
