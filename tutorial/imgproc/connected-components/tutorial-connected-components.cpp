//! \example tutorial-connected-components.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_MODULE_IMGPROC)
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string input_filename = "img.pgm";
  vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--connexity" && i + 1 < argc) {
      connexity = (vpImageMorphology::vpConnexityType)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input image>] [--connexity <0: 4-connexity, "
        "1: 8-connexity>] [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Read]
  vpImage<unsigned char> I;
  vpImageIo::read(I, input_filename);
  //! [Read]

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 0, 0, "Input image");
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay(I, 0, 0, "Input image");
#endif

  //! [Connected components]
  vpImage<int> labels;
  int nbComponents = 0;
  VISP_NAMESPACE_NAME::connectedComponents(I, labels, nbComponents, connexity);
  std::cout << "nbComponents=" << nbComponents << std::endl;
  //! [Connected components]

  //! [Draw connected components]
  vpImage<vpRGBa> I_conn(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < I_conn.getHeight(); i++) {
    for (unsigned int j = 0; j < I_conn.getWidth(); j++) {
      if (labels[i][j] != 0) {
        I_conn[i][j] =
          vpRGBa(vpColor::getColor((unsigned int)labels[i][j]).R, vpColor::getColor((unsigned int)labels[i][j]).G,
                 vpColor::getColor((unsigned int)labels[i][j]).B);
      }
    }
  }
  //! [Draw connected components]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display2 = vpDisplayFactory::createDisplay(I_conn, I.getWidth(), 10, "Connected components");
#else
  vpDisplay *display2 = vpDisplayFactory::allocateDisplay(I_conn, I.getWidth(), 10, "Connected components");
#endif

  vpDisplay::display(I);
  vpDisplay::display(I_conn);
  vpDisplay::displayText(I_conn, 20, 20, "Click to quit.", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::flush(I_conn);
  vpDisplay::getClick(I_conn);

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
