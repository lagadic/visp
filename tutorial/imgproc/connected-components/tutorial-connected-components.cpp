//! \example tutorial-connected-components.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
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

#ifdef VISP_HAVE_X11
  vpDisplayX d, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d, d2;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d, d2;
#endif
  d.init(I, 0, 0, "Input image");

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
  d2.init(I_conn, I.getWidth(), 10, "Connected components");

  vpDisplay::display(I);
  vpDisplay::display(I_conn);
  vpDisplay::displayText(I_conn, 20, 20, "Click to quit.", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::flush(I_conn);
  vpDisplay::getClick(I_conn);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
