//! \example tutorial-contrast-sharpening.cpp

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
  //!

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string input_filename = "Crayfish-low-contrast.png";
  int blockRadius = 150;
  int bins = 256;
  float slope = 3.0f;
  float sigma = 2.0f;
  double weight = 0.5;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--blockRadius" && i + 1 < argc) {
      blockRadius = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--bins" && i + 1 < argc) {
      bins = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--slope" && i + 1 < argc) {
      slope = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--sigma" && i + 1 < argc) {
      sigma = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--weight" && i + 1 < argc) {
      weight = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input image>]"
        " [--blockRadius <block radius for CLAHE>] "
        " [--bins <nb histogram bins for CLAHE>] [--slope <slope for CLAHE>]"
        " [--sigma <Gaussian kernel standard deviation>] [--weight <unsharp mask weighting>]"
        " [--help] [-h]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Read]
  vpImage<vpRGBa> I_color;
  vpImageIo::read(I_color, input_filename);
  //! [Read]

#ifdef VISP_HAVE_X11
  vpDisplayX d, d2, d3, d4, d5, d6;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d, d2, d3, d4, d5, d6;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d, d2, d3, d4, d5, d6;
#endif
  d.init(I_color, 0, 0, "Input color image");

  //! [Stretch contrast]
  vpImage<vpRGBa> I_stretch;
  VISP_NAMESPACE_NAME::stretchContrast(I_color, I_stretch);
  //! [Stretch contrast]
  d2.init(I_stretch, I_color.getWidth(), 10, "Stretch contrast");

  //! [Stretch contrast HSV]
  vpImage<vpRGBa> I_stretch_hsv;
  VISP_NAMESPACE_NAME::stretchContrastHSV(I_color, I_stretch_hsv);
  //! [Stretch contrast HSV]
  d3.init(I_stretch_hsv, 0, I_color.getHeight() + 80, "Stretch contrast HSV");

  //! [Histogram equalization]
  vpImage<vpRGBa> I_hist_eq;
  VISP_NAMESPACE_NAME::equalizeHistogram(I_color, I_hist_eq);
  //! [Histogram equalization]
  d4.init(I_hist_eq, I_color.getWidth(), I_color.getHeight() + 80, "Histogram equalization");

  //! [CLAHE]
  vpImage<vpRGBa> I_clahe;
  VISP_NAMESPACE_NAME::clahe(I_color, I_clahe, blockRadius, bins, slope);
  //! [CLAHE]
  d5.init(I_clahe, 0, 2 * I_color.getHeight() + 80, "CLAHE");

  //! [Unsharp mask]
  vpImage<vpRGBa> I_unsharp;
  VISP_NAMESPACE_NAME::unsharpMask(I_clahe, I_unsharp, sigma, weight);
  //! [Unsharp mask]
  d6.init(I_unsharp, I_color.getWidth(), 2 * I_color.getHeight() + 80, "Unsharp mask");

  vpDisplay::display(I_color);
  vpDisplay::display(I_stretch);
  vpDisplay::display(I_stretch_hsv);
  vpDisplay::display(I_hist_eq);
  vpDisplay::display(I_clahe);
  vpDisplay::display(I_unsharp);
  vpDisplay::displayText(I_unsharp, 20, 20, "Click to quit.", vpColor::red);
  vpDisplay::flush(I_color);
  vpDisplay::flush(I_stretch);
  vpDisplay::flush(I_stretch_hsv);
  vpDisplay::flush(I_hist_eq);
  vpDisplay::flush(I_clahe);
  vpDisplay::flush(I_unsharp);
  vpDisplay::getClick(I_unsharp);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
