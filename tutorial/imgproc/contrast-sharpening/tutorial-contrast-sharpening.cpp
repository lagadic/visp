//! \example tutorial-contrast-sharpening.cpp

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
#if defined(VISP_HAVE_MODULE_IMGPROC) && defined(VISP_HAVE_DISPLAY)
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
      input_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--blockRadius" && i + 1 < argc) {
      blockRadius = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--bins" && i + 1 < argc) {
      bins = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--slope" && i + 1 < argc) {
      slope = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--sigma" && i + 1 < argc) {
      sigma = static_cast<float>(atof(argv[++i]));
    }
    else if (std::string(argv[i]) == "--weight" && i + 1 < argc) {
      weight = atof(argv[++i]);
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

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I_color, 0, 0, "Input color image");
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay(I_color, 0, 0, "Input color image");
#endif

  //! [Stretch contrast]
  vpImage<vpRGBa> I_stretch;
  VISP_NAMESPACE_NAME::stretchContrast(I_color, I_stretch);
  //! [Stretch contrast]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display2 = vpDisplayFactory::createDisplay(I_stretch, I_color.getWidth(), 10, "Stretch contrast");
#else
  vpDisplay *display2 = vpDisplayFactory::allocateDisplay(I_stretch, I_color.getWidth(), 10, "Stretch contrast");
#endif

  //! [Stretch contrast HSV]
  vpImage<vpRGBa> I_stretch_hsv;
  VISP_NAMESPACE_NAME::stretchContrastHSV(I_color, I_stretch_hsv);
  //! [Stretch contrast HSV]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display3 = vpDisplayFactory::createDisplay(I_stretch_hsv, 0, I_color.getHeight() + 80, "Stretch contrast HSV");
#else
  vpDisplay *display3 = vpDisplayFactory::allocateDisplay(I_stretch_hsv, 0, I_color.getHeight() + 80, "Stretch contrast HSV");
#endif

  //! [Histogram equalization]
  vpImage<vpRGBa> I_hist_eq;
  VISP_NAMESPACE_NAME::equalizeHistogram(I_color, I_hist_eq);
  //! [Histogram equalization]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display4 = vpDisplayFactory::createDisplay(I_hist_eq, I_color.getWidth(), I_color.getHeight() + 80, "Histogram equalization");
#else
  vpDisplay *display4 = vpDisplayFactory::allocateDisplay(I_hist_eq, I_color.getWidth(), I_color.getHeight() + 80, "Histogram equalization");
#endif

  //! [CLAHE]
  vpImage<vpRGBa> I_clahe;
  VISP_NAMESPACE_NAME::clahe(I_color, I_clahe, blockRadius, bins, slope);
  //! [CLAHE]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display5 = vpDisplayFactory::createDisplay(I_clahe, 0, 2 * I_color.getHeight() + 80, "CLAHE");
#else
  vpDisplay *display5 = vpDisplayFactory::allocateDisplay(I_clahe, 0, 2 * I_color.getHeight() + 80, "CLAHE");
#endif

  //! [Unsharp mask]
  vpImage<vpRGBa> I_unsharp;
  VISP_NAMESPACE_NAME::unsharpMask(I_clahe, I_unsharp, sigma, weight);
  //! [Unsharp mask]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display6 = vpDisplayFactory::createDisplay(I_unsharp, I_color.getWidth(), 2 * I_color.getHeight() + 80, "Unsharp mask");
#else
  vpDisplay *display6 = vpDisplayFactory::allocateDisplay(I_unsharp, I_color.getWidth(), 2 * I_color.getHeight() + 80, "Unsharp mask");
#endif

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

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
  if (display != nullptr) {
    delete display;
  }

  if (display2 != nullptr) {
    delete display2;
  }

  if (display3 != nullptr) {
    delete display3;
  }

  if (display4 != nullptr) {
    delete display4;
  }

  if (display5 != nullptr) {
    delete display5;
  }

  if (display6 != nullptr) {
    delete display6;
  }
#endif
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
