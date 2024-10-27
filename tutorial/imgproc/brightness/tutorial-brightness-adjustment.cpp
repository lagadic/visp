//! \example tutorial-brightness-adjustment.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_MODULE_IMGPROC)
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L)))
#include <memory>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void display(vpImage<vpRGBa> &I_display, vpImage<vpRGBa> &I_color_res, const vpImage<vpRGBa> &I_color_adjust,
  vpImage<unsigned char> &I_gray_res, vpImage<unsigned char> &I_gray_adjust, vpImage<vpRGBa> &I_gray_display,
  const std::string &title, const std::string &filename_color, const std::string &filename_gray,
  const std::string &title_2 = "")
{
  I_color_res.insert(I_color_adjust, vpImagePoint(0, I_color_adjust.getWidth()));
  I_display.insert(I_color_adjust, vpImagePoint(0, I_color_adjust.getWidth()));

  I_gray_res.insert(I_gray_adjust, vpImagePoint(0, I_gray_adjust.getWidth()));
  vpImageConvert::convert(I_gray_adjust, I_gray_display);
  I_display.insert(I_gray_display, vpImagePoint(I_color_adjust.getHeight(), I_color_adjust.getWidth()));

  vpImageIo::write(I_color_res, filename_color);
  vpImageIo::write(I_gray_res, filename_gray);

  vpDisplay::display(I_display);
  vpDisplay::displayText(I_display, 20, 20, title, vpColor::red);
  if (!title_2.empty()) {
    vpDisplay::displayText(I_display, 40, static_cast<unsigned int>(I_color_adjust.getWidth()*0.85),
      title_2, vpColor::green);
  }
  vpDisplay::flush(I_display);
  vpDisplay::getClick(I_display);
}
}

int main(int argc, const char **argv)
{
  //! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && defined(VISP_HAVE_DISPLAY) && \
  (defined(VISP_HAVE_PNG) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_STBIMAGE) || defined(VISP_HAVE_SIMDLIB)) && \
  ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L)))
  //! [Macro defined]

  std::string input_filename = "Sample_low_brightness.png";
  double alpha = 10.0, beta = 50.0;
  double gamma = 3.5;
  VISP_NAMESPACE_NAME::vpGammaMethod method = VISP_NAMESPACE_NAME::GAMMA_MANUAL;
  VISP_NAMESPACE_NAME::vpGammaColorHandling colorHandling = VISP_NAMESPACE_NAME::GAMMA_HSV;
  int scale = 240, scaleDiv = 3, level = 0, kernelSize = -1;
  double dynamic = 3.0;
  int scale_display = 2;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      ++i;
      input_filename = std::string(argv[i]);
    }
    else if (std::string(argv[i]) == "--alpha" && i + 1 < argc) {
      ++i;
      alpha = atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--beta" && i + 1 < argc) {
      ++i;
      beta = atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--gamma" && i + 1 < argc) {
      ++i;
      gamma = atof(argv[i]);
    }
    else if ((std::string(argv[i]) == "--gamma-color-handling") && ((i + 1) < argc)) {
      ++i;
      colorHandling = VISP_NAMESPACE_NAME::vpGammaColorHandlingFromString(argv[i]);
    }
    else if ((std::string(argv[i]) == "--gamma-method") && ((i + 1) < argc)) {
      ++i;
      method = VISP_NAMESPACE_NAME::vpGammaMethodFromString(argv[i]);
    }
    else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
      ++i;
      scale = atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--scaleDiv" && i + 1 < argc) {
      ++i;
      scaleDiv = atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--level" && i + 1 < argc) {
      ++i;
      level = atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--kernelSize" && i + 1 < argc) {
      ++i;
      kernelSize = atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--dynamic" && i + 1 < argc) {
      ++i;
      dynamic = atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--scale-display" && i + 1 < argc) {
      ++i;
      scale_display = atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input image>]"
        " [--alpha <alpha for adjust()>] [--beta <beta for adjust()>]"
        " [--gamma <gamma for gammaCorrection()>]"
        " [--gamma-color-handling " << VISP_NAMESPACE_NAME::vpGammaColorHandlingList() << "]"
        " [--gamma-method " << VISP_NAMESPACE_NAME::vpGammaMethodList() << "]"
        " [--scale <scale for retinex()> [--scaleDiv for retinex()]"
        " [--level <level for retinex()> [--kernelSize <kernelSize for retinex()>]"
        " [--dynamic <dynamic for retinex()>] "
        " [--scale-display <display downscaling factor>] "
        " [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // Filename without extension to save the results
  const std::string input_name = vpIoTools::getNameWE(input_filename);

  vpImage<vpRGBa> I_color;
  vpImageIo::read(I_color, input_filename);
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I_color, I_gray);
  vpImage<vpRGBa> I_gray_display;
  vpImageConvert::convert(I_gray, I_gray_display);

  // Side-by-side images
  vpImage<vpRGBa> I_color_res(I_color.getHeight(), 2 * I_color.getWidth());
  I_color_res.insert(I_color, vpImagePoint());
  vpImage<unsigned char> I_gray_res(I_gray.getHeight(), 2 * I_gray.getWidth());
  I_gray_res.insert(I_gray, vpImagePoint());

  // Side-by-side display for color (top) and gray (bottom) images
  vpImage<vpRGBa> I_display(2 * I_color.getHeight(), 2 * I_color.getWidth());
  I_display.insert(I_color, vpImagePoint());
  I_display.insert(I_gray_display, vpImagePoint(I_color.getHeight(), 0));
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay();
  d->setDownScalingFactor(static_cast<vpDisplay::vpScaleType>(scale_display));
  d->init(I_display, 10, 10, "Brightness adjustment results");

  //! [Brightness contrast adjustment]
  vpImage<vpRGBa> I_color_adjust;
  VISP_NAMESPACE_NAME::adjust(I_color, I_color_adjust, alpha, beta);
  vpImage<unsigned char> I_gray_adjust;
  VISP_NAMESPACE_NAME::adjust(I_gray, I_gray_adjust, alpha, beta);
  //! [Brightness contrast adjustment]

  std::stringstream ss_color;
  ss_color << input_name << "_adjust_alpha=" << alpha << "_beta=" << beta << ".png";
  std::stringstream ss_gray;
  ss_gray << input_name << "_adjust_alpha=" << alpha << "_beta=" << beta << "_gray.png";
  display(I_display, I_color_res, I_color_adjust, I_gray_res, I_gray_adjust, I_gray_display,
    "Brightness and contrast adjustment. Click to continue.", ss_color.str(), ss_gray.str());

  //! [Gamma correction]
  if (method != VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
    // If the user wants to use an automatic method, the gamma factor must be negative.
    gamma = -1.;
  }

  if (gamma > 0.) {
    // If the user wants to set a constant user-defined gamma factor, the method must be set to manual.
    method = VISP_NAMESPACE_NAME::GAMMA_MANUAL;
  }
  vpImage<vpRGBa> I_color_gamma_correction;
  VISP_NAMESPACE_NAME::gammaCorrection(I_color, I_color_gamma_correction, static_cast<float>(gamma), colorHandling, method);
  vpImage<unsigned char> I_gray_gamma_correction;
  VISP_NAMESPACE_NAME::gammaCorrection(I_gray, I_gray_gamma_correction, static_cast<float>(gamma), method);
  //! [Gamma correction]

  ss_color.str("");
  ss_color << input_name << "_gamma=" << gamma << ".png";
  ss_gray.str("");
  ss_gray << input_name << "_gamma=" << gamma << "_gray.png";
  display(I_display, I_color_res, I_color_gamma_correction, I_gray_res, I_gray_gamma_correction, I_gray_display,
    "Gamma correction. Click to continue.", ss_color.str(), ss_gray.str());

  // Display results for the different Gamma correction method
  for (int gamma_idx = 0; gamma_idx < VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT; ++gamma_idx) {
    gamma = -1.;
    VISP_NAMESPACE_NAME::vpGammaMethod gamma_method = static_cast<VISP_NAMESPACE_NAME::vpGammaMethod>(gamma_idx);
    if (gamma_method == VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
      continue;
    }

    vpImage<vpRGBa> I_color_gamma_correction;
    VISP_NAMESPACE_NAME::gammaCorrection(I_color, I_color_gamma_correction, static_cast<float>(gamma), colorHandling,
      gamma_method);
    vpImage<unsigned char> I_gray_gamma_correction;
    VISP_NAMESPACE_NAME::gammaCorrection(I_gray, I_gray_gamma_correction, static_cast<float>(gamma), gamma_method);

    const std::string gamma_name = VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method);
    ss_color.str("");
    ss_color << input_name << "_" << gamma_name << ".png";
    ss_gray.str("");
    ss_gray << input_name << "_" << gamma_name << "_gray.png";
    display(I_display, I_color_res, I_color_gamma_correction, I_gray_res, I_gray_gamma_correction, I_gray_display,
      "Gamma correction. Click to continue.", ss_color.str(), ss_gray.str(), gamma_name);
  }

  //! [Histogram equalization]
  vpImage<vpRGBa> I_color_equalize_histogram;
  VISP_NAMESPACE_NAME::equalizeHistogram(I_color, I_color_equalize_histogram);
  vpImage<unsigned char> I_gray_equalize_histogram;
  VISP_NAMESPACE_NAME::equalizeHistogram(I_gray, I_gray_equalize_histogram);
  //! [Histogram equalization]

  ss_color.str("");
  ss_color << input_name << "_eqHist.png";
  ss_gray.str("");
  ss_gray << input_name << "_eqHist_gray.png";
  display(I_display, I_color_res, I_color_equalize_histogram, I_gray_res, I_gray_equalize_histogram, I_gray_display,
    "Histogram equalization. Click to continue.", ss_color.str(), ss_gray.str());

  //! [Retinex]
  vpImage<vpRGBa> I_color_retinex;
  VISP_NAMESPACE_NAME::retinex(I_color, I_color_retinex, scale, scaleDiv, level, dynamic, kernelSize);
  // Retinex uses color image as input
  // Convert gray image into RGBa format for quick test
  vpImage<vpRGBa> I_gray_color;
  vpImageConvert::convert(I_gray, I_gray_color);
  vpImage<vpRGBa> I_gray_color_retinex;
  VISP_NAMESPACE_NAME::retinex(I_gray_color, I_gray_color_retinex, scale, scaleDiv, level, dynamic, kernelSize);
  // Convert back to gray
  vpImage<unsigned char> I_gray_retinex;
  vpImageConvert::convert(I_gray_color_retinex, I_gray_retinex);
  //! [Retinex]

  ss_color.str("");
  ss_color << input_name << "_Retinex_scale=" << scale << "_scaleDiv=" << scaleDiv << "_level=" << level
    << "_dynamic=" << dynamic << "_kernelSize=" << kernelSize << ".png";
  ss_gray.str("");
  ss_gray << input_name << "_Retinex_scale=" << scale << "_scaleDiv=" << scaleDiv << "_level=" << level
    << "_dynamic=" << dynamic << "_kernelSize=" << kernelSize << "_gray.png";
  display(I_display, I_color_res, I_color_retinex, I_gray_res, I_gray_retinex, I_gray_display,
    "Retinex. Click to quit.", ss_color.str(), ss_gray.str());
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
