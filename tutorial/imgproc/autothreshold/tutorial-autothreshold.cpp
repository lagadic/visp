//! \example tutorial-autothreshold.cpp

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

  std::string input_filename = "grid36-03.pgm";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0] << " [--input <input image>] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpImage<unsigned char> I;
  vpImageIo::read(I, input_filename);

  vpImage<unsigned char> I_res(3 * I.getHeight(), 3 * I.getWidth());
  I_res.insert(I, vpImagePoint(I.getHeight(), I.getWidth()));

#ifdef VISP_HAVE_X11
  vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d;
#endif
  d.setDownScalingFactor(vpDisplay::SCALE_2);
  d.init(I_res);

  //! [Huang]
  vpImage<unsigned char> I_huang = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_huang, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_HUANG);
  //! [Huang]
  I_res.insert(I_huang, vpImagePoint());

  //! [Intermodes]
  vpImage<unsigned char> I_intermodes = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_intermodes, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_INTERMODES);
  //! [Intermodes]
  I_res.insert(I_intermodes, vpImagePoint(0, I.getWidth()));

  //! [IsoData]
  vpImage<unsigned char> I_isodata = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_isodata, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_ISODATA);
  //! [IsoData]
  I_res.insert(I_isodata, vpImagePoint(0, 2 * I.getWidth()));

  //! [Mean]
  vpImage<unsigned char> I_mean = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_mean, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_MEAN);
  //! [Mean]
  I_res.insert(I_mean, vpImagePoint(I.getHeight(), 0));

  //! [Otsu]
  vpImage<unsigned char> I_otsu = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_otsu, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_OTSU);
  //! [Otsu]
  I_res.insert(I_otsu, vpImagePoint(I.getHeight(), 2 * I.getWidth()));

  //! [Triangle]
  vpImage<unsigned char> I_triangle = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_triangle, VISP_NAMESPACE_NAME::AUTO_THRESHOLD_TRIANGLE);
  //! [Triangle]
  I_res.insert(I_triangle, vpImagePoint(2 * I.getHeight(), 0));

  vpDisplay::display(I_res);

  vpDisplay::displayText(I_res, 30, 20, "Huang", vpColor::red);
  vpDisplay::displayText(I_res, 30, 20 + I.getWidth(), "Intermodes", vpColor::red);
  vpDisplay::displayText(I_res, 30, 20 + 2 * I.getWidth(), "IsoData", vpColor::red);
  vpDisplay::displayText(I_res, 30 + I.getHeight(), 20, "Mean", vpColor::red);
  vpDisplay::displayText(I_res, 30 + I.getHeight(), 20 + I.getWidth(), "Original", vpColor::red);
  vpDisplay::displayText(I_res, 30 + I.getHeight(), 20 + 2 * I.getWidth(), "Otsu", vpColor::red);
  vpDisplay::displayText(I_res, 30 + 2 * I.getHeight(), 20, "Triangle", vpColor::red);

  vpDisplay::flush(I_res);
  vpDisplay::getClick(I_res);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
