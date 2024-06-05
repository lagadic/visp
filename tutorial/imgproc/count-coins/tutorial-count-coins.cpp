//! \example tutorial-count-coins.cpp

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
#include <visp3/core/vpMomentObject.h>
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

int main(int argc, char *argv[])
{
//! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string input_filename = "coins1.jpg";
  VISP_NAMESPACE_NAME::vpAutoThresholdMethod method = VISP_NAMESPACE_NAME::AUTO_THRESHOLD_OTSU;
  bool white_foreground = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--method" && i + 1 < argc) {
      method = static_cast<VISP_NAMESPACE_NAME::vpAutoThresholdMethod>(atoi(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--white_foreground") {
      white_foreground = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input image>]"
        " [--method <0: Huang, 1: Intermodes, 2: IsoData, 3: "
        "Mean, 4: Otsu, 5: Triangle>]"
        " [--white_foreground]"
        " [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Read]
  vpImage<unsigned char> I;
  vpImageIo::read(I, input_filename);
  //! [Read]

#ifdef VISP_HAVE_X11
  vpDisplayX d, d2, d3, d4, d5;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d, d2, d3, d4, d5;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d, d2, d3, d4, d5;
#endif
  d.init(I, 0, 0, "Coins");

  vpImage<unsigned char> I_bin, I_fill;
  //! [Binarisation]
  I_bin = I;
  VISP_NAMESPACE_NAME::autoThreshold(I_bin, method, white_foreground ? 0 : 255, white_foreground ? 255 : 0);
  //! [Binarisation]
  d2.init(I_bin, I.getWidth(), 0, "Binarisation");

  //! [Fill holes]
  I_fill = I_bin;
  VISP_NAMESPACE_NAME::fillHoles(I_fill);
  //! [Fill holes]
  d3.init(I_fill, 0, I.getHeight() + 80, "Fill holes");

  //! [Opening]
  vpImage<unsigned char> I_open = I_fill;
  vpImageMorphology::erosion<unsigned char>(I_open, vpImageMorphology::CONNEXITY_4);
  vpImageMorphology::dilatation<unsigned char>(I_open, vpImageMorphology::CONNEXITY_4);
  //! [Opening]

  //! [Closing]
  vpImage<unsigned char> I_close = I_open;
  vpImageMorphology::dilatation<unsigned char>(I_close, vpImageMorphology::CONNEXITY_4);
  vpImageMorphology::erosion<unsigned char>(I_close, vpImageMorphology::CONNEXITY_4);
  //! [Closing]
  d4.init(I_close, I.getWidth(), I.getHeight() + 80, "Closing");

  //! [Find contours]
  vpImage<unsigned char> I_contours(I_close.getHeight(), I_close.getWidth());
  for (unsigned int cpt = 0; cpt < I_close.getSize(); cpt++)
    I_contours.bitmap[cpt] = I_close.bitmap[cpt] ? 1 : 0;

  VISP_NAMESPACE_NAME::vpContour vp_contours;
  std::vector<std::vector<vpImagePoint> > contours;
  VISP_NAMESPACE_NAME::findContours(I_contours, vp_contours, contours, VISP_NAMESPACE_NAME::CONTOUR_RETR_EXTERNAL);
  //! [Find contours]

  //! [Draw contours]
  vpImage<vpRGBa> I_draw_contours(I_contours.getHeight(), I_contours.getWidth(), vpRGBa());
  VISP_NAMESPACE_NAME::drawContours(I_draw_contours, contours, vpColor::red);
  //! [Draw contours]
  d5.init(I_draw_contours, 0, 2 * I.getHeight() + 80, "Contours");

  vpDisplay::display(I);
  vpDisplay::display(I_bin);
  vpDisplay::display(I_fill);
  vpDisplay::display(I_close);
  vpDisplay::display(I_draw_contours);

  //! [Count coins]
  int nb_coins = 0;
  for (size_t i = 0; i < contours.size(); i++) {
    std::vector<vpPoint> vec_p;

    for (size_t j = 0; j < contours[i].size(); j++) {
      vpPoint pt;
      pt.set_x(contours[i][j].get_u());
      pt.set_y(contours[i][j].get_v());
      vec_p.push_back(pt);
    }

    vpMomentObject obj(1);
    obj.setType(vpMomentObject::DENSE_POLYGON);
    obj.fromVector(vec_p);

    // sign(m00) depends of the contour orientation (clockwise or
    // counter-clockwise)  that's why we use fabs
    if (std::fabs(obj.get(0, 0)) >= I.getSize() / 200) {
      nb_coins++;
      std::stringstream ss;
      ss << "Coin " << nb_coins;

      int centroid_x = (int)std::fabs(obj.get(1, 0) / obj.get(0, 0));
      int centroid_y = (int)std::fabs(obj.get(0, 1) / obj.get(0, 0));
      vpDisplay::displayText(I_draw_contours, centroid_y, centroid_x - 20, ss.str(), vpColor::red);
    }
  }
  //! [Count coins]

  vpDisplay::displayText(I_draw_contours, 20, 20, "Click to quit.", vpColor::red);

  vpDisplay::flush(I);
  vpDisplay::flush(I_bin);
  vpDisplay::flush(I_fill);
  vpDisplay::flush(I_close);
  vpDisplay::flush(I_draw_contours);
  vpDisplay::getClick(I_draw_contours);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
