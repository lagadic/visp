//! \example tutorial-contour.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]

namespace
{
//! [Print contours hierarchy func]
void displayContourInfo(const vp::vpContour &contour, int level)
{
  std::cout << "\nContour:" << std::endl;
  std::cout << "\tlevel: " << level << std::endl;
  std::cout << "\tcontour type: " << (contour.m_contourType == vp::CONTOUR_OUTER ? "outer contour" : "hole contour")
            << std::endl;
  std::cout << "\tcontour size: " << contour.m_points.size() << std::endl;
  std::cout << "\tnb children: " << contour.m_children.size() << std::endl;

  for (std::vector<vp::vpContour *>::const_iterator it = contour.m_children.begin(); it != contour.m_children.end();
       ++it) {
    displayContourInfo(**it, level + 1);
  }
}
//! [Print contours hierarchy func]

//! [Draw contours hierarchical func]
void drawContoursTree(vpImage<vpRGBa> &I, const vp::vpContour &contour)
{
  std::vector<std::vector<vpImagePoint> > contours;
  contours.push_back(contour.m_points);
  vp::drawContours(I, contours, contour.m_contourType == vp::CONTOUR_OUTER ? vpColor::red : vpColor::green);

  for (std::vector<vp::vpContour *>::const_iterator it = contour.m_children.begin(); it != contour.m_children.end();
       ++it) {
    drawContoursTree(I, **it);
  }
}
//! [Draw contours hierarchical func]
} // namespace
#endif

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
  //!
  std::string input_filename = "grid36-03.pgm";
  bool white_foreground = false;
  vp::vpContourRetrievalType extraction_method = vp::CONTOUR_RETR_TREE;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--white_foreground") {
      white_foreground = true;
    } else if (std::string(argv[i]) == "--method" && i + 1 < argc) {
      extraction_method = (vp::vpContourRetrievalType)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <input image>] [--method <0: "
                   "CONTOUR_RETR_TREE, 1: CONTOUR_RETR_LIST, 2: "
                   "CONTOUR_RETR_EXTERNAL>]"
                   " [--white_foreground] [--help]"
                << std::endl;
      return EXIT_SUCCESS;
    }
  }

  //! [Read]
  vpImage<unsigned char> I;
  vpImageIo::read(I, input_filename);
  //! [Read]
  vpImage<unsigned char> I_bin(I.getHeight(), I.getWidth());

  vpImage<vpRGBa> I_draw_contours(I.getHeight(), I.getWidth());

#ifdef VISP_HAVE_X11
  vpDisplayX d, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d, d2;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d, d2;
#endif
  d.init(I_bin, 0, 0, "After binarisation");
  d2.init(I_draw_contours, I_bin.getWidth(), 10, "Contours");

  //! [Otsu]
  vp::autoThreshold(I, vp::AUTO_THRESHOLD_OTSU, white_foreground ? 0 : 1, white_foreground ? 1 : 0);
  //! [Otsu]
  for (unsigned int i = 0; i < I_bin.getSize(); i++) {
    I_bin.bitmap[i] = 255 * I.bitmap[i];
  }

  //! [Find contours]
  vp::vpContour vp_contours;
  std::vector<std::vector<vpImagePoint> > contours;
  vp::findContours(I, vp_contours, contours, extraction_method);
  //! [Find contours]

  //! [Draw contours]
  vp::drawContours(I_draw_contours, contours, vpColor::red);
  //! [Draw contours]

  vpDisplay::display(I_bin);
  vpDisplay::display(I_draw_contours);
  vpDisplay::displayText(I_draw_contours, 20, 20, "Click to draw outer / hole contours.", vpColor::red);
  vpDisplay::flush(I_bin);
  vpDisplay::flush(I_draw_contours);
  vpDisplay::getClick(I_draw_contours);

  I_draw_contours = 0;
  //! [Draw contours hierarchical]
  drawContoursTree(I_draw_contours, vp_contours);
  //! [Draw contours hierarchical]
  displayContourInfo(vp_contours, 0);

  vpDisplay::display(I_bin);
  vpDisplay::display(I_draw_contours);
  vpDisplay::displayText(I_draw_contours, 20, 20, "Click to quit.", vpColor::red);
  vpDisplay::displayText(I_draw_contours, 20, I_draw_contours.getWidth() - 200, "Outer contour", vpColor::red);
  vpDisplay::displayText(I_draw_contours, 20, I_draw_contours.getWidth() - 100, "Hole contour", vpColor::green);
  vpDisplay::flush(I_bin);
  vpDisplay::flush(I_draw_contours);
  vpDisplay::getClick(I_draw_contours);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
