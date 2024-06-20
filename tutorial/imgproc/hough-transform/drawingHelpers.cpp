#include "drawingHelpers.h"

#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_X11)
VISP_NAMESPACE_ADDRESSING vpDisplayX drawingHelpers::d;
#elif defined(VISP_HAVE_OPENCV)
VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV drawingHelpers::d;
#elif defined(VISP_HAVE_GTK)
VISP_NAMESPACE_ADDRESSING vpDisplayGTK drawingHelpers::d;
#elif defined(VISP_HAVE_GDI)
VISP_NAMESPACE_ADDRESSING vpDisplayGDI drawingHelpers::d;
#elif defined(VISP_HAVE_D3D9)
VISP_NAMESPACE_ADDRESSING vpDisplayD3D drawingHelpers::d;
#endif

VISP_NAMESPACE_ADDRESSING vpImage<vpRGBa> drawingHelpers::I_disp;

bool drawingHelpers::display(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, const std::string &title, const bool &blockingMode)
{
  I_disp = I;
#if defined(VISP_HAVE_DISPLAY)
  if (!d.isInitialised()) {
    d.init(I_disp);
    vpDisplay::setTitle(I_disp, title);
  }
#else
  (void)title;
#endif

  vpDisplay::display(I_disp);
  vpDisplay::displayText(I_disp, 15, 15, "Left click to continue...", vpColor::red);
  vpDisplay::displayText(I_disp, 35, 15, "Right click to stop...", vpColor::red);
  vpDisplay::flush(I_disp);
  vpMouseButton::vpMouseButtonType button;
  vpDisplay::getClick(I_disp, button, blockingMode);
  bool hasToContinue = true;
  if (button == vpMouseButton::button3) {
    // Right click => stop the program
    hasToContinue = false;
  }

  return hasToContinue;
}

bool drawingHelpers::display(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<vpRGBa> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
}

bool drawingHelpers::display(VISP_NAMESPACE_ADDRESSING vpImage<double> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
}
