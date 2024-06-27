#include "drawingHelpers.h"

#include <visp3/core/vpImageConvert.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_X11)
vpDisplayX drawingHelpers::d;
#elif defined(VISP_HAVE_OPENCV)
vpDisplayOpenCV drawingHelpers::d;
#elif defined(VISP_HAVE_GTK)
vpDisplayGTK drawingHelpers::d;
#elif defined(VISP_HAVE_GDI)
vpDisplayGDI drawingHelpers::d;
#elif defined(VISP_HAVE_D3D9)
vpDisplayD3D drawingHelpers::d;
#endif

vpImage<vpRGBa> drawingHelpers::I_disp;

bool drawingHelpers::display(vpImage< vpRGBa> &I, const std::string &title, const bool &blockingMode)
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

bool drawingHelpers::display(vpImage<unsigned char> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<vpRGBa> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
}

bool drawingHelpers::display(vpImage<double> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
}

#endif
