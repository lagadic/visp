#include "drawingHelpers.h"

#include <visp3/core/vpImageConvert.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool drawingHelpers::display(vpImage< vpRGBa> &I, const std::string &title, const bool &blockingMode)
{
  vpDisplay::display(I);
  vpDisplay::setTitle(I, title);
  vpDisplay::displayText(I, 15, 15, "Left click to continue...", vpColor::red);
  vpDisplay::displayText(I, 35, 15, "Right click to stop...", vpColor::red);
  vpDisplay::flush(I);
  vpMouseButton::vpMouseButtonType button;
  vpDisplay::getClick(I, button, blockingMode);
  bool hasToContinue = true;
  if (button == vpMouseButton::button3) {
    // Right click => stop the program
    hasToContinue = false;
  }

  return hasToContinue;
}

bool drawingHelpers::display(vpImage<unsigned char> &D, vpImage<vpRGBa> &Idisp, const std::string &title, const bool &blockingMode)
{
  vpImageConvert::convert(D, Idisp);
  return display(Idisp, title, blockingMode);
}

bool drawingHelpers::display(vpImage<double> &D, vpImage<vpRGBa> &Idisp, const std::string &title, const bool &blockingMode)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(D, I);
  vpImageConvert::convert(I, Idisp);
  return display(Idisp, title, blockingMode);
}

#endif
