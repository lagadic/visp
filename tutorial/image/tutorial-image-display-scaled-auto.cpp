//! \example tutorial-image-display-scaled-auto.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);

  //! [vpDisplay scale auto]
  // Initialize the display with the image I. Display and image are now linked together
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Auto scaling display", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Auto scaling display", vpDisplay::SCALE_AUTO);
#endif
    //! [vpDisplay scale auto]

  vpDisplay::display(I);
  //! [Circle]
  vpDisplay::displayCircle(I, I.getHeight() / 2, I.getWidth() / 2, 200, vpColor::red, true);
  //! [Circle]
  vpDisplay::flush(I);
  std::cout << "A click to quit..." << std::endl;
  vpDisplay::getClick(I);

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  delete d;
#endif

#else
  std::cout << "No gui available to display an image..." << std::endl;
#endif

  return EXIT_SUCCESS;
}
