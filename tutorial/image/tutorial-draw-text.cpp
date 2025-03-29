//! \example tutorial-draw-text.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);

  // Initialize the display with the image I. Display and image are now linked together
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Text drawing", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, 10, 10, "Text drawing", vpDisplay::SCALE_AUTO);
#endif

  vpDisplay::display(I);
  vpDisplay::displayRectangle(I, I.getHeight() / 4, I.getWidth() / 4, I.getWidth() / 2, I.getHeight() / 2,
                              vpColor::red, true);
  //! [text]
  vpDisplay::displayText(I, I.getHeight() / 2, I.getWidth() / 2, "Hello World!", vpColor::yellow);
  //! [text]
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
