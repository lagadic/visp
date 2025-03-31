//! \example tutorial-image-display-scaled-manu.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);

  //! [vpDisplay scale manu]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Manual scaling display", vpDisplay::SCALE_5);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Manual scaling display", vpDisplay::SCALE_5);
#endif
    //! [vpDisplay scale manu]
  vpDisplay::display(I);
  vpDisplay::displayCircle(I, I.getHeight() / 2, I.getWidth() / 2, 200, vpColor::red, true);
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
