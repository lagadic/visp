//! \example tutorial-draw-line.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Line drawing", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Line drawing", vpDisplay::SCALE_AUTO);
#endif

  vpDisplay::display(I);
  //! [Line]
  vpDisplay::displayLine(I, I.getHeight() / 4, I.getWidth() / 4, (3 * I.getHeight()) / 4, (3 * I.getWidth()) / 4,
                         vpColor::red, 10);
  //! [Line]
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
