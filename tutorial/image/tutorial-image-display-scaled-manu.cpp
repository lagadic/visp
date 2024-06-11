//! \example tutorial-image-display-scaled-manu.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I(2160, 3840, 128);

  try {
//! [vpDisplay scale manu]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_5);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_5);
#endif
    //! [vpDisplay scale manu]
    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    vpDisplay::displayCircle(I, I.getHeight() / 2, I.getWidth() / 2, 200, vpColor::red, true);
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
