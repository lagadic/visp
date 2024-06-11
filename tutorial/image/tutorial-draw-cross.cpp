//! \example tutorial-draw-cross.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I(2160, 3840, 128);

  try {
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#endif

    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    //! [Cross]
    vpDisplay::displayCross(I, I.getHeight() / 2, I.getWidth() / 2, I.getWidth() / 2, vpColor::red, 2);
    //! [Cross]
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
  std::cout << std::endl;
}
