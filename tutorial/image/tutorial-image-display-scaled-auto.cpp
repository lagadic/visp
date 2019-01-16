//! \example tutorial-image-display-scaled-auto.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
  vpImage<unsigned char> I(2160, 3840, 128);

  try {
//! [vpDisplay scale auto]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#endif
    //! [vpDisplay scale auto]

    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    //! [Circle]
    vpDisplay::displayCircle(I, I.getHeight() / 2, I.getWidth() / 2, 200, vpColor::red, true);
    //! [Circle]
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
