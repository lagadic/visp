//! \example tutorial-draw-rectangle.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
  vpImage<unsigned char> I(2160, 3840, 128);

  try {

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#endif
    
    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    //! [Rectangle]
    vpDisplay::displayRectangle(I, I.getHeight() / 4, I.getWidth() / 4, I.getWidth() / 2, I.getHeight() / 2, vpColor::red, true);
    //! [Rectangle]
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
