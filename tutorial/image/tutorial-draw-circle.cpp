//! \example tutorial-draw-circle.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpImageDraw.h>

int main()
{
  vpImage<unsigned char> I(2160, 3840, 128);
  vpImage<vpRGBa> I_rgb(2160, 3840, vpColor(0, 0, 0));

  try {
    {
#if defined(VISP_HAVE_X11)
      vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#endif

      vpDisplay::setTitle(I, "Gray image");
      vpDisplay::display(I);
      //! [Circle display]
      vpImageCircle circle(vpImagePoint(I.getHeight()/3, I.getWidth()/3), I.getWidth()/10);
      // Displays in overlay a red circle on the image
      // i.e. does not modify I
      vpDisplay::displayCircle(I, circle, vpColor::red, 2);
      //! [Circle display]
      vpDisplay::flush(I);
      vpDisplay::setTitle(I, "Overlay");
      std::cout << "Result of displaying a red circle on overlay on the display..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I);

      //! [Circle draw uchar]
      vpImageCircle circle2(vpImagePoint(I.getHeight()/3, 2*I.getWidth()/3), I.getWidth()/10);
      // Draws a white circle on the image
      // i.e. modifies I
      vpImageDraw::drawCircle(I, circle2, 255, 2);
      //! [Circle draw uchar]
      vpDisplay::display(I);
      vpDisplay::flush(I);
      vpDisplay::setTitle(I, "Modification of a uchar image");
      std::cout << "Result of the modification of a uchar image..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I);
    }

    {
      //! [Circle draw color]
      vpImageCircle circle3(vpImagePoint(2*I.getHeight()/3, I.getWidth()/2), I.getWidth()/10);
      // Draws a blue circle on the image
      // i.e. modifies I_rgb
      vpImageDraw::drawCircle(I_rgb, circle3, vpColor::blue, 2);
      //! [Circle draw color]

#if defined(VISP_HAVE_X11)
      vpDisplayX d_rgb(I_rgb, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI d_rgb(I_rgb, vpDisplay::SCALE_AUTO);
#endif

      vpDisplay::setTitle(I_rgb, "Color image");
      vpDisplay::display(I_rgb);
      vpDisplay::flush(I_rgb);
      vpDisplay::setTitle(I, "Modification of a vpRGBa image");
      std::cout << "Result of the modification of a vpRGBa image..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I_rgb);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
  std::cout << std::endl;
}
