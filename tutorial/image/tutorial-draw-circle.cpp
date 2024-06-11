//! \example tutorial-draw-circle.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpImageDraw.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I(2160, 3840, 128);
  vpImage<vpRGBa> I_rgb(2160, 3840, vpColor(0, 0, 0));

  try {
    {
#if defined(VISP_HAVE_DISPLAY)
      vpDisplay *d = vpDisplayFactory::displayFactory(I, vpDisplay::SCALE_AUTO);
#else
      std::cout << "No gui available to display gray level image..." << std::endl;
#endif

      vpDisplay::setTitle(I, "Gray image");
      vpDisplay::display(I);
      //! [Circle display]
      vpImageCircle circle(vpImagePoint(I.getHeight()/3, I.getWidth()/3), I.getWidth()/10.f);
      // Displays in overlay a red circle on the image
      // i.e. does not modify I
      vpDisplay::displayCircle(I, circle, vpColor::red, false, 2);
      //! [Circle display]
      vpDisplay::setTitle(I, "Display a red circle on gray level image overlay");
      vpDisplay::flush(I);
      std::cout << "Result of displaying a red circle on a gray level image overlay..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I);

      //! [Circle draw uchar]
      vpImageCircle circle2(vpImagePoint(I.getHeight()/3, 2*I.getWidth()/3), I.getWidth()/10.f);
      // Draws a white circle on the image
      // i.e. modifies I
      vpImageDraw::drawCircle(I, circle2, 255, 2);
      //! [Circle draw uchar]
      vpDisplay::display(I);
      vpDisplay::setTitle(I, "Display circle by modifying a gray level image");
      std::cout << "Result of displaying a circle by modifying a gray level image..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::flush(I);
      vpDisplay::getClick(I);

#if defined(VISP_HAVE_DISPLAY)
      if (d) {
        delete d;
    }
#endif
  }

    {
      //! [Circle draw color]
      vpImageCircle circle3(vpImagePoint(2*I.getHeight()/3, I.getWidth()/2), I.getWidth()/10.f);
      // Draws a blue circle on the image
      // i.e. modifies I_rgb
      vpImageDraw::drawCircle(I_rgb, circle3, vpColor::blue, 2);
      //! [Circle draw color]

#if defined(VISP_HAVE_DISPLAY)
      vpDisplay *d = vpDisplayFactory::displayFactory(I_rgb, vpDisplay::SCALE_AUTO);
#else
      std::cout << "No gui available to display color image..." << std::endl;
#endif

      vpDisplay::display(I_rgb);
      vpDisplay::setTitle(I_rgb, "Display blue circle on a modified color image");
      vpDisplay::flush(I_rgb);
      std::cout << "Result of displaying a blue circle on a modified color image..." << std::endl;
      std::cout << "A click to continue..." << std::endl;
      vpDisplay::getClick(I_rgb);
#if defined(VISP_HAVE_DISPLAY)
      if (d) {
        delete d;
    }
#endif
}
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
  std::cout << std::endl;
}
