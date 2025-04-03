//! \example tutorial-draw-circle.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/core/vpImageCircle.h>
#include <visp3/core/vpImageDraw.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);
  vpImage<vpRGBa> I_rgb(2160, 3840, vpColor(0, 0, 0));

  {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "", vpDisplay::SCALE_AUTO);
#else
    vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "", vpDisplay::SCALE_AUTO);
#endif

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


#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    delete d;
#endif
  }

  {
    //! [Circle draw color]
    vpImageCircle circle3(vpImagePoint(2*I.getHeight()/3, I.getWidth()/2), I.getWidth()/10.f);
    // Draws a blue circle on the image
    // i.e. modifies I_rgb
    vpImageDraw::drawCircle(I_rgb, circle3, vpColor::blue, 2);
    //! [Circle draw color]

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I_rgb, -1, -1, "", vpDisplay::SCALE_AUTO);
#else
    vpDisplay *d = vpDisplayFactory::allocateDisplay(I_rgb, -1, -1, "", vpDisplay::SCALE_AUTO);
#endif

    vpDisplay::display(I_rgb);
    vpDisplay::setTitle(I_rgb, "Display blue circle on a modified color image");
    vpDisplay::flush(I_rgb);
    std::cout << "Result of displaying a blue circle on a modified color image..." << std::endl;
    std::cout << "A click to continue..." << std::endl;
    vpDisplay::getClick(I_rgb);


#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    delete d;
#endif
  }


#else
  std::cout << "No gui available to display an image..." << std::endl;
#endif

  return EXIT_SUCCESS;
}
