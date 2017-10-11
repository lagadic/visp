/*! \example tutorial-grabber-realsense.cpp */
#include <visp3/sensor/vpRealSense.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpImage.h>

/*!
  Grab images from an Intel realsense camera
 */
int main()
{
#ifdef VISP_HAVE_REALSENSE
  try {
    vpImage<unsigned char> I;

    vpRealSense g;
    unsigned int width = 640, height = 480;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(width, height, rs::format::rgba8, 60));
    g.open();
    g.acquire(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::displayText(I, 10, 10, "A click to quit", vpColor::red);
      std::stringstream ss;
      ss << "Acquisition time: " << vpTime::measureTimeMs() - t << " ms" ;
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) break;
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
