/*! \example tutorial-grabber-CMU1394.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vp1394CMUGrabber.h>

int main()
{
#ifdef VISP_HAVE_CMU1394
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    vpImage<unsigned char> I;

    vp1394CMUGrabber g;
    g.setVideoMode(0, 1); // 640x480 MONO8
    g.setAutoShutter();
    g.setAutoGain();
    g.setFramerate(4); // 30 fps
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_DISPLAY
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while (1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // A click to exit
        break;
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
#else
  std::cout << "Install CMU1394 SDK, configure and build ViSP again to use this example" << std::endl;
#endif
}
