/*! \example tutorial-grabber-CMU1394.cpp */
#include <visp/vp1394CMUGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpImage.h>

int main()
{
#ifdef VISP_HAVE_CMU1394
  try {
    vpImage<unsigned char> I;

    vp1394CMUGrabber g;
    g.setVideoMode(0, 1); // 640x480 MONO8
    g.setAutoShutter();
    g.setAutoGain();
    g.setFramerate(4); // 30 fps
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_GDI
    vpDisplayGDI d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // A click to exit
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
