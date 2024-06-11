//! \example tutorial-export-image.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I(240, 320, 255); // Create a black grey level image
  vpImage<vpRGBa> Ioverlay;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  vpDisplay *d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  vpDisplay *d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  vpDisplay *d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  vpDisplay *d = new vpDisplayD3D;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplay *d = new vpDisplayOpenCV;
#endif
  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif
  // Set the display background with image I content
  vpDisplay::display(I);
  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);
  // Flush the foreground and background display
  vpDisplay::flush(I);
  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay);
  // Write the color image on the disk
  std::cout << "Save image in overlay.ppm" << std::endl;
  std::string ofilename("overlay.png");
  vpImageIo::write(Ioverlay, ofilename);
  // Wait for a click in the display window
  std::cout << "A click to quit..." << std::endl;
  vpDisplay::getClick(I);
#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
