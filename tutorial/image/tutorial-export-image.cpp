//! \example tutorial-export-image.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(240, 320, 255); // Create a black grey level image
  vpImage<vpRGBa> Ioverlay;

  // Initialize the display with the image I. Display and image are now linked together
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Save overlayed image", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Save overlayed image", vpDisplay::SCALE_AUTO);
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
  std::string ofilename("overlay.png");
  std::cout << "Save overlayed image in: " << ofilename << std::endl;
  vpImageIo::write(Ioverlay, ofilename);
  // Wait for a click in the display window
  std::cout << "A click to quit..." << std::endl;
  vpDisplay::getClick(I);

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  delete d;
#endif
#else
  std::cout << "No gui available to display an image..." << std::endl;
#endif
}
