//! \example tutorial-event-keyboard.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(240, 320); // Create a black image

  // Initialize the display with the image I. Display and image are now linked together
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Keyboard event example", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Keyboard event example", vpDisplay::SCALE_AUTO);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);
  // Flush the foreground and background display
  vpDisplay::flush(I);
  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;
  // Non blocking keyboard event loop
  int cpt_event = 0;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    bool event = vpDisplay::getKeyboardEvent(I, &key[0], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event++;
    }
    vpTime::wait(5); // wait 5 ms
  } while (cpt_event < 5);

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  delete d;
#endif

#else
  std::cout << "No gui available to display an image..." << std::endl;
#endif

  return EXIT_SUCCESS;
}
