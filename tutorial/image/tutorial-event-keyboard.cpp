//! \example tutorial-event-keyboard.cpp
#include <visp/vpConfig.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black image
#if defined(VISP_HAVE_X11)
  vpDisplay *d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  vpDisplay *d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  vpDisplay *d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  vpDisplay *d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplay *d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif
  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
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
      cpt_event ++;
    }
    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);
#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
