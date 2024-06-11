/*! \example tutorial-grabber-1394-writer.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoWriter.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main(int argc, char **)
{
#ifdef VISP_HAVE_DC1394
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#ifdef VISP_HAVE_DISPLAY
  vpDisplay *d = vpDisplayFactory::displayFactory();
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  try {
    bool save = false;
    if (argc == 2) {
      save = true;
    }

    vpImage<vpRGBa> I;         // Create a color image container
    bool reset = true;         // Enable bus reset during construction (default)
    vp1394TwoGrabber g(reset); // Create a grabber based on libdc1394-2.x third party lib

    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpVideoWriter writer;
    writer.setFileName("./I%04d.pgm");
    if (save)
      writer.open(I);

#ifdef VISP_HAVE_DISPLAY
    d->init(I);
    while (1)
#else
    // for loop when no display is available to avoid infinite while loop
    for (unsigned int i = 0; i < 1000; ++i)
#endif
    {
      g.acquire(I);

      if (save)
        writer.saveFrame(I);

#ifdef VISP_HAVE_DISPLAY
      vpDisplay::display(I);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) {
        break;
      }
#endif
    }

    if (save) {
      writer.close();
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#ifdef VISP_HAVE_DISPLAY
  if (d != nullptr) {
    delete d;
  }
#endif
#else
  (void)argc;
#endif

  return EXIT_SUCCESS;
}
