/*! \example tutorial-grabber-v4l2.cpp */
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImage.h>

/*!
  Usage :
    To get the help    : ./tutorial-grabber-v4l2 --help
    To set the device  : ./tutorial-grabber-v4l2 --device 1 (to use /dev/video1)
    To subsample images: ./tutorial-grabber-v4l2 --scale 2
 */
int main(int argc, const char* argv[])
{
#ifdef VISP_HAVE_V4L2
  try {
    unsigned int opt_device = 0;
    unsigned int opt_scale = 2; // Default value is 2 in the constructor. Turn it to 1 to avoid subsampling

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--device")
        opt_device = (unsigned int)atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--scale")
        opt_scale = (unsigned int)atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--device <camera device>] [--scale <subsampling factor>] [--help]" << std::endl;
        return 0;
      }
    }

    vpImage<unsigned char> I;

    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(opt_scale);
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::displayText(I, 10, 10, "A click to quit", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) break;
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
