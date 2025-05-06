//! \example tutorial-viewer.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpFont.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
//! [Include display]
#include <visp3/gui/vpDisplayFactory.h>
//! [Include display]
//! [Include io]
#include <visp3/io/vpImageIo.h>
//! [Include io]

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main(int argc, char **argv)
{
#ifdef VISP_HAVE_DISPLAY
  if (argc != 2) {
    printf("Usage: %s <image name.[pgm,ppm,jpeg,png,tiff,bmp,ras,jp2]>\n", argv[0]);
    return EXIT_FAILURE;
  }

  //! [vpImage construction]
  vpImage<vpRGBa> I;
  //! [vpImage construction]

  //! [vpImage reading]
  try {
    vpImageIo::read(I, argv[1]);
  }
  catch (...) {
    std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
    return EXIT_FAILURE;
  }
  //! [vpImage reading]

  try {
    //! [vpDisplay construction]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> pdisp = vpDisplayFactory::createDisplay(I, 10, 10, vpIoTools::getName(argv[1]), vpDisplay::SCALE_AUTO);
#else
    vpDisplay *pdisp = vpDisplayFactory::allocateDisplay(I, 10, 10, vpIoTools::getName(argv[1]), vpDisplay::SCALE_AUTO);
#endif
    //! [vpDisplay construction]

    //! [vpDisplay set title]
    vpDisplay::setTitle(I, vpIoTools::getName(argv[1]));
    //! [vpDisplay set title]

    //! [vpDisplay display]
    vpDisplay::display(I);
    vpDisplay::flush(I);
    //! [vpDisplay display]
    std::cout << "Right click to quit" << std::endl;
    std::cout << "Left click to inspect pixel position (i,j) and RGBa values\n" << std::endl;
    //! [vpDisplay get click]
    bool quit = false;
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button;
    unsigned int scale_factor = vpDisplay::getDownScalingFactor(I);
    vpFont font(14, vpFont::GENERIC_MONOSPACE);
    while (!quit) {
      if (vpDisplay::getClick(I, ip, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        }
        else {
          std::stringstream ss;
          unsigned int i = static_cast<unsigned int>(ip.get_i());
          unsigned int j = static_cast<unsigned int>(ip.get_j());
          ss << i << " " << j << ": " << I[i][j];
          std::cout << ss.str() << std::endl;
          font.drawText(I, ss.str(), vpImagePoint(I.getHeight() - 20*scale_factor, 10), vpColor::red, vpColor::white);
          vpDisplay::display(I);
          vpDisplay::flush(I);
        }
      }
      vpTime::sleepMs(40);
    }

    //! [vpDisplay get click]
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    delete pdisp;
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "No display available!" << std::endl;
#endif
}
