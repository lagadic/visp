//! \example tutorial-grabber-flycapture.cpp
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpImage.h>

int main()
{
#ifdef VISP_HAVE_FLYCAPTURE
  try {
    vpImage<unsigned char> I; // Create a gray level image container
    //! [vpFlyCaptureGrabber construction]
    vpFlyCaptureGrabber g; // Create a grabber based on FlyCapture SDK third party lib
    //! [vpFlyCaptureGrabber construction]

    //! [vpFlyCaptureGrabber settings]
    try {
      g.setShutter(true); // Turn auto shutter on
      g.setGain(true);    // Turn auto gain on
      g.setVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960Y8, FlyCapture2::FRAMERATE_60);
    }
    catch(...) { // If settings are not available just catch execption to continue with default settings
    }
    //! [vpFlyCaptureGrabber settings]
    //! [vpFlyCaptureGrabber open]
    g.open(I);
    //! [vpFlyCaptureGrabber open]

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      //! [vpFlyCaptureGrabber acquire]
      g.acquire(I);
      //! [vpFlyCaptureGrabber acquire]
      //! [vpFlyCaptureGrabber display]
      vpDisplay::display(I);
      //! [vpFlyCaptureGrabber display]
      //! [vpFlyCaptureGrabber click to exit]
      vpDisplay::displayText(I, 15, 15, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(I, false))
        break;
      //! [vpFlyCaptureGrabber click to exit]
      vpDisplay::flush(I);
    }
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#endif
}
