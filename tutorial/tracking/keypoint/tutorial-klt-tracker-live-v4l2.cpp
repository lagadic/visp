/*! \example tutorial-klt-tracker-live-v4l2.cpp */
#include <visp/vpImageConvert.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpVideoReader.h>
#include <visp/vpV4l2Grabber.h>

int main(int argc, const char *argv[])
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100) && defined(VISP_HAVE_V4L2)
  try {
    bool opt_init_by_click = false;
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--init-by-click")
        opt_init_by_click = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--init-by-click] [--help]" << std::endl;
        return 0;
      }
    }
#else
    (void)argc;
    (void)argv;
#endif
    vpImage<unsigned char> I;
    vpV4l2Grabber g;
    g.acquire(I);

#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    IplImage * cvI = NULL;
#else
    cv::Mat cvI;
#endif
    vpImageConvert::convert(I, cvI);

    // Display initialisation
    vpDisplayOpenCV d(I, 0, 0, "Klt tracking");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpKltOpencv tracker;
    // Set tracker parameters
    tracker.setMaxFeatures(200);
    tracker.setWindowSize(10);
    tracker.setQuality(0.01);
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);

    // Initialise the tracking
    if (opt_init_by_click) {
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
      vpMouseButton::vpMouseButtonType button;
      std::vector<cv::Point2f> guess;
      vpImagePoint ip;
      do {
        vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
        if (vpDisplay::getClick(I, ip, button, false)) {
          if (button == vpMouseButton::button1) {
            guess.push_back(cv::Point2f(ip.get_u(), ip.get_v()));
            vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
            vpDisplay::displayCross(I, ip, 12, vpColor::green);
          }
        }
        vpDisplay::flush(I);
        vpTime::wait(20);
      } while(button != vpMouseButton::button3);
      tracker.initTracking(guess);
#endif
    }
    else {
      tracker.initTracking(cvI);
    }

    while ( 1 ) {
      g.acquire(I);
      vpDisplay::display(I);

      vpImageConvert::convert(I, cvI);
      tracker.track(cvI);

      tracker.display(I, vpColor::red);
      vpDisplay::displayText(I, 10, 10, "Click to quit", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }

#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    cvReleaseImage(&cvI);
#endif

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
