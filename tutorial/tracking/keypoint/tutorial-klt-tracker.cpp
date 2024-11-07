//! \example tutorial-klt-tracker.cpp
//! [Include]
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>
//! [Include]

int main(int argc, const char *argv[])
{
//! [Check 3rd party]
#if (defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO) || defined(VISP_HAVE_V4L2)) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  //! [Check 3rd party]
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    std::string opt_videoname = "video-postcard.mp4";
    bool opt_init_by_click = false;
    unsigned int opt_subsample = 1;
    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--videoname")
        opt_videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--init-by-click")
        opt_init_by_click = true;
      else if (std::string(argv[i]) == "--subsample")
        opt_subsample = static_cast<unsigned int>(std::atoi(argv[i + 1]));
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "Usage: " << argv[0]
          << " [--videoname <video name>] [--subsample <scale factor>] [--init-by-click]"
          << " [--help] [-h]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    //! [Create reader]
    vpVideoReader reader;
    reader.setFileName(opt_videoname);
    //! [Create reader]

    //! [Acquire]
    vpImage<unsigned char> I, Iacq;
    reader.acquire(Iacq);
    Iacq.subsample(opt_subsample, opt_subsample, I);
    //! [Acquire]

    //! [Convert to OpenCV image]
    cv::Mat cvI;
    vpImageConvert::convert(I, cvI);
    //! [Convert to OpenCV image]

    //! [Init display]
    vpDisplayOpenCV d(I, 0, 0, "Klt tracking");
    vpDisplay::display(I);
    vpDisplay::flush(I);
    //! [Init display]

    //! [Create tracker]
    vpKltOpencv tracker;
    tracker.setMaxFeatures(200);
    tracker.setWindowSize(10);
    //! [Quality]
    tracker.setQuality(0.01);
    //! [Quality]
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);
    //! [Create tracker]

    // Initialise the tracking
    if (opt_init_by_click) {
      vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
      std::vector<cv::Point2f> feature;
      vpImagePoint ip;
      do {
        vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
        if (vpDisplay::getClick(I, ip, button, false)) {
          if (button == vpMouseButton::button1) {
            feature.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
            vpDisplay::displayCross(I, ip, 12, vpColor::green);
          }
        }
        vpDisplay::flush(I);
        vpTime::wait(20);
      } while (button != vpMouseButton::button3);
      tracker.initTracking(cvI, feature);
    }
    else {
   //! [Init tracker]
      tracker.initTracking(cvI);
      //! [Init tracker]
    }

    //! [How many features]
    std::cout << "Tracker initialized with " << tracker.getNbFeatures() << " features" << std::endl;
    //! [How many features]

    //! [While loop]
    while (!reader.end()) {
      double t = vpTime::measureTimeMs();
      reader.acquire(Iacq);
      Iacq.subsample(opt_subsample, opt_subsample, I);
      vpDisplay::display(I);

      vpImageConvert::convert(I, cvI);

      if (opt_init_by_click && reader.getFrameIndex() == reader.getFirstFrameIndex() + 20) {
        vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
        std::vector<cv::Point2f> feature;
        vpImagePoint ip;
        do {
          vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
          if (vpDisplay::getClick(I, ip, button, false)) {
            if (button == vpMouseButton::button1) {
              feature.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
              vpDisplay::displayCross(I, ip, 12, vpColor::green);
            }
          }
          vpDisplay::flush(I);
          vpTime::wait(20);
        } while (button != vpMouseButton::button3);
        tracker.initTracking(cvI, feature);
      }

      tracker.track(cvI);

      tracker.display(I, vpColor::red);

      vpDisplay::displayText(I, 10, 10, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(I, false))
        break;

      vpDisplay::flush(I);
      if (!reader.isVideoFormat()) {
        vpTime::wait(t, 40);
      }
    }
    //! [While loop]

    //! [Wait click]
    vpDisplay::getClick(I);
    //! [Wait click]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
}
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
